"""AnyDexRetarget-informed Quest-21 to simulated RH56DFX adapter.

The implementation is intentionally project-native: it uses the committed
RH56DFX MuJoCo actuator/equality semantics and has no NLopt, Pinocchio, SDK, or
physical-hand transport dependency.  AnyDexRetarget's adaptive/key-vector,
warm-start, pinch weighting, and scale-calibration concepts informed the two
small backends below; no upstream source is copied.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Mapping, Protocol

import numpy as np

from embodiment_core.config import load_yaml
from motion_input import QuestHandObservation, Side
from motion_input.hts_protocol import HTS_JOINT_NAMES


RH56_CANONICAL_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
RH56_MUJOCO_ACTUATOR_ORDER = (
    "thumb_lateral",
    "thumb_close",
    "index",
    "middle",
    "ring",
    "pinky",
)
RH56_MUJOCO_ACTUATOR_MAX_RAD = {
    "index": 1.70,
    "middle": 1.68,
    "ring": 1.70,
    "pinky": 1.70,
    "thumb_close": 0.50,
    "thumb_lateral": 1.10,
}
RH56_FULL_JOINT_ORDER = (
    "rh56_R_thumb_MCP_joint1",
    "rh56_R_thumb_MCP_joint2",
    "rh56_R_thumb_PIP_joint",
    "rh56_R_thumb_DIP_joint",
    "rh56_R_index_MCP_joint",
    "rh56_R_index_DIP_joint",
    "rh56_R_middle_MCP_joint",
    "rh56_R_middle_DIP_joint",
    "rh56_R_ring_MCP_joint",
    "rh56_R_ring_DIP_joint",
    "rh56_R_pinky_MCP_joint",
    "rh56_R_pinky_DIP_joint",
)


@dataclass(frozen=True, slots=True)
class QuestHandSkeleton:
    timestamp_monotonic_ns: int
    side: Side
    frame_id: str
    joint_names: tuple[str, ...]
    positions_m: tuple[tuple[float, float, float], ...]
    valid: bool
    tracking_confidence: float | None

    @classmethod
    def from_observation(cls, hand: QuestHandObservation) -> "QuestHandSkeleton":
        names = tuple(joint.name for joint in hand.joints)
        valid = (
            hand.side is Side.RIGHT
            and hand.tracking_valid
            and len(hand.joints) == 21
            and names == HTS_JOINT_NAMES
            and hand.host_receive_monotonic_ns is not None
        )
        return cls(
            timestamp_monotonic_ns=int(hand.host_receive_monotonic_ns or 0),
            side=hand.side,
            frame_id=hand.wrist_frame_id,
            joint_names=names,
            positions_m=tuple(joint.position_m for joint in hand.joints),
            valid=valid,
            tracking_confidence=hand.confidence,
        )


@dataclass(frozen=True, slots=True)
class InspireRetargetResult:
    timestamp_monotonic_ns: int
    valid: bool
    backend: str
    joint_targets: Mapping[str, float]
    actuator_targets: Mapping[str, float]
    optimizer_cost: float | None
    tracking_confidence: float | None
    pinch_diagnostics: Mapping[str, float | bool]
    limit_violations: tuple[str, ...]
    rejection_reason: str | None


@dataclass(frozen=True, slots=True)
class HandRetargetCalibration:
    calibration_id: str
    global_scale: float
    palm_scale: float
    finger_scale: tuple[float, float, float, float]
    thumb_scale: float
    key_vector_scale: float
    pinch_weight: float
    maximum_normalized_step: float
    loss_behavior: str

    @classmethod
    def load(cls, path: str | Path) -> tuple[str, "HandRetargetCalibration"]:
        values = load_yaml(path)
        calibration = values["calibration"]
        result = cls(
            calibration_id=str(calibration["calibration_id"]),
            global_scale=float(calibration["global_scale"]),
            palm_scale=float(calibration["palm_scale"]),
            finger_scale=tuple(float(value) for value in calibration["finger_scale"]),
            thumb_scale=float(calibration["thumb_scale"]),
            key_vector_scale=float(calibration["key_vector_scale"]),
            pinch_weight=float(calibration["pinch_weight"]),
            maximum_normalized_step=float(calibration["maximum_normalized_step"]),
            loss_behavior=str(calibration.get("loss_behavior", "safe_open")),
        )
        if (
            len(result.finger_scale) != 4
            or not all(math.isfinite(value) and value > 0 for value in (
                result.global_scale,
                result.palm_scale,
                *result.finger_scale,
                result.thumb_scale,
                result.key_vector_scale,
            ))
            or not 0 <= result.pinch_weight <= 1
            or not 0 < result.maximum_normalized_step <= 1
            or result.loss_behavior not in {"safe_open", "hold"}
        ):
            raise ValueError("malformed RH56 retarget calibration")
        return str(values.get("selected_backend", "adaptive")), result


class HandRetargeter(Protocol):
    def retarget(self, skeleton: QuestHandSkeleton) -> InspireRetargetResult: ...


class ProjectRh56Retargeter:
    """Selectable angle-adaptive or AnyDex-style key-vector feature backend."""

    def __init__(self, calibration: HandRetargetCalibration, *, backend: str) -> None:
        if backend not in {"adaptive", "vector"}:
            raise ValueError("backend must be 'adaptive' or 'vector'")
        self.calibration = calibration
        self.backend = backend
        self._previous: np.ndarray | None = None

    def reset(self) -> None:
        self._previous = None

    def retarget(self, skeleton: QuestHandSkeleton) -> InspireRetargetResult:
        if not skeleton.valid or len(skeleton.positions_m) != 21:
            self.reset()
            return InspireRetargetResult(
                skeleton.timestamp_monotonic_ns,
                False,
                self.backend,
                {},
                {},
                None,
                skeleton.tracking_confidence,
                {},
                (),
                "INVALID_RIGHT_HAND_SKELETON",
            )
        points = np.asarray(skeleton.positions_m, dtype=np.float64)
        if points.shape != (21, 3) or not np.all(np.isfinite(points)):
            self.reset()
            return InspireRetargetResult(
                skeleton.timestamp_monotonic_ns,
                False,
                self.backend,
                {},
                {},
                None,
                skeleton.tracking_confidence,
                {},
                (),
                "NONFINITE_HAND_SKELETON",
            )
        palm = max(float(np.linalg.norm(points[9] - points[0])), 1e-6)
        thumb_index_distance = float(np.linalg.norm(points[4] - points[8]) / palm)
        pinch = float(np.clip(1.0 - thumb_index_distance / 0.70, 0.0, 1.0))
        if self.backend == "adaptive":
            normalized = self._adaptive(points, pinch)
        else:
            normalized = self._vector(points, pinch)
        unbounded = normalized.copy()
        normalized = np.clip(normalized, 0.0, 1.0)
        violations = tuple(
            f"{name}_outside_normalized_limit"
            for name, before in zip(RH56_CANONICAL_ORDER, unbounded, strict=True)
            if before < 0.0 or before > 1.0
        )
        if self._previous is not None:
            delta = np.clip(
                normalized - self._previous,
                -self.calibration.maximum_normalized_step,
                self.calibration.maximum_normalized_step,
            )
            normalized = self._previous + delta
        self._previous = normalized.copy()
        canonical = dict(zip(RH56_CANONICAL_ORDER, normalized.tolist(), strict=True))
        actuators = {
            name: canonical[name] * RH56_MUJOCO_ACTUATOR_MAX_RAD[name]
            for name in RH56_MUJOCO_ACTUATOR_ORDER
        }
        joints = _expand_mimic_targets(actuators)
        cost = float(np.mean((unbounded - normalized) ** 2))
        return InspireRetargetResult(
            skeleton.timestamp_monotonic_ns,
            True,
            self.backend,
            joints,
            actuators,
            cost,
            skeleton.tracking_confidence,
            {
                "thumb_index_distance_palm": thumb_index_distance,
                "thumb_index_pinch_strength": pinch,
                "thumb_index_pinching": pinch > 0.7,
            },
            violations,
            None,
        )

    def _adaptive(self, p: np.ndarray, pinch: float) -> np.ndarray:
        curls = np.asarray(
            [
                _finger_angle_curl(p, (5, 6, 7, 8)),
                _finger_angle_curl(p, (9, 10, 11, 12)),
                _finger_angle_curl(p, (13, 14, 15, 16)),
                _finger_angle_curl(p, (17, 18, 19, 20)),
            ]
        ) * np.asarray(self.calibration.finger_scale)
        thumb_bend = _finger_angle_curl(p, (1, 2, 3, 4))
        thumb_close = (
            (1.0 - self.calibration.pinch_weight) * thumb_bend
            + self.calibration.pinch_weight * pinch
        ) * self.calibration.thumb_scale
        palm_side = p[17] - p[5]
        thumb_side = p[4] - p[1]
        lateral = float(np.clip((_cos(thumb_side, palm_side) + 1.0) / 2.0, 0.0, 1.0))
        return np.asarray([*curls, thumb_close, lateral])

    def _vector(self, p: np.ndarray, pinch: float) -> np.ndarray:
        palm_forward = p[9] - p[0]
        curls = []
        for base, tip, scale in zip((5, 9, 13, 17), (8, 12, 16, 20), self.calibration.finger_scale, strict=True):
            projection = _cos(p[tip] - p[base], palm_forward)
            curls.append(float(np.clip((1.0 - projection) * self.calibration.key_vector_scale, 0.0, 1.0)) * scale)
        thumb_vector = p[4] - p[1]
        index_vector = p[8] - p[5]
        thumb_close = max((1.0 - _cos(thumb_vector, index_vector)) * 0.5, pinch)
        palm_side = p[17] - p[5]
        lateral = (_cos(thumb_vector, palm_side) + 1.0) * 0.5
        return np.asarray([*curls, thumb_close * self.calibration.thumb_scale, lateral])


def _finger_angle_curl(points: np.ndarray, indices: tuple[int, int, int, int]) -> float:
    a, b, c, d = (points[index] for index in indices)
    return float(np.clip(((math.pi - _angle(a, b, c)) + (math.pi - _angle(b, c, d))) / math.pi, 0.0, 1.0))


def _angle(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> float:
    left, right = a - b, c - b
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    return math.pi if denominator <= 1e-9 else math.acos(float(np.clip(np.dot(left, right) / denominator, -1.0, 1.0)))


def _cos(left: np.ndarray, right: np.ndarray) -> float:
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    return 0.0 if denominator <= 1e-9 else float(np.clip(np.dot(left, right) / denominator, -1.0, 1.0))


def _expand_mimic_targets(actuators: Mapping[str, float]) -> dict[str, float]:
    thumb_lateral = actuators["thumb_lateral"]
    thumb_close = actuators["thumb_close"]
    return {
        "rh56_R_thumb_MCP_joint1": thumb_lateral,
        "rh56_R_thumb_MCP_joint2": thumb_close,
        "rh56_R_thumb_PIP_joint": 0.6 * thumb_close,
        "rh56_R_thumb_DIP_joint": 0.8 * thumb_close,
        "rh56_R_index_MCP_joint": actuators["index"],
        "rh56_R_index_DIP_joint": actuators["index"],
        "rh56_R_middle_MCP_joint": actuators["middle"],
        "rh56_R_middle_DIP_joint": actuators["middle"],
        "rh56_R_ring_MCP_joint": actuators["ring"],
        "rh56_R_ring_DIP_joint": actuators["ring"],
        "rh56_R_pinky_MCP_joint": actuators["pinky"],
        "rh56_R_pinky_DIP_joint": actuators["pinky"],
    }
