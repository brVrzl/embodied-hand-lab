"""Quest-21 to simulated RH56DFX adaptive retarget adapter.

The implementation is intentionally project-native: it uses the committed
RH56DFX MuJoCo actuator/equality semantics and has no NLopt, Pinocchio, SDK, or
physical-hand transport dependency.  Adaptive warm-start, pinch weighting,
and scale-calibration concepts are implemented locally; no upstream source is
copied.
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
RH56_DIGIT_FEATURE_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
)
RH56_MUJOCO_ACTUATOR_ORDER = (
    "thumb_lateral",
    "thumb_close",
    "index",
    "middle",
    "ring",
    "pinky",
)
# Relative MuJoCo travel derived from all 1001 rows of the local vendor-angle
# workbook recorded in assets/rh56_thumb_table_calibration.json.
# Vendor absolute angles (for example 170 deg at one bend endpoint) are not
# MuJoCo qpos zero offsets.
RH56_THUMB_CLOSE_RANGE_RAD = math.radians(40.0)
RH56_THUMB_LATERAL_RANGE_RAD = math.radians(80.0)
RH56_THUMB_PIP_RANGE_RAD = math.radians(44.99504)
RH56_THUMB_DIP_RANGE_RAD = math.radians(35.614928)
RH56_THUMB_PIP_POLYCOEF = (
    0.0,
    0.9093,
    0.38691839905184455,
    -0.11191086847189706,
    0.0,
)
RH56_THUMB_DIP_POLYCOEF = (
    0.0,
    1.33911,
    -0.6236015346424374,
    -0.027454109505147616,
    0.0,
)
RH56_MUJOCO_ACTUATOR_MAX_RAD = {
    "index": 1.70,
    "middle": 1.68,
    "ring": 1.70,
    "pinky": 1.70,
    "thumb_close": RH56_THUMB_CLOSE_RANGE_RAD,
    "thumb_lateral": RH56_THUMB_LATERAL_RANGE_RAD,
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
class PalmLocalFrame:
    """Orthonormal right-hand palm frame expressed in Quest wrist coordinates."""

    across_axis: tuple[float, float, float]
    forward_axis: tuple[float, float, float]
    normal_axis: tuple[float, float, float]
    palm_width_m: float


@dataclass(frozen=True, slots=True)
class InspireRetargetResult:
    timestamp_monotonic_ns: int
    valid: bool
    joint_targets: Mapping[str, float]
    actuator_targets: Mapping[str, float]
    normalized_targets: Mapping[str, float]
    optimizer_cost: float | None
    tracking_confidence: float | None
    pinch_diagnostics: Mapping[str, float | bool | str]
    limit_violations: tuple[str, ...]
    rejection_reason: str | None


@dataclass(frozen=True, slots=True)
class HandRetargetCalibration:
    calibration_id: str
    palm_normalization_scale: float
    digit_scale: tuple[float, float, float, float, float]
    finger_feature_open: tuple[float, float, float, float]
    finger_feature_closed: tuple[float, float, float, float]
    finger_curve_exponent: tuple[float, float, float, float]
    thumb_curve_open_rad: float
    thumb_curve_closed_rad: float
    thumb_close_bend_gain: float
    thumb_close_pinch_assist_gain: float
    thumb_pinch_closed_distance_palm: float
    thumb_pinch_open_distance_palm: float
    thumb_lateral_open_across_palm: float
    thumb_lateral_pregrasp_across_palm: float | None
    thumb_lateral_pregrasp_normalized: float | None
    thumb_lateral_opposed_across_palm: float
    thumb_lateral_frame_epsilon_m: float
    pinch_intent_enter_distance_palm: float
    pinch_intent_exit_distance_palm: float
    pinch_intent_tripod_enter_distance_palm: float
    pinch_intent_tripod_exit_distance_palm: float
    pinch_intent_minimum_finger_curl: float
    pinch_intent_power_grasp_curl: float
    pinch_pose_blending_enabled: bool
    pinch_pose_maximum_weight_step: float
    validated_pinch_poses: Mapping[str, tuple[float, float, float, float, float, float]]
    thumb_first_pinch_enabled: bool
    thumb_first_index_activation: float
    thumb_first_thumb_close_activation: float
    thumb_first_lateral_activation: float
    mcp_flexion_weight: float
    mcp_flexion_deadband: float
    maximum_normalized_step: float

    @classmethod
    def load(cls, path: str | Path) -> "HandRetargetCalibration":
        values = load_yaml(path)
        calibration = values["calibration"]
        thumb_close = calibration.get("thumb_close", {})
        thumb_curve = calibration.get("thumb_curve", {})
        thumb_lateral = calibration.get("thumb_lateral", {})
        pinch_intent = calibration.get("pinch_intent", {})
        pinch_pose_blending = calibration.get("pinch_pose_blending", {})
        thumb_first = calibration.get("thumb_first_pinch", {})
        validated_poses = {
            str(name): tuple(float(value) for value in pose)
            for name, pose in pinch_pose_blending.get("validated_poses", {}).items()
        }
        try:
            palm_normalization_scale = float(
                calibration["palm_normalization_scale"]
            )
            raw_digit_scale = calibration["digit_scale"]
            if not isinstance(raw_digit_scale, (list, tuple)):
                raise TypeError("digit_scale must be a sequence")
            digit_scale = tuple(float(value) for value in raw_digit_scale)
        except (TypeError, ValueError) as exc:
            raise ValueError("malformed RH56 retarget calibration") from exc
        result = cls(
            calibration_id=str(calibration["calibration_id"]),
            palm_normalization_scale=palm_normalization_scale,
            digit_scale=digit_scale,
            finger_feature_open=tuple(
                float(value)
                for value in calibration.get("finger_feature", {}).get(
                    "open", (0.0, 0.0, 0.0, 0.0)
                )
            ),
            finger_feature_closed=tuple(
                float(value)
                for value in calibration.get("finger_feature", {}).get(
                    "closed", (1.0, 1.0, 1.0, 1.0)
                )
            ),
            finger_curve_exponent=tuple(
                float(value)
                for value in calibration.get("finger_feature", {}).get(
                    "curve_exponent", (1.0, 1.0, 1.0, 1.0)
                )
            ),
            thumb_curve_open_rad=float(thumb_curve.get("curve_open_rad", 0.0)),
            thumb_curve_closed_rad=float(
                thumb_curve.get("curve_closed_rad", math.pi)
            ),
            thumb_close_bend_gain=float(thumb_close.get("bend_gain", 1.0)),
            thumb_close_pinch_assist_gain=float(
                thumb_close.get("pinch_assist_gain", 0.4)
            ),
            thumb_pinch_closed_distance_palm=float(
                calibration.get("thumb_pinch_closed_distance_palm", 0.0)
            ),
            thumb_pinch_open_distance_palm=float(
                calibration.get("thumb_pinch_open_distance_palm", 0.70)
            ),
            thumb_lateral_open_across_palm=float(
                thumb_lateral.get("open_across_palm", -0.60)
            ),
            thumb_lateral_pregrasp_across_palm=(
                None
                if "pregrasp_across_palm" not in thumb_lateral
                else float(thumb_lateral["pregrasp_across_palm"])
            ),
            thumb_lateral_pregrasp_normalized=(
                None
                if "pregrasp_normalized" not in thumb_lateral
                else float(thumb_lateral["pregrasp_normalized"])
            ),
            thumb_lateral_opposed_across_palm=float(
                thumb_lateral.get("opposed_across_palm", 0.25)
            ),
            thumb_lateral_frame_epsilon_m=float(
                thumb_lateral.get("frame_epsilon_m", 1e-5)
            ),
            pinch_intent_enter_distance_palm=float(
                pinch_intent.get("enter_distance_palm", 0.15)
            ),
            pinch_intent_exit_distance_palm=float(
                pinch_intent.get("exit_distance_palm", 0.22)
            ),
            pinch_intent_tripod_enter_distance_palm=float(
                pinch_intent.get("tripod_enter_distance_palm", 0.22)
            ),
            pinch_intent_tripod_exit_distance_palm=float(
                pinch_intent.get("tripod_exit_distance_palm", 0.30)
            ),
            pinch_intent_minimum_finger_curl=float(
                pinch_intent.get("minimum_finger_curl", 0.12)
            ),
            pinch_intent_power_grasp_curl=float(
                pinch_intent.get("power_grasp_curl", 0.70)
            ),
            pinch_pose_blending_enabled=bool(
                pinch_pose_blending.get("enabled", False)
            ),
            pinch_pose_maximum_weight_step=float(
                pinch_pose_blending.get("maximum_weight_step", 0.05)
            ),
            validated_pinch_poses=validated_poses,
            thumb_first_pinch_enabled=bool(thumb_first.get("enabled", False)),
            thumb_first_index_activation=float(
                thumb_first.get("index_activation", 0.50)
            ),
            thumb_first_thumb_close_activation=float(
                thumb_first.get("thumb_close_activation", 0.35)
            ),
            thumb_first_lateral_activation=float(
                thumb_first.get("lateral_activation", 0.0)
            ),
            mcp_flexion_weight=float(calibration.get("mcp_flexion_weight", 0.0)),
            mcp_flexion_deadband=float(calibration.get("mcp_flexion_deadband", 0.15)),
            maximum_normalized_step=float(calibration["maximum_normalized_step"]),
        )
        if (
            len(result.digit_scale) != len(RH56_DIGIT_FEATURE_ORDER)
            or len(result.finger_feature_open) != 4
            or len(result.finger_feature_closed) != 4
            or len(result.finger_curve_exponent) != 4
            or not all(math.isfinite(value) and value > 0 for value in (
                result.palm_normalization_scale,
                *result.digit_scale,
                *result.finger_curve_exponent,
            ))
            or not all(
                math.isfinite(open_value)
                and math.isfinite(closed_value)
                and closed_value > open_value
                for open_value, closed_value in zip(
                    result.finger_feature_open,
                    result.finger_feature_closed,
                    strict=True,
                )
            )
            or not math.isfinite(result.thumb_close_bend_gain)
            or result.thumb_close_bend_gain < 0.0
            or not math.isfinite(result.thumb_close_pinch_assist_gain)
            or result.thumb_close_pinch_assist_gain < 0.0
            or not math.isfinite(result.thumb_curve_open_rad)
            or not math.isfinite(result.thumb_curve_closed_rad)
            or result.thumb_curve_closed_rad <= result.thumb_curve_open_rad
            or not 0 <= result.thumb_pinch_closed_distance_palm
            < result.thumb_pinch_open_distance_palm
            or not math.isfinite(result.thumb_lateral_open_across_palm)
            or not math.isfinite(result.thumb_lateral_opposed_across_palm)
            or result.thumb_lateral_open_across_palm
            >= result.thumb_lateral_opposed_across_palm
            or (
                (result.thumb_lateral_pregrasp_across_palm is None)
                != (result.thumb_lateral_pregrasp_normalized is None)
            )
            or (
                result.thumb_lateral_pregrasp_across_palm is not None
                and (
                    not math.isfinite(result.thumb_lateral_pregrasp_across_palm)
                    or not result.thumb_lateral_open_across_palm
                    < result.thumb_lateral_pregrasp_across_palm
                    < result.thumb_lateral_opposed_across_palm
                    or result.thumb_lateral_pregrasp_normalized is None
                    or not math.isfinite(result.thumb_lateral_pregrasp_normalized)
                    or not 0.0 < result.thumb_lateral_pregrasp_normalized < 1.0
                )
            )
            or not math.isfinite(result.thumb_lateral_frame_epsilon_m)
            or result.thumb_lateral_frame_epsilon_m <= 0.0
            or not 0.0 < result.pinch_intent_enter_distance_palm
            < result.pinch_intent_exit_distance_palm
            or not 0.0 < result.pinch_intent_tripod_enter_distance_palm
            < result.pinch_intent_tripod_exit_distance_palm
            or not 0.0 <= result.pinch_intent_minimum_finger_curl < 1.0
            or not result.pinch_intent_minimum_finger_curl
            < result.pinch_intent_power_grasp_curl <= 1.0
            or not 0.0 < result.pinch_pose_maximum_weight_step <= 1.0
            or any(
                name not in {"index", "middle", "tripod"}
                or len(pose) != len(RH56_CANONICAL_ORDER)
                or not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in pose)
                for name, pose in result.validated_pinch_poses.items()
            )
            or not 0.0 <= result.thumb_first_index_activation <= 1.0
            or not 0.0 <= result.thumb_first_thumb_close_activation <= 1.0
            or not 0.0 <= result.thumb_first_lateral_activation <= 1.0
            or not 0 <= result.mcp_flexion_weight <= 1
            or not 0 <= result.mcp_flexion_deadband < 1
            or not 0 < result.maximum_normalized_step <= 1
        ):
            raise ValueError("malformed RH56 retarget calibration")
        return result


class HandRetargeter(Protocol):
    def retarget(self, skeleton: QuestHandSkeleton) -> InspireRetargetResult: ...


def calibrate_finger_feature(
    raw_feature: float,
    *,
    open_feature: float,
    closed_feature: float,
    curve_exponent: float,
) -> float:
    """Map one measured human curl feature onto a monotonic normalized span."""

    values = (raw_feature, open_feature, closed_feature, curve_exponent)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("finger calibration inputs must be finite")
    if closed_feature <= open_feature:
        raise ValueError("finger calibration closed feature must exceed open feature")
    if curve_exponent <= 0.0:
        raise ValueError("finger calibration exponent must be positive")
    normalized = float(
        np.clip(
            (raw_feature - open_feature) / (closed_feature - open_feature),
            0.0,
            1.0,
        )
    )
    return normalized**curve_exponent


def calibrate_thumb_curve(
    raw_curve_rad: float,
    *,
    curve_open_rad: float,
    curve_closed_rad: float,
) -> float:
    """Map the local Quest thumb-chain bend onto the calibrated [0, 1] span."""

    values = (raw_curve_rad, curve_open_rad, curve_closed_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("thumb curve calibration inputs must be finite")
    if curve_closed_rad <= curve_open_rad:
        raise ValueError("thumb curve calibration closed feature must exceed open feature")
    return float(
        np.clip(
            (raw_curve_rad - curve_open_rad)
            / (curve_closed_rad - curve_open_rad),
            0.0,
            1.0,
        )
    )


class PinchIntentDetector:
    """Stateful index/middle/tripod classifier with entry/exit hysteresis."""

    def __init__(
        self,
        *,
        enter_distance_palm: float,
        exit_distance_palm: float,
        tripod_enter_distance_palm: float,
        tripod_exit_distance_palm: float,
        minimum_finger_curl: float,
        power_grasp_curl: float,
    ) -> None:
        self.enter_distance_palm = float(enter_distance_palm)
        self.exit_distance_palm = float(exit_distance_palm)
        self.tripod_enter_distance_palm = float(tripod_enter_distance_palm)
        self.tripod_exit_distance_palm = float(tripod_exit_distance_palm)
        self.minimum_finger_curl = float(minimum_finger_curl)
        self.power_grasp_curl = float(power_grasp_curl)
        self.mode = "none"

    def reset(self) -> None:
        self.mode = "none"

    def update(
        self,
        *,
        thumb_index_distance_palm: float,
        thumb_middle_distance_palm: float,
        index_middle_distance_palm: float,
        index_curl: float,
        middle_curl: float,
        ring_curl: float,
        pinky_curl: float,
        tracking_valid: bool = True,
    ) -> tuple[str, float]:
        values = (
            thumb_index_distance_palm,
            thumb_middle_distance_palm,
            index_middle_distance_palm,
            index_curl,
            middle_curl,
            ring_curl,
            pinky_curl,
        )
        if not tracking_valid or not all(math.isfinite(value) for value in values):
            self.reset()
            return self.mode, 0.0
        if (ring_curl + pinky_curl) / 2.0 >= self.power_grasp_curl:
            self.reset()
            return self.mode, 0.0

        distance_limit = (
            self.exit_distance_palm
            if self.mode != "none"
            else self.enter_distance_palm
        )
        tripod_limit = (
            self.tripod_exit_distance_palm
            if self.mode == "tripod"
            else self.tripod_enter_distance_palm
        )
        tripod_fingertip_limit = (
            self.exit_distance_palm
            if self.mode == "tripod"
            else self.enter_distance_palm
        )
        index_ready = (
            thumb_index_distance_palm <= distance_limit
            and index_curl >= self.minimum_finger_curl
        )
        middle_ready = (
            thumb_middle_distance_palm <= distance_limit
            and middle_curl >= self.minimum_finger_curl
        )
        tripod_ready = (
            thumb_index_distance_palm <= tripod_fingertip_limit
            and thumb_middle_distance_palm <= tripod_fingertip_limit
            and index_curl >= self.minimum_finger_curl
            and middle_curl >= self.minimum_finger_curl
            and index_middle_distance_palm <= tripod_limit
        )

        if tripod_ready:
            self.mode = "tripod"
        elif self.mode == "index" and index_ready:
            pass
        elif self.mode == "middle" and middle_ready:
            pass
        elif index_ready and not middle_ready:
            self.mode = "index"
        elif middle_ready and not index_ready:
            self.mode = "middle"
        elif index_ready and middle_ready:
            self.mode = (
                "index"
                if thumb_index_distance_palm <= thumb_middle_distance_palm
                else "middle"
            )
        else:
            self.mode = "none"

        if self.mode == "none":
            return self.mode, 0.0
        relevant_distances = {
            "index": (thumb_index_distance_palm,),
            "middle": (thumb_middle_distance_palm,),
            "tripod": (
                thumb_index_distance_palm,
                thumb_middle_distance_palm,
            ),
        }[self.mode]
        confidence = min(
            float(
                np.clip(
                    (self.exit_distance_palm - distance)
                    / (self.exit_distance_palm - self.enter_distance_palm),
                    0.0,
                    1.0,
                )
            )
            for distance in relevant_distances
        )
        if self.mode == "tripod":
            confidence = min(
                confidence,
                float(
                    np.clip(
                        (
                            self.tripod_exit_distance_palm
                            - index_middle_distance_palm
                        )
                        / (
                            self.tripod_exit_distance_palm
                            - self.tripod_enter_distance_palm
                        ),
                        0.0,
                        1.0,
                    )
                ),
            )
        return self.mode, confidence


class PinchPoseBlender:
    """Continuously blend retarget output toward physically validated poses."""

    def __init__(
        self,
        validated_poses: Mapping[
            str, tuple[float, float, float, float, float, float]
        ],
        *,
        maximum_weight_step: float,
    ) -> None:
        self.validated_poses = {
            name: np.asarray(pose, dtype=np.float64)
            for name, pose in validated_poses.items()
        }
        self.maximum_weight_step = float(maximum_weight_step)
        self.mode = "none"
        self.weight = 0.0

    def reset(self) -> None:
        self.mode = "none"
        self.weight = 0.0

    def update(
        self,
        continuous_target: np.ndarray,
        *,
        detected_mode: str,
        confidence: float,
        tracking_valid: bool = True,
    ) -> tuple[np.ndarray, str, float]:
        continuous = np.asarray(continuous_target, dtype=np.float64)
        if continuous.shape != (len(RH56_CANONICAL_ORDER),) or not np.all(
            np.isfinite(continuous)
        ):
            raise ValueError("continuous pinch-blend target must be six finite channels")
        desired_mode = (
            detected_mode
            if tracking_valid
            and detected_mode in self.validated_poses
            and math.isfinite(confidence)
            and confidence > 0.0
            else "none"
        )
        desired_weight = float(np.clip(confidence, 0.0, 1.0))

        if self.mode == "none" and desired_mode != "none":
            self.mode = desired_mode
        elif self.mode != "none" and desired_mode != self.mode:
            desired_weight = 0.0

        weight_delta = float(
            np.clip(
                desired_weight - self.weight,
                -self.maximum_weight_step,
                self.maximum_weight_step,
            )
        )
        self.weight = float(np.clip(self.weight + weight_delta, 0.0, 1.0))
        if self.weight == 0.0 and desired_mode != self.mode:
            self.mode = desired_mode
        if self.mode == "none":
            return continuous.copy(), self.mode, self.weight

        pose = self.validated_poses[self.mode]
        blended = (1.0 - self.weight) * continuous + self.weight * pose
        return blended, self.mode, self.weight


class ThumbFirstPinchSequencer:
    """Observe verified index-pinch entry without reshaping the target.

    RH56 has one thumb opposition actuator shared by the thumb mechanism.  A
    simultaneous six-channel target can therefore let the index occupy the
    space that the thumb needs for opposition. The validated triplet is
    already below the known self-collision boundary,
    so a smaller index/thumb-close preposition would only create an unnatural
    retreat.  This state machine now provides diagnostics only; every target
    passes through unchanged and remains subject to the normal command delta
    and safety gates.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        index_activation: float = 0.50,
        thumb_close_activation: float = 0.35,
        lateral_activation: float = 0.0,
    ) -> None:
        self.enabled = bool(enabled)
        self.index_activation = float(index_activation)
        self.thumb_close_activation = float(thumb_close_activation)
        self.lateral_activation = float(lateral_activation)
        self.stage = "idle"

    def reset(self) -> None:
        self.stage = "idle"

    def update(
        self,
        canonical_target: np.ndarray,
        *,
        detected_mode: str,
        confidence: float,
        tracking_valid: bool,
        measured_canonical: np.ndarray | None = None,
    ) -> tuple[np.ndarray, str]:
        target = np.asarray(canonical_target, dtype=np.float64).copy()
        if target.shape != (len(RH56_CANONICAL_ORDER),) or not np.all(
            np.isfinite(target)
        ):
            raise ValueError("thumb-first target must be six finite channels")
        active = (
            self.enabled
            and tracking_valid
            and detected_mode == "index"
            and math.isfinite(confidence)
            and confidence > 0.0
        )
        if not active:
            self.reset()
            return target, self.stage
        if measured_canonical is not None:
            measured = np.asarray(measured_canonical, dtype=np.float64)
            if measured.shape != target.shape or not np.all(np.isfinite(measured)):
                measured = None
        else:
            measured = None

        # Do not turn every index-pinch intent into a staged pose.  The gate is
        # only relevant once the continuous retarget is already close to the
        # previously verified physical index-pinch pose.  In particular, an
        # earlier approach target such as [index=.48, thumb_close=.38,
        # lateral=.65] must pass through unchanged so the thumb can continue
        # its natural side swing.
        near_verified_pose = (
            target[0] >= self.index_activation
            and target[4] >= self.thumb_close_activation
            and target[5] >= self.lateral_activation
        )
        if measured is None or not near_verified_pose:
            self.reset()
            return target, self.stage
        # The current measured pose may be far from opposition, but the
        # validated index value is known not to occupy the thumb's lateral
        # workspace. Approach all three pinch channels directly.
        self.stage = "index_approach"
        return target, self.stage


class ProjectRh56Retargeter:
    """Adaptive Quest hand feature retargeter for the six RH56 channels."""

    def __init__(self, calibration: HandRetargetCalibration) -> None:
        self.calibration = calibration
        self._previous: np.ndarray | None = None
        self._last_finger_raw = np.zeros(4, dtype=np.float64)
        self._last_finger_calibrated = np.zeros(4, dtype=np.float64)
        self._pinch_intent = PinchIntentDetector(
            enter_distance_palm=calibration.pinch_intent_enter_distance_palm,
            exit_distance_palm=calibration.pinch_intent_exit_distance_palm,
            tripod_enter_distance_palm=(
                calibration.pinch_intent_tripod_enter_distance_palm
            ),
            tripod_exit_distance_palm=(
                calibration.pinch_intent_tripod_exit_distance_palm
            ),
            minimum_finger_curl=calibration.pinch_intent_minimum_finger_curl,
            power_grasp_curl=calibration.pinch_intent_power_grasp_curl,
        )

    def reset(self) -> None:
        self._previous = None
        self._last_finger_raw.fill(0.0)
        self._last_finger_calibrated.fill(0.0)
        self._pinch_intent.reset()

    def retarget(self, skeleton: QuestHandSkeleton) -> InspireRetargetResult:
        if not skeleton.valid or len(skeleton.positions_m) != 21:
            self.reset()
            return InspireRetargetResult(
                skeleton.timestamp_monotonic_ns,
                False,
                {},
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
                {},
                {},
                {},
                None,
                skeleton.tracking_confidence,
                {},
                (),
                "NONFINITE_HAND_SKELETON",
            )
        palm_frame = right_hand_palm_local_frame(
            points,
            epsilon_m=self.calibration.thumb_lateral_frame_epsilon_m,
        )
        if palm_frame is None:
            return InspireRetargetResult(
                skeleton.timestamp_monotonic_ns,
                False,
                {},
                {},
                {},
                None,
                skeleton.tracking_confidence,
                {},
                (),
                "DEGENERATE_PALM_FRAME",
            )
        palm = max(float(np.linalg.norm(points[9] - points[0])), 1e-6)
        fingertip_indices = (8, 12, 16, 20)
        thumb_tip_distances = np.asarray(
            [np.linalg.norm(points[4] - points[index]) / palm for index in fingertip_indices]
        )
        thumb_tip_distances_m = np.asarray(
            [np.linalg.norm(points[4] - points[index]) for index in fingertip_indices]
        )
        index_middle_distance_palm = float(
            np.linalg.norm(points[8] - points[12]) / palm
        )
        pinch_strengths = np.clip(
            (
                self.calibration.thumb_pinch_open_distance_palm
                - thumb_tip_distances
            )
            / (
                self.calibration.thumb_pinch_open_distance_palm
                - self.calibration.thumb_pinch_closed_distance_palm
            ),
            0.0,
            1.0,
        )
        # AnyDex's adaptive Inspire objective gives the thumb the strongest
        # pinch alpha from all non-thumb fingertips. RH56 has one coupled thumb
        # closing actuator, so the project-native equivalent is a single
        # closest-fingertip strength rather than an index-only special case.
        pinch = float(np.max(pinch_strengths))
        closest_index = int(np.argmin(thumb_tip_distances))
        raw_thumb_bend_rad, _ = _finger_angle_curl_components(points, (1, 2, 3, 4))
        normalized_thumb_bend = calibrate_thumb_curve(
            raw_thumb_bend_rad,
            curve_open_rad=self.calibration.thumb_curve_open_rad,
            curve_closed_rad=self.calibration.thumb_curve_closed_rad,
        )
        thumb_close, bend_contribution, pinch_assist_contribution = (
            thumb_close_bend_primary_feature(
                normalized_thumb_bend,
                pinch,
                bend_gain=self.calibration.thumb_close_bend_gain,
                pinch_assist_gain=self.calibration.thumb_close_pinch_assist_gain,
            )
        )
        thumb_lateral, raw_thumb_lateral = thumb_lateral_opposition_feature(
            points,
            palm_frame,
            open_across_palm=self.calibration.thumb_lateral_open_across_palm,
            pregrasp_across_palm=(
                self.calibration.thumb_lateral_pregrasp_across_palm
            ),
            pregrasp_normalized=(
                self.calibration.thumb_lateral_pregrasp_normalized
            ),
            opposed_across_palm=self.calibration.thumb_lateral_opposed_across_palm,
            palm_normalization_scale=self.calibration.palm_normalization_scale,
        )
        normalized = self._adaptive(points, thumb_close, thumb_lateral)
        pinch_mode, pinch_confidence = self._pinch_intent.update(
            thumb_index_distance_palm=float(thumb_tip_distances[0]),
            thumb_middle_distance_palm=float(thumb_tip_distances[1]),
            index_middle_distance_palm=index_middle_distance_palm,
            index_curl=float(normalized[0]),
            middle_curl=float(normalized[1]),
            ring_curl=float(normalized[2]),
            pinky_curl=float(normalized[3]),
        )
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
        diagnostics: dict[str, float | bool | str] = {
            "thumb_index_distance_palm": float(thumb_tip_distances[0]),
            "thumb_middle_distance_palm": float(thumb_tip_distances[1]),
            "index_middle_distance_palm": index_middle_distance_palm,
            "thumb_ring_distance_palm": float(thumb_tip_distances[2]),
            "thumb_pinky_distance_palm": float(thumb_tip_distances[3]),
            "thumb_index_pinch_strength": float(pinch_strengths[0]),
            "thumb_closest_fingertip_pinch_strength": pinch,
            "thumb_any_fingertip_pinching": pinch > 0.7,
            "pinch_mode": pinch_mode,
            "pinch_confidence": pinch_confidence,
            "thumb_raw_bend_rad": raw_thumb_bend_rad,
            "thumb_normalized_bend": normalized_thumb_bend,
            "thumb_curve_open_rad": self.calibration.thumb_curve_open_rad,
            "thumb_curve_closed_rad": self.calibration.thumb_curve_closed_rad,
            "thumb_closest_fingertip_index": fingertip_indices[closest_index],
            "thumb_raw_pinch_distance_m": float(
                thumb_tip_distances_m[closest_index]
            ),
            "thumb_raw_pinch_distance_palm": float(
                thumb_tip_distances[closest_index]
            ),
            "thumb_normalized_pinch": pinch,
            "thumb_base_bend_contribution": bend_contribution,
            "thumb_pinch_assist_contribution": pinch_assist_contribution,
            "thumb_close_feature": thumb_close,
            "thumb_effective_feature": float(normalized[4]),
            "thumb_lateral_raw_across_palm": raw_thumb_lateral,
            "thumb_lateral_feature": thumb_lateral,
            "thumb_lateral_effective_feature": float(normalized[5]),
            "palm_width_m": palm_frame.palm_width_m,
            "palm_across_x": palm_frame.across_axis[0],
            "palm_across_y": palm_frame.across_axis[1],
            "palm_across_z": palm_frame.across_axis[2],
            "palm_forward_x": palm_frame.forward_axis[0],
            "palm_forward_y": palm_frame.forward_axis[1],
            "palm_forward_z": palm_frame.forward_axis[2],
            "palm_normal_x": palm_frame.normal_axis[0],
            "palm_normal_y": palm_frame.normal_axis[1],
            "palm_normal_z": palm_frame.normal_axis[2],
        }
        for index, name in enumerate(RH56_CANONICAL_ORDER[:4]):
            diagnostics[f"{name}_raw_curl_feature"] = float(
                self._last_finger_raw[index]
            )
            diagnostics[f"{name}_calibrated_curl_feature"] = float(
                self._last_finger_calibrated[index]
            )
        return InspireRetargetResult(
            skeleton.timestamp_monotonic_ns,
            True,
            joints,
            actuators,
            canonical,
            cost,
            skeleton.tracking_confidence,
            diagnostics,
            violations,
            None,
        )

    def _adaptive(
        self,
        p: np.ndarray,
        thumb_close: float,
        thumb_lateral: float,
    ) -> np.ndarray:
        palm_forward = p[9] - p[0]
        raw_curls = np.asarray(
            [
                _finger_full_hand_curl(
                    p, (5, 6, 7, 8), palm_forward,
                    weight=self.calibration.mcp_flexion_weight,
                    deadband=self.calibration.mcp_flexion_deadband,
                ),
                _finger_full_hand_curl(
                    p, (9, 10, 11, 12), palm_forward,
                    weight=self.calibration.mcp_flexion_weight,
                    deadband=self.calibration.mcp_flexion_deadband,
                ),
                _finger_full_hand_curl(
                    p, (13, 14, 15, 16), palm_forward,
                    weight=self.calibration.mcp_flexion_weight,
                    deadband=self.calibration.mcp_flexion_deadband,
                ),
                _finger_full_hand_curl(
                    p, (17, 18, 19, 20), palm_forward,
                    weight=self.calibration.mcp_flexion_weight,
                    deadband=self.calibration.mcp_flexion_deadband,
                ),
            ]
        )
        curls = self._calibrate_finger_features(raw_curls)
        # Canonical feature order is index, middle, ring, pinky, thumb_close,
        # thumb_lateral. Only the first five are digit-derived and scaled.
        digit_features = np.asarray([*curls, thumb_close])
        digit_features *= np.asarray(self.calibration.digit_scale)
        return np.asarray([*digit_features, thumb_lateral])

    def _calibrate_finger_features(self, raw: np.ndarray) -> np.ndarray:
        curved = np.asarray(
            [
                calibrate_finger_feature(
                    float(value),
                    open_feature=open_feature,
                    closed_feature=closed_feature,
                    curve_exponent=exponent,
                )
                for value, open_feature, closed_feature, exponent in zip(
                    raw,
                    self.calibration.finger_feature_open,
                    self.calibration.finger_feature_closed,
                    self.calibration.finger_curve_exponent,
                    strict=True,
                )
            ],
            dtype=np.float64,
        )
        self._last_finger_raw = raw.copy()
        self._last_finger_calibrated = curved.copy()
        return curved


def right_hand_palm_local_frame(
    points: np.ndarray,
    *,
    epsilon_m: float,
) -> PalmLocalFrame | None:
    """Build a right-handed orthonormal frame from wrist-local Quest landmarks."""

    if (
        points.shape != (21, 3)
        or not np.all(np.isfinite(points))
        or not math.isfinite(epsilon_m)
        or epsilon_m <= 0.0
    ):
        return None
    across_raw = points[17] - points[5]  # right index MCP -> right pinky MCP
    palm_width = float(np.linalg.norm(across_raw))
    if palm_width <= epsilon_m:
        return None
    across = across_raw / palm_width
    forward_raw = points[9] - points[0]  # wrist -> middle MCP
    forward_orthogonal = forward_raw - float(np.dot(forward_raw, across)) * across
    forward_norm = float(np.linalg.norm(forward_orthogonal))
    if forward_norm <= epsilon_m:
        return None
    forward = forward_orthogonal / forward_norm
    normal = np.cross(across, forward)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= epsilon_m:
        return None
    normal /= normal_norm
    # Recompute forward from the two normalized axes to remove accumulated
    # floating-point skew while preserving wrist-to-middle direction.
    forward = np.cross(normal, across)
    forward /= float(np.linalg.norm(forward))
    return PalmLocalFrame(
        tuple(float(value) for value in across),
        tuple(float(value) for value in forward),
        tuple(float(value) for value in normal),
        palm_width,
    )


def thumb_lateral_opposition_feature(
    points: np.ndarray,
    frame: PalmLocalFrame,
    *,
    open_across_palm: float,
    pregrasp_across_palm: float | None = None,
    pregrasp_normalized: float | None = None,
    opposed_across_palm: float,
    palm_normalization_scale: float,
) -> tuple[float, float]:
    """Map right-thumb base-to-tip motion toward the pinky to [0, 1]."""

    values = (
        open_across_palm,
        opposed_across_palm,
        palm_normalization_scale,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("thumb-lateral calibration must be finite")
    if (
        open_across_palm >= opposed_across_palm
        or palm_normalization_scale <= 0.0
    ):
        raise ValueError("invalid thumb-lateral calibration")
    if (pregrasp_across_palm is None) != (pregrasp_normalized is None):
        raise ValueError("thumb-lateral pregrasp anchor must be complete")
    if pregrasp_across_palm is not None:
        assert pregrasp_normalized is not None
        if (
            not math.isfinite(pregrasp_across_palm)
            or not math.isfinite(pregrasp_normalized)
            or not open_across_palm < pregrasp_across_palm < opposed_across_palm
            or not 0.0 < pregrasp_normalized < 1.0
        ):
            raise ValueError("invalid thumb-lateral pregrasp anchor")
    across = np.asarray(frame.across_axis, dtype=np.float64)
    palm_width = frame.palm_width_m * palm_normalization_scale
    if not math.isfinite(palm_width) or palm_width <= 0.0:
        raise ValueError("invalid thumb-lateral palm normalization scale")
    raw = float(np.dot(points[4] - points[1], across) / palm_width)
    if pregrasp_across_palm is None:
        normalized = float(
            np.clip(
                (raw - open_across_palm)
                / (opposed_across_palm - open_across_palm),
                0.0,
                1.0,
            )
        )
    else:
        assert pregrasp_normalized is not None
        normalized = float(
            np.interp(
                raw,
                (open_across_palm, pregrasp_across_palm, opposed_across_palm),
                (0.0, pregrasp_normalized, 1.0),
            )
        )
    return normalized, raw


def _finger_angle_curl(points: np.ndarray, indices: tuple[int, int, int, int]) -> float:
    return _finger_angle_curl_components(points, indices)[1]


def _finger_angle_curl_components(
    points: np.ndarray,
    indices: tuple[int, int, int, int],
) -> tuple[float, float]:
    a, b, c, d = (points[index] for index in indices)
    raw_bend_rad = (math.pi - _angle(a, b, c)) + (math.pi - _angle(b, c, d))
    return raw_bend_rad, float(np.clip(raw_bend_rad / math.pi, 0.0, 1.0))


def thumb_close_bend_primary_feature(
    normalized_thumb_bend: float,
    normalized_pinch: float,
    *,
    bend_gain: float,
    pinch_assist_gain: float,
) -> tuple[float, float, float]:
    """Return bend-primary thumb-close feature and its two contributions."""

    values = (
        normalized_thumb_bend,
        normalized_pinch,
        bend_gain,
        pinch_assist_gain,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("thumb-close blend inputs must be finite")
    if bend_gain < 0.0 or pinch_assist_gain < 0.0:
        raise ValueError("thumb-close blend gains must be non-negative")
    bend = float(np.clip(normalized_thumb_bend, 0.0, 1.0))
    pinch = float(np.clip(normalized_pinch, 0.0, 1.0))
    base = bend_gain * bend
    assist = pinch_assist_gain * max(0.0, pinch - bend)
    return float(np.clip(base + assist, 0.0, 1.0)), base, assist


def _finger_full_hand_curl(
    points: np.ndarray,
    indices: tuple[int, int, int, int],
    palm_forward: np.ndarray,
    *,
    weight: float,
    deadband: float,
) -> float:
    """Blend MCP flexion with the established PIP/DIP curl feature.

    The previous project-native feature ignored MCP-only motion because it
    measured just the two distal bends.  AnyDex's full-hand vector objective
    also constrains wrist-to-PIP/DIP/tip vectors; this lightweight equivalent
    preserves the validated distal feature exactly and adds MCP flexion only
    in its remaining unsaturated range.  It does not introduce an optimizer or
    change RH56 ordering.
    """

    a, b, _, _ = (points[index] for index in indices)
    distal = _finger_angle_curl(points, indices)
    mcp = float(np.clip(math.acos(_cos(b - a, palm_forward)) / (math.pi / 2.0), 0.0, 1.0))
    mcp = float(np.clip((mcp - deadband) / (1.0 - deadband), 0.0, 1.0))
    return float(np.clip(distal + weight * mcp * (1.0 - distal), 0.0, 1.0))


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
    thumb_pip, thumb_dip = thumb_close_coupled_joint_positions(thumb_close)
    return {
        "rh56_R_thumb_MCP_joint1": thumb_lateral,
        "rh56_R_thumb_MCP_joint2": thumb_close,
        "rh56_R_thumb_PIP_joint": thumb_pip,
        "rh56_R_thumb_DIP_joint": thumb_dip,
        "rh56_R_index_MCP_joint": actuators["index"],
        "rh56_R_index_DIP_joint": actuators["index"],
        "rh56_R_middle_MCP_joint": actuators["middle"],
        "rh56_R_middle_DIP_joint": actuators["middle"],
        "rh56_R_ring_MCP_joint": actuators["ring"],
        "rh56_R_ring_DIP_joint": actuators["ring"],
        "rh56_R_pinky_MCP_joint": actuators["pinky"],
        "rh56_R_pinky_DIP_joint": actuators["pinky"],
    }


def thumb_close_coupled_joint_positions(thumb_close_rad: float) -> tuple[float, float]:
    """Return relative PIP/DIP qpos from the monotonic 1001-row vendor table fit."""

    value = float(thumb_close_rad)
    if not math.isfinite(value):
        raise ValueError("thumb close qpos must be finite")
    if not 0.0 <= value <= RH56_THUMB_CLOSE_RANGE_RAD + 1e-12:
        raise ValueError("thumb close qpos is outside the calibrated relative range")

    def evaluate(coefficients: tuple[float, ...]) -> float:
        return float(
            sum(
                coefficient * value**power
                for power, coefficient in enumerate(coefficients)
            )
        )

    return evaluate(RH56_THUMB_PIP_POLYCOEF), evaluate(RH56_THUMB_DIP_POLYCOEF)
