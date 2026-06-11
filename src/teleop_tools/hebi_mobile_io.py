from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from teleop_tools.direction_calibration import (
    AxisMapping,
    DEFAULT_PHONE_TO_ROBOT_TRANSLATION_MAP,
    DEFAULT_PHONE_WRIST_ROLL_MAP,
    apply_vector_axis_map,
    parse_scalar_axis_map,
    parse_vector_axis_map,
)


def _unit_quat_wxyz(quat: list[float] | np.ndarray) -> np.ndarray:
    q = np.asarray(quat, dtype=np.float64)
    if q.shape != (4,):
        raise ValueError("quaternion must contain 4 values in wxyz order.")
    norm = float(np.linalg.norm(q))
    if norm <= 1e-9:
        return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def quat_conjugate_wxyz(quat: list[float] | np.ndarray) -> np.ndarray:
    q = _unit_quat_wxyz(quat)
    return np.asarray([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _quat_multiply_raw_wxyz(a: list[float] | np.ndarray, b: list[float] | np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.asarray(a, dtype=np.float64)
    bw, bx, by, bz = np.asarray(b, dtype=np.float64)
    return np.asarray(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def quat_multiply_wxyz(a: list[float] | np.ndarray, b: list[float] | np.ndarray) -> np.ndarray:
    return _unit_quat_wxyz(_quat_multiply_raw_wxyz(_unit_quat_wxyz(a), _unit_quat_wxyz(b)))


def rotate_vector_wxyz(quat: list[float] | np.ndarray, vector: list[float] | np.ndarray) -> np.ndarray:
    q = _unit_quat_wxyz(quat)
    v = np.asarray(vector, dtype=np.float64)
    if v.shape != (3,):
        raise ValueError("vector must contain 3 values.")
    pure = np.asarray([0.0, v[0], v[1], v[2]], dtype=np.float64)
    rotated = _quat_multiply_raw_wxyz(_quat_multiply_raw_wxyz(q, pure), quat_conjugate_wxyz(q))
    return rotated[1:]


def quat_to_rotvec_wxyz(quat: list[float] | np.ndarray) -> np.ndarray:
    q = _unit_quat_wxyz(quat)
    vector = q[1:]
    sin_half = float(np.linalg.norm(vector))
    if sin_half <= 1e-9:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(sin_half, float(q[0]))
    return vector / sin_half * angle


def apply_deadband_vector(values: list[float] | np.ndarray, deadband: float) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    output = array.copy()
    small = np.abs(output) <= max(0.0, float(deadband))
    output[small] = 0.0
    return output


def clip_vector_norm(values: list[float] | np.ndarray, max_norm: float) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    limit = abs(float(max_norm))
    norm = float(np.linalg.norm(array))
    if limit <= 0.0:
        return np.zeros_like(array)
    if norm <= limit or norm <= 1e-9:
        return array
    return array * (limit / norm)


@dataclass(frozen=True, slots=True)
class HebiMobileIOSnapshot:
    timestamp_sec: float
    position_m: list[float]
    quaternion_wxyz: list[float]
    raw_inputs: dict[str, float | int | bool] = field(default_factory=dict)
    valid: bool = True
    reason: str = "ok"

    @property
    def enabled(self) -> bool:
        return bool(self.raw_inputs.get("b1", False))

    @property
    def quaternion_xyzw(self) -> list[float]:
        w, x, y, z = self.quaternion_wxyz
        return [x, y, z, w]

    def to_dict(self, *, elapsed_sec: float | None = None) -> dict[str, Any]:
        payload = {
            "timestamp_sec": float(self.timestamp_sec),
            "position_m": [float(v) for v in self.position_m],
            "quaternion_wxyz": [float(v) for v in self.quaternion_wxyz],
            "raw_inputs": dict(self.raw_inputs),
            "valid": bool(self.valid),
            "reason": self.reason,
        }
        if elapsed_sec is not None:
            payload["elapsed_sec"] = float(elapsed_sec)
        return payload

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "HebiMobileIOSnapshot":
        return cls(
            timestamp_sec=float(payload.get("timestamp_sec", time.time())),
            position_m=[float(v) for v in payload.get("position_m", [0.0, 0.0, 0.0])],
            quaternion_wxyz=[float(v) for v in payload.get("quaternion_wxyz", [1.0, 0.0, 0.0, 0.0])],
            raw_inputs=dict(payload.get("raw_inputs", {})),
            valid=bool(payload.get("valid", True)),
            reason=str(payload.get("reason", "ok")),
        )


@dataclass(frozen=True, slots=True)
class HebiPalmTargetAction:
    palm_displacement_m: list[float]
    palm_velocity_m_s: list[float]
    wrist_roll_velocity_rad_s: float
    deadman: bool
    calibrated_position_m: list[float]
    calibrated_rotvec_rad: list[float]
    raw_inputs: dict[str, float | int | bool] = field(default_factory=dict)
    precision: bool = False


class MobileIOLike(Protocol):
    def update(self, timeout_ms: float | None = None) -> bool: ...
    def get_button_state(self, index: int) -> bool: ...
    def get_axis_state(self, index: int) -> float: ...


class HebiMobileIOClient:
    """Small wrapper around HEBI Mobile I/O with ARKit pose feedback."""

    def __init__(
        self,
        *,
        family: str = "HEBI",
        name: str = "mobileIO",
        lookup_wait_sec: float = 2.0,
        setup_ui: bool = True,
        max_stale_feedback_sec: float = 0.25,
    ) -> None:
        self.family = family
        self.name = name
        self.lookup_wait_sec = float(lookup_wait_sec)
        self.setup_ui = bool(setup_ui)
        self.max_stale_feedback_sec = float(max_stale_feedback_sec)
        self.hebi: Any | None = None
        self.mobile_io: Any | None = None
        self.last_snapshot: HebiMobileIOSnapshot | None = None

    @property
    def is_connected(self) -> bool:
        return self.mobile_io is not None

    def connect(self) -> None:
        try:
            import hebi  # type: ignore
        except Exception as exc:
            raise RuntimeError("HEBI Python API is required. Install with: pip install hebi-py") from exc
        self.hebi = hebi
        lookup = hebi.Lookup()
        time.sleep(self.lookup_wait_sec)
        self.mobile_io = hebi.util.create_mobile_io(lookup, family=self.family, name=self.name)
        if self.mobile_io is None:
            entries = [f"{entry.family}/{entry.name}" for entry in getattr(lookup, "entrylist", [])]
            raise RuntimeError(
                f"HEBI Mobile I/O not found for family={self.family!r}, name={self.name!r}. "
                f"Check the iOS app settings and that the phone is on the same network. "
                f"Discovered HEBI entries: {entries}"
            )
        if self.setup_ui:
            self._setup_mobile_ui()

    def _setup_mobile_ui(self) -> None:
        if self.mobile_io is None:
            return
        for idx, label in ((1, "B1 deadman"), (8, "B8 quit")):
            try:
                self.mobile_io.set_button_label(idx, label)
            except Exception:
                pass
        try:
            self.mobile_io.set_axis_label(3, "precision")
        except Exception:
            pass

    def read(self, *, timeout_ms: float | None = None) -> HebiMobileIOSnapshot:
        if self.mobile_io is None:
            raise RuntimeError("HEBI Mobile I/O client is not connected.")
        now = time.time()
        if not self.mobile_io.update(timeout_ms=timeout_ms):
            return self._make_stale_snapshot(now, "feedback_timeout")
        position = getattr(self.mobile_io, "position", None)
        orientation = getattr(self.mobile_io, "orientation", None)
        if position is None or orientation is None:
            return self._make_stale_snapshot(now, "missing_ar_pose")
        q = np.asarray(orientation, dtype=np.float64)
        if q.shape == (4,):
            # HEBI Mobile IO exposes xyzw in current hebi-py; store wxyz internally.
            quaternion_wxyz = [float(q[3]), float(q[0]), float(q[1]), float(q[2])]
        else:
            quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]
        raw_inputs: dict[str, float | int | bool] = {}
        for idx in range(1, 9):
            try:
                raw_inputs[f"b{idx}"] = bool(self.mobile_io.get_button_state(idx))
            except Exception:
                pass
        for idx in range(1, 9):
            try:
                raw_inputs[f"a{idx}"] = float(self.mobile_io.get_axis_state(idx))
            except Exception:
                pass
        snapshot = HebiMobileIOSnapshot(
            timestamp_sec=now,
            position_m=[float(v) for v in np.asarray(position, dtype=np.float64)[:3]],
            quaternion_wxyz=_unit_quat_wxyz(quaternion_wxyz).astype(float).tolist(),
            raw_inputs=raw_inputs,
            valid=True,
            reason="ok",
        )
        self.last_snapshot = snapshot
        return snapshot

    def _make_stale_snapshot(self, now: float, reason: str) -> HebiMobileIOSnapshot:
        if (
            self.last_snapshot is not None
            and now - self.last_snapshot.timestamp_sec <= self.max_stale_feedback_sec
        ):
            return self.last_snapshot
        return HebiMobileIOSnapshot(
            timestamp_sec=now,
            position_m=[0.0, 0.0, 0.0],
            quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
            raw_inputs={},
            valid=False,
            reason=reason,
        )


class HebiMobileIOReplay:
    """Replay recorded Mobile I/O snapshots on the original relative timeline."""

    def __init__(
        self,
        path: str | Path,
        *,
        speed: float = 1.0,
        loop: bool = False,
        max_stale_feedback_sec: float = 0.25,
    ) -> None:
        self.records = self._load_records(Path(path))
        if not self.records:
            raise RuntimeError(f"HEBI replay file has no snapshot rows: {path}")
        self.speed = max(float(speed), 1e-6)
        self.loop = bool(loop)
        self.max_stale_feedback_sec = float(max_stale_feedback_sec)
        self.start_wall_sec = time.time()
        self.start_elapsed_sec = float(self.records[0].get("elapsed_sec", 0.0))
        self.last_snapshot: HebiMobileIOSnapshot | None = None

    @staticmethod
    def _load_records(path: Path) -> list[dict[str, Any]]:
        records: list[dict[str, Any]] = []
        with path.open("r", encoding="utf-8") as stream:
            for line in stream:
                payload = json.loads(line)
                snapshot = payload.get("snapshot", payload)
                if "position_m" in snapshot and "quaternion_wxyz" in snapshot:
                    records.append(snapshot)
        return sorted(records, key=lambda item: float(item.get("elapsed_sec", item.get("timestamp_sec", 0.0))))

    @property
    def exhausted(self) -> bool:
        if self.loop:
            return False
        elapsed = (time.time() - self.start_wall_sec) * self.speed + self.start_elapsed_sec
        return elapsed > float(self.records[-1].get("elapsed_sec", self.records[-1].get("timestamp_sec", 0.0)))

    def read_at_elapsed(self, elapsed_sec: float, *, wall_timestamp_sec: float | None = None) -> HebiMobileIOSnapshot:
        return self._read_at_replay_elapsed(float(elapsed_sec), wall_timestamp_sec=wall_timestamp_sec)

    def read(self, *, timeout_ms: float | None = None) -> HebiMobileIOSnapshot:
        del timeout_ms
        replay_elapsed = (time.time() - self.start_wall_sec) * self.speed + self.start_elapsed_sec
        return self._read_at_replay_elapsed(replay_elapsed)

    def _read_at_replay_elapsed(
        self,
        replay_elapsed: float,
        *,
        wall_timestamp_sec: float | None = None,
    ) -> HebiMobileIOSnapshot:
        duration = float(self.records[-1].get("elapsed_sec", replay_elapsed))
        if self.loop and duration > 0.0:
            replay_elapsed = replay_elapsed % duration
        record = self.records[-1]
        for candidate in self.records:
            if float(candidate.get("elapsed_sec", candidate.get("timestamp_sec", 0.0))) <= replay_elapsed:
                record = candidate
            else:
                break
        snapshot = HebiMobileIOSnapshot.from_dict(record)
        snapshot = HebiMobileIOSnapshot(
            timestamp_sec=time.time() if wall_timestamp_sec is None else float(wall_timestamp_sec),
            position_m=snapshot.position_m,
            quaternion_wxyz=snapshot.quaternion_wxyz,
            raw_inputs=snapshot.raw_inputs,
            valid=snapshot.valid,
            reason=snapshot.reason,
        )
        self.last_snapshot = snapshot
        return snapshot


class HebiPalmTargetMapper:
    """Map calibrated phone displacement into bounded palm-target velocity."""

    def __init__(
        self,
        *,
        max_translation_velocity_m_s: float,
        max_wrist_roll_velocity_rad_s: float,
        translation_gain_s_inv: float = 1.0,
        rotation_gain_s_inv: float = 1.0,
        translation_deadband_m: float = 0.005,
        rotation_deadband_rad: float = math.radians(2.0),
        precision_axis: str = "a3",
        precision_min_scale: float = 0.25,
        velocity_filter_time_constant_sec: float = 0.0,
        max_translation_velocity_slew_m_s2: float = math.inf,
        max_wrist_roll_velocity_slew_rad_s2: float = math.inf,
        phone_to_robot_axis_map: dict[str, Any] | None = None,
        wrist_roll_axis_map: dict[str, Any] | str | None = None,
    ) -> None:
        self.max_translation_velocity_m_s = abs(float(max_translation_velocity_m_s))
        self.max_wrist_roll_velocity_rad_s = abs(float(max_wrist_roll_velocity_rad_s))
        self.translation_gain_s_inv = float(translation_gain_s_inv)
        self.rotation_gain_s_inv = float(rotation_gain_s_inv)
        self.translation_deadband_m = abs(float(translation_deadband_m))
        self.rotation_deadband_rad = abs(float(rotation_deadband_rad))
        self.precision_axis = str(precision_axis)
        self.precision_min_scale = max(0.0, min(1.0, float(precision_min_scale)))
        self.velocity_filter_time_constant_sec = max(0.0, float(velocity_filter_time_constant_sec))
        self.max_translation_velocity_slew_m_s2 = abs(float(max_translation_velocity_slew_m_s2))
        self.max_wrist_roll_velocity_slew_rad_s2 = abs(float(max_wrist_roll_velocity_slew_rad_s2))
        self.phone_to_robot_axis_map = parse_vector_axis_map(
            phone_to_robot_axis_map,
            default=DEFAULT_PHONE_TO_ROBOT_TRANSLATION_MAP,
        )
        self.wrist_roll_axis_map: AxisMapping = parse_scalar_axis_map(
            wrist_roll_axis_map,
            default=DEFAULT_PHONE_WRIST_ROLL_MAP,
        )
        self.reference_snapshot: HebiMobileIOSnapshot | None = None
        self._filtered_palm_velocity = np.zeros(3, dtype=np.float64)
        self._filtered_wrist_velocity = 0.0
        self._last_filter_timestamp_sec: float | None = None

    def reset_reference(self, snapshot: HebiMobileIOSnapshot) -> None:
        self.reference_snapshot = snapshot
        self._reset_velocity_filter()

    def _reset_velocity_filter(self) -> None:
        self._filtered_palm_velocity[:] = 0.0
        self._filtered_wrist_velocity = 0.0
        self._last_filter_timestamp_sec = None

    def _filter_velocity(
        self,
        palm_velocity: np.ndarray,
        wrist_velocity: float,
        *,
        timestamp_sec: float,
    ) -> tuple[np.ndarray, float]:
        if self.velocity_filter_time_constant_sec <= 0.0:
            self._filtered_palm_velocity = palm_velocity.copy()
            self._filtered_wrist_velocity = float(wrist_velocity)
            self._last_filter_timestamp_sec = timestamp_sec
            return self._filtered_palm_velocity.copy(), self._filtered_wrist_velocity
        if self._last_filter_timestamp_sec is None:
            alpha = 1.0
        else:
            dt = max(0.0, min(timestamp_sec - self._last_filter_timestamp_sec, 0.1))
            alpha = dt / (self.velocity_filter_time_constant_sec + dt) if dt > 0.0 else 0.0
        self._last_filter_timestamp_sec = timestamp_sec
        self._filtered_palm_velocity = self._filtered_palm_velocity + alpha * (
            palm_velocity - self._filtered_palm_velocity
        )
        self._filtered_wrist_velocity += alpha * (float(wrist_velocity) - self._filtered_wrist_velocity)
        return self._filtered_palm_velocity.copy(), self._filtered_wrist_velocity

    def map(self, snapshot: HebiMobileIOSnapshot) -> HebiPalmTargetAction:
        if not snapshot.valid or not snapshot.enabled:
            self._reset_velocity_filter()
            return HebiPalmTargetAction(
                palm_displacement_m=[0.0, 0.0, 0.0],
                palm_velocity_m_s=[0.0, 0.0, 0.0],
                wrist_roll_velocity_rad_s=0.0,
                deadman=False,
                calibrated_position_m=snapshot.position_m,
                calibrated_rotvec_rad=[0.0, 0.0, 0.0],
                raw_inputs=snapshot.raw_inputs,
                precision=False,
            )
        if self.reference_snapshot is None:
            self.reset_reference(snapshot)
        assert self.reference_snapshot is not None
        raw_delta = np.asarray(snapshot.position_m, dtype=np.float64) - np.asarray(
            self.reference_snapshot.position_m,
            dtype=np.float64,
        )
        values = {"x": raw_delta[0], "y": raw_delta[1], "z": raw_delta[2]}
        robot_displacement = apply_vector_axis_map(values, self.phone_to_robot_axis_map)
        robot_displacement = apply_deadband_vector(robot_displacement, self.translation_deadband_m)

        delta_q = quat_multiply_wxyz(snapshot.quaternion_wxyz, quat_conjugate_wxyz(self.reference_snapshot.quaternion_wxyz))
        rotvec = quat_to_rotvec_wxyz(delta_q)
        rot_values = {"rot_x": rotvec[0], "rot_y": rotvec[1], "rot_z": rotvec[2]}
        wrist_velocity = self.wrist_roll_axis_map.apply(rot_values) * self.rotation_gain_s_inv
        if abs(wrist_velocity) <= self.rotation_deadband_rad * self.rotation_gain_s_inv:
            wrist_velocity = 0.0

        palm_velocity = robot_displacement * self.translation_gain_s_inv
        palm_velocity = clip_vector_norm(palm_velocity, self.max_translation_velocity_m_s)
        wrist_velocity = max(-self.max_wrist_roll_velocity_rad_s, min(self.max_wrist_roll_velocity_rad_s, wrist_velocity))

        precision_raw = float(snapshot.raw_inputs.get(self.precision_axis, 1.0))
        precision_scale = self.precision_min_scale + (1.0 - self.precision_min_scale) * max(0.0, min(1.0, precision_raw))
        palm_velocity *= precision_scale
        wrist_velocity *= precision_scale

        palm_velocity, wrist_velocity = self._filter_velocity(
            palm_velocity,
            wrist_velocity,
            timestamp_sec=snapshot.timestamp_sec,
        )
        return HebiPalmTargetAction(
            palm_displacement_m=robot_displacement.astype(float).tolist(),
            palm_velocity_m_s=palm_velocity.astype(float).tolist(),
            wrist_roll_velocity_rad_s=float(wrist_velocity),
            deadman=True,
            calibrated_position_m=snapshot.position_m,
            calibrated_rotvec_rad=rotvec.astype(float).tolist(),
            raw_inputs=snapshot.raw_inputs,
            precision=precision_scale < 0.99,
        )
