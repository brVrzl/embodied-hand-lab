from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from teleop_tools.direction_calibration import DEFAULT_PHONE_TO_ROBOT_TRANSLATION_MAP, apply_vector_axis_map, parse_vector_axis_map
from teleop_tools.hebi_mobile_io import (
    HebiMobileIOSnapshot,
    quat_conjugate_wxyz,
    quat_multiply_wxyz,
    quat_to_rotvec_wxyz,
)


IDENTITY_QUAT_WXYZ = [1.0, 0.0, 0.0, 0.0]


@dataclass(slots=True)
class TcpPose:
    position_m: list[float]
    quaternion_wxyz: list[float] = field(default_factory=lambda: list(IDENTITY_QUAT_WXYZ))

    def __post_init__(self) -> None:
        if len(self.position_m) != 3:
            raise ValueError("TcpPose.position_m must contain 3 values.")
        if len(self.quaternion_wxyz) != 4:
            raise ValueError("TcpPose.quaternion_wxyz must contain 4 values in wxyz order.")
        self.position_m = [float(v) for v in self.position_m]
        self.quaternion_wxyz = _unit_quat(self.quaternion_wxyz).astype(float).tolist()

    def to_dict(self) -> dict[str, Any]:
        return {
            "position_m": [float(v) for v in self.position_m],
            "quaternion_wxyz": [float(v) for v in self.quaternion_wxyz],
        }


@dataclass(frozen=True, slots=True)
class RelativePoseLagFollowConfig:
    target_response_mode: str = "lag_follow"
    position_scale: float = 1.0
    max_step_position_m: float = 0.01
    max_step_rotation_rad: float = math.radians(2.0)
    max_target_lead_m: float = 0.08
    workspace_min_m: tuple[float, float, float] = (-1.0, -1.0, -1.0)
    workspace_max_m: tuple[float, float, float] = (1.0, 1.0, 1.0)
    max_pos_tracking_error_warn_m: float = 0.03
    max_pos_tracking_error_pause_m: float = 0.08
    max_rot_tracking_error_warn_rad: float = math.radians(8.0)
    max_rot_tracking_error_pause_rad: float = math.radians(20.0)
    max_q_tracking_error_pause_rad: float = 0.25
    min_warn_time_scale: float = 0.25
    phone_translation_deadband_m: float = 0.003
    phone_rotation_deadband_rad: float = math.radians(1.0)
    phone_jump_reject_translation_m: float = 0.25
    phone_jump_reject_rotation_rad: float = math.radians(45.0)
    phone_still_translation_m: float = 0.002
    phone_still_rotation_rad: float = math.radians(0.5)
    phone_still_min_sec: float = 0.0
    phone_still_freeze_tracking_error_m: float = 0.03
    freeze_when_phone_still: bool = True
    target_filter_time_constant_sec: float = 0.10
    max_target_velocity_m_s: float = 0.02
    max_target_acceleration_m_s2: float = 0.0
    max_target_jump_m: float = 0.05
    target_update_deadband_m: float = 0.0
    target_update_release_m: float = 0.0
    reanchor_requires_deadman_release: bool = False
    phone_to_robot_axis_map: dict[str, Any] | None = None


@dataclass(frozen=True, slots=True)
class IkCheckResult:
    success: bool
    q_cmd: list[float] | None = None
    safety_limited: bool = False
    reason: str = "ok"


@dataclass(frozen=True, slots=True)
class RelativePoseLagFollowOutput:
    command_deadman: bool
    palm_target_position_m: list[float] | None
    wrist_roll_velocity_rad_s: float
    log: dict[str, Any]


IkChecker = Callable[[TcpPose, list[float]], IkCheckResult]


def _unit_quat(values: list[float] | np.ndarray) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (4,):
        raise ValueError("quaternion must contain 4 values.")
    norm = float(np.linalg.norm(array))
    if norm <= 1e-9:
        return np.asarray(IDENTITY_QUAT_WXYZ, dtype=np.float64)
    return array / norm


def pose_rotation_error_rad(a: TcpPose, b: TcpPose) -> float:
    delta = quat_multiply_wxyz(a.quaternion_wxyz, quat_conjugate_wxyz(b.quaternion_wxyz))
    return float(np.linalg.norm(quat_to_rotvec_wxyz(delta)))


def _pose_to_dict_or_none(pose: TcpPose | None) -> dict[str, Any] | None:
    return None if pose is None else pose.to_dict()


def _as_position(values: list[float] | np.ndarray) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (3,):
        raise ValueError("position must contain 3 values.")
    return array


def _rotvec_to_quat_wxyz(rotvec: list[float] | np.ndarray) -> np.ndarray:
    vector = np.asarray(rotvec, dtype=np.float64)
    angle = float(np.linalg.norm(vector))
    if angle <= 1e-9:
        return np.asarray(IDENTITY_QUAT_WXYZ, dtype=np.float64)
    axis = vector / angle
    half = angle * 0.5
    return np.asarray([math.cos(half), *(axis * math.sin(half))], dtype=np.float64)


def _quat_slerp_step(current: list[float], target: list[float], max_step_rad: float) -> np.ndarray:
    current_q = _unit_quat(current)
    target_q = _unit_quat(target)
    delta = quat_multiply_wxyz(target_q, quat_conjugate_wxyz(current_q))
    rotvec = quat_to_rotvec_wxyz(delta)
    angle = float(np.linalg.norm(rotvec))
    if angle <= abs(float(max_step_rad)):
        return target_q
    stepped_rotvec = rotvec * (abs(float(max_step_rad)) / max(angle, 1e-9))
    return quat_multiply_wxyz(_rotvec_to_quat_wxyz(stepped_rotvec), current_q)


def _warn_ratio(value: float, *, warn: float, pause: float) -> float:
    if pause <= warn:
        return 1.0 if value >= pause else 0.0
    return float(max(0.0, min(1.0, (value - warn) / (pause - warn))))


class RelativePoseLagFollower:
    """Lag-aware relative phone-pose target generator for JAKA palm-target teleop."""

    def __init__(
        self,
        config: RelativePoseLagFollowConfig | None = None,
        *,
        ik_checker: IkChecker | None = None,
    ) -> None:
        self.config = config or RelativePoseLagFollowConfig()
        if self.config.target_response_mode not in {"lag_follow", "direct"}:
            raise ValueError("target_response_mode must be 'lag_follow' or 'direct'.")
        self.ik_checker = ik_checker
        self.phone_to_robot_axis_map = parse_vector_axis_map(
            self.config.phone_to_robot_axis_map,
            default=DEFAULT_PHONE_TO_ROBOT_TRANSLATION_MAP,
        )
        self.phone_anchor_pose: TcpPose | None = None
        self.robot_anchor_pose: TcpPose | None = None
        self.last_phone_pose: TcpPose | None = None
        self.last_target_pose: TcpPose | None = None
        self.last_raw_target_pose: TcpPose | None = None
        self.last_q_cmd: list[float] | None = None
        self.last_timestamp_sec: float | None = None
        self.last_target_velocity_m_s = np.zeros(3, dtype=np.float64)
        self.phone_still_started_at_sec: float | None = None
        self.target_deadband_holding = False
        self.waiting_for_deadman_release_after_reject = False

    def reset(self) -> None:
        self.phone_anchor_pose = None
        self.robot_anchor_pose = None
        self.last_phone_pose = None
        self.last_target_pose = None
        self.last_raw_target_pose = None
        self.last_q_cmd = None
        self.last_timestamp_sec = None
        self.last_target_velocity_m_s = np.zeros(3, dtype=np.float64)
        self.phone_still_started_at_sec = None
        self.target_deadband_holding = False

    def step(
        self,
        snapshot: HebiMobileIOSnapshot,
        actual_tcp_pose: TcpPose,
        q_current: list[float],
        *,
        timestamp_sec: float | None = None,
    ) -> RelativePoseLagFollowOutput:
        if len(q_current) != 6:
            raise ValueError("q_current must contain 6 values.")
        timestamp = float(snapshot.timestamp_sec if timestamp_sec is None else timestamp_sec)
        phone_pose = TcpPose(snapshot.position_m, snapshot.quaternion_wxyz)
        log = self._base_log(timestamp, snapshot, actual_tcp_pose, q_current)
        if not snapshot.valid or not snapshot.enabled:
            self.reset()
            self.waiting_for_deadman_release_after_reject = False
            log.update({"command_deadman": False, "reason": snapshot.reason})
            return RelativePoseLagFollowOutput(False, None, 0.0, log)
        if self.waiting_for_deadman_release_after_reject:
            log.update(
                {
                    "command_deadman": False,
                    "reason": "waiting_for_deadman_release_after_reject",
                    "reanchor_requires_deadman_release": True,
                }
            )
            return RelativePoseLagFollowOutput(False, None, 0.0, log)
        if self.phone_anchor_pose is None or self.robot_anchor_pose is None:
            self._anchor(phone_pose, actual_tcp_pose)
        assert self.phone_anchor_pose is not None
        assert self.robot_anchor_pose is not None

        dt = self._step_dt(timestamp)
        phone_step_translation_m, phone_step_rotation_rad = self._phone_step(phone_pose)
        phone_pose_jump_rejected = (
            phone_step_translation_m > self.config.phone_jump_reject_translation_m
            or phone_step_rotation_rad > self.config.phone_jump_reject_rotation_rad
        )
        if phone_pose_jump_rejected:
            self.reset()
            self.waiting_for_deadman_release_after_reject = (
                self.config.reanchor_requires_deadman_release
            )
            log.update(
                {
                    "command_deadman": False,
                    "reason": "phone_pose_jump_rejected",
                    "phone_step_translation_m": phone_step_translation_m,
                    "phone_step_rotation_rad": phone_step_rotation_rad,
                    "reanchor_requires_deadman_release": self.config.reanchor_requires_deadman_release,
                }
            )
            return RelativePoseLagFollowOutput(False, None, 0.0, log)

        phone_delta_pose = self._phone_delta(phone_pose)
        mapped_delta = self._map_phone_delta_to_robot(phone_delta_pose.position_m)
        desired_position = _as_position(self.robot_anchor_pose.position_m) + mapped_delta * self.config.position_scale
        desired_raw = TcpPose(desired_position.astype(float).tolist(), self.robot_anchor_pose.quaternion_wxyz)
        raw_target_jump_rejected = self._raw_target_jump_rejected(desired_raw)
        if raw_target_jump_rejected:
            self.reset()
            log.update(
                {
                    "command_deadman": False,
                    "reason": "raw_target_jump_rejected",
                    "desired_tcp_pose_raw": desired_raw.to_dict(),
                }
            )
            return RelativePoseLagFollowOutput(False, None, 0.0, log)

        desired_workspace_bounded = self._workspace_bounded_target(desired_raw)
        phone_still_now = self._phone_still(phone_step_translation_m, phone_step_rotation_rad)
        phone_still, phone_still_duration_sec = self._update_phone_still(
            phone_still_now,
            timestamp,
        )
        (
            desired_bounded,
            target_lead_limited,
            target_filtered,
            target_velocity_limited,
            target_acceleration_limited,
            target_deadband_hold,
            still_freeze,
        ) = self._command_target(
            actual_tcp_pose=actual_tcp_pose,
            desired_workspace_bounded=desired_workspace_bounded,
            dt=dt,
            phone_still=phone_still,
        )
        pos_error, rot_error, q_error = self._tracking_errors(desired_bounded, actual_tcp_pose, q_current)
        time_scale, lag_warn, lag_pause = self._time_scale(pos_error, rot_error, q_error)
        command_deadman = not lag_pause
        ik = self._check_ik(desired_bounded, q_current)
        if not ik.success:
            command_deadman = False

        self.last_phone_pose = phone_pose
        self.last_timestamp_sec = timestamp
        self.last_raw_target_pose = desired_raw
        if command_deadman and not still_freeze:
            self.last_target_pose = desired_bounded
            self.last_q_cmd = ik.q_cmd
        log.update(
            {
                "command_deadman": command_deadman,
                "target_response_mode": self.config.target_response_mode,
                "phone_anchor_pose": _pose_to_dict_or_none(self.phone_anchor_pose),
                "phone_delta_pose": phone_delta_pose.to_dict(),
                "phone_step_translation_m": phone_step_translation_m,
                "phone_step_rotation_rad": phone_step_rotation_rad,
                "phone_still": phone_still,
                "phone_still_now": phone_still_now,
                "phone_still_duration_sec": phone_still_duration_sec,
                "phone_still_min_sec": self.config.phone_still_min_sec,
                "mapped_phone_delta_m": mapped_delta.astype(float).tolist(),
                "desired_tcp_pose_raw": desired_raw.to_dict(),
                "accepted_desired_tcp_pose_raw": desired_raw.to_dict() if command_deadman else None,
                "desired_tcp_pose_workspace_bounded": desired_workspace_bounded.to_dict(),
                "desired_tcp_pose_bounded": desired_bounded.to_dict(),
                "still_freeze_tracking_error_m": self._position_error(
                    desired_workspace_bounded,
                    actual_tcp_pose,
                ),
                "still_freeze_tracking_error_limit_m": self.config.phone_still_freeze_tracking_error_m,
                "actual_tcp_pose": actual_tcp_pose.to_dict(),
                "target_filtered": target_filtered,
                "target_velocity_limited": target_velocity_limited,
                "target_acceleration_limited": target_acceleration_limited,
                "target_deadband_hold": target_deadband_hold,
                "still_freeze": still_freeze,
                "tcp_tracking_error_pos": pos_error,
                "tcp_tracking_error_rot": rot_error,
                "q_cmd": ik.q_cmd,
                "q_current": list(q_current),
                "q_tracking_error": q_error,
                "time_scale": time_scale,
                "effective_time_scale": time_scale if command_deadman else 0.0,
                "lag_warn": lag_warn,
                "lag_pause": lag_pause,
                "target_lead_limited": target_lead_limited,
                "reanchor_requires_deadman_release": self.config.reanchor_requires_deadman_release,
                "ik_success": ik.success,
                "ik_reason": ik.reason,
                "teleop_mode": "relative_pose_lag_follow",
                "locked_anchor_p0": True,
            }
        )
        return RelativePoseLagFollowOutput(
            command_deadman=command_deadman,
            palm_target_position_m=desired_bounded.position_m if command_deadman else None,
            wrist_roll_velocity_rad_s=0.0,
            log=log,
        )

    def _anchor(self, phone_pose: TcpPose, actual_tcp_pose: TcpPose) -> None:
        self.phone_anchor_pose = phone_pose
        self.robot_anchor_pose = actual_tcp_pose
        self.last_phone_pose = phone_pose
        self.last_target_pose = actual_tcp_pose
        self.last_raw_target_pose = actual_tcp_pose
        self.last_timestamp_sec = None

    def _step_dt(self, timestamp: float) -> float:
        if self.last_timestamp_sec is None:
            return 1.0 / 30.0
        dt = timestamp - self.last_timestamp_sec
        if not math.isfinite(dt):
            return 1.0 / 30.0
        return float(max(1e-3, min(0.2, dt)))

    def _phone_step(self, phone_pose: TcpPose) -> tuple[float, float]:
        if self.last_phone_pose is None:
            return 0.0, 0.0
        translation = float(np.linalg.norm(_as_position(phone_pose.position_m) - _as_position(self.last_phone_pose.position_m)))
        rotation = pose_rotation_error_rad(phone_pose, self.last_phone_pose)
        return translation, rotation

    def _phone_still(self, translation_m: float, rotation_rad: float) -> bool:
        return (
            translation_m <= self.config.phone_still_translation_m
            and rotation_rad <= self.config.phone_still_rotation_rad
        )

    def _update_phone_still(self, phone_still_now: bool, timestamp: float) -> tuple[bool, float]:
        if not phone_still_now:
            self.phone_still_started_at_sec = None
            return False, 0.0
        if self.phone_still_started_at_sec is None:
            self.phone_still_started_at_sec = timestamp
        duration = max(0.0, timestamp - self.phone_still_started_at_sec)
        return duration >= max(0.0, self.config.phone_still_min_sec), duration

    def _raw_target_jump_rejected(self, desired_raw: TcpPose) -> bool:
        limit = float(self.config.max_target_jump_m)
        if limit <= 0.0 or self.last_raw_target_pose is None:
            return False
        jump = float(np.linalg.norm(_as_position(desired_raw.position_m) - _as_position(self.last_raw_target_pose.position_m)))
        return jump > limit

    def _phone_delta(self, phone_pose: TcpPose) -> TcpPose:
        assert self.phone_anchor_pose is not None
        return TcpPose(
            (_as_position(phone_pose.position_m) - _as_position(self.phone_anchor_pose.position_m)).astype(float).tolist(),
            quat_multiply_wxyz(phone_pose.quaternion_wxyz, quat_conjugate_wxyz(self.phone_anchor_pose.quaternion_wxyz)).astype(float).tolist(),
        )

    def _map_phone_delta_to_robot(self, phone_delta_m: list[float] | np.ndarray) -> np.ndarray:
        raw = _as_position(phone_delta_m)
        values = {"x": raw[0], "y": raw[1], "z": raw[2]}
        mapped = apply_vector_axis_map(values, self.phone_to_robot_axis_map)
        mapped[np.abs(mapped) <= self.config.phone_translation_deadband_m] = 0.0
        return mapped

    def _workspace_bounded_target(self, target: TcpPose) -> TcpPose:
        target_pos = _as_position(target.position_m)
        min_bound, max_bound = self._effective_workspace_bounds()
        bounded = np.minimum(np.maximum(target_pos, min_bound), max_bound)
        return TcpPose(bounded.astype(float).tolist(), target.quaternion_wxyz)

    def _effective_workspace_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        min_bound = np.asarray(self.config.workspace_min_m, dtype=np.float64)
        max_bound = np.asarray(self.config.workspace_max_m, dtype=np.float64)
        if self.robot_anchor_pose is not None:
            anchor_position = _as_position(self.robot_anchor_pose.position_m)
            min_bound = np.minimum(min_bound, anchor_position)
            max_bound = np.maximum(max_bound, anchor_position)
        return min_bound, max_bound

    def _command_target(
        self,
        *,
        actual_tcp_pose: TcpPose,
        desired_workspace_bounded: TcpPose,
        dt: float,
        phone_still: bool,
    ) -> tuple[TcpPose, bool, bool, bool, bool, bool, bool]:
        if self.config.target_response_mode == "lag_follow":
            target, lead_limited = self._rate_limited_target(
                current=self.last_target_pose or actual_tcp_pose,
                target=desired_workspace_bounded,
            )
            target, deadband_hold = self._target_update_deadbanded(
                current=self.last_target_pose or actual_tcp_pose,
                target=target,
            )
            return target, lead_limited or deadband_hold, False, False, False, deadband_hold, False

        last_target = self.last_target_pose or actual_tcp_pose
        tracking_error = self._position_error(desired_workspace_bounded, actual_tcp_pose)
        still_freeze = self._should_freeze_when_phone_still(phone_still, tracking_error)
        if still_freeze:
            self.last_target_velocity_m_s = np.zeros(3, dtype=np.float64)
            return actual_tcp_pose, False, False, False, False, False, True

        target, filtered = self._filtered_target(current=last_target, target=desired_workspace_bounded, dt=dt)
        target, velocity_limited = self._velocity_limited_target(current=last_target, target=target, dt=dt)
        target, acceleration_limited = self._acceleration_limited_target(
            current=last_target,
            target=target,
            dt=dt,
        )
        target, deadband_hold = self._target_update_deadbanded(current=last_target, target=target)
        return (
            target,
            filtered or velocity_limited or acceleration_limited or deadband_hold,
            filtered,
            velocity_limited,
            acceleration_limited,
            deadband_hold,
            False,
        )

    def _position_error(self, target: TcpPose, actual_tcp_pose: TcpPose) -> float:
        return float(np.linalg.norm(_as_position(target.position_m) - _as_position(actual_tcp_pose.position_m)))

    def _should_freeze_when_phone_still(self, phone_still: bool, tracking_error_m: float) -> bool:
        if not self.config.freeze_when_phone_still or not phone_still:
            return False
        limit = max(0.0, float(self.config.phone_still_freeze_tracking_error_m))
        return tracking_error_m <= limit

    def _filtered_target(self, *, current: TcpPose, target: TcpPose, dt: float) -> tuple[TcpPose, bool]:
        tau = float(self.config.target_filter_time_constant_sec)
        if tau <= 0.0:
            return target, False
        alpha = dt / (tau + dt)
        current_pos = _as_position(current.position_m)
        target_pos = _as_position(target.position_m)
        filtered_pos = current_pos + (target_pos - current_pos) * alpha
        return TcpPose(filtered_pos.astype(float).tolist(), target.quaternion_wxyz), True

    def _velocity_limited_target(self, *, current: TcpPose, target: TcpPose, dt: float) -> tuple[TcpPose, bool]:
        max_velocity = float(self.config.max_target_velocity_m_s)
        if max_velocity <= 0.0:
            return current, True
        current_pos = _as_position(current.position_m)
        target_pos = _as_position(target.position_m)
        delta = target_pos - current_pos
        max_step = max_velocity * max(dt, 1e-3)
        norm = float(np.linalg.norm(delta))
        if norm <= max_step or norm <= 1e-9:
            return target, False
        limited_pos = current_pos + delta * (max_step / norm)
        return TcpPose(limited_pos.astype(float).tolist(), target.quaternion_wxyz), True

    def _acceleration_limited_target(self, *, current: TcpPose, target: TcpPose, dt: float) -> tuple[TcpPose, bool]:
        max_acceleration = float(self.config.max_target_acceleration_m_s2)
        if max_acceleration <= 0.0 or dt <= 0.0:
            self.last_target_velocity_m_s = (
                (_as_position(target.position_m) - _as_position(current.position_m))
                / max(dt, 1e-3)
            )
            return target, False
        current_pos = _as_position(current.position_m)
        target_pos = _as_position(target.position_m)
        desired_velocity = (target_pos - current_pos) / max(dt, 1e-3)
        velocity_delta = desired_velocity - self.last_target_velocity_m_s
        max_delta = max_acceleration * max(dt, 1e-3)
        delta_norm = float(np.linalg.norm(velocity_delta))
        limited = delta_norm > max_delta > 0.0
        if limited:
            desired_velocity = self.last_target_velocity_m_s + velocity_delta * (max_delta / max(delta_norm, 1e-9))
        self.last_target_velocity_m_s = desired_velocity
        return TcpPose((current_pos + desired_velocity * dt).astype(float).tolist(), target.quaternion_wxyz), limited

    def _target_update_deadbanded(self, *, current: TcpPose, target: TcpPose) -> tuple[TcpPose, bool]:
        deadband = max(0.0, float(self.config.target_update_deadband_m))
        if deadband <= 0.0:
            self.target_deadband_holding = False
            return target, False
        release = max(deadband, float(self.config.target_update_release_m or deadband))
        delta = float(np.linalg.norm(_as_position(target.position_m) - _as_position(current.position_m)))
        threshold = release if self.target_deadband_holding else deadband
        if delta <= threshold:
            self.target_deadband_holding = True
            self.last_target_velocity_m_s = np.zeros(3, dtype=np.float64)
            return current, True
        self.target_deadband_holding = False
        return target, False

    def _tracking_errors(
        self,
        desired_bounded: TcpPose,
        actual_tcp_pose: TcpPose,
        q_current: list[float],
    ) -> tuple[float, float, float]:
        pos_error = float(np.linalg.norm(_as_position(desired_bounded.position_m) - _as_position(actual_tcp_pose.position_m)))
        rot_error = pose_rotation_error_rad(desired_bounded, actual_tcp_pose)
        q_error = 0.0
        if self.last_q_cmd is not None:
            q_error = float(max(abs(a - b) for a, b in zip(self.last_q_cmd, q_current, strict=True)))
        return pos_error, rot_error, q_error

    def _time_scale(
        self,
        position_error_m: float,
        rotation_error_rad: float,
        q_tracking_error_rad: float,
    ) -> tuple[float, bool, bool]:
        pos_ratio = _warn_ratio(position_error_m, warn=self.config.max_pos_tracking_error_warn_m, pause=self.config.max_pos_tracking_error_pause_m)
        rot_ratio = _warn_ratio(rotation_error_rad, warn=self.config.max_rot_tracking_error_warn_rad, pause=self.config.max_rot_tracking_error_pause_rad)
        q_ratio = _warn_ratio(q_tracking_error_rad, warn=self.config.max_q_tracking_error_pause_rad * 0.5, pause=self.config.max_q_tracking_error_pause_rad)
        ratio = max(pos_ratio, rot_ratio, q_ratio)
        lag_warn = ratio > 0.0
        lag_pause = position_error_m >= self.config.max_pos_tracking_error_pause_m or rotation_error_rad >= self.config.max_rot_tracking_error_pause_rad or q_tracking_error_rad >= self.config.max_q_tracking_error_pause_rad
        return max(self.config.min_warn_time_scale, 1.0 - ratio), lag_warn, lag_pause

    def _rate_limited_target(self, current: TcpPose, target: TcpPose) -> tuple[TcpPose, bool]:
        current_pos = _as_position(current.position_m)
        target_pos = _as_position(target.position_m)
        delta = target_pos - current_pos
        lead_limited = False
        norm = float(np.linalg.norm(delta))
        max_step = abs(float(self.config.max_step_position_m))
        if self.config.target_response_mode == "lag_follow" and norm > max_step > 0.0:
            target_pos = current_pos + delta * (max_step / max(norm, 1e-9))
            lead_limited = True
        target_pos = np.minimum(np.maximum(target_pos, np.asarray(self.config.workspace_min_m)), np.asarray(self.config.workspace_max_m))
        return TcpPose(target_pos.astype(float).tolist(), target.quaternion_wxyz), lead_limited

    def _check_ik(self, pose: TcpPose, q_current: list[float]) -> IkCheckResult:
        if self.ik_checker is None:
            return IkCheckResult(True, q_cmd=list(q_current))
        result = self.ik_checker(pose, q_current)
        if result.q_cmd is not None and len(result.q_cmd) != 6:
            return IkCheckResult(False, None, True, "ik_q_cmd_wrong_size")
        return result

    def _base_log(
        self,
        timestamp: float,
        snapshot: HebiMobileIOSnapshot,
        actual_tcp_pose: TcpPose,
        q_current: list[float],
    ) -> dict[str, Any]:
        return {
            "timestamp": timestamp,
            "timestamp_sec": timestamp,
            "b1_state": bool(snapshot.raw_inputs.get("b1", False)),
            "phone_pose_raw": {
                "position_m": list(snapshot.position_m),
                "quaternion_wxyz": list(snapshot.quaternion_wxyz),
                "raw_inputs": dict(snapshot.raw_inputs),
                "valid": snapshot.valid,
                "reason": snapshot.reason,
            },
            "phone_anchor_pose": _pose_to_dict_or_none(self.phone_anchor_pose),
            "actual_tcp_pose": actual_tcp_pose.to_dict(),
            "q_current": list(q_current),
            "source": "relative_pose_lag_follow",
        }
