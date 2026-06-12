from __future__ import annotations

import json
import math
import socket
import time
from dataclasses import dataclass
from typing import Any, Callable

from .jaka_sdk_backend import JakaSDKBackend
from .palm_target_ik import (
    DEFAULT_JOINT_LIMIT_MARGIN_RAD,
    DEFAULT_MJCF,
    PalmTargetIkState,
    clip_joints_to_safe_limits,
    joint_limit_margin_blockers,
    safe_joint_limits_rad,
)


ABS_MOVE_MODE = 0


def joint_limit_remaining_rad(
    joints_rad: list[float],
    *,
    margin_rad: float,
) -> list[float]:
    remaining: list[float] = []
    for value, (low, high) in zip(joints_rad, safe_joint_limits_rad(margin_rad), strict=True):
        remaining.append(max(0.0, min(float(value) - low, high - float(value))))
    return remaining


def nearest_joint_limit_summary(
    joints_rad: list[float],
    *,
    margin_rad: float,
) -> dict[str, Any]:
    remaining = joint_limit_remaining_rad(joints_rad, margin_rad=margin_rad)
    if not remaining:
        return {
            "joint_limit_remaining_rad": [],
            "nearest_joint_limit_index_1_based": None,
            "nearest_joint_limit_remaining_rad": None,
        }
    nearest_index = min(range(len(remaining)), key=remaining.__getitem__)
    return {
        "joint_limit_remaining_rad": remaining,
        "nearest_joint_limit_index_1_based": nearest_index + 1,
        "nearest_joint_limit_remaining_rad": remaining[nearest_index],
    }


def resolve_edg_stat_ip(controller_ip: str, configured_ip: str = "auto") -> str:
    configured_ip = str(configured_ip).strip()
    if configured_ip and configured_ip.lower() not in {"auto", "default"}:
        return configured_ip
    if not controller_ip:
        return "127.0.0.1"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((controller_ip, 10001))
        return str(sock.getsockname()[0])
    finally:
        sock.close()


@dataclass(frozen=True, slots=True)
class JointJogCommand:
    deadman: bool
    joint_velocity_rad_s: list[float]


@dataclass(frozen=True, slots=True)
class PalmTargetJogCommand:
    deadman: bool
    palm_velocity_m_s: list[float]
    wrist_roll_velocity_rad_s: float
    palm_target_position_m: list[float] | None = None
    hold_current: bool = False


def parse_joint_jog_command(message: Any) -> JointJogCommand:
    if isinstance(message, str):
        payload = json.loads(message)
    elif hasattr(message, "data"):
        payload = json.loads(str(message.data))
    elif isinstance(message, dict):
        payload = dict(message)
    else:
        raise TypeError(f"Unsupported joint jog payload type: {type(message)!r}")
    if not isinstance(payload, dict):
        raise ValueError("Joint jog command must decode to an object.")
    velocity = payload.get("joint_velocity_rad_s")
    if not isinstance(velocity, list) or len(velocity) != 6:
        raise ValueError("joint_velocity_rad_s must contain 6 numeric values.")
    if not all(isinstance(value, (int, float)) for value in velocity):
        raise ValueError("joint_velocity_rad_s must contain only numeric values.")
    deadman = payload.get("deadman", False)
    if not isinstance(deadman, bool):
        raise ValueError("deadman must be a boolean.")
    return JointJogCommand(
        deadman=deadman,
        joint_velocity_rad_s=[float(value) for value in velocity],
    )


def parse_palm_target_jog_command(message: Any) -> PalmTargetJogCommand:
    if isinstance(message, str):
        payload = json.loads(message)
    elif hasattr(message, "data"):
        payload = json.loads(str(message.data))
    elif isinstance(message, dict):
        payload = dict(message)
    else:
        raise TypeError(f"Unsupported palm target jog payload type: {type(message)!r}")
    if not isinstance(payload, dict):
        raise ValueError("Palm target jog command must decode to an object.")
    velocity = payload.get("palm_velocity_m_s")
    if not isinstance(velocity, list) or len(velocity) != 3:
        raise ValueError("palm_velocity_m_s must contain 3 numeric values.")
    if not all(isinstance(value, (int, float)) for value in velocity):
        raise ValueError("palm_velocity_m_s must contain only numeric values.")
    wrist_roll_velocity = payload.get("wrist_roll_velocity_rad_s", 0.0)
    if not isinstance(wrist_roll_velocity, (int, float)):
        raise ValueError("wrist_roll_velocity_rad_s must be numeric.")
    deadman = payload.get("deadman", False)
    if not isinstance(deadman, bool):
        raise ValueError("deadman must be a boolean.")
    palm_target_position = payload.get("palm_target_position_m")
    if palm_target_position is not None:
        if not isinstance(palm_target_position, list) or len(palm_target_position) != 3:
            raise ValueError("palm_target_position_m must contain 3 numeric values.")
        if not all(isinstance(value, (int, float)) for value in palm_target_position):
            raise ValueError("palm_target_position_m must contain only numeric values.")
    hold_current = payload.get("hold_current", False)
    if not isinstance(hold_current, bool):
        raise ValueError("hold_current must be a boolean.")
    return PalmTargetJogCommand(
        deadman=deadman,
        palm_velocity_m_s=[float(value) for value in velocity],
        wrist_roll_velocity_rad_s=float(wrist_roll_velocity),
        palm_target_position_m=(
            None
            if palm_target_position is None
            else [float(value) for value in palm_target_position]
        ),
        hold_current=hold_current,
    )


def find_servo_safety_blockers(flags: dict[str, Any]) -> list[str]:
    blockers: list[str] = []
    for field, label in (
        ("is_in_estop", "robot_in_estop"),
        ("is_in_collision", "robot_in_collision"),
        ("is_on_limit", "robot_on_limit"),
        ("is_in_drag_mode", "robot_in_drag_mode"),
    ):
        if flags.get(field, 0) != 0:
            blockers.append(label)
    last_error = flags.get("last_error_raw")
    if isinstance(last_error, list) and last_error and last_error[0] != 0:
        blockers.append("robot_last_error")
    protective_stop = flags.get("protective_stop_status")
    if isinstance(protective_stop, dict):
        if protective_stop.get("protective_stop", 0) != 0:
            blockers.append("robot_protective_stop")
    elif protective_stop not in (None, 0, False):
        blockers.append("robot_protective_stop")
    return blockers


class JakaServoJogController:
    """Own the bounded EDG servo lifecycle for Xbox-style joint jogging."""

    def __init__(
        self,
        backend: JakaSDKBackend,
        *,
        state_flags: Callable[[], dict[str, Any]],
        watchdog_sec: float = 0.25,
        max_joint_velocity_rad_s: float = 0.12,
        max_joint_acceleration_rad_s2: float = math.inf,
        max_session_excursion_rad: float = 0.0,
        joint_limit_margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
        prime_after_enable_ticks: int = 0,
        step_num: int = 1,
        robot_index: int = 0,
        edg_stat_ip: str = "auto",
        sdk_servo_filter: str = "auto",
        sdk_servo_filter_cutoff_hz: float = 0.5,
        sdk_servo_filter_jerk_deg_s3: float = 50.0,
        now: Callable[[], float] = time.monotonic,
    ) -> None:
        self.backend = backend
        self.state_flags = state_flags
        self.watchdog_sec = float(watchdog_sec)
        self.max_joint_velocity_rad_s = abs(float(max_joint_velocity_rad_s))
        self.max_joint_acceleration_rad_s2 = abs(float(max_joint_acceleration_rad_s2))
        self.max_session_excursion_rad = abs(float(max_session_excursion_rad))
        self.session_excursion_enabled = (
            math.isfinite(self.max_session_excursion_rad)
            and self.max_session_excursion_rad > 0.0
        )
        self.joint_limit_margin_rad = abs(float(joint_limit_margin_rad))
        self.prime_after_enable_ticks = max(0, int(prime_after_enable_ticks))
        self.step_num = max(1, int(step_num))
        self.robot_index = int(robot_index)
        backend_config = getattr(backend, "config", {})
        self.edg_stat_ip = resolve_edg_stat_ip(str(backend_config.get("ip", "")), edg_stat_ip)
        self.sdk_servo_filter = str(sdk_servo_filter)
        self.sdk_servo_filter_cutoff_hz = max(0.01, abs(float(sdk_servo_filter_cutoff_hz)))
        self.sdk_servo_filter_jerk_deg_s3 = abs(float(sdk_servo_filter_jerk_deg_s3))
        self.now = now
        self.enabled = False
        self.command: JointJogCommand | None = None
        self.last_command_time: float | None = None
        self.last_tick_time: float | None = None
        self.anchor_joints: list[float] | None = None
        self.target_joints: list[float] | None = None
        self.last_disable_reason = "not_started"
        self.fault_latched = False
        self.fault_reason = ""
        self.joint_limit_limited = False
        self.limited_joint_indices_1_based: list[int] = []
        self._prime_ticks_remaining = 0
        self._last_joint_velocity_rad_s = [0.0] * 6
        self._active_sdk_servo_filter = "none"

    def accept(self, command: JointJogCommand) -> None:
        self.command = command
        self.last_command_time = self.now()
        if not command.deadman:
            self.disable("deadman_released")

    def tick(self) -> bool:
        now = self.now()
        if self.fault_latched:
            return False
        if self.command is None or self.last_command_time is None:
            self.disable("waiting_for_command")
            return False
        if now - self.last_command_time > self.watchdog_sec:
            self.disable("command_timeout")
            return False
        if not self.command.deadman:
            self.disable("deadman_released")
            return False
        if not self.enabled:
            self._enable(now)

        flags = self.state_flags()
        blockers = find_servo_safety_blockers(flags)
        if blockers:
            self.latch_fault(f"safety_blocked:{','.join(blockers)}")
            raise RuntimeError(f"JAKA servo jog blocked by safety flags: {blockers}")

        assert self.anchor_joints is not None
        assert self.target_joints is not None
        if self._prime_ticks_remaining > 0:
            self._stream_target(self.target_joints)
            self._prime_ticks_remaining -= 1
            self.last_tick_time = now
            return False
        previous_tick = now if self.last_tick_time is None else self.last_tick_time
        dt = max(0.0, min(now - previous_tick, self.watchdog_sec))
        velocity = [
            max(-self.max_joint_velocity_rad_s, min(self.max_joint_velocity_rad_s, value))
            for value in self.command.joint_velocity_rad_s
        ]
        velocity = self._limit_joint_acceleration(velocity, dt)
        next_target_joints = [
            target + speed * dt
            for target, speed in zip(self.target_joints, velocity, strict=True)
        ]
        if self.session_excursion_enabled:
            low = [value - self.max_session_excursion_rad for value in self.anchor_joints]
            high = [value + self.max_session_excursion_rad for value in self.anchor_joints]
            next_target_joints = [
                max(lo, min(hi, target))
                for target, lo, hi in zip(next_target_joints, low, high, strict=True)
            ]
        self.target_joints = next_target_joints
        self.target_joints, limited = self._clip_to_safe_joint_limits(self.target_joints)
        self.joint_limit_limited = bool(limited)
        self.limited_joint_indices_1_based = [index + 1 for index in limited]
        self._stream_target(self.target_joints)
        self.last_tick_time = now
        return True

    def disable(self, reason: str) -> None:
        if self.enabled:
            self._ensure_success(
                self.backend.call_sdk_method("servo_move_enable", False),
                "servo_move_enable(False)",
            )
        self._clear_sdk_servo_filter()
        self.enabled = False
        self.anchor_joints = None
        self.target_joints = None
        self.last_tick_time = None
        self.last_disable_reason = reason
        self._prime_ticks_remaining = 0
        self._last_joint_velocity_rad_s = [0.0] * 6
        self._active_sdk_servo_filter = "none"

    def latch_fault(self, reason: str) -> None:
        if self.fault_latched:
            return
        self.disable(reason)
        self.fault_latched = True
        self.fault_reason = reason

    def status(self) -> dict[str, Any]:
        target_joints = self.target_joints or self.anchor_joints or [0.0] * 6
        status = {
            "enabled": self.enabled,
            "last_disable_reason": self.last_disable_reason,
            "fault_latched": self.fault_latched,
            "fault_reason": self.fault_reason,
            "anchor_joints": self.anchor_joints,
            "target_joints": self.target_joints,
            "watchdog_sec": self.watchdog_sec,
            "max_joint_velocity_rad_s": self.max_joint_velocity_rad_s,
            "max_joint_acceleration_rad_s2": self.max_joint_acceleration_rad_s2,
            "max_session_excursion_rad": self.max_session_excursion_rad,
            "session_excursion_enabled": self.session_excursion_enabled,
            "joint_limit_margin_rad": self.joint_limit_margin_rad,
            "joint_limit_limited": self.joint_limit_limited,
            "limited_joint_indices_1_based": self.limited_joint_indices_1_based,
            "last_joint_velocity_rad_s": self._last_joint_velocity_rad_s,
            "step_num": self.step_num,
            "edg_stat_ip": self.edg_stat_ip,
            "sdk_servo_filter": self.sdk_servo_filter,
            "sdk_servo_filter_cutoff_hz": self.sdk_servo_filter_cutoff_hz,
            "active_sdk_servo_filter": self._active_sdk_servo_filter,
        }
        status.update(
            nearest_joint_limit_summary(
                [float(value) for value in target_joints],
                margin_rad=self.joint_limit_margin_rad,
            )
        )
        return status

    def reset_velocity_limiter(self) -> None:
        self._last_joint_velocity_rad_s = [0.0] * 6

    def _enable(self, now: float) -> None:
        blockers = find_servo_safety_blockers(self.state_flags())
        if blockers:
            self.latch_fault(f"safety_blocked:{','.join(blockers)}")
            raise RuntimeError(f"JAKA servo jog enable blocked by safety flags: {blockers}")
        joints = [float(value) for value in self.backend.get_joint_state().positions]
        if len(joints) != 6:
            raise RuntimeError(f"Expected 6 JAKA joints, got {len(joints)}.")
        joint_limit_blockers = joint_limit_margin_blockers(
            joints,
            margin_rad=self.joint_limit_margin_rad,
        )
        if joint_limit_blockers:
            self.latch_fault(f"joint_limit_margin:{','.join(joint_limit_blockers)}")
            raise RuntimeError(
                f"JAKA servo jog enable blocked by configured joint limit margins: {joint_limit_blockers}"
            )
        self._ensure_success(
            self.backend.call_sdk_method("edg_init", True, self.edg_stat_ip),
            "edg_init",
        )
        self._apply_sdk_servo_filter()
        self._ensure_success(
            self.backend.call_sdk_method("servo_move_enable", True),
            "servo_move_enable(True)",
        )
        self.enabled = True
        self.anchor_joints = joints
        self.target_joints = list(joints)
        self.last_tick_time = now
        self.last_disable_reason = ""
        self._prime_ticks_remaining = self.prime_after_enable_ticks
        self._last_joint_velocity_rad_s = [0.0] * 6

    def _clip_to_safe_joint_limits(self, joints: list[float]) -> tuple[list[float], list[int]]:
        clipped, limited = clip_joints_to_safe_limits(
            joints,
            margin_rad=self.joint_limit_margin_rad,
        )
        return clipped.tolist(), limited

    def _stream_target(self, target_joints: list[float]) -> None:
        self._ensure_success(
            self.backend.call_sdk_method(
                "edg_servo_j",
                target_joints,
                ABS_MOVE_MODE,
                self.step_num,
                self.robot_index,
            ),
            "edg_servo_j",
        )

    def _limit_joint_acceleration(self, desired_velocity: list[float], dt: float) -> list[float]:
        if (
            not math.isfinite(self.max_joint_acceleration_rad_s2)
            or self.max_joint_acceleration_rad_s2 <= 0.0
        ):
            self._last_joint_velocity_rad_s = [float(value) for value in desired_velocity]
            return self._last_joint_velocity_rad_s
        if dt <= 0.0:
            return list(self._last_joint_velocity_rad_s)
        max_delta = self.max_joint_acceleration_rad_s2 * dt
        limited: list[float] = []
        for desired, previous in zip(desired_velocity, self._last_joint_velocity_rad_s, strict=True):
            delta = max(-max_delta, min(max_delta, float(desired) - previous))
            limited.append(previous + delta)
        self._last_joint_velocity_rad_s = limited
        return limited

    def _apply_sdk_servo_filter(self) -> None:
        requested = self.sdk_servo_filter.lower().strip()
        if requested in ("", "none", "off", "false"):
            self._clear_sdk_servo_filter()
            self._active_sdk_servo_filter = "none"
            return
        robot = getattr(self.backend, "_robot", None)
        if robot is None:
            self._active_sdk_servo_filter = "none"
            return
        if requested == "auto":
            if hasattr(robot, "servo_move_use_joint_NLF"):
                requested = "joint_nlf"
            elif hasattr(robot, "servo_move_use_joint_LPF"):
                requested = "joint_lpf"
            else:
                self._active_sdk_servo_filter = "unavailable"
                return
        try:
            if requested == "joint_nlf":
                max_v_deg_s = max(0.1, math.degrees(self.max_joint_velocity_rad_s))
                if math.isfinite(self.max_joint_acceleration_rad_s2):
                    max_a_deg_s2 = max(0.1, math.degrees(self.max_joint_acceleration_rad_s2))
                else:
                    max_a_deg_s2 = max(0.1, max_v_deg_s * 10.0)
                self._ensure_success(
                    self.backend.call_sdk_method(
                        "servo_move_use_joint_NLF",
                        max_v_deg_s,
                        max_a_deg_s2,
                        self.sdk_servo_filter_jerk_deg_s3,
                    ),
                    "servo_move_use_joint_NLF",
                )
                self._active_sdk_servo_filter = "joint_nlf"
                return
            if requested == "joint_lpf":
                self._ensure_success(
                    self.backend.call_sdk_method(
                        "servo_move_use_joint_LPF",
                        self.sdk_servo_filter_cutoff_hz,
                    ),
                    "servo_move_use_joint_LPF",
                )
                self._active_sdk_servo_filter = "joint_lpf"
                return
            raise RuntimeError(f"Unsupported SDK SERVO filter: {self.sdk_servo_filter!r}.")
        except Exception:
            if self.sdk_servo_filter.lower().strip() == "auto":
                self._active_sdk_servo_filter = "failed_auto"
                return
            raise

    def _clear_sdk_servo_filter(self) -> None:
        robot = getattr(self.backend, "_robot", None)
        if hasattr(robot, "servo_move_use_none_filter"):
            self._ensure_success(
                self.backend.call_sdk_method("servo_move_use_none_filter"),
                "servo_move_use_none_filter",
            )

    @staticmethod
    def _ensure_success(result: Any, method_name: str) -> None:
        JakaSDKBackend.ensure_success(result, method_name)


class JakaPalmTargetJogController:
    """Convert bounded TCP velocity commands into short-horizon EDG joint servo streams."""

    def __init__(
        self,
        backend: JakaSDKBackend,
        *,
        state_flags: Callable[[], dict[str, Any]],
        mjcf_path: str = str(DEFAULT_MJCF),
        watchdog_sec: float = 0.25,
        max_palm_velocity_m_s: float = 0.12,
        max_wrist_roll_velocity_rad_s: float = 0.25,
        max_joint_velocity_rad_s: float = 1.20,
        max_joint_acceleration_rad_s2: float = 6.00,
        max_session_excursion_rad: float = 0.0,
        max_session_palm_excursion_m: float = 0.0,
        tcp_velocity_horizon_sec: float = 0.20,
        max_tcp_target_offset_m: float = 0.04,
        max_raw_ik_error_rad: float = 0.12,
        target_deadband_m: float = 0.0,
        max_joint_tracking_error_rad: float = 0.012,
        joint_tracking_release_rad: float = 0.0,
        max_joint_tracking_error_fault_rad: float = 0.025,
        joint_tracking_hold_min_sec: float = 0.0,
        saturation_velocity_ratio: float = 0.98,
        saturation_min_joints: int = 6,
        saturation_hold_sec: float = 0.0,
        joint_limit_margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
        prime_after_enable_ticks: int = 3,
        step_num: int = 1,
        robot_index: int = 0,
        edg_stat_ip: str = "auto",
        sdk_servo_filter: str = "auto",
        sdk_servo_filter_cutoff_hz: float = 0.5,
        sdk_servo_filter_jerk_deg_s3: float = 50.0,
        now: Callable[[], float] = time.monotonic,
    ) -> None:
        self.now = now
        self.max_palm_velocity_m_s = abs(float(max_palm_velocity_m_s))
        self.max_wrist_roll_velocity_rad_s = abs(float(max_wrist_roll_velocity_rad_s))
        self.max_session_palm_excursion_m = abs(float(max_session_palm_excursion_m))
        self.tcp_velocity_horizon_sec = max(0.0, float(tcp_velocity_horizon_sec))
        self.max_tcp_target_offset_m = abs(float(max_tcp_target_offset_m))
        self.max_raw_ik_error_rad = abs(float(max_raw_ik_error_rad))
        self.target_deadband_m = abs(float(target_deadband_m))
        self.max_joint_tracking_error_rad = abs(float(max_joint_tracking_error_rad))
        self.joint_tracking_release_rad = abs(float(joint_tracking_release_rad))
        self.max_joint_tracking_error_fault_rad = abs(float(max_joint_tracking_error_fault_rad))
        self.joint_tracking_hold_min_sec = max(0.0, float(joint_tracking_hold_min_sec))
        self.saturation_velocity_ratio = max(0.0, min(1.0, float(saturation_velocity_ratio)))
        self.saturation_min_joints = max(1, int(saturation_min_joints))
        self.saturation_hold_sec = max(0.0, float(saturation_hold_sec))
        self.joint_limit_margin_rad = abs(float(joint_limit_margin_rad))
        self.mjcf_path = str(mjcf_path)
        self.servo = JakaServoJogController(
            backend,
            state_flags=state_flags,
            watchdog_sec=watchdog_sec,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
            max_session_excursion_rad=max_session_excursion_rad,
            joint_limit_margin_rad=self.joint_limit_margin_rad,
            prime_after_enable_ticks=prime_after_enable_ticks,
            step_num=step_num,
            robot_index=robot_index,
            edg_stat_ip=edg_stat_ip,
            sdk_servo_filter=sdk_servo_filter,
            sdk_servo_filter_cutoff_hz=sdk_servo_filter_cutoff_hz,
            sdk_servo_filter_jerk_deg_s3=sdk_servo_filter_jerk_deg_s3,
            now=now,
        )
        self.prime_after_enable_ticks = max(0, int(prime_after_enable_ticks))
        self.command: PalmTargetJogCommand | None = None
        self.ik_state = PalmTargetIkState(
            [0.0] * 6,
            mjcf_path=self.mjcf_path,
            target_workspace_radius_m=self.max_session_palm_excursion_m,
            joint_limit_margin_rad=self.joint_limit_margin_rad,
        )
        self.last_tick_time: float | None = None
        self._last_q_cmd: list[float] | None = None
        self._last_raw_ik_q = [0.0] * 6
        self._last_q_current = [0.0] * 6
        self._last_joint_error = [0.0] * 6
        self._last_qdot_cmd = [0.0] * 6
        self._joint_tracking_error_rad = 0.0
        self._joint_tracking_error_indices_1_based: list[int] = []
        self._joint_tracking_error_limited = False
        self._joint_tracking_error_faulted = False
        self._joint_tracking_hold_active = False
        self._joint_tracking_hold_until_sec = 0.0
        self._last_tcp_current: list[float] | None = None
        self._last_tcp_target: list[float] | None = None
        self._last_tcp_target_error_m: float | None = None
        self._last_tick_dt_sec = 0.0
        self._raw_ik_error_limited = False
        self._is_saturated = False
        self._saturated_joint_indices_1_based: list[int] = []
        self._saturation_time_sec = 0.0
        self._target_deadband_hold = False
        self._last_position_target_m: list[float] | None = None
        self._watchdog_active = False
        self._watchdog_reason = ""
        self._hold_current_active = False

    @property
    def enabled(self) -> bool:
        return self.servo.enabled

    @property
    def fault_latched(self) -> bool:
        return self.servo.fault_latched

    def accept(self, command: PalmTargetJogCommand) -> None:
        self.command = command
        self.servo.accept(JointJogCommand(deadman=command.deadman, joint_velocity_rad_s=[0.0] * 6))
        if not command.deadman:
            self.last_tick_time = None
            self._last_position_target_m = None
            self._target_deadband_hold = False
            self._joint_tracking_hold_active = False

    def tick(self) -> bool:
        now = self.now()
        if self.command is None:
            self.servo.command = None
            return self.servo.tick()
        if not self.command.deadman:
            self._hold_current_active = False
            return self.servo.tick()
        if not self.servo.enabled:
            streamed = self.servo.tick()
            if self.servo.enabled and self.servo.target_joints is not None:
                assert self.servo.target_joints is not None
                self.ik_state.reset_session(list(self.servo.target_joints))
                self._last_q_cmd = list(self.servo.target_joints)
                self._reset_motion_watchdogs()
                self.last_tick_time = now
            return streamed

        assert self.servo.target_joints is not None
        previous_tick = now if self.last_tick_time is None else self.last_tick_time
        dt = max(0.0, min(now - previous_tick, self.servo.watchdog_sec))
        actual_joints = self._actual_joints()
        q_cmd_reference = self._last_q_cmd or list(self.servo.target_joints)
        self._update_joint_tracking_error(q_cmd_reference, actual_joints)
        if self._joint_tracking_error_fault_exceeded():
            self._record_debug_state(
                dt=dt,
                q_current=actual_joints,
                q_cmd=q_cmd_reference,
                raw_ik_q=actual_joints,
                qdot_cmd=[0.0] * 6,
            )
            self.latch_fault("joint_tracking_error_fault")
            return False
        if self._joint_tracking_hold_should_continue(now):
            return self._hold_actual_joints(
                now=now,
                dt=dt,
                actual_joints=actual_joints,
                reason="joint_tracking_error",
            )
        palm_velocity = self._clip_palm_velocity(self.command.palm_velocity_m_s)
        wrist_roll_velocity = max(
            -self.max_wrist_roll_velocity_rad_s,
            min(self.max_wrist_roll_velocity_rad_s, self.command.wrist_roll_velocity_rad_s),
        )
        target_position = self.command.palm_target_position_m
        if target_position is not None and len(target_position) != 3:
            raise ValueError("palm_target_position_m must contain 3 values.")
        has_position_target = target_position is not None
        if (
            not self.command.hold_current
            and not has_position_target
            and max(abs(float(value)) for value in palm_velocity) < 1e-9
            and abs(float(wrist_roll_velocity)) < 1e-9
        ):
            hold_joints = list(self._last_q_cmd or self.servo.target_joints or actual_joints)
            self.ik_state.reset_session(actual_joints)
            self.ik_state.hold_current_target()
            self._reset_motion_watchdogs()
            self._record_debug_state(
                dt=dt,
                q_current=actual_joints,
                q_cmd=hold_joints,
                raw_ik_q=actual_joints,
                qdot_cmd=[0.0] * 6,
            )
            self.servo.target_joints = hold_joints
            self.servo.command = JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6)
            streamed = self.servo.tick()
            self._last_q_cmd = list(self.servo.target_joints or hold_joints)
            self.last_tick_time = now
            if not streamed and not self.servo.enabled:
                self.last_tick_time = None
            return streamed
        self.servo.target_joints = list(q_cmd_reference)
        self.ik_state.set_arm_joints_rad(actual_joints)
        tcp_current = self.ik_state.current_palm_position_m.tolist()
        if self.command.hold_current:
            self._last_position_target_m = None
            self._target_deadband_hold = False
            if self._hold_current_active:
                hold_joints = list(self._last_q_cmd or self.servo.target_joints or actual_joints)
            else:
                hold_joints = list(actual_joints)
                self._hold_current_active = True
            self.ik_state.reset_session(hold_joints)
            self.ik_state.hold_current_target()
            self._reset_motion_watchdogs()
            self._record_debug_state(
                dt=dt,
                q_current=actual_joints,
                q_cmd=hold_joints,
                raw_ik_q=actual_joints,
                qdot_cmd=[0.0] * 6,
                tcp_current=tcp_current,
            )
            self.servo.target_joints = hold_joints
            self.servo.command = JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6)
            streamed = self.servo.tick()
            self._last_q_cmd = list(self.servo.target_joints or hold_joints)
            self.last_tick_time = now
            if not streamed and not self.servo.enabled:
                self.last_tick_time = None
            return streamed
        self._hold_current_active = False
        if has_position_target:
            assert target_position is not None
            if self._position_target_deadband_holds(target_position):
                return self._hold_commanded_joints(
                    now=now,
                    dt=dt,
                    actual_joints=actual_joints,
                    q_cmd_reference=q_cmd_reference,
                    tcp_current=tcp_current,
                    reason="target_deadband",
                )
            self.ik_state.apply_position_target(
                palm_target_position_m=[float(value) for value in target_position],
                wrist_roll_velocity_rad_s=wrist_roll_velocity,
                dt=dt,
            )
        else:
            self._last_position_target_m = None
            self._target_deadband_hold = False
            self.ik_state.apply_short_horizon(
                palm_velocity_m_s=palm_velocity,
                wrist_roll_velocity_rad_s=wrist_roll_velocity,
                dt=dt,
                horizon_sec=self.tcp_velocity_horizon_sec,
                max_target_offset_m=self.max_tcp_target_offset_m,
            )
        raw_ik_q = self.ik_state.arm_joints_rad.tolist()
        bounded_ik_q, raw_ik_error_limited = self._clamp_raw_ik_q(
            raw_ik_q,
            q_current=actual_joints,
            q_cmd_reference=q_cmd_reference,
        )
        self._raw_ik_error_limited = raw_ik_error_limited
        if dt > 0.0:
            joint_velocity = [
                (float(target) - reference) / dt
                for target, reference in zip(bounded_ik_q, q_cmd_reference, strict=True)
            ]
        else:
            joint_velocity = [0.0] * 6
        self._update_saturation_watchdog(joint_velocity, dt)
        if self._watchdog_active:
            joint_velocity = [0.0] * 6
            self.servo.reset_velocity_limiter()
            self.ik_state.set_arm_joints_rad(actual_joints)
            self.ik_state.hold_current_target()
        self.servo.command = JointJogCommand(deadman=True, joint_velocity_rad_s=joint_velocity)
        streamed = self.servo.tick()
        self._last_q_cmd = list(self.servo.target_joints or actual_joints)
        qdot_cmd = self.servo.status()["last_joint_velocity_rad_s"]
        self._record_debug_state(
            dt=dt,
            q_current=actual_joints,
            q_cmd=self._last_q_cmd,
            raw_ik_q=raw_ik_q,
            qdot_cmd=qdot_cmd,
            tcp_current=tcp_current,
        )
        self.last_tick_time = now
        if not streamed and not self.servo.enabled:
            self.last_tick_time = None
        return streamed

    def disable(self, reason: str) -> None:
        self.servo.disable(reason)
        self.last_tick_time = None
        self._hold_current_active = False

    def latch_fault(self, reason: str) -> None:
        self.servo.latch_fault(reason)
        self.last_tick_time = None
        self._hold_current_active = False

    def status(self) -> dict[str, Any]:
        status = self.servo.status()
        if self.command is not None and self.command.hold_current:
            mode = "hold_current_edg_servo_j"
        elif self.command is not None and self.command.palm_target_position_m is not None:
            mode = "tcp_position_target_ik_edg_servo_j"
        else:
            mode = "tcp_velocity_short_horizon_ik_edg_servo_j"
        status.update(
            {
                "mode": mode,
                "max_palm_velocity_m_s": self.max_palm_velocity_m_s,
                "max_wrist_roll_velocity_rad_s": self.max_wrist_roll_velocity_rad_s,
                "max_session_palm_excursion_m": self.max_session_palm_excursion_m,
                "tcp_velocity_horizon_sec": self.tcp_velocity_horizon_sec,
                "max_tcp_target_offset_m": self.max_tcp_target_offset_m,
                "max_raw_ik_error_rad": self.max_raw_ik_error_rad,
                "target_deadband_m": self.target_deadband_m,
                "max_joint_tracking_error_rad": self.max_joint_tracking_error_rad,
                "joint_tracking_release_rad": self._effective_joint_tracking_release_rad(),
                "max_joint_tracking_error_fault_rad": self.max_joint_tracking_error_fault_rad,
                "joint_tracking_hold_min_sec": self.joint_tracking_hold_min_sec,
                "saturation_velocity_ratio": self.saturation_velocity_ratio,
                "saturation_min_joints": self.saturation_min_joints,
                "saturation_hold_sec": self.saturation_hold_sec,
                "joint_limit_margin_rad": self.joint_limit_margin_rad,
                "prime_after_enable_ticks": self.prime_after_enable_ticks,
                "tick_dt_sec": self._last_tick_dt_sec,
                "q_current": self._last_q_current,
                "q_cmd": self._last_q_cmd,
                "raw_ik_q": self._last_raw_ik_q,
                "qdot_cmd": self._last_qdot_cmd,
                "joint_error": self._last_joint_error,
                "raw_ik_error_limited": self._raw_ik_error_limited,
                "target_deadband_hold": self._target_deadband_hold,
                "joint_tracking_error_rad": self._joint_tracking_error_rad,
                "joint_tracking_error_indices_1_based": self._joint_tracking_error_indices_1_based,
                "joint_tracking_error_limited": self._joint_tracking_error_limited,
                "joint_tracking_error_faulted": self._joint_tracking_error_faulted,
                "joint_tracking_hold_active": self._joint_tracking_hold_active,
                "joint_tracking_hold_until_sec": self._joint_tracking_hold_until_sec,
                "tcp_current": self._last_tcp_current,
                "tcp_target": self._last_tcp_target,
                "tcp_target_error_m": self._last_tcp_target_error_m,
                "is_saturated": self._is_saturated,
                "saturated_joint_indices_1_based": self._saturated_joint_indices_1_based,
                "near_local_velocity_cap_joint_indices_1_based": self._saturated_joint_indices_1_based,
                "saturation_time_sec": self._saturation_time_sec,
                "watchdog_active": self._watchdog_active,
                "watchdog_reason": self._watchdog_reason,
                "palm_target_position_m": (
                    None if not self.enabled else self.ik_state.target_palm_position_m.tolist()
                ),
                "palm_preview_position_m": (
                    None if not self.enabled else self.ik_state.current_palm_position_m.tolist()
                ),
                "palm_target_error_m": (
                    None if not self.enabled else self.ik_state.target_error_m
                ),
                "palm_target_workspace_limited": (
                    None if not self.enabled else self.ik_state.target_workspace_limited
                ),
                "ik_joint_limit_limited": (
                    None if not self.enabled else self.ik_state.joint_limit_limited
                ),
                "ik_limited_joint_indices_1_based": (
                    [] if not self.enabled else self.ik_state.limited_joint_indices_1_based
                ),
                "feedback_closed_loop": True,
                "hold_current": bool(self.command.hold_current) if self.command is not None else False,
            }
        )
        return status

    def _actual_joints(self) -> list[float]:
        joints = [float(value) for value in self.servo.backend.get_joint_state().positions]
        if len(joints) != 6:
            raise RuntimeError(f"Expected 6 JAKA joints, got {len(joints)}.")
        return joints

    def _clip_palm_velocity(self, velocity: list[float]) -> list[float]:
        norm = math.sqrt(sum(float(value) ** 2 for value in velocity))
        if norm <= self.max_palm_velocity_m_s or norm == 0.0:
            return [float(value) for value in velocity]
        scale = self.max_palm_velocity_m_s / norm
        return [float(value) * scale for value in velocity]

    def _clamp_raw_ik_q(
        self,
        raw_ik_q: list[float],
        *,
        q_current: list[float],
        q_cmd_reference: list[float],
    ) -> tuple[list[float], bool]:
        if self.max_raw_ik_error_rad <= 0.0:
            return [float(value) for value in raw_ik_q], False
        bounded: list[float] = []
        limited = False
        for raw, current, q_cmd in zip(raw_ik_q, q_current, q_cmd_reference, strict=True):
            low = max(current - self.max_raw_ik_error_rad, q_cmd - self.max_raw_ik_error_rad)
            high = min(current + self.max_raw_ik_error_rad, q_cmd + self.max_raw_ik_error_rad)
            if low > high:
                value = float(current)
                limited = True
            else:
                value = max(low, min(high, float(raw)))
                limited = limited or abs(value - float(raw)) > 1e-9
            bounded.append(value)
        return bounded, limited

    def _update_saturation_watchdog(self, desired_velocity: list[float], dt: float) -> None:
        threshold = self.servo.max_joint_velocity_rad_s * self.saturation_velocity_ratio
        if threshold <= 0.0 or dt <= 0.0:
            self._is_saturated = False
            self._saturated_joint_indices_1_based = []
            self._saturation_time_sec = 0.0
            self._watchdog_active = False
            self._watchdog_reason = ""
            return
        saturated = [
            index + 1
            for index, value in enumerate(desired_velocity)
            if abs(float(value)) >= threshold
        ]
        self._is_saturated = len(saturated) >= self.saturation_min_joints
        self._saturated_joint_indices_1_based = saturated
        if self._is_saturated:
            self._saturation_time_sec += dt
        else:
            self._saturation_time_sec = 0.0
            self._watchdog_active = False
            self._watchdog_reason = ""
        if self.saturation_hold_sec > 0.0 and self._saturation_time_sec >= self.saturation_hold_sec:
            self._watchdog_active = True
            self._watchdog_reason = "saturation_hold"

    def _update_joint_tracking_error(
        self,
        q_cmd_reference: list[float],
        actual_joints: list[float],
    ) -> None:
        errors = [
            abs(float(commanded) - float(actual))
            for commanded, actual in zip(q_cmd_reference, actual_joints, strict=True)
        ]
        self._joint_tracking_error_rad = max(errors) if errors else 0.0
        limit = self.max_joint_tracking_error_rad
        self._joint_tracking_error_indices_1_based = [
            index + 1
            for index, value in enumerate(errors)
            if limit > 0.0 and value > limit
        ]
        self._joint_tracking_error_limited = bool(self._joint_tracking_error_indices_1_based)

    def _joint_tracking_error_exceeded(self) -> bool:
        return (
            self.max_joint_tracking_error_rad > 0.0
            and self._joint_tracking_error_rad > self.max_joint_tracking_error_rad
        )

    def _effective_joint_tracking_release_rad(self) -> float:
        if self.joint_tracking_release_rad > 0.0:
            return self.joint_tracking_release_rad
        return self.max_joint_tracking_error_rad

    def _joint_tracking_hold_should_continue(self, now: float) -> bool:
        if self._joint_tracking_error_exceeded() and not self._joint_tracking_hold_active:
            self._joint_tracking_hold_active = True
            self._joint_tracking_hold_until_sec = now + self.joint_tracking_hold_min_sec
            return True
        if not self._joint_tracking_hold_active:
            return False
        release = self._effective_joint_tracking_release_rad()
        if now < self._joint_tracking_hold_until_sec:
            return True
        if release > 0.0 and self._joint_tracking_error_rad > release:
            return True
        self._joint_tracking_hold_active = False
        return False

    def _joint_tracking_error_fault_exceeded(self) -> bool:
        self._joint_tracking_error_faulted = (
            self.max_joint_tracking_error_fault_rad > 0.0
            and self._joint_tracking_error_rad > self.max_joint_tracking_error_fault_rad
        )
        return self._joint_tracking_error_faulted

    def _hold_actual_joints(
        self,
        *,
        now: float,
        dt: float,
        actual_joints: list[float],
        reason: str,
    ) -> bool:
        self._watchdog_active = True
        self._watchdog_reason = reason
        self._hold_current_active = False
        self.ik_state.reset_session(actual_joints)
        self.ik_state.hold_current_target()
        self._record_debug_state(
            dt=dt,
            q_current=actual_joints,
            q_cmd=actual_joints,
            raw_ik_q=actual_joints,
            qdot_cmd=[0.0] * 6,
            tcp_current=self.ik_state.current_palm_position_m.tolist(),
        )
        self.servo.target_joints = list(actual_joints)
        self.servo.command = JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6)
        self.servo.reset_velocity_limiter()
        streamed = self.servo.tick()
        self._last_q_cmd = list(self.servo.target_joints or actual_joints)
        self.last_tick_time = now
        if not streamed and not self.servo.enabled:
            self.last_tick_time = None
        return streamed

    def _position_target_deadband_holds(self, target_position: list[float]) -> bool:
        if self.target_deadband_m <= 0.0:
            self._target_deadband_hold = False
            self._last_position_target_m = [float(value) for value in target_position]
            return False
        target = [float(value) for value in target_position]
        if self._last_position_target_m is None:
            self._last_position_target_m = target
            self._target_deadband_hold = False
            return False
        delta = math.sqrt(
            sum(
                (current - previous) ** 2
                for current, previous in zip(target, self._last_position_target_m, strict=True)
            )
        )
        if delta <= self.target_deadband_m:
            self._target_deadband_hold = True
            return True
        self._last_position_target_m = target
        self._target_deadband_hold = False
        return False

    def _hold_commanded_joints(
        self,
        *,
        now: float,
        dt: float,
        actual_joints: list[float],
        q_cmd_reference: list[float],
        tcp_current: list[float],
        reason: str,
    ) -> bool:
        self._watchdog_active = True
        self._watchdog_reason = reason
        hold_joints = list(q_cmd_reference)
        self._record_debug_state(
            dt=dt,
            q_current=actual_joints,
            q_cmd=hold_joints,
            raw_ik_q=actual_joints,
            qdot_cmd=[0.0] * 6,
            tcp_current=tcp_current,
        )
        self.servo.target_joints = hold_joints
        self.servo.command = JointJogCommand(deadman=True, joint_velocity_rad_s=[0.0] * 6)
        self.servo.reset_velocity_limiter()
        streamed = self.servo.tick()
        self._last_q_cmd = list(self.servo.target_joints or hold_joints)
        self.last_tick_time = now
        if not streamed and not self.servo.enabled:
            self.last_tick_time = None
        return streamed

    def _reset_motion_watchdogs(self) -> None:
        self._is_saturated = False
        self._saturated_joint_indices_1_based = []
        self._saturation_time_sec = 0.0
        self._watchdog_active = False
        self._watchdog_reason = ""
        self._raw_ik_error_limited = False
        self._target_deadband_hold = False
        self._joint_tracking_error_limited = False
        self._joint_tracking_error_faulted = False
        self._joint_tracking_hold_active = False
        self._joint_tracking_hold_until_sec = 0.0
        self.servo.reset_velocity_limiter()

    def _record_debug_state(
        self,
        *,
        dt: float,
        q_current: list[float],
        q_cmd: list[float],
        raw_ik_q: list[float],
        qdot_cmd: list[float],
        tcp_current: list[float] | None = None,
    ) -> None:
        self._last_tick_dt_sec = float(dt)
        self._last_q_current = [float(value) for value in q_current]
        self._last_q_cmd = [float(value) for value in q_cmd]
        self._last_raw_ik_q = [float(value) for value in raw_ik_q]
        self._last_qdot_cmd = [float(value) for value in qdot_cmd]
        self._last_joint_error = [
            float(raw) - float(current)
            for raw, current in zip(raw_ik_q, q_current, strict=True)
        ]
        if tcp_current is None:
            tcp_current = self.ik_state.current_palm_position_m.tolist()
        self._last_tcp_current = [float(value) for value in tcp_current]
        self._last_tcp_target = self.ik_state.target_palm_position_m.tolist()
        self._last_tcp_target_error_m = self.ik_state.target_error_m
