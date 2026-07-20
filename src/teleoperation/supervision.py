from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .contracts import (
    HealthLevel,
    JointTarget,
    Pose3D,
    PoseTarget,
    SafetyAction,
    SafetyState,
)
from .transforms.se3 import quaternion_angle


@dataclass(frozen=True, slots=True)
class JointSafetyLimits:
    lower_rad: tuple[float, float, float, float, float, float]
    upper_rad: tuple[float, float, float, float, float, float]
    soft_margin_rad: float = math.radians(5.0)
    maximum_velocity_rad_s: float = 0.08
    maximum_acceleration_rad_s2: float = 0.25
    maximum_jerk_rad_s3: float = 1.0

    def __post_init__(self) -> None:
        if len(self.lower_rad) != 6 or len(self.upper_rad) != 6:
            raise ValueError("joint limits require six lower and upper values")
        if not all(math.isfinite(v) for v in (*self.lower_rad, *self.upper_rad)):
            raise ValueError("joint limits must be finite")
        if not all(low < high for low, high in zip(self.lower_rad, self.upper_rad)):
            raise ValueError("joint lower limits must be below upper limits")
        if min(
            self.soft_margin_rad,
            self.maximum_velocity_rad_s,
            self.maximum_acceleration_rad_s2,
            self.maximum_jerk_rad_s3,
        ) <= 0.0:
            raise ValueError("joint dynamic limits and margin must be positive")

    @classmethod
    def jaka_mini2_first_test(cls) -> "JointSafetyLimits":
        return cls(
            lower_rad=(-6.28, -2.09, -2.27, -6.28, -2.09, -6.28),
            upper_rad=(6.28, 2.09, 2.27, 6.28, 2.09, 6.28),
        )


@dataclass(frozen=True, slots=True)
class SafetyEnvelope:
    startup_tcp_pose: Pose3D
    workspace_half_extent_m: tuple[float, float, float]
    maximum_orientation_deviation_rad: float
    maximum_session_ns: int = 10_000_000_000


class ArmSafetySupervisor:
    def __init__(self, envelope: SafetyEnvelope, joint_limits: JointSafetyLimits) -> None:
        self.envelope = envelope
        self.joint_limits = joint_limits
        self.started_ns: int | None = None
        self.fault_latched = False

    def start(self, now_ns: int) -> None:
        self.started_ns = now_ns

    def stop(self) -> None:
        self.started_ns = None

    def _decision(self, action: SafetyAction, now_ns: int, *reasons: str) -> SafetyState:
        if action == SafetyAction.ABORT:
            self.fault_latched = True
        return SafetyState(action, now_ns, tuple(reasons), self.fault_latched)

    def evaluate_cartesian(self, target: PoseTarget, *, now_ns: int) -> SafetyState:
        if self.fault_latched:
            return self._decision(SafetyAction.ABORT, now_ns, "fault_latched")
        if self.started_ns is None:
            return self._decision(SafetyAction.HOLD, now_ns, "session_not_started")
        if now_ns - self.started_ns >= self.envelope.maximum_session_ns:
            return self._decision(SafetyAction.CONTROLLED_STOP, now_ns, "maximum_session_duration")
        delta = np.abs(
            np.asarray(target.pose.position_m) - np.asarray(self.envelope.startup_tcp_pose.position_m)
        )
        if np.any(delta > np.asarray(self.envelope.workspace_half_extent_m) + 1e-12):
            return self._decision(SafetyAction.ABORT, now_ns, "absolute_workspace_envelope")
        angle = quaternion_angle(
            target.pose.quaternion_xyzw,
            self.envelope.startup_tcp_pose.quaternion_xyzw,
        )
        if angle > self.envelope.maximum_orientation_deviation_rad + 1e-12:
            return self._decision(SafetyAction.ABORT, now_ns, "absolute_orientation_envelope")
        return self._decision(SafetyAction.ALLOW, now_ns, "cartesian_target_safe")

    def evaluate_joint_target(
        self,
        target: JointTarget,
        *,
        previous: JointTarget | None,
        dt_s: float | None,
        now_ns: int,
    ) -> SafetyState:
        lower = np.asarray(self.joint_limits.lower_rad) + self.joint_limits.soft_margin_rad
        upper = np.asarray(self.joint_limits.upper_rad) - self.joint_limits.soft_margin_rad
        position = np.asarray(target.joint_position_rad)
        if np.any(position < lower) or np.any(position > upper):
            return self._decision(SafetyAction.ABORT, now_ns, "joint_soft_limit")
        if np.max(np.abs(target.joint_velocity_rad_s)) > self.joint_limits.maximum_velocity_rad_s + 1e-12:
            return self._decision(SafetyAction.ABORT, now_ns, "joint_velocity_limit")
        if np.max(np.abs(target.joint_acceleration_rad_s2)) > self.joint_limits.maximum_acceleration_rad_s2 + 1e-12:
            return self._decision(SafetyAction.ABORT, now_ns, "joint_acceleration_limit")
        if previous is not None:
            if dt_s is None or dt_s <= 0.0:
                return self._decision(SafetyAction.ABORT, now_ns, "invalid_joint_target_dt")
            jerk = (
                np.asarray(target.joint_acceleration_rad_s2)
                - np.asarray(previous.joint_acceleration_rad_s2)
            ) / dt_s
            if np.max(np.abs(jerk)) > self.joint_limits.maximum_jerk_rad_s3 + 1e-9:
                return self._decision(SafetyAction.ABORT, now_ns, "joint_jerk_limit")
        return self._decision(SafetyAction.ALLOW, now_ns, "joint_target_safe")

    def reset_fault(self, *, safe: bool) -> None:
        if not safe:
            raise RuntimeError("cannot reset safety fault while unsafe")
        self.fault_latched = False
