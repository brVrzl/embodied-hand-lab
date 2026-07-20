from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ..contracts import Pose3D, PoseTarget, TimestampSet
from ..transforms.se3 import (
    quaternion_conjugate,
    quaternion_exp,
    quaternion_log,
    quaternion_multiply,
)


@dataclass(frozen=True, slots=True)
class CartesianMotionLimits:
    workspace_half_extent_m: tuple[float, float, float] = (0.015, 0.015, 0.015)
    maximum_orientation_deviation_rad: float = math.radians(4.0)
    maximum_linear_speed_m_s: float = 0.008
    maximum_linear_acceleration_m_s2: float = 0.030
    maximum_linear_jerk_m_s3: float = 0.150
    maximum_angular_speed_rad_s: float = math.radians(4.0)
    maximum_angular_acceleration_rad_s2: float = math.radians(15.0)
    maximum_angular_jerk_rad_s3: float = math.radians(60.0)
    maximum_step_dt_s: float = 0.050

    def __post_init__(self) -> None:
        values = (
            *self.workspace_half_extent_m,
            self.maximum_orientation_deviation_rad,
            self.maximum_linear_speed_m_s,
            self.maximum_linear_acceleration_m_s2,
            self.maximum_linear_jerk_m_s3,
            self.maximum_angular_speed_rad_s,
            self.maximum_angular_acceleration_rad_s2,
            self.maximum_angular_jerk_rad_s3,
            self.maximum_step_dt_s,
        )
        if len(self.workspace_half_extent_m) != 3 or not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Cartesian motion limits must be finite and positive")
        if any(value > 0.0200001 for value in self.workspace_half_extent_m):
            raise ValueError("first-test workspace may not exceed 20 mm per axis")
        if self.maximum_orientation_deviation_rad > math.radians(5.0) + 1e-12:
            raise ValueError("first-test orientation deviation may not exceed 5 degrees")
        if self.maximum_linear_speed_m_s > 0.0100001:
            raise ValueError("first-test linear speed may not exceed 10 mm/s")
        if self.maximum_angular_speed_rad_s > math.radians(5.0) + 1e-12:
            raise ValueError("first-test angular speed may not exceed 5 deg/s")


@dataclass(frozen=True, slots=True)
class ShaperDiagnostics:
    workspace_clipped: bool
    orientation_clipped: bool
    linear_velocity_m_s: tuple[float, float, float]
    linear_acceleration_m_s2: tuple[float, float, float]
    angular_velocity_rad_s: tuple[float, float, float]
    angular_acceleration_rad_s2: tuple[float, float, float]


def _bounded_dynamics_step(
    error: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    *,
    dt: float,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
) -> tuple[np.ndarray, np.ndarray]:
    natural_frequency = min(4.0, max_acceleration / max_velocity)
    desired_velocity = np.clip(natural_frequency * error, -max_velocity, max_velocity)
    desired_acceleration = np.clip(
        (desired_velocity - velocity) / dt,
        -max_acceleration,
        max_acceleration,
    )
    acceleration_delta = np.clip(
        desired_acceleration - acceleration,
        -max_jerk * dt,
        max_jerk * dt,
    )
    new_acceleration = np.clip(
        acceleration + acceleration_delta,
        -max_acceleration,
        max_acceleration,
    )
    new_velocity = np.clip(
        velocity + new_acceleration * dt,
        -max_velocity,
        max_velocity,
    )
    return new_velocity, new_acceleration


class JerkLimitedPoseShaper:
    """Interruptible latest-target Cartesian shaper with explicit third-order bounds.

    This layer shapes Cartesian targets only.  The native joint trajectory layer
    remains independently responsible for joint velocity, acceleration, jerk,
    and tracking limits.
    """

    def __init__(self, limits: CartesianMotionLimits) -> None:
        self.limits = limits
        self._anchor: Pose3D | None = None
        self._pose: Pose3D | None = None
        self._timestamp_ns: int | None = None
        self._linear_velocity = np.zeros(3)
        self._linear_acceleration = np.zeros(3)
        self._angular_velocity = np.zeros(3)
        self._angular_acceleration = np.zeros(3)
        self.last_diagnostics: ShaperDiagnostics | None = None

    def reset(self, pose: Pose3D, *, timestamp_ns: int) -> None:
        self._anchor = pose
        self._pose = pose
        self._timestamp_ns = int(timestamp_ns)
        self._linear_velocity[:] = 0.0
        self._linear_acceleration[:] = 0.0
        self._angular_velocity[:] = 0.0
        self._angular_acceleration[:] = 0.0
        self.last_diagnostics = ShaperDiagnostics(False, False, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _clip_target(self, target: Pose3D) -> tuple[Pose3D, bool, bool]:
        if self._anchor is None:
            raise RuntimeError("pose shaper is not initialized")
        anchor_position = np.asarray(self._anchor.position_m)
        target_position = np.asarray(target.position_m)
        extent = np.asarray(self.limits.workspace_half_extent_m)
        # Preserve an absolute-envelope guard for bounded deceleration while
        # acceleration itself is jerk-limited.  The safety supervisor still
        # checks the full configured envelope independently.
        linear_guard = (
            self.limits.maximum_linear_speed_m_s**2
            / (2.0 * self.limits.maximum_linear_acceleration_m_s2)
            + self.limits.maximum_linear_speed_m_s
            * self.limits.maximum_linear_acceleration_m_s2
            / self.limits.maximum_linear_jerk_m_s3
        )
        effective_extent = extent - linear_guard
        if np.any(effective_extent <= 0.0):
            raise ValueError("workspace is too small for configured dynamic stopping guard")
        clipped_position = np.clip(
            target_position,
            anchor_position - effective_extent,
            anchor_position + effective_extent,
        )
        workspace_clipped = not np.allclose(clipped_position, target_position, atol=0.0, rtol=0.0)

        orientation_delta = quaternion_multiply(
            target.quaternion_xyzw,
            quaternion_conjugate(self._anchor.quaternion_xyzw),
        )
        rotation_vector = quaternion_log(orientation_delta)
        angle = float(np.linalg.norm(rotation_vector))
        angular_guard = (
            self.limits.maximum_angular_speed_rad_s**2
            / (2.0 * self.limits.maximum_angular_acceleration_rad_s2)
            + self.limits.maximum_angular_speed_rad_s
            * self.limits.maximum_angular_acceleration_rad_s2
            / self.limits.maximum_angular_jerk_rad_s3
        )
        effective_orientation = self.limits.maximum_orientation_deviation_rad - angular_guard
        if effective_orientation <= 0.0:
            raise ValueError("orientation envelope is too small for configured dynamic stopping guard")
        orientation_clipped = angle > effective_orientation
        if orientation_clipped:
            rotation_vector *= effective_orientation / angle
        clipped_orientation = quaternion_multiply(
            quaternion_exp(rotation_vector),
            self._anchor.quaternion_xyzw,
        )
        return (
            Pose3D(tuple(float(value) for value in clipped_position), clipped_orientation),
            workspace_clipped,
            orientation_clipped,
        )

    def update(
        self,
        target: Pose3D,
        *,
        source_id: str,
        sequence: int,
        source_timestamps: TimestampSet,
        now_ns: int,
    ) -> PoseTarget:
        if self._pose is None or self._timestamp_ns is None:
            raise RuntimeError("pose shaper must be reset at clutch/recenter")
        dt = (now_ns - self._timestamp_ns) / 1e9
        if dt <= 0.0:
            raise ValueError("pose shaper timestamps must increase")
        if dt > self.limits.maximum_step_dt_s:
            raise RuntimeError("pose shaper update gap exceeds configured maximum")
        clipped, workspace_clipped, orientation_clipped = self._clip_target(target)

        position = np.asarray(self._pose.position_m)
        position_error = np.asarray(clipped.position_m) - position
        self._linear_velocity, self._linear_acceleration = _bounded_dynamics_step(
            position_error,
            self._linear_velocity,
            self._linear_acceleration,
            dt=dt,
            max_velocity=self.limits.maximum_linear_speed_m_s,
            max_acceleration=self.limits.maximum_linear_acceleration_m_s2,
            max_jerk=self.limits.maximum_linear_jerk_m_s3,
        )
        next_position = position + self._linear_velocity * dt

        orientation_error_q = quaternion_multiply(
            clipped.quaternion_xyzw,
            quaternion_conjugate(self._pose.quaternion_xyzw),
        )
        orientation_error = quaternion_log(orientation_error_q)
        self._angular_velocity, self._angular_acceleration = _bounded_dynamics_step(
            orientation_error,
            self._angular_velocity,
            self._angular_acceleration,
            dt=dt,
            max_velocity=self.limits.maximum_angular_speed_rad_s,
            max_acceleration=self.limits.maximum_angular_acceleration_rad_s2,
            max_jerk=self.limits.maximum_angular_jerk_rad_s3,
        )
        next_orientation = quaternion_multiply(
            quaternion_exp(self._angular_velocity * dt),
            self._pose.quaternion_xyzw,
        )
        self._pose = Pose3D(tuple(float(value) for value in next_position), next_orientation)
        self._timestamp_ns = now_ns
        self.last_diagnostics = ShaperDiagnostics(
            workspace_clipped,
            orientation_clipped,
            tuple(float(value) for value in self._linear_velocity),
            tuple(float(value) for value in self._linear_acceleration),
            tuple(float(value) for value in self._angular_velocity),
            tuple(float(value) for value in self._angular_acceleration),
        )
        timestamps = source_timestamps.with_stage(processing_ns=now_ns)
        return PoseTarget(
            source_id=source_id,
            sequence=sequence,
            target_frame_id="robot_base",
            pose=self._pose,
            timestamps=timestamps,
            linear_velocity_m_s=self.last_diagnostics.linear_velocity_m_s,
            angular_velocity_rad_s=self.last_diagnostics.angular_velocity_rad_s,
        )
