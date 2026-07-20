from __future__ import annotations

import math

import numpy as np
import pytest

from teleoperation.contracts import JointTarget, Pose3D, TimestampSet
from teleoperation.processing.target_shaper import CartesianMotionLimits, JerkLimitedPoseShaper
from teleoperation.supervision import ArmSafetySupervisor, JointSafetyLimits, SafetyEnvelope
from teleoperation.transforms.se3 import quaternion_angle, quaternion_exp


IDENTITY = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_cartesian_shaper_clips_workspace_orientation_and_dynamic_derivatives() -> None:
    limits = CartesianMotionLimits()
    shaper = JerkLimitedPoseShaper(limits)
    start = 1_000_000_000
    shaper.reset(IDENTITY, timestamp_ns=start)
    target = Pose3D((1.0, -1.0, 1.0), quaternion_exp((0.0, 0.0, math.pi)))
    previous_velocity = np.zeros(3)
    previous_acceleration = np.zeros(3)
    previous_angular_velocity = np.zeros(3)
    previous_angular_acceleration = np.zeros(3)
    outputs = []
    for index in range(1, 301):
        now = start + index * 8_000_000
        output = shaper.update(
            target,
            source_id="synthetic",
            sequence=index,
            source_timestamps=TimestampSet(now - 1_000_000),
            now_ns=now,
        )
        diagnostics = shaper.last_diagnostics
        assert diagnostics is not None
        velocity = np.asarray(diagnostics.linear_velocity_m_s)
        acceleration = np.asarray(diagnostics.linear_acceleration_m_s2)
        angular_velocity = np.asarray(diagnostics.angular_velocity_rad_s)
        angular_acceleration = np.asarray(diagnostics.angular_acceleration_rad_s2)
        assert np.max(np.abs(velocity)) <= limits.maximum_linear_speed_m_s + 1e-12
        assert np.max(np.abs(acceleration)) <= limits.maximum_linear_acceleration_m_s2 + 1e-12
        assert np.max(np.abs((acceleration - previous_acceleration) / 0.008)) <= limits.maximum_linear_jerk_m_s3 + 1e-9
        assert np.max(np.abs(angular_velocity)) <= limits.maximum_angular_speed_rad_s + 1e-12
        assert np.max(np.abs(angular_acceleration)) <= limits.maximum_angular_acceleration_rad_s2 + 1e-12
        assert np.max(np.abs((angular_acceleration - previous_angular_acceleration) / 0.008)) <= limits.maximum_angular_jerk_rad_s3 + 1e-9
        previous_velocity = velocity
        previous_acceleration = acceleration
        previous_angular_velocity = angular_velocity
        previous_angular_acceleration = angular_acceleration
        outputs.append(output)
    final = outputs[-1]
    assert np.all(np.abs(final.pose.position_m) <= np.asarray(limits.workspace_half_extent_m) + 1e-12)
    assert quaternion_angle(final.pose.quaternion_xyzw, IDENTITY.quaternion_xyzw) <= limits.maximum_orientation_deviation_rad + 1e-12
    assert shaper.last_diagnostics is not None
    assert shaper.last_diagnostics.workspace_clipped
    assert shaper.last_diagnostics.orientation_clipped


def test_shaper_rejects_timing_gap_instead_of_hiding_dropout() -> None:
    shaper = JerkLimitedPoseShaper(CartesianMotionLimits())
    shaper.reset(IDENTITY, timestamp_ns=1)
    with pytest.raises(RuntimeError, match="gap"):
        shaper.update(
            IDENTITY,
            source_id="source",
            sequence=1,
            source_timestamps=TimestampSet(1),
            now_ns=100_000_000,
        )


def joint_target(
    sequence: int,
    *,
    position: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    velocity: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    acceleration: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
) -> JointTarget:
    return JointTarget("source", sequence, sequence, position, velocity, acceleration, 0, 1)  # type: ignore[arg-type]


def test_joint_workspace_velocity_acceleration_and_jerk_limits_are_independent() -> None:
    limits = JointSafetyLimits.jaka_mini2_first_test()
    supervisor = ArmSafetySupervisor(
        SafetyEnvelope(IDENTITY, (0.015, 0.015, 0.015), math.radians(4.0)),
        limits,
    )
    assert supervisor.evaluate_joint_target(joint_target(1), previous=None, dt_s=None, now_ns=1).action.value == "allow"
    near_limit = joint_target(2, position=(6.25, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert "joint_soft_limit" in supervisor.evaluate_joint_target(near_limit, previous=None, dt_s=None, now_ns=2).reasons

    supervisor.reset_fault(safe=True)
    fast = joint_target(3, velocity=(0.081, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert "joint_velocity_limit" in supervisor.evaluate_joint_target(fast, previous=None, dt_s=None, now_ns=3).reasons

    supervisor.reset_fault(safe=True)
    previous = joint_target(4)
    jerk = joint_target(5, acceleration=(0.02, 0.0, 0.0, 0.0, 0.0, 0.0))
    assert "joint_jerk_limit" in supervisor.evaluate_joint_target(jerk, previous=previous, dt_s=0.008, now_ns=5).reasons
