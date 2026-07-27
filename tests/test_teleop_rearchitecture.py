from __future__ import annotations

import math

import pytest

from teleop_rearchitecture import (
    CommandState,
    JerkBoundedPositionServo,
    JointCommand,
    LatestCommandMailbox,
    ResolvedRateVelocityServo,
    ShaperLimits,
    StopReason,
    output_must_terminate,
)


def _command(sequence: int, position: tuple[float, ...]) -> JointCommand:
    return JointCommand(sequence, sequence + 1, position, CommandState.ACTIVE, "test")


def test_latest_mailbox_has_no_backlog_and_preserves_latest() -> None:
    mailbox = LatestCommandMailbox()
    mailbox.publish(_command(1, (0.0,) * 6))
    mailbox.publish(_command(2, (0.2,) * 6))
    assert mailbox.depth == 1
    assert mailbox.replaced == 1
    assert mailbox.take_latest() == _command(2, (0.2,) * 6)
    assert mailbox.depth == 0


def test_non_active_command_cannot_replace_joint_target() -> None:
    with pytest.raises(ValueError, match="must not smuggle"):
        JointCommand(1, 1, (0.0,) * 6, CommandState.HOLD_REJECTED, "infeasible")


def test_feasibility_hold_is_not_a_liveness_stop_but_all_hardware_faults_are() -> None:
    assert not output_must_terminate(CommandState.HOLD_REJECTED)
    assert output_must_terminate(CommandState.DISENGAGED, StopReason.CLUTCH_RELEASE)
    for reason in (
        StopReason.STALE_INPUT,
        StopReason.CONTROLLER_ALARM,
        StopReason.SDK_FAILURE,
        StopReason.TIMING_FAULT,
    ):
        assert output_must_terminate(CommandState.HARD_STOP, reason)


@pytest.mark.parametrize("servo_type", (ResolvedRateVelocityServo, JerkBoundedPositionServo))
def test_shapers_bound_velocity_acceleration_and_jerk(servo_type: type[object]) -> None:
    limits = ShaperLimits(
        maximum_velocity_rad_s=1.0,
        maximum_acceleration_rad_s2=2.0,
        maximum_jerk_rad_s3=10.0,
    )
    servo = servo_type((0.0,) * 6, limits)  # type: ignore[call-arg]
    servo.set_target((1.0,) * 6)  # type: ignore[attr-defined]
    for _ in range(100):
        point = servo.tick()  # type: ignore[attr-defined]
        assert max(abs(value) for value in point.velocity_rad_s) <= 1.0 + 1e-12
        assert max(abs(value) for value in point.acceleration_rad_s2) <= 2.0 + 1e-12
        assert max(abs(value) for value in point.jerk_rad_s3) <= 10.0 + 1e-12


@pytest.mark.parametrize("servo_type", (ResolvedRateVelocityServo, JerkBoundedPositionServo))
def test_clutch_controlled_stop_settles_without_a_position_jump(servo_type: type[object]) -> None:
    limits = ShaperLimits(maximum_velocity_rad_s=1.0, maximum_acceleration_rad_s2=2.0, maximum_jerk_rad_s3=10.0)
    servo = servo_type((0.0,) * 6, limits)  # type: ignore[call-arg]
    servo.set_target((1.0,) * 6)  # type: ignore[attr-defined]
    for _ in range(20):
        previous = servo.tick()  # type: ignore[attr-defined]
    servo.request_controlled_stop()  # type: ignore[attr-defined]
    next_point = servo.tick()  # type: ignore[attr-defined]
    assert max(abs(a - b) for a, b in zip(next_point.position_rad, previous.position_rad, strict=True)) < limits.maximum_velocity_rad_s * limits.period_s + 1e-12
    for _ in range(1000):
        next_point = servo.tick()  # type: ignore[attr-defined]
        if max(abs(value) for value in next_point.velocity_rad_s) < 1e-3:
            break
    assert max(abs(value) for value in next_point.velocity_rad_s) < 1e-3
    assert all(math.isfinite(value) for value in next_point.position_rad)


@pytest.mark.parametrize("servo_type", (ResolvedRateVelocityServo, JerkBoundedPositionServo))
def test_target_replacement_is_continuous_without_a_queue(servo_type: type[object]) -> None:
    limits = ShaperLimits(maximum_velocity_rad_s=1.0, maximum_acceleration_rad_s2=2.0, maximum_jerk_rad_s3=10.0)
    servo = servo_type((0.0,) * 6, limits)  # type: ignore[call-arg]
    servo.set_target((1.0,) * 6)  # type: ignore[attr-defined]
    previous = servo.tick()  # type: ignore[attr-defined]
    servo.set_target((-1.0,) * 6)  # type: ignore[attr-defined]
    replacement = servo.tick()  # type: ignore[attr-defined]
    assert max(abs(a - b) for a, b in zip(replacement.position_rad, previous.position_rad, strict=True)) <= limits.maximum_velocity_rad_s * limits.period_s + 1e-12
