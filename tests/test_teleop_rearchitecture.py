from __future__ import annotations

import math
from pathlib import Path

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
from teleop_rearchitecture.replay import (
    ReplaySample,
    causal_joint_target,
    interpolated_joint_target,
    run_replay,
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


def test_intended_target_selection_never_uses_a_future_sample() -> None:
    samples = [
        ReplaySample(1_000_000_000, (0.0,) * 6),
        ReplaySample(1_020_000_000, (1.0,) * 6),
    ]
    assert causal_joint_target(samples, 1_010_000_000) == (0.0,) * 6
    assert interpolated_joint_target(samples, 1_010_000_000) == (0.5,) * 6


def test_position_feed_forward_uses_source_target_timestamp_delta() -> None:
    servo = JerkBoundedPositionServo((0.0,) * 6, ShaperLimits())
    servo.set_target((0.0,) * 6, timestamp_ns=1_000_000_000)
    servo.set_target((0.2,) * 6, timestamp_ns=1_020_000_000)
    assert servo.target_velocity_rad_s == pytest.approx((10.0,) * 6)
    with pytest.raises(ValueError, match="must increase"):
        servo.set_target((0.3,) * 6, timestamp_ns=1_020_000_000)


def test_clutch_release_clears_pending_position_feed_forward() -> None:
    servo = JerkBoundedPositionServo((0.0,) * 6, ShaperLimits())
    servo.set_target((0.0,) * 6, timestamp_ns=1_000_000_000)
    servo.set_target((0.2,) * 6, timestamp_ns=1_020_000_000)
    assert max(abs(value) for value in servo.target_velocity_rad_s) > 0.0
    servo.request_controlled_stop()
    assert servo.target_velocity_rad_s == (0.0,) * 6


def test_replay_reports_active_interpolated_and_settling_metrics_separately() -> None:
    samples = [
        ReplaySample(1_000_000_000, (0.0,) * 6),
        ReplaySample(1_020_000_000, (0.01,) * 6),
        ReplaySample(1_040_000_000, (0.02,) * 6),
    ]
    result = run_replay(
        samples,
        prototype="jerk_bounded_position",
        xml_path=Path("data/sim_assets/jaka_rh56.xml"),
    )
    assert result["active_tracking"]["reference"] == "latest target with timestamp <= servo tick"
    assert result["timestamp_interpolated_tracking"]["reference"] == "joint-linear interpolation at servo tick timestamp"
    assert result["settling"]["reference"] == "final target after final target timestamp"
    assert result["active_tracking"]["sample_count"] > 0
    assert result["settling"]["sample_count"] > 0


@pytest.mark.parametrize("servo_type", (ResolvedRateVelocityServo, JerkBoundedPositionServo))
def test_clutch_release_during_motion_takes_multiple_bounded_ticks(servo_type: type[object]) -> None:
    limits = ShaperLimits(maximum_velocity_rad_s=1.0, maximum_acceleration_rad_s2=2.0, maximum_jerk_rad_s3=10.0)
    servo = servo_type((0.0,) * 6, limits)  # type: ignore[call-arg]
    servo.set_target((1.0,) * 6, timestamp_ns=1_000_000_000)  # type: ignore[attr-defined]
    for _ in range(20):
        moving = servo.tick()  # type: ignore[attr-defined]
    assert max(abs(value) for value in moving.velocity_rad_s) > 0.05
    release_position = moving.position_rad
    servo.request_controlled_stop()  # type: ignore[attr-defined]
    stop_ticks = 0
    while stop_ticks < 1000:
        stopped = servo.tick()  # type: ignore[attr-defined]
        stop_ticks += 1
        if max(abs(value) for value in stopped.velocity_rad_s) < 1e-3 and max(abs(value) for value in stopped.acceleration_rad_s2) < 1e-2:
            break
    assert 1 < stop_ticks < 1000
    assert max(abs(value) for value in stopped.jerk_rad_s3) <= limits.maximum_jerk_rad_s3 + 1e-12
    assert max(abs(a - b) for a, b in zip(stopped.position_rad, release_position, strict=True)) > 0.0
