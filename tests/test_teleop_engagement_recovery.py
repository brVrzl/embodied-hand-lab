from __future__ import annotations

import math
from pathlib import Path

import pytest

from teleop_rearchitecture.cpp_shaping import CppReferenceShaper, OutputMode
from teleop_rearchitecture.engagement import (
    EngagementCoordinator,
    EngagementMode,
    EngagementResult,
    MeasuredJointState,
    SpatialPose,
)


PERIOD_NS = 8_000_000


def measured(*, sequence: int = 1, q: tuple[float, ...] = (0.0,) * 6,
             dq: tuple[float, ...] = (0.0,) * 6,
             ddq: tuple[float, ...] = (0.0,) * 6, valid: bool = True) -> MeasuredJointState:
    return MeasuredJointState(sequence, 1_000_000_000, q, dq, ddq, valid)


def pose(x: float = 0.0, orientation: tuple[float, float, float, float] = (1, 0, 0, 0)) -> SpatialPose:
    return SpatialPose.checked((x, 0.0, 0.0), orientation)


def engaged() -> EngagementCoordinator:
    coordinator = EngagementCoordinator()
    assert coordinator.initialize_disengaged(measured()) is EngagementResult.OK
    assert coordinator.observe_input(1, pose()) is EngagementResult.OK
    result, capture = coordinator.begin_engagement(measured(), 1_000_000_000)
    assert result is EngagementResult.OK and capture is not None
    assert coordinator.complete_engagement() is EngagementResult.OK
    return coordinator


@pytest.mark.parametrize(
    "release_pose,new_pose",
    [
        (pose(), pose()),
        (pose(), pose(0.75)),
        (pose(), pose(0.75, (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0))),
    ],
)
def test_release_motion_is_not_chased_after_reference_recapture(
    release_pose: SpatialPose, new_pose: SpatialPose
) -> None:
    coordinator = engaged()
    assert coordinator.note_target(1, 1, True) is EngagementResult.OK
    assert coordinator.observe_input(2, release_pose) is EngagementResult.OK
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.observe_input(3, new_pose) is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(measured(sequence=2), 2_000_000_000)
    assert result is EngagementResult.OK and capture is not None
    assert capture.safety_epoch == 2
    assert capture.input_reference == new_pose
    assert coordinator.complete_engagement() is EngagementResult.OK
    relative = coordinator.relative_pose()
    assert relative is not None
    assert relative.translation_m == pytest.approx((0.0, 0.0, 0.0), abs=1e-15)
    assert relative.rotation_wxyz == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-15)
    assert coordinator.snapshot().last_target_sequence == 0


def test_hold_rejected_release_and_reengage_clears_rejected_history() -> None:
    coordinator = engaged()
    assert coordinator.note_target(1, 1, False) is EngagementResult.OK
    assert coordinator.snapshot().mode is EngagementMode.HOLD_REJECTED
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.snapshot().frozen_source_sequence == 1
    assert coordinator.braking_complete() is EngagementResult.OK
    assert coordinator.observe_input(2, pose(0.2)) is EngagementResult.OK
    result, _ = coordinator.begin_engagement(measured(sequence=2), 2_000_000_000)
    assert result is EngagementResult.OK
    assert coordinator.complete_engagement() is EngagementResult.OK
    assert coordinator.snapshot().mode is EngagementMode.ACTIVE_TRACKING
    assert coordinator.snapshot().last_target_sequence == 0


def test_reengage_during_braking_waits_and_repeated_events_are_idempotent() -> None:
    coordinator = engaged()
    assert coordinator.note_target(1, 1, True) is EngagementResult.OK
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.request_release() is EngagementResult.ALREADY_APPLIED
    result, capture = coordinator.begin_engagement(measured(sequence=2), 2_000_000_000)
    assert result is EngagementResult.WAIT_FOR_STOPPED and capture is None
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(measured(sequence=2), 2_100_000_000)
    assert result is EngagementResult.OK and capture is not None
    duplicate, duplicate_capture = coordinator.begin_engagement(measured(sequence=2), 2_100_000_000)
    assert duplicate is EngagementResult.ALREADY_APPLIED
    assert duplicate_capture == capture
    assert coordinator.complete_engagement() is EngagementResult.OK
    assert coordinator.complete_engagement() is EngagementResult.ALREADY_APPLIED


def test_old_epoch_and_input_backlog_are_rejected() -> None:
    coordinator = engaged()
    for sequence in range(2, 102):
        assert coordinator.observe_input(sequence, pose(sequence / 100.0)) is EngagementResult.OK
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(measured(sequence=2), 2_000_000_000)
    assert result is EngagementResult.OK and capture is not None
    assert capture.input_sequence == 101
    assert capture.input_reference == pose(1.01)
    assert coordinator.complete_engagement() is EngagementResult.OK
    assert coordinator.note_target(500, 1, True) is EngagementResult.OLD_EPOCH
    assert coordinator.note_target(1, 2, True) is EngagementResult.OK
    assert coordinator.note_target(1, 2, True) is EngagementResult.OLD_SEQUENCE
    snapshot = coordinator.snapshot()
    assert snapshot.old_target_rejection_count == 1
    assert snapshot.input_replacement_count == 100


def test_hard_stop_requires_explicit_valid_measured_reset() -> None:
    coordinator = engaged()
    coordinator.hard_stop()
    assert coordinator.snapshot().mode is EngagementMode.HARD_STOPPED
    result, _ = coordinator.begin_engagement(measured(), 2_000_000_000)
    assert result is EngagementResult.INVALID_STATE
    assert coordinator.reset_hard_stop(measured(valid=False)) is EngagementResult.INVALID_MEASUREMENT
    assert coordinator.reset_hard_stop(measured(sequence=2)) is EngagementResult.OK
    assert coordinator.snapshot().mode is EngagementMode.DISENGAGED
    assert coordinator.snapshot().safety_epoch == 2


def test_stopped_ready_can_wait_without_emitting_a_motion_target() -> None:
    coordinator = engaged()
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    for sequence in range(2, 1_002):
        assert coordinator.observe_input(sequence, pose(sequence * 1e-4)) is EngagementResult.OK
        assert coordinator.relative_pose() is None
    assert coordinator.snapshot().mode is EngagementMode.STOPPED_READY


def test_first_reengaged_cpp_output_is_continuous_with_measured_state(
    teleop_shaping_library: Path,
) -> None:
    q = (0.1, -0.2, 0.3, -0.1, 0.2, -0.3)
    state = measured(sequence=8, q=q)
    coordinator = engaged()
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.observe_input(2, pose(0.8, (0.9238795325, 0, 0.3826834324, 0))) is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(state, 2_000_000_000)
    assert result is EngagementResult.OK and capture is not None
    assert coordinator.complete_engagement() is EngagementResult.OK

    with CppReferenceShaper(teleop_shaping_library) as shaper:
        shaper.initialize(
            position_rad=q,
            velocity_rad_s=(0.0,) * 6,
            acceleration_rad_s2=(0.0,) * 6,
            minimum_position_rad=(-3.0,) * 6,
            maximum_position_rad=(3.0,) * 6,
            maximum_velocity_rad_s=(math.pi,) * 6,
            maximum_acceleration_rad_s2=(4 * math.pi,) * 6,
            maximum_jerk_rad_s3=(50.0,) * 6,
            now_ns=2_000_000_000,
            safety_epoch=capture.safety_epoch,
        )
        shaper.replace_target(
            q,
            sequence=1,
            source_monotonic_ns=2_000_000_000,
            accepted_monotonic_ns=2_000_000_000,
            valid_until_monotonic_ns=3_000_000_000,
        )
        first = shaper.tick(2_000_000_000)
    joint_delta = max(abs(left - right) for left, right in zip(first.position_rad, q))
    assert first.output_mode is OutputMode.ACTIVE_TRACKING
    assert joint_delta == 0.0
    assert max(abs(value) for value in first.velocity_rad_s) == 0.0
    assert max(abs(value) for value in first.acceleration_rad_s2) == 0.0


def test_residual_measured_velocity_is_preserved_not_reconstructed_from_old_target() -> None:
    q = (0.0,) * 6
    dq = (0.0, 0.002, 0.0, 0.0, 0.0, 0.0)
    state = measured(sequence=9, q=q, dq=dq)
    coordinator = engaged()
    assert coordinator.request_release() is EngagementResult.OK
    assert coordinator.braking_complete() is EngagementResult.OK
    result, capture = coordinator.begin_engagement(state, 2_000_000_000)
    assert result is EngagementResult.OK and capture is not None
    assert capture.robot_reference.velocity_rad_s == dq
    assert capture.robot_reference.position_rad == q
