from __future__ import annotations

from dataclasses import replace

from teleoperation.contracts import (
    ArmPoseSample,
    DiscontinuityKind,
    Pose3D,
    TimestampSet,
    TrackingState,
)
from teleoperation.processing.pose_validator import PoseValidationConfig, PoseValidator, ValidationAction


POSE = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def sample(sequence: int, receive_ns: int, **updates: object) -> ArmPoseSample:
    base = ArmPoseSample("source", sequence, "frame", POSE, TimestampSet(receive_ns))
    return replace(base, **updates)


def validator() -> PoseValidator:
    return PoseValidator(PoseValidationConfig("frame"))


def test_duplicate_reordered_and_optional_source_sequence() -> None:
    check = validator()
    assert check.validate(sample(1, 10, source_sequence=5), now_ns=11).action == ValidationAction.ACCEPT
    assert check.validate(sample(1, 12, source_sequence=6), now_ns=13).reason == "duplicate_sequence"
    assert check.validate(sample(0, 14, source_sequence=7), now_ns=15).reason == "reordered_sequence"


def test_stale_dropout_and_fatal_age_actions() -> None:
    assert validator().validate(sample(1, 0, sample_age_ns=110_000_000), now_ns=110_000_000).action == ValidationAction.HOLD
    assert validator().validate(sample(1, 0, sample_age_ns=600_000_000), now_ns=600_000_000).action == ValidationAction.CONTROLLED_STOP
    assert validator().validate(sample(1, 0, sample_age_ns=2_100_000_000), now_ns=2_100_000_000).action == ValidationAction.ABORT


def test_reconnect_tracking_recovery_and_discontinuity_require_reclutch() -> None:
    check = validator()
    assert check.validate(sample(1, 10), now_ns=11).action == ValidationAction.ACCEPT
    reconnect = sample(2, 20, connection_epoch=1, discontinuity=DiscontinuityKind.RECONNECT)
    assert check.validate(reconnect, now_ns=21).action == ValidationAction.RECLUTCH_REQUIRED

    check = validator()
    invalid = sample(1, 10, tracking_valid=False, tracking_state=TrackingState.INVALID)
    assert check.validate(invalid, now_ns=11).action == ValidationAction.RECLUTCH_REQUIRED
    assert check.validate(sample(2, 20), now_ns=21).action == ValidationAction.RECLUTCH_REQUIRED


def test_large_jump_is_not_smoothed_into_motion() -> None:
    check = validator()
    check.validate(sample(1, 1_000_000_000), now_ns=1_000_000_001)
    jumped = sample(2, 1_010_000_000, pose=Pose3D((0.3, 0.0, 0.0), POSE.quaternion_xyzw))
    assert check.validate(jumped, now_ns=1_010_000_001).action == ValidationAction.RECLUTCH_REQUIRED
