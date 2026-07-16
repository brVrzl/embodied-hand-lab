from __future__ import annotations

import json

import pytest

from teleoperation.contracts import (
    ArmPoseSample,
    Pose3D,
    TimestampSet,
    arm_pose_sample_from_dict,
    contract_json_dumps,
)
from teleoperation.sequence import SequenceDisposition, SequenceTracker


def test_arm_pose_contract_round_trip_and_separate_clock_domains() -> None:
    sample = ArmPoseSample("tracker", 7, "tracker_world", Pose3D((1, 2, 3), (0, 0, 0, 1)),
                           TimestampSet(100, source_capture_ns=9999, processing_ns=110, dispatch_ns=120))
    restored = arm_pose_sample_from_dict(json.loads(contract_json_dumps(sample)))
    assert restored == sample
    assert restored.timestamps.source_capture_ns != restored.timestamps.local_receive_ns


def test_local_pipeline_timestamps_must_follow_stage_order() -> None:
    with pytest.raises(ValueError, match="monotonic by stage"):
        TimestampSet(100, processing_ns=90)


@pytest.mark.parametrize("value", [True, 1.5, "10"])
def test_timestamps_reject_non_integer_nanoseconds(value: object) -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        TimestampSet(value)  # type: ignore[arg-type]


@pytest.mark.parametrize("value", [True, 1.0])
def test_sequences_reject_boolean_and_float(value: object) -> None:
    with pytest.raises(ValueError, match="non-negative integer"):
        ArmPoseSample("tracker", value, "world", Pose3D((0, 0, 0), (0, 0, 0, 1)),
                      TimestampSet(1))  # type: ignore[arg-type]


def test_pose_rejects_non_unit_quaternion() -> None:
    with pytest.raises(ValueError, match="unit length"):
        Pose3D((0, 0, 0), (0, 0, 0, 2))


def test_sequence_tracker_rejects_duplicate_and_reordered() -> None:
    tracker = SequenceTracker()
    assert tracker.observe(3) is SequenceDisposition.FIRST
    assert tracker.observe(4) is SequenceDisposition.NEW
    assert tracker.observe(4) is SequenceDisposition.DUPLICATE
    assert tracker.observe(2) is SequenceDisposition.REORDERED
    assert tracker.last_sequence == 4
