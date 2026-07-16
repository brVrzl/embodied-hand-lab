from __future__ import annotations

import pytest

from teleoperation.runtime.synthetic import FaultSchedule, SyntheticPattern, SyntheticPoseSource


@pytest.mark.parametrize("pattern", list(SyntheticPattern))
def test_every_synthetic_pattern_emits_valid_six_dof_pose(pattern: SyntheticPattern) -> None:
    target = SyntheticPoseSource(pattern).samples(1.25)[0]
    assert len(target.pose.position_m) == 3
    assert len(target.pose.quaternion_xyzw) == 4
    assert target.target_frame_id == "robot_base"


def test_dropout_and_slowdown_schedule() -> None:
    source = SyntheticPoseSource(SyntheticPattern.FIXED, faults=FaultSchedule(
        dropout_start_s=1.0, dropout_duration_s=0.5,
        slowdown_after_s=2.0, slowdown_factor=4.0,
    ))
    assert not source.should_drop(0.9)
    assert source.should_drop(1.25)
    assert not source.should_drop(1.5)
    assert source.period_scale(1.9) == 1.0
    assert source.period_scale(2.0) == 4.0


def test_timestamp_jitter_changes_only_source_clock_domain() -> None:
    target = SyntheticPoseSource(SyntheticPattern.FIXED, faults=FaultSchedule(
        timestamp_jitter_ns=1_000_000,
    )).samples(0.0)[0]
    assert target.timestamps.processing_ns == target.timestamps.local_receive_ns
    assert target.timestamps.source_capture_ns is not None


def test_duplicate_and_reordered_sequences_are_injected() -> None:
    duplicate = SyntheticPoseSource(SyntheticPattern.FIXED, faults=FaultSchedule(duplicate_every=2))
    assert [duplicate.samples(0.0)[0].sequence for _ in range(4)] == [0, 1, 1, 3]
    reordered = SyntheticPoseSource(SyntheticPattern.FIXED, faults=FaultSchedule(reorder_every=3))
    assert [reordered.samples(0.0)[0].sequence for _ in range(5)] == [0, 1, 2, 1, 4]


def test_burst_contains_new_samples_not_repeated_packet() -> None:
    source = SyntheticPoseSource(SyntheticPattern.FIXED, faults=FaultSchedule(
        burst_every=1, burst_count=3,
    ))
    assert [sample.sequence for sample in source.samples(0.0)] == [0, 1, 2]
