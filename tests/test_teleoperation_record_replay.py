from __future__ import annotations

from teleoperation.contracts import ArmPoseSample, Pose3D, RunGateSample, TimestampSet
from teleoperation.input.interface import AdapterSnapshot
from teleoperation.input.replay import PoseStreamRecorder, ReplayPoseInput


def snapshot(generation: int, timestamp: int) -> AdapterSnapshot:
    pose = ArmPoseSample(
        "source",
        generation,
        "frame",
        Pose3D((generation * 0.001, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        TimestampSet(timestamp),
    )
    gate = RunGateSample("source", generation, timestamp, False, True)
    return AdapterSnapshot(pose, gate, True, generation, "ok")


def test_record_round_trip_and_replay_skips_backlog(tmp_path) -> None:
    path = tmp_path / "stream.jsonl"
    with PoseStreamRecorder(path, metadata={"test": True}) as recorder:
        for index in range(1, 5):
            recorder.write(snapshot(index, index * 10_000_000))
    replay = ReplayPoseInput(path, start_ns=1_000_000_000)
    first = replay.latest(now_ns=1_000_000_000)
    assert first is not None and first.generation == 1
    latest = replay.latest(now_ns=1_030_000_000)
    assert latest is not None and latest.generation == 4
    assert replay.skipped_backlog == 2
    assert replay.latest(now_ns=1_040_000_000, after_generation=4) is None
