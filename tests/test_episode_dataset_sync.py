from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from episode_dataset.collector import SingleEpisodeCollector
from episode_dataset.episode import (
    CameraSample,
    CanonicalEpisodeWriter,
    CanonicalSample,
    ControlSample,
    EpisodeStatus,
    StartPrerequisites,
)
from episode_dataset.lerobot_staging import (
    LeRobotStagingWriter,
    materialize_staging_episode,
    set_staging_review,
)
from episode_dataset.process_runtime import _frame_number_gap
from episode_dataset.synchronization import synchronize_staging_episode
from episode_dataset.timeline import advance_fixed_deadline


def _camera(role: str, timestamp_ns: int, frame_number: int) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=np.full((8, 32, 3), frame_number, dtype=np.uint8),
        depth_raw=np.zeros((8, 32), dtype=np.uint16),
        device_rgb_timestamp_ms=1000.0 + frame_number,
        device_depth_timestamp_ms=1000.0 + frame_number,
        rgb_frame_number=frame_number,
        depth_frame_number=frame_number,
        rgb_timestamp_domain="hardware_clock",
        depth_timestamp_domain="hardware_clock",
    )


def _control(timestamp_ns: int, *, force: tuple[float, ...] | None = None) -> ControlSample:
    zeros = (0.0,) * 6
    return ControlSample(
        host_monotonic_ns=timestamp_ns,
        accepted_arm_q=zeros,
        arm_q_measured=zeros,
        arm_dq_measured=zeros,
        tcp_pose_xyzw=(0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0),
        hand_observation=zeros,
        hand_source="measured",
        hand_target=zeros,
        arm_trigger=True,
        hand_grip=True,
        force_observation=force,
        source_timestamps_ns={
            "jaka_observation": timestamp_ns - 1_000_000,
            "jaka_command": timestamp_ns - 500_000,
            "rh56_angle_act": timestamp_ns - 2_000_000,
            "rh56_force_act": timestamp_ns - 3_000_000,
        },
        source_timestamp_domains={
            "jaka_observation": "host_monotonic_ns",
            "jaka_command": "host_monotonic_ns",
            "rh56_angle_act": "host_monotonic_ns",
            "rh56_force_act": "host_monotonic_ns",
        },
    )


def _timed_control(
    host_ns: int,
    *,
    jaka_ns: int,
    angle_ns: int,
    force_ns: int,
    marker: float,
) -> ControlSample:
    return replace(
        _control(host_ns, force=(marker,) * 6),
        arm_q_measured=(marker,) * 6,
        hand_observation=(marker,) * 6,
        source_timestamps_ns={
            "jaka_observation": jaka_ns,
            "jaka_command": host_ns,
            "rh56_angle_act": angle_ns,
            "rh56_force_act": force_ns,
        },
    )


def test_fixed_deadline_skips_expired_slots_without_burst() -> None:
    assert advance_fixed_deadline(100, 10, 105) == 110
    assert advance_fixed_deadline(100, 10, 137) == 140
    assert _frame_number_gap(10, 13) == 2
    assert _frame_number_gap(-1, 13) == 0


def test_staging_force_and_timing_materialize_without_changing_state_action(
    tmp_path: Path,
) -> None:
    root = tmp_path / "staging"
    writer = LeRobotStagingWriter(
        root, episode_index=0, task_name="pick", operator="test", dataset_fps=30
    )
    base = 1_000_000_000
    control = _control(base, force=(10.0, 20.0, 30.0, 40.0, 50.0, 60.0))
    workspace = _camera("workspace", base, 1)
    wrist = _camera("wrist", base, 1)
    writer.begin(
        StartPrerequisites(
            trigger_press_monotonic_ns=base,
            reference_established=True,
            accepted=control,
            workspace=workspace,
            wrist=wrist,
            maximum_start_delta_rad=0.2,
            maximum_hand_start_delta_rad=0.2,
        ),
        camera_max_age_ns=100_000_000,
    )
    writer.append_sample(
        CanonicalSample(
            frame_index=0,
            timestamp_ns=base,
            control=control,
            workspace=workspace,
            wrist=wrist,
            source_offsets_ns={"control": 0, "workspace": 0, "wrist": 0},
            synchronization_valid=True,
        )
    )
    writer.finalize(
        EpisodeStatus.COMPLETED,
        termination_reason="test_complete",
        trigger_release_monotonic_ns=None,
    )
    row = json.loads(
        (root / "data/chunk-000/episode_000000.jsonl").read_text().splitlines()[0]
    )
    assert row["observation.state"] == [*control.arm_q_measured, *control.hand_observation]
    assert row["action"] == [*control.accepted_arm_q, *control.hand_target]
    assert row["observation.force"] == [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert row["timing"]["rh56_force_act_timestamp_ns"] == base - 3_000_000
    assert row["timing"]["rh56_force_act_age_ns"] == 3_000_000
    assert row["timing"]["rh56_force_act_valid"] is True
    assert row["camera"]["workspace"]["rgb_timestamp_domain"] == "hardware_clock"

    set_staging_review(root, 0, status="approved")
    parquet_path = materialize_staging_episode(root, 0, tmp_path / "materialized")
    import pyarrow.parquet as parquet

    table = parquet.read_table(parquet_path)
    assert table.column("observation.state").type.list_size == 12
    assert table.column("action").type.list_size == 12
    assert table.column("observation.force").type.list_size == 6
    assert json.loads(table.column("timing")[0].as_py())["rh56_force_act_valid"] is True


def test_collector_counts_repeated_camera_source_frames(tmp_path: Path) -> None:
    base = 2_000_000_000
    writer = CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="test")
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=0.2,
        maximum_hand_start_delta_rad=0.2,
    )
    workspace = _camera("workspace", base, 1)
    wrist = _camera("wrist", base, 1)
    collector.ingest_camera(workspace)
    collector.ingest_camera(wrist)
    collector.ingest_control(_control(base), reference_established=True)
    collector.ingest_control(
        _control(base + 34_000_000), reference_established=True
    )
    quality = collector.diagnostics()["data_quality"]
    assert quality["workspace_repeated_source_frame_count"] == 1
    assert quality["wrist_repeated_source_frame_count"] == 1
    writer.flush_pending()
    record = json.loads(
        (writer.partial_dir / "canonical/samples.jsonl").read_text().splitlines()[-1]
    )
    assert record["timing"]["repeated_source_frames"] == ["workspace", "wrist"]


def test_delayed_producer_never_selects_future_angle_or_force(tmp_path: Path) -> None:
    root = tmp_path / "staging"
    writer = LeRobotStagingWriter(
        root, episode_index=0, task_name="pick", operator="test", dataset_fps=30
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=2.0,
        maximum_hand_start_delta_rad=2.0,
    )
    base = 4_000_000_000
    initial = _timed_control(
        base, jaka_ns=base, angle_ns=base, force_ns=base, marker=1.0
    )
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(initial, reference_established=True, capture_active=True)
    collector.ingest_camera(_camera("workspace", base + 30_000_000, 2))
    collector.ingest_camera(_camera("wrist", base + 30_000_000, 2))
    # The outer snapshot is before the 33.333 ms deadline, but its concurrently
    # updated ANGLE/FORCE values are after that deadline.
    future = _timed_control(
        base + 33_000_000,
        jaka_ns=base + 40_000_000,
        angle_ns=base + 40_000_000,
        force_ns=base + 40_000_000,
        marker=2.0,
    )
    collector.ingest_control(future, reference_established=True, capture_active=True)
    collector.ingest_control(
        replace(future, host_monotonic_ns=base + 50_000_000),
        reference_established=True,
        capture_active=True,
    )
    writer.flush_pending()

    rows = [
        json.loads(line)
        for line in (
            root / "data/chunk-000/episode_000000.jsonl.partial"
        ).read_text(encoding="utf-8").splitlines()
    ]
    canonical = rows[1]
    assert canonical["observation.state"] == [1.0] * 12
    assert canonical["observation.force"] == [1.0] * 6
    assert canonical["timing"]["source_timestamps_ns"]["jaka_observation"] == base
    assert canonical["timing"]["source_timestamps_ns"]["rh56_angle_act"] == base
    assert canonical["timing"]["source_timestamps_ns"]["rh56_force_act"] == base
    assert all(
        timestamp is None or timestamp <= canonical["timestamp_ns"]
        for timestamp in canonical["timing"]["source_timestamps_ns"].values()
    )


def test_source_published_after_deadline_is_not_backdated_into_slot(
    tmp_path: Path,
) -> None:
    root = tmp_path / "staging"
    writer = LeRobotStagingWriter(
        root, episode_index=0, task_name="pick", operator="test", dataset_fps=30
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=2.0,
        maximum_hand_start_delta_rad=2.0,
    )
    base = 4_500_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(
        _timed_control(
            base, jaka_ns=base, angle_ns=base, force_ns=base, marker=1.0
        ),
        reference_established=True,
        capture_active=True,
    )
    collector.ingest_camera(_camera("workspace", base + 30_000_000, 2))
    collector.ingest_camera(_camera("wrist", base + 30_000_000, 2))
    collector.ingest_control(
        _timed_control(
            base + 50_000_000,
            jaka_ns=base + 30_000_000,
            angle_ns=base + 30_000_000,
            force_ns=base + 30_000_000,
            marker=2.0,
        ),
        reference_established=True,
        capture_active=True,
    )
    writer.flush_pending()
    rows = [
        json.loads(line)
        for line in (
            root / "data/chunk-000/episode_000000.jsonl.partial"
        ).read_text(encoding="utf-8").splitlines()
    ]
    assert rows[1]["observation.state"] == [1.0] * 12
    assert rows[1]["observation.force"] == [1.0] * 6


def test_sixty_hz_snapshots_build_complete_30_hz_causal_multirate_timeline(
    tmp_path: Path,
) -> None:
    root = tmp_path / "staging"
    writer = LeRobotStagingWriter(
        root, episode_index=0, task_name="pick", operator="test", dataset_fps=30
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=2.0,
        maximum_hand_start_delta_rad=2.0,
    )
    base = 5_000_000_000
    camera_index = 0
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    for outer_index in range(61):
        host_ns = base + round(outer_index * 1_000_000_000 / 60)
        due_camera_index = (host_ns - base) // 33_333_333
        if due_camera_index > camera_index:
            camera_index = int(due_camera_index)
            camera_ns = base + camera_index * 33_333_333
            # One absent new frame must reuse the still-valid causal frame and
            # must not erase the canonical slot.
            if camera_index != 10:
                collector.ingest_camera(
                    _camera("workspace", camera_ns, camera_index + 1)
                )
                collector.ingest_camera(
                    _camera("wrist", camera_ns, camera_index + 1)
                )
        force_ns = base + ((host_ns - base) // 100_000_000) * 100_000_000
        angle_ns = base + ((host_ns - base) // 66_666_667) * 66_666_667
        jaka_ns = base + ((host_ns - base) // 32_000_000) * 32_000_000
        marker = float(outer_index)
        collector.ingest_control(
            _timed_control(
                host_ns,
                jaka_ns=jaka_ns,
                angle_ns=angle_ns,
                force_ns=force_ns,
                marker=marker,
            ),
            reference_established=True,
            capture_active=True,
        )
    writer.flush_pending()

    rows = [
        json.loads(line)
        for line in (
            root / "data/chunk-000/episode_000000.jsonl.partial"
        ).read_text(encoding="utf-8").splitlines()
    ]
    assert len(rows) == 31
    assert collector.clock.total_missed_slots == 0
    sources = {
        name: [
            row["timing"]["source_timestamps_ns"][name]
            for row in rows
        ]
        for name in ("jaka_observation", "rh56_angle_act", "rh56_force_act")
    }
    assert len(set(sources["jaka_observation"])) == 30
    assert len(set(sources["rh56_angle_act"])) == 15
    assert len(set(sources["rh56_force_act"])) == 10
    assert all(
        source_ns <= row["timestamp_ns"]
        for row in rows
        for source_ns in row["timing"]["source_timestamps_ns"].values()
        if source_ns is not None
    )
    assert any(row["camera"]["workspace"]["repeated_source_frame"] for row in rows)


def test_control_source_timestamp_domain_mismatch_is_rejected(tmp_path: Path) -> None:
    writer = CanonicalEpisodeWriter(tmp_path, task_name="pick", operator="test")
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=2.0,
        maximum_hand_start_delta_rad=2.0,
    )
    sample = _timed_control(
        6_000_000_000,
        jaka_ns=6_000_000_000,
        angle_ns=6_000_000_000,
        force_ns=6_000_000_000,
        marker=1.0,
    )
    collector.ingest_control(
        replace(
            sample,
            source_timestamp_domains={
                **dict(sample.source_timestamp_domains or {}),
                "rh56_force_act": "device_clock",
            },
        ),
        reference_established=True,
        capture_active=True,
    )
    assert collector.state.value == "DONE"
    assert collector.termination_reason == (
        "control_source_timestamp_domain_mismatch:rh56_force_act"
    )


def test_sync_check_is_causal_and_never_uses_future_force(tmp_path: Path) -> None:
    root = tmp_path / "staging"
    name = "episode_000000"
    (root / "data/chunk-000").mkdir(parents=True)
    (root / "audit/chunk-000" / name).mkdir(parents=True)
    base = 3_000_000_000
    period = 33_333_333
    rows = []
    for index in range(3):
        timestamp = base + index * period
        rows.append(
            {
                "frame_index": index,
                "timestamp_ns": timestamp,
                "observation.state": [0.0] * 12,
                "observation.force": [float(index)] * 6,
                "camera": {
                    role: {
                        "host_monotonic_ns": timestamp,
                        "rgb_device_timestamp_ms": 1000.0 + index,
                        "rgb_timestamp_domain": "hardware_clock",
                        "rgb_frame_number": index + 1,
                    }
                    for role in ("workspace", "wrist")
                },
                "timing": {
                    "source_timestamps_ns": {
                        "jaka_observation": timestamp - 1_000_000,
                        "rh56_angle_act": timestamp - 1_000_000,
                        "rh56_force_act": timestamp - 1_000_000,
                    }
                },
            }
        )
    (root / "data/chunk-000" / f"{name}.jsonl").write_text(
        "".join(json.dumps(row) + "\n" for row in rows)
    )
    (root / "audit/chunk-000" / name / "jaka_state.jsonl").write_text(
        json.dumps({"read_host_monotonic_ns": base - 1_000_000, "measured_joint_position_rad": [1.0] * 6})
        + "\n"
    )
    (root / "audit/chunk-000" / name / "rh56_feedback.jsonl").write_text(
        "".join(
            json.dumps(
                {
                    "hand_feedback_register_timestamps_ns": {
                        "ANGLE_ACT": base - 1_000_000,
                        "FORCE_ACT": timestamp,
                    },
                    "rh56_registers": {
                        "ANGLE_ACT": [2.0] * 6,
                        "FORCE_ACT": [9.0] * 6,
                    },
                }
            )
            + "\n"
            for timestamp in (base - 1_000_000, base + period + 10_000_000)
        )
    )

    report = synchronize_staging_episode(root, 0, camera_tolerance_ms=5.0)
    assert report["future_force_samples_used"] is False
    assert report["camera_clock_maps"]["workspace"]["valid"] is True
    first_force = report["timeline"][1]["rh56_force"]
    assert first_force["timestamp_ns"] == base - 1_000_000
    assert first_force["age_ns"] >= 0
    assert all(
        row["rh56_force"]["timestamp_ns"] is None
        or row["rh56_force"]["timestamp_ns"] <= row["timestamp_ns"]
        for row in report["timeline"]
    )
    assert report["timeline"][1]["validity"]["camera.workspace"] is True
