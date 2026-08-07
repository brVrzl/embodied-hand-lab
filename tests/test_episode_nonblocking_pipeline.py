from __future__ import annotations

import threading
import time
from pathlib import Path
import json

import numpy as np
import pytest

from episode_dataset.async_writer import AsyncEpisodeWriter
from episode_dataset.camera import CameraFrameRing
from episode_dataset.camera import AsyncRGBDCamera
from episode_dataset.collector import CaptureState, SingleEpisodeCollector
from episode_dataset.episode import (
    CameraFrameUnavailable,
    CameraSample,
    CanonicalEpisodeWriter,
    ControlSample,
)
from embodiment_core.types import CameraIntrinsics
from vision_interface.interfaces import RGBDFrame


def _camera(role: str, timestamp_ns: int, number: int, value: int = 0) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=np.full((12, 16, 3), value, dtype=np.uint8),
        depth_raw=np.full((12, 16), value, dtype=np.uint16),
        depth_aligned_to_rgb=np.full((12, 16), value, dtype=np.uint16),
        depth_scale_m=0.001,
        device_rgb_timestamp_ms=float(number),
        device_depth_timestamp_ms=float(number),
        rgb_frame_number=number,
        depth_frame_number=number,
        rgb_timestamp_domain="hardware_clock",
        depth_timestamp_domain="hardware_clock",
    )


def _control(timestamp_ns: int, *, trigger: bool = True) -> ControlSample:
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
        arm_trigger=trigger,
        hand_grip=False,
    )


class _SlowSink:
    root = Path("/tmp/offline-slow-sink")
    temporary_id = "offline"
    dataset_fps = 30
    sample_count = 0
    start_monotonic_ns = None

    def __init__(self, delay_s: float = 0.01, *, fail: bool = False) -> None:
        self.delay_s = delay_s
        self.fail = fail
        self.rows = 0

    def append_raw(self, _stream: str, _record: object) -> None:
        time.sleep(self.delay_s)
        if self.fail:
            raise OSError("synthetic writer failure")
        self.rows += 1

    def append_raw_camera(self, _camera: object) -> None:
        time.sleep(self.delay_s)
        self.rows += 1

    def finalize(
        self,
        _status: object,
        *,
        termination_reason: str,
        trigger_release_monotonic_ns: int | None,
        report: object,
    ) -> Path:
        del termination_reason, trigger_release_monotonic_ns, report
        time.sleep(self.delay_s)
        return self.root


class _FastCamera:
    def __init__(self) -> None:
        self.number = 0
        self.closed = False

    def capture(self) -> RGBDFrame:
        if self.closed:
            raise RuntimeError("closed")
        time.sleep(0.001)
        self.number += 1
        sample = _camera("workspace", time.monotonic_ns(), self.number, self.number % 250)
        return RGBDFrame(
            rgb=sample.rgb,
            depth_m=sample.depth_raw.astype(np.float32) * 0.001,
            intrinsics=CameraIntrinsics(16, 12, 1.0, 1.0, 8.0, 6.0, "camera"),
            host_timestamp_s=time.time(),
            color_timestamp_ms=float(self.number),
            depth_timestamp_ms=float(self.number),
            color_timestamp_domain="hardware_clock",
            depth_timestamp_domain="hardware_clock",
            color_frame_number=self.number,
            depth_frame_number=self.number,
            depth_aligned_to_color=True,
            host_monotonic_ns=sample.host_monotonic_ns,
            depth_raw_units=sample.depth_raw,
            depth_aligned_to_color_units=sample.depth_aligned_to_rgb,
            depth_scale_m=0.001,
        )

    def close(self) -> None:
        self.closed = True


def test_recorder_backpressure_is_nonblocking_and_counted() -> None:
    writer = AsyncEpisodeWriter(
        _SlowSink(0.02), capacity=3, batch_size=2, shutdown_timeout_s=1.0
    )
    durations = []
    for index in range(100):
        started = time.perf_counter_ns()
        writer.append_raw("control", {"index": index})
        durations.append(time.perf_counter_ns() - started)
    diagnostics = writer.diagnostics()
    assert max(durations) < 10_000_000
    assert diagnostics["recorder_queue_full_count"] > 0
    assert diagnostics["recorder_drop_count"] > 0
    assert diagnostics["queue_max_depth"] <= 3
    writer.close()


def test_live_writer_rejects_ndarray_camera_queue_payloads() -> None:
    writer = AsyncEpisodeWriter(
        _SlowSink(0.0), require_frame_references=True, shutdown_timeout_s=0.5
    )
    sample = _camera("wrist", 1, 1)
    with pytest.raises(TypeError, match="ring reference"):
        writer.append_raw_camera(sample)
    reference = CameraFrameRing(2).publish(sample)
    assert writer.append_raw_camera(reference) is True
    writer.close()


def test_ring_overwrite_never_returns_a_half_written_frame() -> None:
    ring = CameraFrameRing(2)
    old = ring.publish(_camera("wrist", 1, 1, 1))
    ring.publish(_camera("wrist", 2, 2, 2))
    newest = ring.publish(_camera("wrist", 3, 3, 3))
    with pytest.raises(CameraFrameUnavailable):
        old.snapshot()
    snapshot = newest.snapshot()
    assert np.all(snapshot.rgb == 3)
    assert np.all(snapshot.depth_raw == 3)
    assert ring.overwrite_count == 1
    assert ring.slot_reuse_count == 1
    assert ring.inconsistent_read_count == 0

    stop = threading.Event()
    failures: list[str] = []
    latest = [newest]

    def produce() -> None:
        for sequence in range(4, 300):
            latest[0] = ring.publish(
                _camera("wrist", sequence, sequence, sequence % 250)
            )
        stop.set()

    producer = threading.Thread(target=produce)
    producer.start()
    while not stop.is_set():
        try:
            frame = latest[0].snapshot()
        except CameraFrameUnavailable:
            continue
        if not (np.all(frame.rgb == frame.rgb[0, 0, 0]) and np.all(frame.depth_raw == frame.depth_raw[0, 0])):
            failures.append("torn frame")
    producer.join(timeout=1.0)
    assert not failures


def test_delayed_ring_references_never_substitute_a_later_sequence() -> None:
    """A two-slot ring may expire references, but it must never relabel them."""

    ring = CameraFrameRing(2)
    references = [
        ring.publish(_camera("wrist", sequence, sequence, sequence % 251))
        for sequence in range(100)
    ]
    expired = 0
    for reference in references:
        try:
            sample = reference.snapshot()
        except CameraFrameUnavailable:
            expired += 1
            continue
        assert sample.ring_sequence == reference.sequence
        assert int(sample.rgb[0, 0, 0]) == reference.sequence % 251
        assert sample.host_monotonic_ns == reference.host_monotonic_ns
    assert expired == 98
    assert ring.overwrite_count == 98
    assert ring.reference_expired_count == expired
    assert ring.slot_reuse_count == 98
    assert ring.inconsistent_read_count == 0


def test_canonical_writer_materializes_ring_refs_before_metadata(tmp_path: Path) -> None:
    """Canonical metadata must use the materialized sample, not the ref object."""

    from episode_dataset.episode import CanonicalEpisodeWriter

    workspace_ring = CameraFrameRing(2)
    wrist_ring = CameraFrameRing(2)
    base = 2_000_000_000
    workspace_ref = workspace_ring.publish(_camera("workspace", base, 1))
    wrist_ref = wrist_ring.publish(_camera("wrist", base, 1))
    collector = SingleEpisodeCollector(
        CanonicalEpisodeWriter(tmp_path, task_name="offline", operator="test"),
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    collector.ingest_camera(workspace_ref)
    collector.ingest_camera(wrist_ref)
    collector.ingest_control(_control(base, trigger=True), reference_established=True)
    collector.ingest_control(
        _control(base + 10_000_000, trigger=False), reference_established=True
    )

    assert collector.result is not None
    rows = (collector.result / "canonical" / "samples.jsonl").read_text().splitlines()
    record = json.loads(rows[0])
    assert record["camera"]["workspace"]["ring_sequence"] == workspace_ref.sequence
    assert record["camera"]["wrist"]["ring_sequence"] == wrist_ref.sequence


def test_latest_frame_mailbox_does_not_accumulate_preview_lag() -> None:
    source = _FastCamera()
    camera = AsyncRGBDCamera(
        "workspace", lambda: source, queue_capacity=4, copy_arrays=False
    )
    camera.start()
    deadline = time.monotonic() + 0.5
    while camera.latest() is None and time.monotonic() < deadline:
        time.sleep(0.001)
    first, _ = camera.latest_after(-1)
    assert first is not None
    time.sleep(0.04)  # GUI-like consumer is much slower than capture.
    newest, skipped = camera.latest_after(first.host_monotonic_ns)
    assert newest is not None
    assert skipped > 0
    assert source.number - newest.rgb_frame_number <= 1
    assert camera.queue_depth <= 4
    camera.stop()


def test_canonical_alignment_is_causal_and_does_not_wait_for_slow_source(tmp_path: Path) -> None:
    from episode_dataset.episode import CanonicalEpisodeWriter

    collector = SingleEpisodeCollector(
        CanonicalEpisodeWriter(tmp_path, task_name="offline", operator="test"),
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 2_000_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base), reference_established=True)
    collector.ingest_camera(_camera("workspace", base + 25_000_000, 2))
    started = time.perf_counter_ns()
    collector.ingest_control(_control(base + 34_000_000), reference_established=True)
    elapsed_ns = time.perf_counter_ns() - started
    assert elapsed_ns < 10_000_000
    collector.writer.flush_pending()
    rows = (collector.writer.partial_dir / "canonical/samples.jsonl").read_text().splitlines()
    import json

    row = json.loads(rows[-1])
    assert row["timing"]["source_timestamps_ns"]["workspace"] == base + 25_000_000
    assert row["timing"]["source_timestamps_ns"]["wrist"] == base
    assert row["timing"]["signed_offsets_ns"]["workspace"] == -8_333_333
    assert row["timing"]["signed_offsets_ns"]["wrist"] == -33_333_333


def test_canonical_timing_fields_are_not_aliased(tmp_path: Path) -> None:
    writer = CanonicalEpisodeWriter(tmp_path, task_name="offline", operator="test")
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=100_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )

    data_quality = collector.diagnostics()["data_quality"]
    assert "canonical_compute_duration_ns" in data_quality
    assert "canonical_metadata_duration_ns" not in data_quality
    assert writer.timing_diagnostics()["canonical_metadata_duration_ns"]["count"] == 0


def test_wrist_stale_is_quality_fault_then_persistent_recording_fault(tmp_path: Path) -> None:
    from episode_dataset.episode import CanonicalEpisodeWriter

    collector = SingleEpisodeCollector(
        CanonicalEpisodeWriter(tmp_path, task_name="offline", operator="test"),
        camera_max_age_ns=20_000_000,
        camera_severe_stale_ns=60_000_000,
        camera_consecutive_stale_limit=3,
        camera_missing_timeout_ns=200_000_000,
        control_max_age_ns=40_000_000,
        maximum_start_delta_rad=0.02,
        maximum_hand_start_delta_rad=0.02,
    )
    base = 1_000_000_000
    collector.ingest_camera(_camera("workspace", base, 1))
    collector.ingest_camera(_camera("wrist", base, 1))
    collector.ingest_control(_control(base), reference_established=True)
    collector.ingest_control(_control(base + 34_000_000), reference_established=True)
    assert collector.state is CaptureState.REC
    assert collector.diagnostics()["data_quality"]["wrist_stale_count"] == 1
    collector.ingest_control(_control(base + 67_000_000), reference_established=True)
    assert collector.state is CaptureState.REC
    collector.ingest_control(_control(base + 100_000_000), reference_established=True)
    assert collector.state is CaptureState.DONE
    assert collector.termination_reason == "persistent_camera_acquisition_fault:workspace,wrist"
    assert collector.completion_status.value == "aborted"


def test_writer_failure_is_recording_only_and_shutdown_is_bounded() -> None:
    writer = AsyncEpisodeWriter(
        _SlowSink(0.001, fail=True), capacity=2, shutdown_timeout_s=0.5
    )
    writer.append_raw("quality", {"sample": 1})
    deadline = time.monotonic() + 0.2
    while writer.diagnostics()["writer_error_count"] == 0 and time.monotonic() < deadline:
        time.sleep(0.001)
    with pytest.raises(OSError, match="worker failed"):
        writer.append_raw("quality", {"sample": 2})
    started = time.monotonic()
    writer.close()
    assert time.monotonic() - started < 0.5


def test_writer_finalization_timeout_is_bounded_and_thread_exits() -> None:
    from episode_dataset.episode import EpisodeStatus

    writer = AsyncEpisodeWriter(
        _SlowSink(0.08), capacity=2, shutdown_timeout_s=0.02
    )
    started = time.monotonic()
    with pytest.raises(TimeoutError, match="bounded shutdown"):
        writer.finalize(
            EpisodeStatus.ABORTED,
            termination_reason="recorder_stop",
            trigger_release_monotonic_ns=None,
        )
    assert time.monotonic() - started < 0.1
    time.sleep(0.1)
    assert writer.diagnostics()["thread_alive"] is False


def test_teleop_event_log_publication_is_bounded_and_nonblocking(tmp_path: Path) -> None:
    from tools.quest_jaka_hardware import _AsyncEventLog

    path = tmp_path / "events.jsonl"
    started = time.perf_counter_ns()
    with _AsyncEventLog(path, capacity=2) as log:
        for index in range(500):
            log.write({"record_type": "event", "index": index})
    elapsed_ns = time.perf_counter_ns() - started
    assert elapsed_ns < 500_000_000
    assert path.is_file()
    assert log.drop_count >= 0
    assert log.error_count == 0
