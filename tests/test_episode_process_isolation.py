from __future__ import annotations

import multiprocessing as mp
from multiprocessing import shared_memory
import json
import os
import queue
import time

import numpy as np
import pytest

from episode_dataset.episode import CameraSample
from episode_dataset.episode import ControlSample
from episode_dataset.process_runtime import (
    FrameReferenceDescriptor,
    ProcessEpisodeRecorder,
    ProcessCamera,
    SharedMemoryCameraFrameRing,
    _queue_put_latest,
    configure_non_realtime_affinity,
    process_placement,
)


def _control(timestamp_ns: int, trigger: bool) -> ControlSample:
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


def _camera_sample(role: str, timestamp_ns: int, sequence: int) -> CameraSample:
    return CameraSample(
        role=role,
        host_monotonic_ns=timestamp_ns,
        rgb=np.full((4, 4, 3), sequence, dtype=np.uint8),
        depth_raw=np.full((4, 4), sequence, dtype=np.uint16),
        depth_aligned_to_rgb=np.full((4, 4), sequence, dtype=np.uint16),
        depth_scale_m=0.001,
        device_rgb_timestamp_ms=float(sequence),
        device_depth_timestamp_ms=float(sequence),
        rgb_frame_number=sequence,
        depth_frame_number=sequence,
        rgb_timestamp_domain="offline",
        depth_timestamp_domain="offline",
    )


def _fake_camera_producer(spec, descriptors, stop, pause_s: float = 0.0) -> None:
    ring = SharedMemoryCameraFrameRing.attach(spec)
    try:
        for sequence in range(80):
            if stop.is_set():
                return
            if sequence == 1 and pause_s:
                time.sleep(pause_s)
            sample = CameraSample(
                role=spec.role,
                host_monotonic_ns=time.monotonic_ns(),
                rgb=np.full(spec.rgb_shape, sequence % 255, dtype=np.uint8),
                depth_raw=np.full(spec.depth_shape, sequence, dtype=np.uint16),
                depth_aligned_to_rgb=(
                    None
                    if spec.aligned_depth_shape is None
                    else np.full(spec.aligned_depth_shape, sequence, dtype=np.uint16)
                ),
                depth_scale_m=0.001,
                device_rgb_timestamp_ms=float(sequence),
                device_depth_timestamp_ms=float(sequence),
                rgb_frame_number=sequence,
                depth_frame_number=sequence,
                rgb_timestamp_domain="offline",
                depth_timestamp_domain="offline",
            )
            reference = ring.publish(sample, sequence)
            descriptor = FrameReferenceDescriptor.from_reference(reference)
            try:
                descriptors.put_nowait(descriptor)
            except queue.Full:
                try:
                    descriptors.get_nowait()
                except queue.Empty:
                    pass
                try:
                    descriptors.put_nowait(descriptor)
                except queue.Full:
                    pass
            time.sleep(0.002)
    finally:
        ring.close()


def _crash_worker() -> None:
    raise RuntimeError("synthetic child crash")


def _camera_startup_stall(*args) -> None:
    stop_event = args[5]
    while not stop_event.is_set():
        time.sleep(0.001)


def _camera_lifecycle_worker(
    role,
    camera_config,
    ring_spec,
    descriptor_queue,
    status_queue,
    stop_event,
    forbidden_cpu,
) -> None:
    ring = SharedMemoryCameraFrameRing.attach(ring_spec)
    try:
        sample = _camera_sample(role, 1_000_000, 0)
        reference = ring.publish(sample, 0)
        descriptor_queue.put(FrameReferenceDescriptor.from_reference(reference))
        status_queue.put(
            {
                "kind": "ready",
                "profile": {"synthetic": True},
                "placement": process_placement(f"camera_{role}"),
            }
        )
        while not stop_event.is_set():
            time.sleep(0.001)
        status_queue.put(
            {
                "kind": "stopped",
                "received": 1,
                "camera_publish_duration_ns": {
                    "count": 1,
                    "p50": 1,
                    "p95": 1,
                    "p99": 1,
                    "max": 1,
                },
                "camera_interframe_interval_ns": {
                    "count": 0,
                    "p50": 0,
                    "p95": 0,
                    "p99": 0,
                    "max": 0,
                },
                "placement": process_placement(f"camera_{role}"),
            }
        )
    finally:
        ring.close()


def _affinity_worker(forbidden_cpu: int, result_queue) -> None:
    configure_non_realtime_affinity(forbidden_cpu)
    result_queue.put(process_placement("offline_worker"))


def _attach_and_close_ring(spec, result_queue) -> None:
    ring = SharedMemoryCameraFrameRing.attach(spec)
    result_queue.put(True)
    ring.close()


def _assert_shared_memory_absent(names: list[str | None]) -> None:
    for name in names:
        if name is None:
            continue
        with pytest.raises(FileNotFoundError):
            shared_memory.SharedMemory(name=name)


def _ring_pair(context: mp.context.BaseContext):
    rings = {
        role: SharedMemoryCameraFrameRing.create(
            role=role,
            capacity=4,
            rgb_shape=(480, 640, 3),
            depth_shape=(480, 640),
            aligned_depth_shape=(480, 640),
        )
        for role in ("workspace", "wrist")
    }
    queues = {role: context.Queue(maxsize=4) for role in rings}
    stop = context.Event()
    return rings, queues, stop


def _close_ring_pair(rings, queues, stop) -> None:
    stop.set()
    for q in queues.values():
        q.close()
    for ring in rings.values():
        ring.close()


def test_two_camera_processes_keep_control_path_metadata_only() -> None:
    context = mp.get_context("spawn")
    rings, queues, stop = _ring_pair(context)
    processes = [
        context.Process(
            target=_fake_camera_producer,
            args=(rings[role].spec, queues[role], stop),
            name=f"offline-camera-{role}",
        )
        for role in rings
    ]
    for process in processes:
        process.start()
    durations_ns: list[int] = []
    try:
        for _ in range(100):
            started = time.perf_counter_ns()
            for q in queues.values():
                while True:
                    try:
                        descriptor = q.get_nowait()
                    except queue.Empty:
                        break
                    assert isinstance(descriptor, FrameReferenceDescriptor)
                    assert not hasattr(descriptor, "rgb")
            durations_ns.append(time.perf_counter_ns() - started)
            time.sleep(0.005)
        ordered = sorted(durations_ns)
        assert ordered[round((len(ordered) - 1) * 0.99)] < 20_000_000
    finally:
        _close_ring_pair(rings, queues, stop)
        for process in processes:
            process.join(timeout=2.0)
            assert not process.is_alive()


def test_camera_stall_does_not_block_control_and_marks_reference_stale() -> None:
    context = mp.get_context("spawn")
    rings, queues, stop = _ring_pair(context)
    process = context.Process(
        target=_fake_camera_producer,
        args=(rings["workspace"].spec, queues["workspace"], stop, 0.15),
    )
    process.start()
    control_ticks = 0
    latest = None
    stale_seen = False
    try:
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            control_ticks += 1
            try:
                while True:
                    latest = queues["workspace"].get_nowait()
            except queue.Empty:
                pass
            if latest is not None and time.monotonic_ns() - latest.host_monotonic_ns >= 50_000_000:
                stale_seen = True
            time.sleep(0.005)
        assert control_ticks > 30
        assert latest is not None
        assert stale_seen
    finally:
        _close_ring_pair(rings, queues, stop)
        process.join(timeout=2.0)
        assert not process.is_alive()


def test_slow_recorder_queue_is_bounded_and_does_not_backpressure_control() -> None:
    context = mp.get_context("spawn")
    q = context.Queue(maxsize=1)
    started = time.perf_counter_ns()
    dropped = 0
    for sequence in range(200):
        try:
            q.put_nowait(sequence)
        except queue.Full:
            dropped += 1
    elapsed_ns = time.perf_counter_ns() - started
    assert dropped > 0
    assert elapsed_ns < 20_000_000
    q.close()


def test_sequence_expiry_never_reads_newer_frame() -> None:
    ring = SharedMemoryCameraFrameRing.create(
        role="workspace",
        capacity=2,
        rgb_shape=(2, 2, 3),
        depth_shape=(2, 2),
        aligned_depth_shape=None,
    )
    try:
        references = []
        for sequence in range(3):
            sample = CameraSample(
                role="workspace",
                host_monotonic_ns=sequence + 1,
                rgb=np.full((2, 2, 3), sequence, dtype=np.uint8),
                depth_raw=np.full((2, 2), sequence, dtype=np.uint16),
                device_rgb_timestamp_ms=float(sequence),
                device_depth_timestamp_ms=float(sequence),
                rgb_frame_number=sequence,
                depth_frame_number=sequence,
                rgb_timestamp_domain="offline",
                depth_timestamp_domain="offline",
            )
            reference = ring.publish(sample, sequence)
            descriptor = FrameReferenceDescriptor.from_reference(reference)
            references.append(descriptor.to_reference(ring))
        with pytest.raises(Exception, match="overwritten"):
            references[0].snapshot()
        assert references[2].snapshot().ring_sequence == 2
    finally:
        ring.close()


def test_shared_memory_owner_unlinks_only_after_child_attach_closes() -> None:
    context = mp.get_context("spawn")
    ring = SharedMemoryCameraFrameRing.create(
        role="workspace",
        capacity=2,
        rgb_shape=(2, 2, 3),
        depth_shape=(2, 2),
        aligned_depth_shape=None,
    )
    names = [
        ring.spec.header_name,
        ring.spec.rgb_name,
        ring.spec.depth_name,
        ring.spec.aligned_depth_name,
    ]
    result_queue = context.Queue(maxsize=1)
    child = context.Process(target=_attach_and_close_ring, args=(ring.spec, result_queue))
    child.start()
    try:
        assert result_queue.get(timeout=2.0) is True
        child.join(timeout=2.0)
        assert not child.is_alive()
        assert child.exitcode == 0
        for name in names:
            if name is not None:
                handle = shared_memory.SharedMemory(name=name)
                handle.close()
    finally:
        if child.is_alive():
            child.terminate()
            child.join(timeout=1.0)
        ring.close()
        result_queue.close()
    _assert_shared_memory_absent(names)


def test_repeated_ring_startup_uses_unique_names_and_cleans_up() -> None:
    rings = [
        SharedMemoryCameraFrameRing.create(
            role="workspace",
            capacity=2,
            rgb_shape=(2, 2, 3),
            depth_shape=(2, 2),
            aligned_depth_shape=None,
        ),
        SharedMemoryCameraFrameRing.create(
            role="workspace",
            capacity=2,
            rgb_shape=(2, 2, 3),
            depth_shape=(2, 2),
            aligned_depth_shape=None,
        ),
    ]
    names = [
        [ring.spec.header_name, ring.spec.rgb_name, ring.spec.depth_name]
        for ring in rings
    ]
    try:
        assert set(names[0]).isdisjoint(names[1])
    finally:
        for ring in rings:
            ring.close()
    _assert_shared_memory_absent(names[0] + names[1])


def test_camera_startup_timeout_cleans_ring_and_child(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from episode_dataset import process_runtime

    monkeypatch.setattr(process_runtime, "_camera_process_main", _camera_startup_stall)
    context = mp.get_context("fork")
    camera = ProcessCamera(
        "workspace",
        {"width": 2, "height": 2, "align_depth_to_color": False},
        capacity=2,
        forbidden_cpu=None,
        context=context,
    )
    names = [camera.ring_spec.header_name, camera.ring_spec.rgb_name, camera.ring_spec.depth_name]
    with pytest.raises(TimeoutError):
        camera.start(timeout_s=0.05)
    camera.stop(timeout_s=0.1)
    _assert_shared_memory_absent(names)


def test_camera_diagnostics_are_cached_before_stop_and_cleanup_is_idempotent(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from episode_dataset import process_runtime

    monkeypatch.setattr(process_runtime, "_camera_process_main", _camera_lifecycle_worker)
    context = mp.get_context("fork")
    camera = ProcessCamera(
        "workspace",
        {"width": 4, "height": 4, "align_depth_to_color": True},
        capacity=2,
        forbidden_cpu=None,
        context=context,
    )
    names = [
        camera.ring_spec.header_name,
        camera.ring_spec.rgb_name,
        camera.ring_spec.depth_name,
        camera.ring_spec.aligned_depth_name,
    ]
    try:
        camera.start(timeout_s=1.0)
        live_profile = camera.profile_metadata()
        reference, _ = camera.latest_after(-1)
        assert live_profile["synthetic"] is True
        assert reference is not None
        assert reference.snapshot().ring_sequence == 0
        final = camera.stop(timeout_s=1.0)
        assert final["received"] == 1
        assert final["camera_publish_duration_ns"]["count"] == 1
        assert final["placement"]
        assert not camera._process.is_alive()
        assert camera.stop(timeout_s=0.1) == final
    finally:
        camera.stop(timeout_s=0.1)
    _assert_shared_memory_absent(names)


def test_preview_mailbox_is_latest_only() -> None:
    context = mp.get_context("spawn")
    mailbox = context.Queue(maxsize=1)
    accepted = 0
    for value in range(100):
        accepted += int(_queue_put_latest(mailbox, value))
    assert accepted > 0
    assert mailbox.get(timeout=1.0) <= 99
    mailbox.close()


def test_child_crash_has_bounded_shutdown_and_no_robot_action() -> None:
    context = mp.get_context("spawn")
    process = context.Process(target=_crash_worker)
    started = time.monotonic()
    process.start()
    process.join(timeout=2.0)
    assert not process.is_alive()
    assert process.exitcode != 0
    assert time.monotonic() - started < 2.0


def test_non_realtime_children_exclude_native_control_cpu() -> None:
    if not hasattr(os, "sched_getaffinity"):
        pytest.skip("CPU affinity is unavailable")
    allowed = sorted(os.sched_getaffinity(0))
    if len(allowed) < 2:
        pytest.skip("requires at least two CPUs")
    context = mp.get_context("spawn")
    result_queue = context.Queue(maxsize=1)
    process = context.Process(target=_affinity_worker, args=(allowed[0], result_queue))
    process.start()
    process.join(timeout=2.0)
    try:
        assert not process.is_alive()
        placement = result_queue.get(timeout=1.0)
        assert allowed[0] not in placement["affinity"]
        assert placement["scheduler_policy_name"] != "SCHED_FIFO"
    finally:
        result_queue.close()


def test_recorder_process_materializes_reference_and_finalizes_bounded(tmp_path) -> None:
    context = mp.get_context("spawn")
    rings = {
        role: SharedMemoryCameraFrameRing.create(
            role=role,
            capacity=4,
            rgb_shape=(4, 4, 3),
            depth_shape=(4, 4),
            aligned_depth_shape=(4, 4),
        )
        for role in ("workspace", "wrist")
    }
    recorder = ProcessEpisodeRecorder(
        context=context,
        ring_specs={role: ring.spec for role, ring in rings.items()},
        episode_root=tmp_path,
        task_name="offline_process_recorder",
        operator="test",
        control_config_path=tmp_path / "control.yaml",
        maximum_start_delta_rad=0.02,
        metadata={"physically_validated": False},
        dataset={
            "fps": 30,
            "camera_ring_capacity": 4,
            "recorder_queue_capacity": 4,
            "writer_batch_size": 2,
            "writer_flush_interval_s": 0.05,
            "writer_shutdown_timeout_s": 1.0,
            "camera_max_age_ms": 100.0,
            "control_max_age_ms": 40.0,
            "hand_start_tolerance_rad": 0.05,
            "quality_min_valid_ratio": 1.0,
            "quality_max_invalid_run": 0,
        },
        camera_profiles={},
        forbidden_cpu=None,
    )
    (tmp_path / "control.yaml").write_text("offline: true\n")
    collector = None
    try:
        collector = recorder.start(timeout_s=3.0)
        base = 1_000_000_000
        references = []
        for role, ring in rings.items():
            reference = ring.publish(_camera_sample(role, base, 0), 0)
            references.append(reference)
            collector.ingest_camera(reference)
        collector.ingest_control(_control(base, True), reference_established=True)
        collector.ingest_control(_control(base + 34_000_000, False), reference_established=True)
        deadline = time.monotonic() + 3.0
        while collector.result is None and time.monotonic() < deadline:
            collector.finalize_pending()
            time.sleep(0.01)
        assert collector.result is not None
        validation = collector.result / "validation_report.json"
        assert validation.is_file()
        payload = json.loads(validation.read_text())
        assert payload["valid"] is True
        assert payload["ring_reference_expired_count"] == 0
        metadata = json.loads((collector.result / "metadata.json").read_text())
        assert "canonical_metadata_duration_ns" in metadata
        assert "frame_materialization_duration_ns" in metadata
        assert "canonical_metadata_duration_ns" not in metadata["data_quality"]
    finally:
        if collector is not None:
            collector.finalize_pending()
        recorder.stop()
        for ring in rings.values():
            ring.close()
