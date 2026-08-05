"""Process-isolated RGB-D capture and episode recording primitives.

The live control process owns only immutable camera references.  Camera
processes publish into a versioned shared-memory ring; recorder and preview
processes open the same ring and are the only consumers allowed to copy image
arrays.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import multiprocessing as mp
from multiprocessing import shared_memory
import os
from pathlib import Path
import queue
import struct
import time
from typing import Any, Mapping

import numpy as np

from vision_interface.realsense_adapter import RealSenseCamera

from .async_writer import AsyncEpisodeWriter
from .camera import camera_sample_from_rgbd
from .collector import CaptureState, SingleEpisodeCollector
from .episode import (
    CameraFrameRef,
    CameraFrameUnavailable,
    CameraSample,
    CanonicalEpisodeWriter,
    ControlSample,
    EpisodeStatus,
    SCHEMA_VERSION,
    StartPrerequisites,
)
from .preview import DualCameraPreview, PreviewStatus, require_preview_dependencies


_HEADER_STRUCT = struct.Struct("<Qq")


def _current_cpu() -> int | None:
    getter = getattr(os, "sched_getcpu", None)
    if getter is not None:
        try:
            return int(getter())
        except OSError:
            return None
    return None


def process_placement(role: str) -> dict[str, object]:
    """Return lightweight process placement evidence."""

    affinity = (
        sorted(os.sched_getaffinity(0))
        if hasattr(os, "sched_getaffinity")
        else None
    )
    policy = None
    priority = None
    if hasattr(os, "sched_getscheduler"):
        try:
            policy = int(os.sched_getscheduler(0))
            priority = int(os.sched_getparam(0).sched_priority)
        except OSError:
            pass
    return {
        "role": role,
        "pid": os.getpid(),
        "affinity": affinity,
        "current_cpu": _current_cpu(),
        "scheduler_policy": policy,
        "scheduler_policy_name": {
            getattr(os, "SCHED_OTHER", -1): "SCHED_OTHER",
            getattr(os, "SCHED_FIFO", -2): "SCHED_FIFO",
            getattr(os, "SCHED_RR", -3): "SCHED_RR",
        }.get(policy, None),
        "scheduler_priority": priority,
    }


def configure_non_realtime_affinity(forbidden_cpu: int | None) -> None:
    """Keep a child off the native control CPU without using SCHED_FIFO."""

    if forbidden_cpu is None or not hasattr(os, "sched_getaffinity"):
        return
    allowed = set(os.sched_getaffinity(0))
    target = allowed - {int(forbidden_cpu)}
    if not target:
        raise RuntimeError(
            f"cannot isolate process from native control CPU {forbidden_cpu}: no CPUs remain"
        )
    os.sched_setaffinity(0, target)


@dataclass(frozen=True, slots=True)
class SharedCameraRingSpec:
    role: str
    capacity: int
    rgb_shape: tuple[int, ...]
    depth_shape: tuple[int, ...]
    aligned_depth_shape: tuple[int, ...] | None
    rgb_dtype: str
    depth_dtype: str
    header_name: str
    rgb_name: str
    depth_name: str
    aligned_depth_name: str | None


@dataclass(frozen=True, slots=True)
class FrameReferenceDescriptor:
    """Pickleable metadata-only equivalent of CameraFrameRef."""

    role: str
    host_monotonic_ns: int
    sequence: int
    device_rgb_timestamp_ms: float
    device_depth_timestamp_ms: float
    rgb_frame_number: int
    depth_frame_number: int
    rgb_timestamp_domain: str
    depth_timestamp_domain: str
    depth_scale_m: float | None
    rgb_shape: tuple[int, ...]
    depth_shape: tuple[int, ...]
    rgb_dtype: str
    depth_dtype: str

    @classmethod
    def from_reference(cls, reference: CameraFrameRef) -> "FrameReferenceDescriptor":
        return cls(
            role=reference.role,
            host_monotonic_ns=reference.host_monotonic_ns,
            sequence=reference.sequence,
            device_rgb_timestamp_ms=reference.device_rgb_timestamp_ms,
            device_depth_timestamp_ms=reference.device_depth_timestamp_ms,
            rgb_frame_number=reference.rgb_frame_number,
            depth_frame_number=reference.depth_frame_number,
            rgb_timestamp_domain=reference.rgb_timestamp_domain,
            depth_timestamp_domain=reference.depth_timestamp_domain,
            depth_scale_m=reference.depth_scale_m,
            rgb_shape=reference.rgb_shape,
            depth_shape=reference.depth_shape,
            rgb_dtype=reference.rgb_dtype,
            depth_dtype=reference.depth_dtype,
        )

    def to_reference(self, ring: "SharedMemoryCameraFrameRing") -> CameraFrameRef:
        ring.register_descriptor(self)
        return CameraFrameRef(
            role=self.role,
            host_monotonic_ns=self.host_monotonic_ns,
            sequence=self.sequence,
            device_rgb_timestamp_ms=self.device_rgb_timestamp_ms,
            device_depth_timestamp_ms=self.device_depth_timestamp_ms,
            rgb_frame_number=self.rgb_frame_number,
            depth_frame_number=self.depth_frame_number,
            rgb_timestamp_domain=self.rgb_timestamp_domain,
            depth_timestamp_domain=self.depth_timestamp_domain,
            depth_scale_m=self.depth_scale_m,
            rgb_shape=self.rgb_shape,
            depth_shape=self.depth_shape,
            rgb_dtype=self.rgb_dtype,
            depth_dtype=self.depth_dtype,
            _reader=ring.read,
        )


class SharedMemoryCameraFrameRing:
    """Cross-process versioned ring using the existing CameraFrameRef contract."""

    def __init__(
        self,
        spec: SharedCameraRingSpec,
        *,
        owner: bool,
        handles: tuple[shared_memory.SharedMemory, ...] | None = None,
    ) -> None:
        self.spec = spec
        self.capacity = spec.capacity
        self._owner = owner
        if handles is None:
            self._header = shared_memory.SharedMemory(name=spec.header_name, create=False)
            self._rgb = shared_memory.SharedMemory(name=spec.rgb_name, create=False)
            self._depth = shared_memory.SharedMemory(name=spec.depth_name, create=False)
            self._aligned = (
                None
                if spec.aligned_depth_name is None
                else shared_memory.SharedMemory(name=spec.aligned_depth_name, create=False)
            )
        else:
            if len(handles) != 4:
                raise ValueError("shared camera ring requires four shared-memory handles")
            self._header, self._rgb, self._depth, self._aligned = handles
        self._rgb_bytes = int(np.prod(spec.rgb_shape)) * np.dtype(spec.rgb_dtype).itemsize
        self._depth_bytes = int(np.prod(spec.depth_shape)) * np.dtype(spec.depth_dtype).itemsize
        self._aligned_bytes = (
            None
            if spec.aligned_depth_shape is None
            else int(np.prod(spec.aligned_depth_shape)) * np.dtype(spec.depth_dtype).itemsize
        )
        self._slot_header_bytes = _HEADER_STRUCT.size
        self._slot_reuse_count = 0
        self._expired_count = 0
        self._inconsistent_count = 0
        self._metadata: dict[int, FrameReferenceDescriptor] = {}
        self._closed = False

    @classmethod
    def create(
        cls,
        *,
        role: str,
        capacity: int,
        rgb_shape: tuple[int, ...],
        depth_shape: tuple[int, ...],
        aligned_depth_shape: tuple[int, ...] | None,
    ) -> "SharedMemoryCameraFrameRing":
        if capacity <= 1:
            raise ValueError("camera ring capacity must be greater than one")
        if role not in {"workspace", "wrist"}:
            raise ValueError("camera role must be workspace or wrist")
        rgb_bytes = int(np.prod(rgb_shape)) * np.dtype(np.uint8).itemsize
        depth_bytes = int(np.prod(depth_shape)) * np.dtype(np.uint16).itemsize
        aligned_bytes = (
            0
            if aligned_depth_shape is None
            else int(np.prod(aligned_depth_shape)) * np.dtype(np.uint16).itemsize
        )
        blocks: list[shared_memory.SharedMemory] = []
        try:
            blocks = [
                shared_memory.SharedMemory(create=True, size=capacity * _HEADER_STRUCT.size),
                shared_memory.SharedMemory(create=True, size=capacity * rgb_bytes),
                shared_memory.SharedMemory(create=True, size=capacity * depth_bytes),
            ]
            if aligned_bytes:
                blocks.append(shared_memory.SharedMemory(create=True, size=capacity * aligned_bytes))
            spec = SharedCameraRingSpec(
                role=role,
                capacity=capacity,
                rgb_shape=rgb_shape,
                depth_shape=depth_shape,
                aligned_depth_shape=aligned_depth_shape,
                rgb_dtype=np.dtype(np.uint8).str,
                depth_dtype=np.dtype(np.uint16).str,
                header_name=blocks[0].name,
                rgb_name=blocks[1].name,
                depth_name=blocks[2].name,
                aligned_depth_name=None if aligned_bytes == 0 else blocks[3].name,
            )
            ring = cls(
                spec,
                owner=True,
                handles=(blocks[0], blocks[1], blocks[2], None if aligned_bytes == 0 else blocks[3]),
            )
            for index in range(capacity):
                _HEADER_STRUCT.pack_into(ring._header.buf, index * _HEADER_STRUCT.size, 0, -1)
            return ring
        except BaseException:
            for block in blocks:
                try:
                    block.close()
                    block.unlink()
                except FileNotFoundError:
                    pass
            raise

    @classmethod
    def attach(cls, spec: SharedCameraRingSpec) -> "SharedMemoryCameraFrameRing":
        return cls(spec, owner=False)

    def _array(
        self,
        block: shared_memory.SharedMemory,
        shape: tuple[int, ...],
        dtype: str,
        sequence: int,
        bytes_per_slot: int,
    ) -> np.ndarray:
        return np.ndarray(
            shape,
            dtype=np.dtype(dtype),
            buffer=block.buf,
            offset=(sequence % self.capacity) * bytes_per_slot,
        )

    def publish(self, sample: CameraSample, sequence: int) -> CameraFrameRef:
        if sample.role != self.spec.role:
            raise ValueError("camera sample role does not match shared ring")
        if sample.rgb.shape != self.spec.rgb_shape or sample.depth_raw.shape != self.spec.depth_shape:
            raise RuntimeError("camera profile changed after shared ring allocation")
        if (sample.depth_aligned_to_rgb is None) != (self.spec.aligned_depth_shape is None):
            raise RuntimeError("camera alignment mode changed after shared ring allocation")
        slot = sequence % self.capacity
        previous_sequence = _HEADER_STRUCT.unpack_from(
            self._header.buf, slot * _HEADER_STRUCT.size
        )[1]
        if previous_sequence >= 0:
            self._slot_reuse_count += 1
        version_in_progress = sequence * 2 + 1
        version_stable = version_in_progress + 1
        _HEADER_STRUCT.pack_into(
            self._header.buf, slot * _HEADER_STRUCT.size, version_in_progress, sequence
        )
        np.copyto(
            self._array(self._rgb, self.spec.rgb_shape, self.spec.rgb_dtype, sequence, self._rgb_bytes),
            sample.rgb,
        )
        np.copyto(
            self._array(self._depth, self.spec.depth_shape, self.spec.depth_dtype, sequence, self._depth_bytes),
            sample.depth_raw,
        )
        if self._aligned is not None and sample.depth_aligned_to_rgb is not None:
            assert self._aligned_bytes is not None
            np.copyto(
                self._array(
                    self._aligned,
                    self.spec.aligned_depth_shape or (),
                    self.spec.depth_dtype,
                    sequence,
                    self._aligned_bytes,
                ),
                sample.depth_aligned_to_rgb,
            )
        _HEADER_STRUCT.pack_into(
            self._header.buf, slot * _HEADER_STRUCT.size, version_stable, sequence
        )
        return CameraFrameRef(
            role=sample.role,
            host_monotonic_ns=sample.host_monotonic_ns,
            sequence=sequence,
            device_rgb_timestamp_ms=sample.device_rgb_timestamp_ms,
            device_depth_timestamp_ms=sample.device_depth_timestamp_ms,
            rgb_frame_number=sample.rgb_frame_number,
            depth_frame_number=sample.depth_frame_number,
            rgb_timestamp_domain=sample.rgb_timestamp_domain,
            depth_timestamp_domain=sample.depth_timestamp_domain,
            depth_scale_m=sample.depth_scale_m,
            rgb_shape=tuple(sample.rgb.shape),
            depth_shape=tuple(sample.depth_raw.shape),
            rgb_dtype=sample.rgb.dtype.str,
            depth_dtype=sample.depth_raw.dtype.str,
            _reader=self.read,
        )

    def read(self, sequence: int) -> CameraSample | None:
        slot = sequence % self.capacity
        version, published_sequence = _HEADER_STRUCT.unpack_from(
            self._header.buf, slot * _HEADER_STRUCT.size
        )
        expected_version = sequence * 2 + 2
        if version != expected_version or version & 1 or published_sequence != sequence:
            self._expired_count += 1
            return None
        metadata = self._metadata.get(sequence)
        if metadata is None:
            self._expired_count += 1
            return None
        rgb = self._array(self._rgb, self.spec.rgb_shape, self.spec.rgb_dtype, sequence, self._rgb_bytes).copy()
        depth = self._array(self._depth, self.spec.depth_shape, self.spec.depth_dtype, sequence, self._depth_bytes).copy()
        aligned = (
            None
            if self._aligned is None
            else self._array(
                self._aligned,
                self.spec.aligned_depth_shape or (),
                self.spec.depth_dtype,
                sequence,
                self._aligned_bytes or 0,
            ).copy()
        )
        version_after, published_after = _HEADER_STRUCT.unpack_from(
            self._header.buf, slot * _HEADER_STRUCT.size
        )
        if version_after != version or published_after != sequence:
            self._inconsistent_count += 1
            return None
        return CameraSample(
            role=self.spec.role,
            host_monotonic_ns=metadata.host_monotonic_ns,
            rgb=rgb,
            depth_raw=depth,
            depth_aligned_to_rgb=aligned,
            depth_scale_m=metadata.depth_scale_m,
            device_rgb_timestamp_ms=metadata.device_rgb_timestamp_ms,
            device_depth_timestamp_ms=metadata.device_depth_timestamp_ms,
            rgb_frame_number=metadata.rgb_frame_number,
            depth_frame_number=metadata.depth_frame_number,
            rgb_timestamp_domain=metadata.rgb_timestamp_domain,
            depth_timestamp_domain=metadata.depth_timestamp_domain,
            ring_sequence=sequence,
        )

    def register_descriptor(self, descriptor: FrameReferenceDescriptor) -> None:
        if descriptor.role != self.spec.role:
            raise ValueError("frame descriptor role does not match shared ring")
        self._metadata[descriptor.sequence] = descriptor
        expired_before = descriptor.sequence - self.capacity
        if expired_before >= 0:
            self._metadata.pop(expired_before, None)
        while len(self._metadata) > self.capacity * 2:
            self._metadata.pop(min(self._metadata), None)

    def metrics(self) -> dict[str, int]:
        return {
            "camera_ring_slot_reuse_count": self._slot_reuse_count,
            "camera_ring_reference_expired_count": self._expired_count,
            "camera_inconsistent_read_count": self._inconsistent_count,
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for block in (self._header, self._rgb, self._depth, self._aligned):
            if block is not None:
                block.close()
        if self._owner:
            for name in (
                self.spec.header_name,
                self.spec.rgb_name,
                self.spec.depth_name,
                self.spec.aligned_depth_name,
            ):
                if name is None:
                    continue
                try:
                    shared_memory.SharedMemory(name=name).unlink()
                except FileNotFoundError:
                    pass


def _queue_put_latest(q: Any, item: object) -> bool:
    try:
        q.put_nowait(item)
        return True
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        try:
            q.put_nowait(item)
            return True
        except queue.Full:
            return False


def _camera_process_main(
    role: str,
    camera_config: Mapping[str, Any],
    ring_spec: SharedCameraRingSpec,
    descriptor_queue: Any,
    status_queue: Any,
    stop_event: Any,
    forbidden_cpu: int | None,
) -> None:
    configure_non_realtime_affinity(forbidden_cpu)
    ring = SharedMemoryCameraFrameRing.attach(ring_spec)
    received = 0
    dropped_descriptors = 0
    publish_durations: deque[int] = deque(maxlen=4096)
    interframe_intervals: deque[int] = deque(maxlen=4096)
    previous_frame_ns: int | None = None
    camera: RealSenseCamera | None = None
    try:
        camera = RealSenseCamera(dict(camera_config))
        while not stop_event.is_set():
            frame = camera.capture()
            sample = camera_sample_from_rgbd(role, frame, copy_arrays=False)
            sequence = received
            started_ns = time.perf_counter_ns()
            reference = ring.publish(sample, sequence)
            publish_durations.append(time.perf_counter_ns() - started_ns)
            if previous_frame_ns is not None:
                interframe_intervals.append(sample.host_monotonic_ns - previous_frame_ns)
            previous_frame_ns = sample.host_monotonic_ns
            descriptor = FrameReferenceDescriptor.from_reference(reference)
            if not _queue_put_latest(descriptor_queue, descriptor):
                dropped_descriptors += 1
            received += 1
            if received == 1:
                _queue_put_latest(
                    status_queue,
                    {
                        "kind": "ready",
                        "profile": camera.profile_metadata(),
                        "placement": process_placement(f"camera_{role}"),
                    },
                )
    except BaseException as exc:
        _queue_put_latest(
            status_queue,
            {"kind": "error", "error": f"{type(exc).__name__}: {exc}"},
        )
    finally:
        if camera is not None:
            camera.close()
        _queue_put_latest(
            status_queue,
            {
                "kind": "stopped",
                "received": received,
                "dropped_descriptors": dropped_descriptors,
                "camera_publish_duration_ns": _summary(list(publish_durations)),
                "camera_interframe_interval_ns": _summary(list(interframe_intervals)),
                "camera_frame_age_ns": {
                    "count": 0 if previous_frame_ns is None else 1,
                    "p50": 0 if previous_frame_ns is None else max(0, time.monotonic_ns() - previous_frame_ns),
                    "p95": 0 if previous_frame_ns is None else max(0, time.monotonic_ns() - previous_frame_ns),
                    "p99": 0 if previous_frame_ns is None else max(0, time.monotonic_ns() - previous_frame_ns),
                    "max": 0 if previous_frame_ns is None else max(0, time.monotonic_ns() - previous_frame_ns),
                },
                "placement": process_placement(f"camera_{role}"),
            },
        )
        ring.close()


def _summary(values: list[int]) -> dict[str, int]:
    if not values:
        return {"count": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0}
    values = sorted(values)
    last = len(values) - 1
    return {
        "count": len(values),
        "p50": values[round(last * 0.50)],
        "p95": values[round(last * 0.95)],
        "p99": values[round(last * 0.99)],
        "max": values[-1],
    }


class ProcessCamera:
    """Parent-side camera handle that never snapshots a shared frame."""

    def __init__(
        self,
        role: str,
        camera_config: Mapping[str, Any],
        *,
        capacity: int,
        forbidden_cpu: int | None,
        context: mp.context.BaseContext,
    ) -> None:
        width = int(camera_config.get("width", 640))
        height = int(camera_config.get("height", 480))
        if width <= 0 or height <= 0:
            raise ValueError("camera width and height must be positive")
        aligned = bool(camera_config.get("align_depth_to_color", False))
        self.role = role
        self._ring = SharedMemoryCameraFrameRing.create(
            role=role,
            capacity=capacity,
            rgb_shape=(height, width, 3),
            depth_shape=(height, width),
            aligned_depth_shape=(height, width) if aligned else None,
        )
        self._closed = False
        try:
            self._descriptors = context.Queue(maxsize=capacity)
            self._status = context.Queue(maxsize=16)
            self._stop = context.Event()
            self._references: deque[CameraFrameRef] = deque(maxlen=capacity)
            self._profile: dict[str, object] = {}
            self._error: str | None = None
            self._last_timestamp_ns = -1
            self._process = context.Process(
                target=_camera_process_main,
                args=(
                    role,
                    dict(camera_config),
                    self._ring.spec,
                    self._descriptors,
                    self._status,
                    self._stop,
                    forbidden_cpu,
                ),
                name=f"camera-{role}",
            )
        except BaseException:
            self._ring.close()
            raise

    @property
    def ring_spec(self) -> SharedCameraRingSpec:
        return self._ring.spec

    def start(self, timeout_s: float) -> None:
        if self._closed:
            raise RuntimeError(f"{self.role} camera handle is closed")
        try:
            self._process.start()
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                self._poll_status()
                if self._profile:
                    return
                if not self._process.is_alive() and self._error is None:
                    self._error = "camera process exited before first frame"
                    break
                time.sleep(0.005)
            raise TimeoutError(
                f"{self.role} camera process did not publish a frame before startup deadline"
            )
        except BaseException:
            self.stop()
            raise

    @property
    def error(self) -> str | None:
        self._poll_status()
        if self._error is None and not self._process.is_alive() and self._process.exitcode not in (None, 0):
            self._error = f"camera process exited with code {self._process.exitcode}"
        return self._error

    @property
    def latest(self) -> CameraFrameRef | None:
        self._drain_descriptors()
        return None if not self._references else self._references[-1]

    def latest_after(self, host_monotonic_ns: int) -> tuple[CameraFrameRef | None, int]:
        self._poll_status()
        self._drain_descriptors()
        unread = [frame for frame in self._references if frame.host_monotonic_ns > host_monotonic_ns]
        if not unread:
            return None, 0
        return unread[-1], len(unread) - 1

    def profile_metadata(self) -> dict[str, object]:
        self._poll_status()
        return {
            **self._profile,
            "role": self.role,
            "process_alive": self._process.is_alive(),
            "shared_ring": {
                "capacity": self._ring.capacity,
                "rgb_shape": self._ring.spec.rgb_shape,
                "depth_shape": self._ring.spec.depth_shape,
                "aligned_depth_shape": self._ring.spec.aligned_depth_shape,
                **self._ring.metrics(),
            },
        }

    def stop(self, timeout_s: float = 3.0) -> None:
        if self._closed:
            return
        try:
            self._stop.set()
            if self._process.pid is not None:
                self._process.join(timeout=timeout_s)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=timeout_s)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(timeout=timeout_s)
            self._poll_status()
        finally:
            self._ring.close()
            self._descriptors.close()
            self._status.close()
            self._closed = True

    def _drain_descriptors(self) -> None:
        while True:
            try:
                descriptor = self._descriptors.get_nowait()
            except queue.Empty:
                return
            if not isinstance(descriptor, FrameReferenceDescriptor):
                continue
            reference = descriptor.to_reference(self._ring)
            self._references.append(reference)
            self._last_timestamp_ns = max(self._last_timestamp_ns, reference.host_monotonic_ns)

    def _poll_status(self) -> None:
        while True:
            try:
                message = self._status.get_nowait()
            except queue.Empty:
                return
            kind = message.get("kind")
            if kind == "ready":
                self._profile = dict(message.get("profile") or {})
                self._profile["placement"] = message.get("placement")
            elif kind == "error":
                self._error = str(message.get("error"))
            elif kind == "stopped":
                self._profile.update({key: value for key, value in message.items() if key != "kind"})


class _ClockProxy:
    def __init__(self, period_ns: int) -> None:
        self.period_ns = int(period_ns)


class _WriterProxy:
    def __init__(self, temporary_id: str, root: Path) -> None:
        self.temporary_id = temporary_id
        self.root = root
        self.sample_count = 0
        self._diagnostics: dict[str, object] = {}

    def diagnostics(self) -> dict[str, object]:
        return dict(self._diagnostics)


class ProcessEpisodeCollectorProxy:
    """Control-side facade for a recorder-process SingleEpisodeCollector."""

    def __init__(self, recorder: "ProcessEpisodeRecorder", *, fps: int, temporary_id: str, root: Path) -> None:
        self._recorder = recorder
        self.clock = _ClockProxy(1_000_000_000 // int(fps))
        self.writer = _WriterProxy(temporary_id, root)
        self._state = CaptureState.IDLE
        self._completion_status: EpisodeStatus | None = None
        self._termination_reason: str | None = None
        self._result: Path | None = None
        self._diagnostics: dict[str, object] = {}

    @property
    def state(self) -> CaptureState:
        self._sync()
        return self._state

    @state.setter
    def state(self, value: CaptureState) -> None:
        self._state = value

    @property
    def completion_status(self) -> EpisodeStatus | None:
        self._sync()
        return self._completion_status

    @property
    def termination_reason(self) -> str | None:
        self._sync()
        return self._termination_reason

    @property
    def result(self) -> Path | None:
        self._sync()
        return self._result

    @property
    def recorder_failure(self) -> str | None:
        return self._recorder.error

    def ingest_camera(self, frame: CameraFrameRef, *, skipped_frames: int = 0) -> None:
        if self.state is CaptureState.DONE:
            return
        try:
            self._recorder.send(
                "camera",
                FrameReferenceDescriptor.from_reference(frame),
                int(skipped_frames),
            )
        except OSError as exc:
            self._mark_failed(str(exc))

    def ingest_control(
        self,
        sample: ControlSample,
        *,
        reference_established: bool,
        raw_records: Mapping[str, Mapping[str, Any]] | None = None,
        capture_active: bool | None = None,
    ) -> None:
        if self.state is CaptureState.DONE:
            return
        try:
            self._recorder.send(
                "control",
                sample,
                bool(reference_established),
                raw_records,
                capture_active,
            )
        except OSError as exc:
            self._mark_failed(str(exc))

    def camera_fault(self, role: str, reason: str) -> None:
        if self.state is not CaptureState.DONE:
            self.abort(f"{role}_camera_disconnected:{reason}")

    def finish(self, reason: str, *, release_ns: int | None = None) -> None:
        self.state = CaptureState.DONE
        if self._recorder.error is None:
            try:
                self._recorder.send("finish", reason, release_ns)
            except OSError:
                pass

    def abort(self, reason: str, *, invalid: bool = False, detail: str | None = None) -> None:
        self.state = CaptureState.DONE
        if self._recorder.error is None:
            try:
                self._recorder.send("abort", reason, invalid, detail)
            except OSError:
                pass

    def shutdown(self, reason: str) -> None:
        if self.state is not CaptureState.DONE:
            self.state = CaptureState.DONE
        if self._recorder.error is None:
            try:
                self._recorder.send("shutdown", reason)
            except OSError:
                pass

    def finalize_pending(self) -> None:
        if self._recorder.error is not None:
            return
        try:
            response = self._recorder.request("finalize_pending")
        except (OSError, RuntimeError, TimeoutError) as exc:
            self._mark_failed(str(exc))
            return
        self._apply_status(response)

    def diagnostics(self) -> dict[str, object]:
        self._recorder.poll()
        self._sync()
        return dict(self._diagnostics)

    def _sync(self) -> None:
        latest = self._recorder.latest_status
        if latest is not None:
            self._apply_status(latest)
        if self._recorder.error is not None and self._state is not CaptureState.DONE:
            self._mark_failed(self._recorder.error)

    def _mark_failed(self, reason: str) -> None:
        self._state = CaptureState.DONE
        self._completion_status = EpisodeStatus.ABORTED
        self._termination_reason = "recording_process_failure"
        self._diagnostics = {"recording_process_error": reason}

    def _apply_status(self, message: Mapping[str, object]) -> None:
        if "state" in message:
            self.state = CaptureState(str(message["state"]))
        if "completion_status" in message and message["completion_status"] is not None:
            self._completion_status = EpisodeStatus(str(message["completion_status"]))
        if message.get("termination_reason") is not None:
            self._termination_reason = str(message["termination_reason"])
        result = message.get("result")
        if result is not None:
            self._result = Path(str(result))
        self.writer.sample_count = int(message.get("sample_count", self.writer.sample_count))
        diagnostics = message.get("diagnostics")
        if isinstance(diagnostics, Mapping):
            self._diagnostics = dict(diagnostics)
        writer_diagnostics = message.get("writer_diagnostics")
        if isinstance(writer_diagnostics, Mapping):
            self.writer._diagnostics = dict(writer_diagnostics)


class ProcessEpisodeRecorder:
    """Recorder child process with bounded command and response queues."""

    def __init__(
        self,
        *,
        context: mp.context.BaseContext,
        ring_specs: Mapping[str, SharedCameraRingSpec],
        episode_root: str | Path | None,
        task_name: str,
        operator: str,
        control_config_path: str | Path,
        maximum_start_delta_rad: float,
        metadata: Mapping[str, Any],
        dataset: Mapping[str, Any],
        camera_profiles: Mapping[str, Any],
        forbidden_cpu: int | None,
        schema_version: str = SCHEMA_VERSION,
    ) -> None:
        self._commands = context.Queue(maxsize=max(8, int(dataset.get("recorder_queue_capacity", 16))))
        self._status = context.Queue(maxsize=32)
        self._process = context.Process(
            target=_recorder_process_main,
            args=(
                dict(ring_specs),
                self._commands,
                self._status,
                {
                    "episode_root": str(episode_root or dataset.get("root", "data/episodes")),
                    "task_name": task_name,
                    "operator": operator,
                    "control_config_path": str(control_config_path),
                    "maximum_start_delta_rad": maximum_start_delta_rad,
                    "metadata": dict(metadata),
                    "dataset": dict(dataset),
                    "camera_profiles": dict(camera_profiles),
                    "schema_version": schema_version,
                    "forbidden_cpu": forbidden_cpu,
                },
            ),
            name="episode-recorder",
        )
        self._error: str | None = None
        self._closed = False
        self._placement: dict[str, object] | None = None
        self._responses: dict[int, Mapping[str, object]] = {}
        self._next_request_id = 1
        self._config = {
            "dataset": dict(dataset),
        }
        self._latest_status: Mapping[str, object] | None = None

    @property
    def latest_status(self) -> Mapping[str, object] | None:
        return self._latest_status

    @property
    def error(self) -> str | None:
        self.poll()
        if self._error is None and not self._process.is_alive() and self._process.exitcode not in (None, 0):
            self._error = f"recorder process exited with code {self._process.exitcode}"
        return self._error

    def start(self, timeout_s: float) -> ProcessEpisodeCollectorProxy:
        if self._closed:
            raise RuntimeError("recorder handle is closed")
        try:
            self._process.start()
            deadline = time.monotonic() + timeout_s
            ready: Mapping[str, object] | None = None
            while time.monotonic() < deadline:
                self.poll()
                if self._responses:
                    pass
                try:
                    message = self._status.get(timeout=0.01)
                except queue.Empty:
                    message = None
                if isinstance(message, Mapping) and message.get("kind") == "ready":
                    ready = message
                    break
                if self.error is not None:
                    break
            if ready is None:
                raise TimeoutError(
                    "recorder process did not become ready before startup deadline"
                )
            return ProcessEpisodeCollectorProxy(
                self,
                fps=int(self._config["dataset"].get("fps", 30)),
                temporary_id=str(ready["temporary_id"]),
                root=Path(str(ready["root"])),
            )
        except BaseException:
            self.stop()
            raise

    def send(self, kind: str, *payload: object) -> None:
        self.poll()
        if self.error is not None:
            raise OSError(f"recorder process failed: {self.error}")
        try:
            self._commands.put_nowait((kind, payload, None))
        except queue.Full as exc:
            self._error = "recorder command queue full"
            raise OSError(self._error) from exc

    def request(self, kind: str, *payload: object, timeout_s: float = 8.0) -> Mapping[str, object]:
        self.poll()
        request_id = self._next_request_id
        self._next_request_id += 1
        try:
            self._commands.put((kind, payload, request_id), timeout=timeout_s)
        except queue.Full as exc:
            self._error = "recorder request queue remained full"
            raise TimeoutError(self._error) from exc
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            response = self._responses.pop(request_id, None)
            if response is not None:
                if response.get("error"):
                    raise RuntimeError(str(response["error"]))
                return response
            if self.error is not None:
                raise RuntimeError(self.error)
            time.sleep(0.005)
        raise TimeoutError(f"recorder request {kind} exceeded bounded timeout")

    def poll(self) -> None:
        while True:
            try:
                message = self._status.get_nowait()
            except queue.Empty:
                return
            if not isinstance(message, Mapping):
                continue
            if message.get("kind") == "response":
                self._responses[int(message["request_id"])] = message
            elif message.get("kind") == "state":
                self._latest_status = message
            elif message.get("kind") == "error":
                self._error = str(message.get("error"))

    def stop(self, timeout_s: float = 5.0) -> None:
        if self._closed:
            return
        try:
            if self._process.pid is not None and self._process.is_alive():
                try:
                    self._commands.put(("stop", (), None), timeout=timeout_s)
                except queue.Full:
                    pass
                self._process.join(timeout=timeout_s)
            if self._process.pid is not None and self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=timeout_s)
            if self._process.pid is not None and self._process.is_alive():
                self._process.kill()
                self._process.join(timeout=timeout_s)
            self.poll()
        finally:
            self._commands.close()
            self._status.close()
            self._closed = True


def _recorder_process_main(
    ring_specs: Mapping[str, SharedCameraRingSpec],
    commands: Any,
    status_queue: Any,
    config: Mapping[str, Any],
) -> None:
    configure_non_realtime_affinity(config.get("forbidden_cpu"))
    rings: dict[str, SharedMemoryCameraFrameRing] = {}
    try:
        for role, spec in ring_specs.items():
            rings[role] = SharedMemoryCameraFrameRing.attach(spec)
    except BaseException as exc:
        for ring in rings.values():
            ring.close()
        _queue_put_latest(
            status_queue,
            {"kind": "error", "error": f"{type(exc).__name__}: {exc}"},
        )
        return
    dataset = dict(config["dataset"])
    writer = AsyncEpisodeWriter(
        CanonicalEpisodeWriter(
            config["episode_root"],
            task_name=str(config["task_name"]),
            operator=str(config["operator"]),
            dataset_fps=int(dataset.get("fps", 30)),
            schema_version=str(config.get("schema_version", SCHEMA_VERSION)),
            metadata={
                **dict(config["metadata"]),
                "camera_profiles": dict(config["camera_profiles"]),
                "process_placement": process_placement("episode_recorder"),
            },
        ),
        capacity=min(
            int(dataset.get("recorder_queue_capacity", 16)),
            int(dataset.get("camera_ring_capacity", 16)),
        ),
        batch_size=int(dataset.get("writer_batch_size", 8)),
        flush_interval_s=float(dataset.get("writer_flush_interval_s", 1.0)),
        shutdown_timeout_s=float(dataset.get("writer_shutdown_timeout_s", 5.0)),
        require_frame_references=True,
    )
    collector = SingleEpisodeCollector(
        writer,
        camera_max_age_ns=round(float(dataset.get("camera_max_age_ms", 100.0)) * 1e6),
        control_max_age_ns=round(float(dataset.get("control_max_age_ms", 40.0)) * 1e6),
        maximum_start_delta_rad=float(config["maximum_start_delta_rad"]),
        maximum_hand_start_delta_rad=float(dataset.get("hand_start_tolerance_rad", 0.05)),
        defer_finalization=True,
        camera_severe_stale_ns=round(float(dataset.get("camera_severe_stale_limit_ms", 500.0)) * 1e6),
        camera_consecutive_stale_limit=int(dataset.get("camera_consecutive_stale_limit", 15)),
        camera_missing_timeout_ns=round(float(dataset.get("camera_missing_timeout_ms", 1000.0)) * 1e6),
        quality_min_valid_ratio=float(dataset.get("quality_min_valid_ratio", 1.0)),
        quality_max_invalid_run=int(dataset.get("quality_max_invalid_run", 0)),
    )
    def final_metadata() -> dict[str, object]:
        writer_diagnostics = writer.diagnostics()
        return {
            "camera_profiles": dict(config["camera_profiles"]),
            "process_placement": process_placement("episode_recorder"),
            **collector.diagnostics(),
            "frame_materialization_duration_ns": writer_diagnostics[
                "frame_materialization_duration_ns"
            ],
            "canonical_metadata_duration_ns": writer_diagnostics[
                "canonical_metadata_duration_ns"
            ],
        }

    writer.set_final_metadata_provider(final_metadata)
    _queue_put_latest(
        status_queue,
        {
            "kind": "ready",
            "temporary_id": writer.temporary_id,
            "root": str(writer.root),
            "placement": process_placement("episode_recorder"),
        },
    )

    def report(request_id: int | None = None, error: BaseException | None = None) -> None:
        message: dict[str, object] = {
            "kind": "response" if request_id is not None else "state",
            "state": collector.state.value,
            "completion_status": None if collector.completion_status is None else collector.completion_status.value,
            "termination_reason": collector.termination_reason,
            "result": None if collector.result is None else str(collector.result),
            "sample_count": writer.sample_count,
            "diagnostics": collector.diagnostics(),
            "writer_diagnostics": writer.diagnostics(),
        }
        if request_id is not None:
            message["request_id"] = request_id
        if error is not None:
            message["error"] = f"{type(error).__name__}: {error}"
        _queue_put_latest(status_queue, message)

    try:
        while True:
            kind, payload, request_id = commands.get()
            error: BaseException | None = None
            try:
                if kind == "camera":
                    descriptor, skipped = payload
                    reference = descriptor.to_reference(rings[descriptor.role])
                    collector.ingest_camera(reference, skipped_frames=int(skipped))
                elif kind == "control":
                    sample, reference_established, raw_records, capture_active = payload
                    collector.ingest_control(
                        sample,
                        reference_established=bool(reference_established),
                        raw_records=raw_records,
                        capture_active=capture_active,
                    )
                elif kind == "finish":
                    reason, release_ns = payload
                    collector.finish(str(reason), release_ns=release_ns)
                elif kind == "abort":
                    reason, invalid, detail = payload
                    collector.abort(str(reason), invalid=bool(invalid), detail=detail)
                elif kind == "shutdown":
                    collector.shutdown(str(payload[0]))
                elif kind == "finalize_pending":
                    collector.finalize_pending()
                elif kind == "stop":
                    break
                else:
                    raise ValueError(f"unknown recorder command {kind!r}")
            except BaseException as exc:
                error = exc
                if kind in {"camera", "control"}:
                    collector.abort("recording_writer_failure", detail=str(exc))
            report(request_id, error)
            if kind == "finalize_pending":
                break
    except BaseException as exc:
        _queue_put_latest(status_queue, {"kind": "error", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        try:
            if collector.state is CaptureState.IDLE:
                writer.close()
        except BaseException as exc:
            _queue_put_latest(status_queue, {"kind": "error", "error": f"{type(exc).__name__}: {exc}"})
        for ring in rings.values():
            ring.close()


class ProcessPreview:
    """Latest-only preview process; frame display cannot backpressure capture."""

    def __init__(
        self,
        *,
        context: mp.context.BaseContext,
        ring_specs: Mapping[str, SharedCameraRingSpec],
        refresh_hz: float,
        forbidden_cpu: int | None,
    ) -> None:
        self._updates = context.Queue(maxsize=1)
        self._status = context.Queue(maxsize=4)
        self._stop = context.Event()
        self._process = context.Process(
            target=_preview_process_main,
            args=(dict(ring_specs), self._updates, self._status, self._stop, refresh_hz, forbidden_cpu),
            name="episode-preview",
        )
        self._error: str | None = None
        self._placement: dict[str, object] | None = None
        self._closed = False

    def start(self) -> None:
        if self._closed:
            raise RuntimeError("preview handle is closed")
        self._process.start()

    def update(
        self,
        workspace: FrameReferenceDescriptor | None,
        wrist: FrameReferenceDescriptor | None,
        status: PreviewStatus,
    ) -> None:
        _queue_put_latest(self._updates, (workspace, wrist, status))
        self.poll()

    @property
    def error(self) -> str | None:
        self.poll()
        return self._error

    def diagnostics(self) -> dict[str, object]:
        self.poll()
        return {"error": self._error, "placement": self._placement}

    def poll(self) -> None:
        while True:
            try:
                message = self._status.get_nowait()
            except queue.Empty:
                return
            if message.get("kind") == "error":
                self._error = str(message.get("error"))
            elif message.get("kind") == "ready":
                placement = message.get("placement")
                if isinstance(placement, Mapping):
                    self._placement = dict(placement)

    def stop(self, timeout_s: float = 2.0) -> None:
        if self._closed:
            return
        try:
            self._stop.set()
            if self._process.pid is not None:
                self._process.join(timeout=timeout_s)
                if self._process.is_alive():
                    self._process.terminate()
                    self._process.join(timeout=timeout_s)
                if self._process.is_alive():
                    self._process.kill()
                    self._process.join(timeout=timeout_s)
            self.poll()
        finally:
            self._updates.close()
            self._status.close()
            self._closed = True


class _PreviewCameraSource:
    def __init__(self, role: str, ring: SharedMemoryCameraFrameRing) -> None:
        self.role = role
        self.ring = ring
        self._latest: CameraFrameRef | None = None

    def set_latest(self, descriptor: FrameReferenceDescriptor | None) -> None:
        self._latest = None if descriptor is None else descriptor.to_reference(self.ring)

    def latest(self) -> CameraFrameRef | None:
        return self._latest

    actual_fps = 0.0
    dropped_frames = 0


def _preview_process_main(
    ring_specs: Mapping[str, SharedCameraRingSpec],
    updates: Any,
    status_queue: Any,
    stop_event: Any,
    refresh_hz: float,
    forbidden_cpu: int | None,
) -> None:
    configure_non_realtime_affinity(forbidden_cpu)
    rings: dict[str, SharedMemoryCameraFrameRing] = {}
    try:
        for role, spec in ring_specs.items():
            rings[role] = SharedMemoryCameraFrameRing.attach(spec)
    except BaseException as exc:
        for ring in rings.values():
            ring.close()
        _queue_put_latest(
            status_queue,
            {"kind": "error", "error": f"{type(exc).__name__}: {exc}"},
        )
        return
    try:
        require_preview_dependencies()
        renderer = DualCameraPreview()
        _queue_put_latest(
            status_queue,
            {"kind": "ready", "placement": process_placement("episode_preview")},
        )
        sources = {role: _PreviewCameraSource(role, rings[role]) for role in rings}
        latest: tuple[FrameReferenceDescriptor | None, FrameReferenceDescriptor | None, PreviewStatus] | None = None
        while not stop_event.is_set():
            try:
                latest = updates.get_nowait()
            except queue.Empty:
                pass
            if latest is not None:
                workspace, wrist, status = latest
                sources["workspace"].set_latest(workspace)
                sources["wrist"].set_latest(wrist)
                if not renderer.render(sources["workspace"], sources["wrist"], status):
                    return
            stop_event.wait(max(1.0 / float(refresh_hz), 0.001))
    except BaseException as exc:
        _queue_put_latest(status_queue, {"kind": "error", "error": f"{type(exc).__name__}: {exc}"})
    finally:
        try:
            if "renderer" in locals():
                renderer.close()
        finally:
            for ring in rings.values():
                ring.close()
