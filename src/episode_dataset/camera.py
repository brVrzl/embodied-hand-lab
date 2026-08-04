from __future__ import annotations

from collections import deque
import threading
import time
from typing import Callable

import numpy as np

from vision_interface.interfaces import CameraInterface, RGBDFrame

from .episode import CameraFrameRef, CameraSample


class _FrameSlot:
    __slots__ = ("version", "sequence", "rgb", "depth_raw", "depth_aligned")

    def __init__(self, sample: CameraSample) -> None:
        self.version = 0
        self.sequence = -1
        self.rgb = np.empty_like(sample.rgb)
        self.depth_raw = np.empty_like(sample.depth_raw)
        self.depth_aligned = (
            None
            if sample.depth_aligned_to_rgb is None
            else np.empty_like(sample.depth_aligned_to_rgb)
        )


class CameraFrameRing:
    """Single-producer, multi-consumer preallocated RGB-D seqlock ring."""

    def __init__(self, capacity: int) -> None:
        if capacity <= 1:
            raise ValueError("camera ring capacity must be greater than one")
        self.capacity = int(capacity)
        self._slots: list[_FrameSlot] | None = None
        self._metadata: dict[int, CameraFrameRef] = {}
        # Metadata lifetime and slot publication are guarded separately from
        # the counters.  This lock is held only for dictionary operations; it
        # never spans an ndarray copy or camera capture.
        self._metadata_lock = threading.Lock()
        self._next_sequence = 0
        self._counter_lock = threading.Lock()
        self._overwrite_count = 0
        self._slot_reuse_count = 0
        self._inconsistent_read_count = 0

    @property
    def overwrite_count(self) -> int:
        """Number of published frames that reused an occupied slot."""
        with self._counter_lock:
            return self._slot_reuse_count

    @property
    def reference_expired_count(self) -> int:
        """Number of consumer reads rejected after the referenced slot expired."""
        with self._counter_lock:
            return self._overwrite_count

    @property
    def inconsistent_read_count(self) -> int:
        with self._counter_lock:
            return self._inconsistent_read_count

    @property
    def slot_reuse_count(self) -> int:
        with self._counter_lock:
            return self._slot_reuse_count

    def publish(self, sample: CameraSample) -> CameraFrameRef:
        if self._slots is None:
            self._slots = [_FrameSlot(sample) for _ in range(self.capacity)]
        slots = self._slots
        assert slots is not None
        sequence = self._next_sequence
        self._next_sequence += 1
        slot = slots[sequence % self.capacity]
        if slot.sequence >= 0:
            with self._counter_lock:
                self._slot_reuse_count += 1
            with self._metadata_lock:
                self._metadata.pop(slot.sequence, None)
        if (
            slot.rgb.shape != sample.rgb.shape
            or slot.depth_raw.shape != sample.depth_raw.shape
            or (slot.depth_aligned is None) != (sample.depth_aligned_to_rgb is None)
            or (
                slot.depth_aligned is not None
                and sample.depth_aligned_to_rgb is not None
                and slot.depth_aligned.shape != sample.depth_aligned_to_rgb.shape
            )
        ):
            raise RuntimeError("camera profile changed after ring allocation")

        # Odd versions are being written; even versions are stable.  NumPy
        # copies may release the GIL, so consumers verify the version again
        # after copying instead of relying on Python object atomicity.
        stable_version = sequence * 2 + 2
        slot.version = stable_version - 1
        np.copyto(slot.rgb, sample.rgb)
        np.copyto(slot.depth_raw, sample.depth_raw)
        if slot.depth_aligned is not None and sample.depth_aligned_to_rgb is not None:
            np.copyto(slot.depth_aligned, sample.depth_aligned_to_rgb)
        slot.sequence = sequence
        slot.version = stable_version
        reference = CameraFrameRef(
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
        with self._metadata_lock:
            self._metadata[sequence] = reference
        return reference

    def read(self, sequence: int) -> CameraSample | None:
        slots = self._slots
        with self._metadata_lock:
            reference = self._metadata.get(int(sequence))
        if slots is None or reference is None:
            with self._counter_lock:
                self._overwrite_count += 1
            return None
        slot = slots[sequence % self.capacity]
        expected_version = sequence * 2 + 2
        for _ in range(3):
            before = slot.version
            if before != expected_version or before & 1 or slot.sequence != sequence:
                continue
            rgb = slot.rgb.copy()
            depth_raw = slot.depth_raw.copy()
            depth_aligned = (
                None if slot.depth_aligned is None else slot.depth_aligned.copy()
            )
            if slot.version == before and slot.sequence == sequence:
                return CameraSample(
                    role=reference.role,
                    host_monotonic_ns=reference.host_monotonic_ns,
                    rgb=rgb,
                    depth_raw=depth_raw,
                    depth_aligned_to_rgb=depth_aligned,
                    depth_scale_m=reference.depth_scale_m,
                    device_rgb_timestamp_ms=reference.device_rgb_timestamp_ms,
                    device_depth_timestamp_ms=reference.device_depth_timestamp_ms,
                    rgb_frame_number=reference.rgb_frame_number,
                    depth_frame_number=reference.depth_frame_number,
                    rgb_timestamp_domain=reference.rgb_timestamp_domain,
                    depth_timestamp_domain=reference.depth_timestamp_domain,
                    ring_sequence=sequence,
                )
        with self._counter_lock:
            self._inconsistent_read_count += 1
        return None


class AsyncRGBDCamera:
    """Continuously capture one camera without blocking control or preview."""

    def __init__(
        self,
        role: str,
        camera_factory: Callable[[], CameraInterface],
        *,
        queue_capacity: int = 16,
        copy_arrays: bool = True,
    ) -> None:
        if role not in {"workspace", "wrist"}:
            raise ValueError("role must be workspace or wrist")
        self.role = role
        self._factory = camera_factory
        # Generic camera sources may reuse their backing buffers.  The live
        # RealSense adapter already returns owned arrays, so the episode
        # runtime can disable this second full-frame copy on that path.
        self._copy_arrays = bool(copy_arrays)
        self._frames: deque[CameraFrameRef] = deque(maxlen=queue_capacity)
        self._ring = CameraFrameRing(queue_capacity)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"rgbd-{role}", daemon=True)
        self._camera: CameraInterface | None = None
        self._error: BaseException | None = None
        self._received = 0
        self._dropped = 0
        self._queue_overflow = 0
        self._previous_numbers: tuple[int, int] | None = None
        self._receive_times: deque[int] = deque(maxlen=90)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        camera = self._camera
        close = getattr(camera, "close", None)
        if callable(close):
            close()
        self._thread.join(timeout=3.0)

    @property
    def error(self) -> BaseException | None:
        with self._lock:
            return self._error

    @property
    def dropped_frames(self) -> int:
        with self._lock:
            return self._dropped

    @property
    def queue_depth(self) -> int:
        with self._lock:
            return len(self._frames)

    @property
    def queue_overflow_count(self) -> int:
        with self._lock:
            return self._queue_overflow

    @property
    def actual_fps(self) -> float:
        with self._lock:
            if len(self._receive_times) < 2:
                return 0.0
            elapsed = (self._receive_times[-1] - self._receive_times[0]) / 1e9
            return 0.0 if elapsed <= 0.0 else (len(self._receive_times) - 1) / elapsed

    def latest(self) -> CameraFrameRef | None:
        with self._lock:
            return None if not self._frames else self._frames[-1]

    def frames_after(self, host_monotonic_ns: int) -> list[CameraFrameRef]:
        with self._lock:
            return [frame for frame in self._frames if frame.host_monotonic_ns > host_monotonic_ns]

    def latest_after(self, host_monotonic_ns: int) -> tuple[CameraFrameRef | None, int]:
        """Return only the newest unread reference and the skipped-frame count."""

        with self._lock:
            unread = sum(
                frame.host_monotonic_ns > host_monotonic_ns for frame in self._frames
            )
            if unread == 0:
                return None, 0
            return self._frames[-1], unread - 1

    def profile_metadata(self) -> dict[str, object]:
        camera = self._camera
        getter = getattr(camera, "profile_metadata", None)
        profile = getter() if callable(getter) else {}
        with self._lock:
            receive_times = list(self._receive_times)
            intervals_ns = [
                later - earlier
                for earlier, later in zip(
                    receive_times[:-1], receive_times[1:], strict=True
                )
            ]
            latest_timestamp_ns = None if not self._frames else self._frames[-1].host_monotonic_ns
        timing = {
            "interframe_interval_ns": _summary(intervals_ns),
            "frame_age_ns": (
                None
                if latest_timestamp_ns is None
                else max(0, time.monotonic_ns() - latest_timestamp_ns)
            ),
        }
        return {
            **profile,
            "role": self.role,
            "actual_fps": self.actual_fps,
            "dropped_frame_count": self.dropped_frames,
            **timing,
            "queue_capacity": self._frames.maxlen,
            "queue_depth": self.queue_depth,
            "queue_overflow_count": self.queue_overflow_count,
            "ring_capacity": self._ring.capacity,
            "camera_ring_overwrite_count": self._ring.overwrite_count,
            "camera_ring_reference_expired_count": self._ring.reference_expired_count,
            "camera_ring_slot_reuse_count": self._ring.slot_reuse_count,
            "camera_inconsistent_read_count": self._ring.inconsistent_read_count,
            "thread_alive": self._thread.is_alive(),
        }

    def _run(self) -> None:
        try:
            self._camera = self._factory()
            while not self._stop.is_set():
                frame = self._camera.capture()
                sample = camera_sample_from_rgbd(
                    self.role, frame, copy_arrays=self._copy_arrays
                )
                reference = self._ring.publish(sample)
                with self._lock:
                    if len(self._frames) == self._frames.maxlen:
                        self._queue_overflow += 1
                    numbers = (sample.rgb_frame_number, sample.depth_frame_number)
                    if self._previous_numbers is not None:
                        color_gap = max(numbers[0] - self._previous_numbers[0] - 1, 0)
                        depth_gap = max(numbers[1] - self._previous_numbers[1] - 1, 0)
                        self._dropped += max(color_gap, depth_gap)
                    self._previous_numbers = numbers
                    self._frames.append(reference)
                    self._receive_times.append(sample.host_monotonic_ns)
                    self._received += 1
        except BaseException as exc:
            with self._lock:
                self._error = exc


def camera_sample_from_rgbd(
    role: str, frame: RGBDFrame, *, copy_arrays: bool = True
) -> CameraSample:
    if frame.depth_raw_units is None:
        depth_scale = frame.depth_scale_m
        if depth_scale is None or depth_scale <= 0:
            raise ValueError("RGBDFrame does not preserve raw depth or declare depth scale")
        raw = np.rint(frame.depth_m / depth_scale).astype(np.uint16)
    else:
        raw = (
            frame.depth_raw_units.copy()
            if copy_arrays
            else frame.depth_raw_units
        )
    host_monotonic_ns = frame.host_monotonic_ns
    if host_monotonic_ns is None:
        # Mock/legacy sources are permitted for offline tests, but are stamped
        # at the adapter boundary rather than pretending wall time is monotonic.
        host_monotonic_ns = time.monotonic_ns()
    return CameraSample(
        role=role,
        host_monotonic_ns=host_monotonic_ns,
        rgb=frame.rgb.copy() if copy_arrays else frame.rgb,
        depth_raw=raw,
        depth_aligned_to_rgb=(
            None
            if frame.depth_aligned_to_color_units is None
            else (
                frame.depth_aligned_to_color_units.copy()
                if copy_arrays
                else frame.depth_aligned_to_color_units
            )
        ),
        depth_scale_m=frame.depth_scale_m,
        device_rgb_timestamp_ms=frame.color_timestamp_ms,
        device_depth_timestamp_ms=frame.depth_timestamp_ms,
        rgb_frame_number=frame.color_frame_number,
        depth_frame_number=frame.depth_frame_number,
        rgb_timestamp_domain=frame.color_timestamp_domain,
        depth_timestamp_domain=frame.depth_timestamp_domain,
    )


def _summary(values: list[int]) -> dict[str, int]:
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "p50": 0, "p95": 0, "p99": 0, "max": 0}
    last = len(ordered) - 1
    return {
        "count": len(ordered),
        "p50": ordered[round(last * 0.50)],
        "p95": ordered[round(last * 0.95)],
        "p99": ordered[round(last * 0.99)],
        "max": ordered[-1],
    }
