from __future__ import annotations

from collections import deque
import threading
import time
from typing import Callable

import numpy as np

from vision_interface.interfaces import CameraInterface, RGBDFrame

from .episode import CameraSample


class AsyncRGBDCamera:
    """Continuously capture one camera without blocking control or preview."""

    def __init__(
        self,
        role: str,
        camera_factory: Callable[[], CameraInterface],
        *,
        queue_capacity: int = 120,
    ) -> None:
        if role not in {"workspace", "wrist"}:
            raise ValueError("role must be workspace or wrist")
        self.role = role
        self._factory = camera_factory
        self._frames: deque[CameraSample] = deque(maxlen=queue_capacity)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"rgbd-{role}", daemon=True)
        self._camera: CameraInterface | None = None
        self._error: BaseException | None = None
        self._received = 0
        self._dropped = 0
        self._previous_numbers: tuple[int, int] | None = None
        self._receive_times: deque[int] = deque(maxlen=90)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)
        camera = self._camera
        close = getattr(camera, "close", None)
        if callable(close):
            close()

    @property
    def error(self) -> BaseException | None:
        with self._lock:
            return self._error

    @property
    def dropped_frames(self) -> int:
        with self._lock:
            return self._dropped

    @property
    def actual_fps(self) -> float:
        with self._lock:
            if len(self._receive_times) < 2:
                return 0.0
            elapsed = (self._receive_times[-1] - self._receive_times[0]) / 1e9
            return 0.0 if elapsed <= 0.0 else (len(self._receive_times) - 1) / elapsed

    def latest(self) -> CameraSample | None:
        with self._lock:
            return None if not self._frames else self._frames[-1]

    def frames_after(self, host_monotonic_ns: int) -> list[CameraSample]:
        with self._lock:
            return [frame for frame in self._frames if frame.host_monotonic_ns > host_monotonic_ns]

    def profile_metadata(self) -> dict[str, object]:
        camera = self._camera
        getter = getattr(camera, "profile_metadata", None)
        profile = getter() if callable(getter) else {}
        return {
            **profile,
            "role": self.role,
            "actual_fps": self.actual_fps,
            "dropped_frame_count": self.dropped_frames,
        }

    def _run(self) -> None:
        try:
            self._camera = self._factory()
            while not self._stop.is_set():
                frame = self._camera.capture()
                sample = camera_sample_from_rgbd(self.role, frame)
                with self._lock:
                    numbers = (sample.rgb_frame_number, sample.depth_frame_number)
                    if self._previous_numbers is not None:
                        color_gap = max(numbers[0] - self._previous_numbers[0] - 1, 0)
                        depth_gap = max(numbers[1] - self._previous_numbers[1] - 1, 0)
                        self._dropped += max(color_gap, depth_gap)
                    self._previous_numbers = numbers
                    self._frames.append(sample)
                    self._receive_times.append(sample.host_monotonic_ns)
                    self._received += 1
        except BaseException as exc:
            with self._lock:
                self._error = exc


def camera_sample_from_rgbd(role: str, frame: RGBDFrame) -> CameraSample:
    if frame.depth_raw_units is None:
        depth_scale = frame.depth_scale_m
        if depth_scale is None or depth_scale <= 0:
            raise ValueError("RGBDFrame does not preserve raw depth or declare depth scale")
        raw = np.rint(frame.depth_m / depth_scale).astype(np.uint16)
    else:
        raw = frame.depth_raw_units.copy()
    host_monotonic_ns = frame.host_monotonic_ns
    if host_monotonic_ns is None:
        # Mock/legacy sources are permitted for offline tests, but are stamped
        # at the adapter boundary rather than pretending wall time is monotonic.
        host_monotonic_ns = time.monotonic_ns()
    return CameraSample(
        role=role,
        host_monotonic_ns=host_monotonic_ns,
        rgb=frame.rgb.copy(),
        depth_raw=raw,
        depth_aligned_to_rgb=(
            None
            if frame.depth_aligned_to_color_units is None
            else frame.depth_aligned_to_color_units.copy()
        ),
        device_rgb_timestamp_ms=frame.color_timestamp_ms,
        device_depth_timestamp_ms=frame.depth_timestamp_ms,
        rgb_frame_number=frame.color_frame_number,
        depth_frame_number=frame.depth_frame_number,
        rgb_timestamp_domain=frame.color_timestamp_domain,
        depth_timestamp_domain=frame.depth_timestamp_domain,
    )
