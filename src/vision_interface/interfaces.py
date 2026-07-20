from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from embodiment_core.types import CameraIntrinsics


@dataclass(frozen=True, slots=True)
class RGBDFrame:
    """One synchronized RGB-D frameset.

    ``intrinsics`` always describes the pixel geometry of ``depth_m``. When
    ``depth_aligned_to_color`` is true, it therefore contains color intrinsics.
    Device timestamps remain in milliseconds because hardware-clock timestamps
    are not necessarily Unix time.
    """

    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: CameraIntrinsics
    host_timestamp_s: float
    color_timestamp_ms: float
    depth_timestamp_ms: float
    color_timestamp_domain: str
    depth_timestamp_domain: str
    color_frame_number: int
    depth_frame_number: int
    depth_aligned_to_color: bool

    @property
    def timestamps_comparable(self) -> bool:
        return self.color_timestamp_domain == self.depth_timestamp_domain

    @property
    def timestamp_skew_ms(self) -> float:
        if not self.timestamps_comparable:
            return float("nan")
        return abs(self.color_timestamp_ms - self.depth_timestamp_ms)


class CameraInterface(ABC):
    @abstractmethod
    def capture(self) -> RGBDFrame:
        raise NotImplementedError

    def get_rgb(self) -> np.ndarray:
        frame = self.capture()
        self._compat_frame = frame
        return frame.rgb.copy()

    def get_depth(self) -> np.ndarray:
        frame = getattr(self, "_compat_frame", None)
        if frame is None:
            frame = self.capture()
        self._compat_frame = None
        return frame.depth_m.copy()

    def get_intrinsics(self) -> CameraIntrinsics:
        frame = getattr(self, "_last_frame", None)
        if frame is None:
            frame = self.capture()
        return frame.intrinsics

    def get_timestamp(self) -> float:
        frame = getattr(self, "_last_frame", None)
        if frame is None:
            frame = self.capture()
        return float(frame.host_timestamp_s)
