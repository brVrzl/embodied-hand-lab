from __future__ import annotations

import time

import numpy as np

from embodiment_core.types import CameraIntrinsics

from .interfaces import CameraInterface


class MockRGBDCamera(CameraInterface):
    def __init__(self, width: int = 64, height: int = 48, frame_id: str = "camera_color_optical_frame") -> None:
        self.width = width
        self.height = height
        self.frame_id = frame_id

    def get_rgb(self) -> np.ndarray:
        x = np.linspace(0, 255, self.width, dtype=np.uint8)
        y = np.linspace(0, 255, self.height, dtype=np.uint8)
        xv, yv = np.meshgrid(x, y)
        return np.stack([xv, yv, np.full_like(xv, 127)], axis=-1)

    def get_depth(self) -> np.ndarray:
        depth = np.linspace(0.4, 1.2, self.width * self.height, dtype=np.float32)
        return depth.reshape((self.height, self.width))

    def get_intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(
            width=self.width,
            height=self.height,
            fx=385.0,
            fy=385.0,
            cx=self.width / 2.0,
            cy=self.height / 2.0,
            frame_id=self.frame_id,
        )

    def get_timestamp(self) -> float:
        return time.time()


MockRGDBCamera = MockRGBDCamera
