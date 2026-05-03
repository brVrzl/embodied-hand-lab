from __future__ import annotations

from .interfaces import CameraInterface


class RealSenseCamera(CameraInterface):
    def __init__(self, config: dict) -> None:
        self.config = config
        raise NotImplementedError(
            "此处为待替换适配点: RealSense backend requires librealsense/ROS2 driver integration."
        )

