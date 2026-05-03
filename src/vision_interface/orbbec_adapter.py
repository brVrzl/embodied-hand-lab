from __future__ import annotations

from .interfaces import CameraInterface


class OrbbecCamera(CameraInterface):
    def __init__(self, config: dict) -> None:
        self.config = config
        raise NotImplementedError(
            "此处为待替换适配点: Orbbec backend requires official SDK or ROS2 driver integration."
        )

