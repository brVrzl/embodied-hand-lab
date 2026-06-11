from __future__ import annotations

import importlib
import time
from types import ModuleType
from typing import Any

import numpy as np

from embodiment_core.types import CameraIntrinsics

from .interfaces import CameraInterface


class RealSenseCamera(CameraInterface):
    def __init__(self, config: dict | None = None) -> None:
        rs = _load_pyrealsense2()
        self.config = config or {}
        self.width = int(self.config.get("width", 640))
        self.height = int(self.config.get("height", 480))
        self.fps = int(self.config.get("fps", 30))
        self.frame_id = str(self.config.get("frame_id", "camera_color_optical_frame"))
        self.align_depth_to_color = bool(self.config.get("align_depth_to_color", True))
        self.serial = self.config.get("serial")
        self.warmup_frames = int(self.config.get("warmup_frames", 5))
        self.timeout_ms = int(self.config.get("timeout_ms", 5000))
        self.rs = rs

        self.pipeline = rs.pipeline()
        pipeline_config = rs.config()
        if self.serial:
            pipeline_config.enable_device(str(self.serial))
        pipeline_config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        pipeline_config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.profile = self.pipeline.start(pipeline_config)
        self.align = rs.align(rs.stream.color) if self.align_depth_to_color else None
        self.depth_scale = _get_depth_scale(rs, self.profile)
        self._intrinsics = _get_color_intrinsics(rs, self.profile, self.frame_id)
        self._last_rgb: np.ndarray | None = None
        self._last_depth: np.ndarray | None = None
        self._last_timestamp = 0.0
        self._closed = False

        for _ in range(max(0, self.warmup_frames)):
            self._read_frames()

    def _read_frames(self) -> None:
        if self._closed:
            raise RuntimeError("RealSenseCamera is closed.")
        frames = self.pipeline.wait_for_frames(self.timeout_ms)
        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense frameset did not contain both color and depth frames.")

        self._last_rgb = np.asanyarray(color_frame.get_data()).copy()
        self._last_depth = (np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale).copy()
        self._last_timestamp = time.time()

    def get_rgb(self) -> np.ndarray:
        self._read_frames()
        if self._last_rgb is None:
            raise RuntimeError("RealSense RGB frame is unavailable.")
        return self._last_rgb.copy()

    def get_depth(self) -> np.ndarray:
        if self._last_depth is None:
            self._read_frames()
        if self._last_depth is None:
            raise RuntimeError("RealSense depth frame is unavailable.")
        return self._last_depth.copy()

    def get_intrinsics(self) -> CameraIntrinsics:
        return self._intrinsics

    def get_timestamp(self) -> float:
        return self._last_timestamp

    def close(self) -> None:
        if not getattr(self, "_closed", True):
            self.pipeline.stop()
            self._closed = True

    def __enter__(self) -> RealSenseCamera:
        return self

    def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass


def _load_pyrealsense2() -> ModuleType:
    try:
        return importlib.import_module("pyrealsense2")
    except ImportError as exc:
        raise ImportError(
            "RealSenseCamera requires the optional pyrealsense2 package. "
            "Install it with: python3 -m pip install pyrealsense2"
        ) from exc


def _get_depth_scale(rs: ModuleType, profile: Any) -> float:
    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()
    return float(depth_sensor.get_depth_scale())


def _get_color_intrinsics(rs: ModuleType, profile: Any, frame_id: str) -> CameraIntrinsics:
    stream_profile = profile.get_stream(rs.stream.color)
    video_profile = stream_profile.as_video_stream_profile()
    intrinsics = video_profile.get_intrinsics()
    return CameraIntrinsics(
        width=int(intrinsics.width),
        height=int(intrinsics.height),
        fx=float(intrinsics.fx),
        fy=float(intrinsics.fy),
        cx=float(intrinsics.ppx),
        cy=float(intrinsics.ppy),
        frame_id=frame_id,
    )


def list_realsense_devices() -> list[dict[str, str]]:
    rs = _load_pyrealsense2()
    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": _device_info(rs, device, rs.camera_info.name),
                "serial": _device_info(rs, device, rs.camera_info.serial_number),
                "usb_type": _device_info(rs, device, rs.camera_info.usb_type_descriptor),
                "firmware": _device_info(rs, device, rs.camera_info.firmware_version),
            }
        )
    return devices


def _device_info(rs: ModuleType, device: Any, info_key: Any) -> str:
    try:
        return str(device.get_info(info_key))
    except Exception:
        return ""
