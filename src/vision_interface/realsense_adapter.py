from __future__ import annotations

import importlib
import importlib.metadata
import platform
import time
from pathlib import Path
from types import ModuleType
from typing import Any, Mapping

import numpy as np

from embodiment_core.config import load_yaml
from embodiment_core.types import CameraIntrinsics

from .interfaces import CameraInterface, RGBDFrame


class RealSenseCamera(CameraInterface):
    """Intel RealSense RGB-D source with an atomic frameset API."""

    def __init__(self, config: Mapping[str, Any] | None = None) -> None:
        self.rs = _load_pyrealsense2()
        self.config = dict(config or {})
        self.width = _positive_int(self.config.get("width", 640), "width")
        self.height = _positive_int(self.config.get("height", 480), "height")
        self.fps = _positive_int(self.config.get("fps", 30), "fps")
        self.align_depth_to_color = bool(self.config.get("align_depth_to_color", True))
        self.serial = self.config.get("serial")
        self.allow_profile_fallback = bool(self.config.get("allow_profile_fallback", False))
        self.warmup_frames = max(int(self.config.get("warmup_frames", 5)), 0)
        self.timeout_ms = _positive_int(self.config.get("timeout_ms", 5000), "timeout_ms")
        max_skew = self.config.get("max_timestamp_skew_ms", 50.0)
        self.max_timestamp_skew_ms = None if max_skew is None else float(max_skew)
        if self.max_timestamp_skew_ms is not None and self.max_timestamp_skew_ms < 0.0:
            raise ValueError("max_timestamp_skew_ms must be non-negative or null.")
        self.sync_retry_frames = max(int(self.config.get("sync_retry_frames", 30)), 0)
        frames = self.config.get("frames", {})
        frames = frames if isinstance(frames, Mapping) else {}
        self.color_frame_id = str(
            self.config.get("frame_id", frames.get("rgb_optical", "camera_color_optical_frame"))
        )
        self.depth_frame_id = str(frames.get("depth_optical", "camera_depth_optical_frame"))

        self.pipeline = self.rs.pipeline()
        pipeline_config = self.rs.config()
        if self.serial:
            pipeline_config.enable_device(str(self.serial))
        color_profile = (self.width, self.height, self.fps)
        depth_profile = (self.width, self.height, self.fps)
        if self.allow_profile_fallback:
            if not self.serial:
                raise ValueError("allow_profile_fallback requires an explicit RealSense serial")
            color_profile, depth_profile = _select_device_profiles(
                self.rs,
                serial=str(self.serial),
                desired_width=self.width,
                desired_height=self.height,
                desired_fps=self.fps,
            )
        pipeline_config.enable_stream(
            self.rs.stream.color,
            color_profile[0],
            color_profile[1],
            self.rs.format.rgb8,
            color_profile[2],
        )
        pipeline_config.enable_stream(
            self.rs.stream.depth,
            depth_profile[0],
            depth_profile[1],
            self.rs.format.z16,
            depth_profile[2],
        )
        self.profile = self.pipeline.start(pipeline_config)
        self.align = self.rs.align(self.rs.stream.color) if self.align_depth_to_color else None
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        _configure_depth_sensor(self.rs, self.depth_sensor, self.config)
        self.depth_scale = _get_depth_scale(self.depth_sensor)
        self.depth_filters = _build_depth_filters(self.rs, self.config.get("filters", {}))
        self._last_frame: RGBDFrame | None = None
        self._compat_frame: RGBDFrame | None = None
        self._closed = False

        for _ in range(self.warmup_frames):
            self.capture()

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        *,
        camera_name: str | None = None,
    ) -> RealSenseCamera:
        return cls(resolve_realsense_config(load_yaml(path), camera_name=camera_name))

    def capture(self) -> RGBDFrame:
        if self._closed:
            raise RuntimeError("RealSenseCamera is closed.")

        last_frame: RGBDFrame | None = None
        for _ in range(self.sync_retry_frames + 1):
            frame = self._capture_once()
            last_frame = frame
            if self.max_timestamp_skew_ms is None or not frame.timestamps_comparable:
                self._last_frame = frame
                return frame
            if frame.timestamp_skew_ms <= self.max_timestamp_skew_ms:
                self._last_frame = frame
                return frame
        assert last_frame is not None
        raise RuntimeError(
            "RealSense RGB/depth timestamp skew remained above "
            f"{self.max_timestamp_skew_ms:.3f} ms after {self.sync_retry_frames + 1} frames; "
            f"last skew was {last_frame.timestamp_skew_ms:.3f} ms."
        )

    def _capture_once(self) -> RGBDFrame:
        frameset = self.pipeline.wait_for_frames(self.timeout_ms)
        host_monotonic_ns = time.monotonic_ns()
        host_wall_timestamp_ns = time.time_ns()
        raw_color_frame = frameset.get_color_frame()
        raw_depth_frame = frameset.get_depth_frame()
        if not raw_color_frame or not raw_depth_frame:
            raise RuntimeError("RealSense frameset did not contain both color and depth frames.")
        depth_raw_units = np.asanyarray(raw_depth_frame.get_data()).copy()
        if depth_raw_units.dtype != np.uint16:
            raise RuntimeError(f"Unexpected RealSense raw depth dtype: {depth_raw_units.dtype}.")
        depth_aligned_units = None
        if self.align is not None:
            frameset = self.align.process(frameset)
        color_frame = frameset.get_color_frame()
        depth_frame = frameset.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("RealSense frameset did not contain both color and depth frames.")

        if self.align is not None:
            depth_aligned_units = np.asanyarray(depth_frame.get_data()).copy()
            if depth_aligned_units.dtype != np.uint16:
                raise RuntimeError(
                    f"Unexpected RealSense aligned depth dtype: {depth_aligned_units.dtype}."
                )

        for depth_filter in self.depth_filters:
            depth_frame = depth_filter.process(depth_frame)

        rgb = np.asanyarray(color_frame.get_data()).copy()
        depth_m = (
            np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        ).copy()
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise RuntimeError(f"Unexpected RealSense RGB shape: {rgb.shape}.")
        if depth_m.ndim != 2:
            raise RuntimeError(f"Unexpected RealSense depth shape: {depth_m.shape}.")

        frame_id = self.color_frame_id if self.align_depth_to_color else self.depth_frame_id
        intrinsics = _get_frame_intrinsics(depth_frame, frame_id)
        if depth_m.shape != (intrinsics.height, intrinsics.width):
            raise RuntimeError(
                "Depth image shape does not match its intrinsics: "
                f"image={depth_m.shape}, intrinsics={(intrinsics.height, intrinsics.width)}."
            )

        frame = RGBDFrame(
            rgb=rgb,
            depth_m=depth_m,
            intrinsics=intrinsics,
            host_timestamp_s=host_wall_timestamp_ns / 1e9,
            color_timestamp_ms=_frame_timestamp_ms(raw_color_frame),
            depth_timestamp_ms=_frame_timestamp_ms(raw_depth_frame),
            color_timestamp_domain=_frame_timestamp_domain(raw_color_frame),
            depth_timestamp_domain=_frame_timestamp_domain(raw_depth_frame),
            color_frame_number=_frame_number(raw_color_frame),
            depth_frame_number=_frame_number(raw_depth_frame),
            depth_aligned_to_color=self.align_depth_to_color,
            host_monotonic_ns=host_monotonic_ns,
            host_wall_timestamp_ns=host_wall_timestamp_ns,
            depth_raw_units=depth_raw_units,
            depth_aligned_to_color_units=depth_aligned_units,
            serial_number=str(self.serial) if self.serial else None,
            depth_scale_m=self.depth_scale,
        )
        return frame

    def profile_metadata(self) -> dict[str, Any]:
        """Return an immutable-friendly snapshot of the active device/profile."""

        device = self.profile.get_device()
        color = self.profile.get_stream(self.rs.stream.color).as_video_stream_profile()
        depth = self.profile.get_stream(self.rs.stream.depth).as_video_stream_profile()
        color_intrinsics = _intrinsics_dict(color.get_intrinsics())
        depth_intrinsics = _intrinsics_dict(depth.get_intrinsics())
        extrinsics = depth.get_extrinsics_to(color)
        return {
            "serial_number": _device_info(device, self.rs.camera_info.serial_number),
            "name": _device_info(device, self.rs.camera_info.name),
            "firmware_version": _device_info(device, self.rs.camera_info.firmware_version),
            "usb_type": _device_info(device, self.rs.camera_info.usb_type_descriptor),
            "librealsense_python_version": _distribution_version("pyrealsense2"),
            "host_platform": platform.platform(),
            "depth_scale_m": self.depth_scale,
            "align_depth_to_color": self.align_depth_to_color,
            "raw_depth_preserved": True,
            "color": _video_profile_dict(color, color_intrinsics),
            "depth_raw": _video_profile_dict(depth, depth_intrinsics),
            "depth_to_color_extrinsics": {
                "rotation_row_major": [float(value) for value in extrinsics.rotation],
                "translation_m": [float(value) for value in extrinsics.translation],
            },
        }

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


def resolve_realsense_config(
    config: Mapping[str, Any],
    *,
    camera_name: str | None = None,
) -> dict[str, Any]:
    """Resolve one device from either a flat or a multi-camera YAML mapping."""

    resolved = {key: value for key, value in config.items() if key != "cameras"}
    cameras = config.get("cameras")
    if not isinstance(cameras, Mapping):
        if camera_name is not None:
            raise ValueError("camera_name was provided but the config has no 'cameras' mapping.")
        return resolved

    if camera_name is None:
        if len(cameras) != 1:
            names = ", ".join(str(name) for name in cameras)
            raise ValueError(f"camera_name is required for multi-camera config; available: {names}.")
        camera_name = str(next(iter(cameras)))
    if camera_name not in cameras:
        names = ", ".join(str(name) for name in cameras)
        raise KeyError(f"Unknown camera {camera_name!r}; available: {names}.")
    camera_config = cameras[camera_name]
    if not isinstance(camera_config, Mapping):
        raise ValueError(f"Camera config {camera_name!r} must be a mapping.")
    resolved.update(camera_config)
    resolved["camera_name"] = camera_name
    return resolved


def _load_pyrealsense2() -> ModuleType:
    try:
        return importlib.import_module("pyrealsense2")
    except ImportError as exc:
        raise ImportError(
            "RealSenseCamera requires the optional pyrealsense2 package. "
            "Install it with: python3 -m pip install pyrealsense2"
        ) from exc


def _positive_int(value: Any, name: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise ValueError(f"{name} must be positive, got {parsed}.")
    return parsed


def _get_depth_scale(depth_sensor: Any) -> float:
    scale = float(depth_sensor.get_depth_scale())
    if not np.isfinite(scale) or scale <= 0.0:
        raise RuntimeError(f"Invalid RealSense depth scale: {scale}.")
    return scale


def _get_frame_intrinsics(frame: Any, frame_id: str) -> CameraIntrinsics:
    profile = frame.get_profile() if hasattr(frame, "get_profile") else frame.profile
    video_profile = profile.as_video_stream_profile()
    intrinsics = video_profile.get_intrinsics()
    model = str(getattr(intrinsics, "model", "none"))
    if "." in model:
        model = model.rsplit(".", 1)[-1]
    return CameraIntrinsics(
        width=int(intrinsics.width),
        height=int(intrinsics.height),
        fx=float(intrinsics.fx),
        fy=float(intrinsics.fy),
        cx=float(intrinsics.ppx),
        cy=float(intrinsics.ppy),
        frame_id=frame_id,
        distortion_model=model,
        distortion_coefficients=[float(value) for value in getattr(intrinsics, "coeffs", [])],
    )


def _frame_timestamp_ms(frame: Any) -> float:
    if hasattr(frame, "get_timestamp"):
        return float(frame.get_timestamp())
    return float(frame.timestamp)


def _frame_timestamp_domain(frame: Any) -> str:
    if hasattr(frame, "get_frame_timestamp_domain"):
        domain = frame.get_frame_timestamp_domain()
    else:
        domain = frame.frame_timestamp_domain
    text = str(domain)
    return text.rsplit(".", 1)[-1]


def _frame_number(frame: Any) -> int:
    if hasattr(frame, "get_frame_number"):
        return int(frame.get_frame_number())
    return int(frame.frame_number)


def _build_depth_filters(rs: ModuleType, config: Any) -> list[Any]:
    if not isinstance(config, Mapping):
        raise ValueError("filters must be a mapping.")
    spatial = config.get("spatial", {})
    temporal = config.get("temporal", {})
    spatial_enabled = _filter_enabled(spatial)
    temporal_enabled = _filter_enabled(temporal)
    use_disparity = bool(config.get("use_disparity", True)) and (
        spatial_enabled or temporal_enabled
    )
    filters: list[Any] = []
    if use_disparity:
        filters.append(rs.disparity_transform(True))
    if spatial_enabled:
        filters.append(
            rs.spatial_filter(
                float(spatial.get("smooth_alpha", 0.5)),
                float(spatial.get("smooth_delta", 20.0)),
                int(spatial.get("magnitude", 2)),
                int(spatial.get("hole_fill", 0)),
            )
        )
    if temporal_enabled:
        filters.append(
            rs.temporal_filter(
                float(temporal.get("smooth_alpha", 0.4)),
                float(temporal.get("smooth_delta", 20.0)),
                int(temporal.get("persistence_control", 3)),
            )
        )
    if use_disparity:
        filters.append(rs.disparity_transform(False))
    hole_filling = config.get("hole_filling", {})
    if _filter_enabled(hole_filling):
        filters.append(rs.hole_filling_filter(int(hole_filling.get("mode", 1))))
    return filters


def _configure_depth_sensor(rs: ModuleType, sensor: Any, config: Mapping[str, Any]) -> None:
    preset = config.get("visual_preset")
    if preset is None:
        return
    preset_values = {
        "custom": 0.0,
        "default": 1.0,
        "hand": 2.0,
        "high_accuracy": 3.0,
        "high_density": 4.0,
        "medium_density": 5.0,
    }
    if isinstance(preset, str):
        key = preset.strip().lower().replace("-", "_").replace(" ", "_")
        if key not in preset_values:
            names = ", ".join(preset_values)
            raise ValueError(f"Unknown visual_preset {preset!r}; expected one of: {names}.")
        value = preset_values[key]
    else:
        value = float(preset)
    option = rs.option.visual_preset
    if hasattr(sensor, "supports") and not sensor.supports(option):
        raise RuntimeError("The selected RealSense depth sensor does not support visual presets.")
    sensor.set_option(option, value)


def _filter_enabled(config: Any) -> bool:
    return isinstance(config, Mapping) and bool(config.get("enabled", False))


def list_realsense_devices() -> list[dict[str, str]]:
    rs = _load_pyrealsense2()
    devices = []
    for device in rs.context().query_devices():
        devices.append(
            {
                "name": _device_info(device, rs.camera_info.name),
                "serial": _device_info(device, rs.camera_info.serial_number),
                "usb_type": _device_info(device, rs.camera_info.usb_type_descriptor),
                "firmware": _device_info(device, rs.camera_info.firmware_version),
            }
        )
    return devices


def _device_info(device: Any, info_key: Any) -> str:
    try:
        return str(device.get_info(info_key))
    except Exception:
        return ""


def _distribution_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def choose_closest_profile(
    profiles: list[tuple[int, int, int]], *, width: int, height: int, fps: int
) -> tuple[int, int, int]:
    """Choose a stable-rate profile without silently preferring resolution over FPS."""

    if not profiles:
        raise RuntimeError("device exposes no compatible stream profiles")
    unique = sorted(set(profiles))
    return min(
        unique,
        key=lambda item: (
            abs(item[2] - fps),
            0 if item[2] == fps else 1,
            abs(item[0] - width) + abs(item[1] - height),
            abs(item[0] * item[1] - width * height),
        ),
    )


def _select_device_profiles(
    rs: ModuleType, *, serial: str, desired_width: int, desired_height: int, desired_fps: int
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    devices = list(rs.context().query_devices())
    device = next(
        (
            item
            for item in devices
            if _device_info(item, rs.camera_info.serial_number) == serial
        ),
        None,
    )
    if device is None:
        available = [_device_info(item, rs.camera_info.serial_number) for item in devices]
        raise RuntimeError(f"RealSense serial {serial!r} not found; available serials: {available}")
    color_profiles: list[tuple[int, int, int]] = []
    depth_profiles: list[tuple[int, int, int]] = []
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            try:
                video = profile.as_video_stream_profile()
                candidate = (int(video.width()), int(video.height()), int(video.fps()))
            except Exception:
                continue
            if profile.stream_type() == rs.stream.color and profile.format() == rs.format.rgb8:
                color_profiles.append(candidate)
            elif profile.stream_type() == rs.stream.depth and profile.format() == rs.format.z16:
                depth_profiles.append(candidate)
    color = choose_closest_profile(
        color_profiles, width=desired_width, height=desired_height, fps=desired_fps
    )
    depth = choose_closest_profile(
        depth_profiles, width=desired_width, height=desired_height, fps=desired_fps
    )
    return color, depth


def _intrinsics_dict(intrinsics: Any) -> dict[str, Any]:
    return {
        "width": int(intrinsics.width),
        "height": int(intrinsics.height),
        "fx": float(intrinsics.fx),
        "fy": float(intrinsics.fy),
        "cx": float(intrinsics.ppx),
        "cy": float(intrinsics.ppy),
        "distortion_model": str(intrinsics.model).rsplit(".", 1)[-1],
        "distortion_coefficients": [float(value) for value in intrinsics.coeffs],
    }


def _video_profile_dict(profile: Any, intrinsics: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "width": int(profile.width()),
        "height": int(profile.height()),
        "nominal_fps": int(profile.fps()),
        "format": str(profile.format()).rsplit(".", 1)[-1],
        "intrinsics": dict(intrinsics),
    }
