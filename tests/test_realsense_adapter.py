from __future__ import annotations

import sys
from types import SimpleNamespace

import numpy as np
import pytest

from vision_interface.realsense_adapter import RealSenseCamera, list_realsense_devices


class _FakeFrame:
    def __init__(self, data: np.ndarray) -> None:
        self._data = data

    def get_data(self) -> np.ndarray:
        return self._data


class _FakeFrames:
    def __init__(self, rgb: np.ndarray, depth: np.ndarray) -> None:
        self.rgb = _FakeFrame(rgb)
        self.depth = _FakeFrame(depth)

    def get_color_frame(self) -> _FakeFrame:
        return self.rgb

    def get_depth_frame(self) -> _FakeFrame:
        return self.depth


class _FakePipeline:
    stopped = False

    def start(self, config: object) -> "_FakeProfile":
        return _FakeProfile()

    def wait_for_frames(self, timeout_ms: int) -> _FakeFrames:
        rgb = np.zeros((2, 3, 3), dtype=np.uint8)
        depth = np.array([[1000, 1200, 0], [800, 900, 1100]], dtype=np.uint16)
        return _FakeFrames(rgb, depth)

    def stop(self) -> None:
        _FakePipeline.stopped = True


class _FakeConfig:
    def enable_device(self, serial: str) -> None:
        self.serial = serial

    def enable_stream(self, *args: object) -> None:
        pass


class _FakeAlign:
    def __init__(self, stream: object) -> None:
        pass

    def process(self, frames: _FakeFrames) -> _FakeFrames:
        return frames


class _FakeDepthSensor:
    def get_depth_scale(self) -> float:
        return 0.001


class _FakeDevice:
    def first_depth_sensor(self) -> _FakeDepthSensor:
        return _FakeDepthSensor()

    def get_info(self, key: str) -> str:
        return {
            "name": "Intel RealSense D435",
            "serial": "123456",
            "usb": "3.2",
            "firmware": "5.0",
        }[key]


class _FakeProfile:
    def get_device(self) -> _FakeDevice:
        return _FakeDevice()

    def get_stream(self, stream: object) -> "_FakeStreamProfile":
        return _FakeStreamProfile()


class _FakeStreamProfile:
    def as_video_stream_profile(self) -> "_FakeVideoProfile":
        return _FakeVideoProfile()


class _FakeVideoProfile:
    def get_intrinsics(self) -> SimpleNamespace:
        return SimpleNamespace(width=3, height=2, fx=100.0, fy=101.0, ppx=1.5, ppy=1.0)


class _FakeContext:
    def query_devices(self) -> list[_FakeDevice]:
        return [_FakeDevice()]


def _fake_rs() -> SimpleNamespace:
    return SimpleNamespace(
        pipeline=_FakePipeline,
        config=_FakeConfig,
        align=_FakeAlign,
        context=_FakeContext,
        stream=SimpleNamespace(color="color", depth="depth"),
        format=SimpleNamespace(rgb8="rgb8", z16="z16"),
        camera_info=SimpleNamespace(
            name="name",
            serial_number="serial",
            usb_type_descriptor="usb",
            firmware_version="firmware",
        ),
    )


def test_realsense_camera_reads_rgb_depth_intrinsics(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "pyrealsense2", _fake_rs())
    camera = RealSenseCamera({"width": 3, "height": 2, "fps": 30, "warmup_frames": 0})

    rgb = camera.get_rgb()
    depth = camera.get_depth()
    intrinsics = camera.get_intrinsics()
    camera.close()

    assert rgb.shape == (2, 3, 3)
    assert rgb.dtype == np.uint8
    assert depth.shape == (2, 3)
    assert depth.dtype == np.float32
    np.testing.assert_allclose(depth[0, 0], 1.0)
    assert intrinsics.width == 3
    assert intrinsics.height == 2
    assert intrinsics.fx == 100.0
    assert _FakePipeline.stopped


def test_list_realsense_devices(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "pyrealsense2", _fake_rs())

    assert list_realsense_devices() == [
        {
            "name": "Intel RealSense D435",
            "serial": "123456",
            "usb_type": "3.2",
            "firmware": "5.0",
        }
    ]
