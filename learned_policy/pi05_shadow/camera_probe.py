#!/usr/bin/env python3
"""Read-only dual-camera timing probe for the OpenPI shadow pipeline.

This module only opens V4L2 video-capture devices. It has no robot imports and
contains no actuator or command path.
"""

from __future__ import annotations

import argparse
import dataclasses
import datetime as dt
import json
import pathlib
import statistics
import threading
import time
from typing import Any

import cv2
import numpy as np


@dataclasses.dataclass(frozen=True)
class CameraSpec:
    role: str
    serial: str
    device: str
    rotate_degrees: int = 0


SCENE_CAMERA = CameraSpec(
    role="scene",
    serial="315223123328",
    device=(
        "/dev/v4l/by-id/"
        "usb-Intel_R__RealSense_TM__Depth_Camera_435_"
        "Intel_R__RealSense_TM__Depth_Camera_435_315223123328-video-index0"
    ),
)
WRIST_CAMERA = CameraSpec(
    role="wrist",
    serial="315223123181",
    device=(
        "/dev/v4l/by-id/"
        "usb-Intel_R__RealSense_TM__Depth_Camera_435_"
        "Intel_R__RealSense_TM__Depth_Camera_435_315223123181-video-index0"
    ),
    # Verified from the live sample on 2026-07-22: this camera is mounted
    # upside-down in its current wrist installation.
    rotate_degrees=180,
)


@dataclasses.dataclass
class CaptureResult:
    spec: CameraSpec
    negotiated_width: int
    negotiated_height: int
    negotiated_fps: float
    timestamps_ns: list[int]
    sample_bgr: np.ndarray
    read_failures: int


def _rotate(frame: np.ndarray, degrees: int) -> np.ndarray:
    if degrees == 0:
        return frame
    if degrees == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    raise ValueError(f"Unsupported camera rotation: {degrees}; only 0 and 180 are accepted")


def _open(spec: CameraSpec, width: int, height: int, fps: float) -> cv2.VideoCapture:
    path = pathlib.Path(spec.device)
    if not path.exists():
        raise FileNotFoundError(f"{spec.role} camera is unavailable: {path}")
    capture = cv2.VideoCapture(str(path), cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"Could not open {spec.role} camera at {path}")
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    return capture


def _capture_worker(
    spec: CameraSpec,
    capture: cv2.VideoCapture,
    barrier: threading.Barrier,
    deadline_ns: int,
    output: dict[str, CaptureResult | BaseException],
) -> None:
    timestamps: list[int] = []
    sample: np.ndarray | None = None
    read_failures = 0
    try:
        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        barrier.wait()
        while time.monotonic_ns() < deadline_ns:
            ok, frame = capture.read()
            timestamp_ns = time.monotonic_ns()
            if not ok:
                read_failures += 1
                continue
            timestamps.append(timestamp_ns)
            sample = _rotate(frame, spec.rotate_degrees)
        if sample is None:
            raise RuntimeError(f"No frames received from {spec.role} camera")
        output[spec.role] = CaptureResult(
            spec=spec,
            negotiated_width=width,
            negotiated_height=height,
            negotiated_fps=fps,
            timestamps_ns=timestamps,
            sample_bgr=sample,
            read_failures=read_failures,
        )
    except BaseException as exc:  # Propagate worker failures to the caller.
        output[spec.role] = exc
    finally:
        capture.release()


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    return float(np.percentile(np.asarray(values), percentile))


def _camera_metrics(result: CaptureResult) -> dict[str, Any]:
    timestamps = result.timestamps_ns
    intervals_ms = [
        (right - left) / 1_000_000 for left, right in zip(timestamps, timestamps[1:], strict=False)
    ]
    elapsed_s = (timestamps[-1] - timestamps[0]) / 1_000_000_000 if len(timestamps) > 1 else 0.0
    actual_fps = (len(timestamps) - 1) / elapsed_s if elapsed_s > 0 else 0.0
    period_ms = 1000.0 / result.negotiated_fps if result.negotiated_fps > 0 else 0.0
    inferred_drops = 0
    if period_ms > 0:
        inferred_drops = sum(max(0, round(interval / period_ms) - 1) for interval in intervals_ms)
    return {
        "role": result.spec.role,
        "serial": result.spec.serial,
        "device": result.spec.device,
        "rotation_degrees": result.spec.rotate_degrees,
        "width": result.negotiated_width,
        "height": result.negotiated_height,
        "reported_fps": result.negotiated_fps,
        "measured_fps": actual_fps,
        "frames": len(timestamps),
        "read_failures": result.read_failures,
        "inferred_dropped_frames": inferred_drops,
        "inter_frame_ms_p50": statistics.median(intervals_ms) if intervals_ms else 0.0,
        "inter_frame_ms_p95": _percentile(intervals_ms, 95),
        "inter_frame_ms_max": max(intervals_ms, default=0.0),
    }


def capture_synchronized(
    scene: CameraSpec = SCENE_CAMERA,
    wrist: CameraSpec = WRIST_CAMERA,
    *,
    duration_s: float = 5.0,
    width: int = 848,
    height: int = 480,
    fps: float = 30.0,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    if duration_s <= 0:
        raise ValueError("duration_s must be positive")
    captures = {
        scene.role: _open(scene, width, height, fps),
        wrist.role: _open(wrist, width, height, fps),
    }
    barrier = threading.Barrier(3)
    output: dict[str, CaptureResult | BaseException] = {}
    # Include a short lead-in so both workers enter read() after the barrier.
    deadline_ns = time.monotonic_ns() + int((duration_s + 0.25) * 1_000_000_000)
    threads = [
        threading.Thread(
            target=_capture_worker,
            args=(spec, captures[spec.role], barrier, deadline_ns, output),
            name=f"{spec.role}-camera-probe",
        )
        for spec in (scene, wrist)
    ]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=duration_s + 5.0)
        if thread.is_alive():
            raise TimeoutError(f"{thread.name} did not stop")
    for role in (scene.role, wrist.role):
        value = output.get(role)
        if isinstance(value, BaseException):
            raise value
        if value is None:
            raise RuntimeError(f"Missing capture result for {role}")
    scene_result = output[scene.role]
    wrist_result = output[wrist.role]
    assert isinstance(scene_result, CaptureResult)
    assert isinstance(wrist_result, CaptureResult)

    # Pair each scene frame with the temporally nearest wrist frame. Timestamps
    # are host CLOCK_MONOTONIC samples taken immediately after capture.read().
    wrist_times = np.asarray(wrist_result.timestamps_ns, dtype=np.int64)
    skews_ms: list[float] = []
    for scene_time in scene_result.timestamps_ns:
        insertion = int(np.searchsorted(wrist_times, scene_time))
        candidates = wrist_times[max(0, insertion - 1) : min(len(wrist_times), insertion + 1)]
        if candidates.size:
            skews_ms.append(float(np.min(np.abs(candidates - scene_time))) / 1_000_000)

    report = {
        "schema": "embodied_lab.pi05_dual_camera_probe.v1",
        "captured_at": dt.datetime.now(dt.timezone.utc).isoformat(),
        "duration_requested_s": duration_s,
        "timestamp_source": "host_monotonic_ns_after_cv2_read",
        "cameras": {
            "scene": _camera_metrics(scene_result),
            "wrist": _camera_metrics(wrist_result),
        },
        "pairing": {
            "pairs": len(skews_ms),
            "timestamp_skew_ms_p50": statistics.median(skews_ms) if skews_ms else 0.0,
            "timestamp_skew_ms_p95": _percentile(skews_ms, 95),
            "timestamp_skew_ms_max": max(skews_ms, default=0.0),
        },
    }
    frames = {
        "scene": scene_result.sample_bgr,
        "wrist": wrist_result.sample_bgr,
    }
    return report, frames


def save_probe(output_dir: pathlib.Path, report: dict[str, Any], frames: dict[str, np.ndarray]) -> None:
    output_dir.mkdir(parents=True, exist_ok=False)
    for role, frame in frames.items():
        if not cv2.imwrite(str(output_dir / f"{role}.jpg"), frame):
            raise RuntimeError(f"Failed to save {role} sample")
    combined = np.concatenate([frames["scene"], frames["wrist"]], axis=1)
    if not cv2.imwrite(str(output_dir / "scene_wrist.jpg"), combined):
        raise RuntimeError("Failed to save combined camera sample")
    (output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--scene-device", default=SCENE_CAMERA.device)
    parser.add_argument("--wrist-device", default=WRIST_CAMERA.device)
    parser.add_argument("--output-dir", type=pathlib.Path)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    scene = dataclasses.replace(SCENE_CAMERA, device=args.scene_device)
    wrist = dataclasses.replace(WRIST_CAMERA, device=args.wrist_device)
    report, frames = capture_synchronized(
        scene,
        wrist,
        duration_s=args.duration_s,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )
    output_dir = args.output_dir
    if output_dir is None:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = pathlib.Path(__file__).parent / "artifacts" / f"camera_probe_{stamp}"
    save_probe(output_dir, report, frames)
    print(json.dumps({**report, "output_dir": str(output_dir.resolve())}, indent=2))


if __name__ == "__main__":
    main()
