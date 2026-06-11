from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from vision_interface.realsense_adapter import RealSenseCamera, list_realsense_devices


def check_realsense_stream(
    *,
    duration_sec: float,
    width: int,
    height: int,
    fps: int,
    serial: str | None,
    snapshot_dir: str,
) -> dict[str, object]:
    devices = list_realsense_devices()
    frames = 0
    first_rgb_shape: list[int] | None = None
    first_depth_shape: list[int] | None = None
    first_depth_m_range: list[float] | None = None
    last_rgb: np.ndarray | None = None
    last_depth: np.ndarray | None = None
    intrinsics: dict[str, object] | None = None

    config: dict[str, object] = {
        "width": width,
        "height": height,
        "fps": fps,
        "warmup_frames": 5,
    }
    if serial:
        config["serial"] = serial

    with RealSenseCamera(config) as camera:
        intrinsics = camera.get_intrinsics().to_dict()
        start = time.time()
        while time.time() - start < duration_sec:
            rgb = camera.get_rgb()
            depth = camera.get_depth()
            frames += 1
            last_rgb = rgb
            last_depth = depth
            if first_rgb_shape is None:
                first_rgb_shape = list(rgb.shape)
                first_depth_shape = list(depth.shape)
                valid_depth = depth[depth > 0.0]
                if valid_depth.size:
                    first_depth_m_range = [float(valid_depth.min()), float(valid_depth.max())]

    out_dir = Path(snapshot_dir)
    snapshot_rgb = ""
    snapshot_depth = ""
    if last_rgb is not None and last_depth is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        snapshot_rgb = str((out_dir / "realsense_rgb.npy").resolve())
        snapshot_depth = str((out_dir / "realsense_depth_m.npy").resolve())
        np.save(snapshot_rgb, last_rgb)
        np.save(snapshot_depth, last_depth)

    elapsed = max(time.time() - start, 1e-6)
    return {
        "ok": frames > 0,
        "devices": devices,
        "frames": frames,
        "duration_sec": elapsed,
        "observed_fps": frames / elapsed,
        "rgb_shape": first_rgb_shape,
        "depth_shape": first_depth_shape,
        "depth_m_range": first_depth_m_range,
        "intrinsics": intrinsics,
        "snapshot_rgb": snapshot_rgb,
        "snapshot_depth_m": snapshot_depth,
        "error": "" if frames > 0 else "no frames captured",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Check an Intel RealSense RGB-D stream through vision_interface.")
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--snapshot-dir", default="data/reports/realsense")
    args = parser.parse_args()
    result = check_realsense_stream(
        duration_sec=args.duration_sec,
        width=args.width,
        height=args.height,
        fps=args.fps,
        serial=args.serial,
        snapshot_dir=args.snapshot_dir,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
