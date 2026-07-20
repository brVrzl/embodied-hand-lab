from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from vision_interface.depth_processing import depth_quality_summary, depth_to_point_cloud
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
    last_frame = None
    intrinsics: dict[str, object] | None = None
    timestamp_skews_ms: list[float] = []

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
            frame = camera.capture()
            frames += 1
            last_frame = frame
            if np.isfinite(frame.timestamp_skew_ms):
                timestamp_skews_ms.append(frame.timestamp_skew_ms)
            if first_rgb_shape is None:
                first_rgb_shape = list(frame.rgb.shape)
                first_depth_shape = list(frame.depth_m.shape)
        elapsed = max(time.time() - start, 1e-6)

    out_dir = Path(snapshot_dir)
    snapshot_rgb = ""
    snapshot_depth = ""
    snapshot_cloud = ""
    snapshot_metadata = ""
    quality: dict[str, float | int] = {}
    if last_frame is not None:
        out_dir.mkdir(parents=True, exist_ok=True)
        snapshot_rgb = str((out_dir / "realsense_rgb.npy").resolve())
        snapshot_depth = str((out_dir / "realsense_depth_m.npy").resolve())
        snapshot_cloud = str((out_dir / "realsense_point_cloud.npz").resolve())
        snapshot_metadata = str((out_dir / "realsense_frame.json").resolve())
        np.save(snapshot_rgb, last_frame.rgb)
        np.save(snapshot_depth, last_frame.depth_m)
        cloud = depth_to_point_cloud(
            last_frame.depth_m,
            last_frame.intrinsics,
            rgb=last_frame.rgb if last_frame.depth_aligned_to_color else None,
            min_depth_m=0.15,
            max_depth_m=3.0,
        )
        cloud_payload = {"points_m": cloud.points_m}
        if cloud.colors_rgb is not None:
            cloud_payload["colors_rgb"] = cloud.colors_rgb
        np.savez_compressed(snapshot_cloud, **cloud_payload)
        quality = depth_quality_summary(last_frame.depth_m, min_depth_m=0.15, max_depth_m=3.0)
        frame_skew_ms = last_frame.timestamp_skew_ms
        frame_metadata = {
            "intrinsics": last_frame.intrinsics.to_dict(),
            "host_timestamp_s": last_frame.host_timestamp_s,
            "color_timestamp_ms": last_frame.color_timestamp_ms,
            "depth_timestamp_ms": last_frame.depth_timestamp_ms,
            "color_timestamp_domain": last_frame.color_timestamp_domain,
            "depth_timestamp_domain": last_frame.depth_timestamp_domain,
            "color_frame_number": last_frame.color_frame_number,
            "depth_frame_number": last_frame.depth_frame_number,
            "timestamp_skew_ms": float(frame_skew_ms) if np.isfinite(frame_skew_ms) else None,
            "depth_aligned_to_color": last_frame.depth_aligned_to_color,
            "depth_quality": quality,
            "point_count": len(cloud),
        }
        Path(snapshot_metadata).write_text(json.dumps(frame_metadata, indent=2), encoding="utf-8")

    return {
        "ok": frames > 0,
        "devices": devices,
        "frames": frames,
        "duration_sec": elapsed,
        "observed_fps": frames / elapsed,
        "rgb_shape": first_rgb_shape,
        "depth_shape": first_depth_shape,
        "depth_quality": quality,
        "timestamp_skew_ms_mean": float(np.mean(timestamp_skews_ms)) if timestamp_skews_ms else None,
        "timestamp_skew_ms_max": float(np.max(timestamp_skews_ms)) if timestamp_skews_ms else None,
        "intrinsics": intrinsics,
        "snapshot_rgb": snapshot_rgb,
        "snapshot_depth_m": snapshot_depth,
        "snapshot_point_cloud": snapshot_cloud,
        "snapshot_metadata": snapshot_metadata,
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
