from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from vision_interface.depth_processing import depth_quality_summary, depth_to_point_cloud
from vision_interface.realsense_adapter import RealSenseCamera, list_realsense_devices


def _viewer_filter_config(profile: str) -> dict[str, object]:
    """Return an explicit depth-filter policy for live inspection.

    The moving-object profile intentionally avoids temporal persistence and
    global hole filling.  The static profile enables temporal smoothing for a
    stationary calibration target, but still leaves global hole filling off so
    invalid pixels remain observable.
    """

    if profile not in {"raw", "spatial", "static"}:
        raise ValueError(
            f"unknown filter profile {profile!r}; expected raw, spatial, or static"
        )
    return {
        "use_disparity": profile != "raw",
        "spatial": {
            "enabled": profile in {"spatial", "static"},
            "magnitude": 2,
            "smooth_alpha": 0.5,
            "smooth_delta": 20,
            "hole_fill": 0,
        },
        "temporal": {
            "enabled": profile == "static",
            "smooth_alpha": 0.4,
            "smooth_delta": 20,
            "persistence_control": 3,
        },
        "hole_filling": {"enabled": False, "mode": 1},
    }


def _make_preview_panel(
    rgb: np.ndarray,
    depth_m: np.ndarray,
    *,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    """Create an RGB/depth panel with a fixed metric depth scale.

    A fixed range makes frames comparable.  Zero, non-finite, and out-of-range
    depth remains black instead of being filled or normalized away.
    """

    rgb_array = np.asarray(rgb)
    depth_array = np.asarray(depth_m, dtype=np.float32)
    if rgb_array.ndim != 3 or rgb_array.shape[2] != 3:
        raise ValueError(f"rgb must have shape (H, W, 3), got {rgb_array.shape}")
    if depth_array.shape != rgb_array.shape[:2]:
        raise ValueError(
            "depth_m must match the RGB height and width: "
            f"rgb={rgb_array.shape[:2]}, depth={depth_array.shape}"
        )
    if not np.isfinite(depth_min_m) or not np.isfinite(depth_max_m):
        raise ValueError("depth preview bounds must be finite")
    if depth_min_m < 0.0 or depth_max_m <= depth_min_m:
        raise ValueError("expected 0 <= depth_min_m < depth_max_m")

    valid = (
        np.isfinite(depth_array)
        & (depth_array >= depth_min_m)
        & (depth_array <= depth_max_m)
    )
    normalized = np.zeros(depth_array.shape, dtype=np.float32)
    normalized[valid] = (
        depth_array[valid] - depth_min_m
    ) / (depth_max_m - depth_min_m)
    # A small, dependency-free blue-to-red map. Invalid pixels stay black.
    depth_rgb = np.zeros((*depth_array.shape, 3), dtype=np.uint8)
    depth_rgb[..., 0][valid] = np.rint(255.0 * normalized[valid]).astype(np.uint8)
    depth_rgb[..., 1][valid] = np.rint(
        255.0 * (1.0 - np.abs(2.0 * normalized[valid] - 1.0))
    ).astype(np.uint8)
    depth_rgb[..., 2][valid] = np.rint(
        255.0 * (1.0 - normalized[valid])
    ).astype(np.uint8)
    return np.concatenate((rgb_array.astype(np.uint8, copy=False), depth_rgb), axis=1)


def check_realsense_stream(
    *,
    duration_sec: float,
    width: int,
    height: int,
    fps: int,
    serial: str | None,
    snapshot_dir: str,
    filter_profile: str = "spatial",
    preview: bool = False,
    preview_depth_min_m: float = 0.15,
    preview_depth_max_m: float = 1.5,
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
        "filters": _viewer_filter_config(filter_profile),
    }
    if serial:
        config["serial"] = serial

    cv2 = None
    if preview:
        try:
            import cv2 as cv2_module
        except ImportError as exc:
            raise RuntimeError(
                "live preview requires OpenCV; install the development extra"
            ) from exc
        cv2 = cv2_module

    try:
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
                if cv2 is not None:
                    panel_rgb = _make_preview_panel(
                        frame.rgb,
                        frame.depth_m,
                        depth_min_m=preview_depth_min_m,
                        depth_max_m=preview_depth_max_m,
                    )
                    cv2.imshow(
                        "RealSense RGB | depth (press q to stop)",
                        panel_rgb[..., ::-1],
                    )
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            elapsed = max(time.time() - start, 1e-6)
    finally:
        if cv2 is not None:
            cv2.destroyAllWindows()

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
        "filter_profile": filter_profile,
        "snapshot_rgb": snapshot_rgb,
        "snapshot_depth_m": snapshot_depth,
        "snapshot_point_cloud": snapshot_cloud,
        "snapshot_metadata": snapshot_metadata,
        "error": "" if frames > 0 else "no frames captured",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Check an Intel RealSense RGB-D stream through vision_interface.")
    parser.add_argument(
        "--list-devices",
        action="store_true",
        help="enumerate RealSense identities and exit without starting a stream",
    )
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--snapshot-dir", default="data/reports/realsense")
    parser.add_argument(
        "--filter-profile",
        choices=["raw", "spatial", "static"],
        default="spatial",
        help="depth-filter policy; use static only for a stationary target",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="show a local RGB/depth window; press q to end early",
    )
    parser.add_argument("--preview-depth-min-m", type=float, default=0.15)
    parser.add_argument("--preview-depth-max-m", type=float, default=1.5)
    args = parser.parse_args()
    if args.list_devices:
        print(json.dumps(list_realsense_devices(), indent=2))
        return
    if not args.serial:
        parser.error("--serial is required to start a stream; use --list-devices first")
    result = check_realsense_stream(
        duration_sec=args.duration_sec,
        width=args.width,
        height=args.height,
        fps=args.fps,
        serial=args.serial,
        snapshot_dir=args.snapshot_dir,
        filter_profile=args.filter_profile,
        preview=args.preview,
        preview_depth_min_m=args.preview_depth_min_m,
        preview_depth_max_m=args.preview_depth_max_m,
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
