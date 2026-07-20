from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from embodiment_core.config import load_yaml
from embodiment_core.types import CameraIntrinsics
from vision_interface.depth_processing import TabletopConfig, process_tabletop_frame
from vision_interface.interfaces import RGBDFrame


def process_saved_frame(
    *,
    depth_path: str | Path,
    metadata_path: str | Path,
    output_dir: str | Path,
    config_path: str | Path,
    rgb_path: str | Path | None = None,
    target_from_camera_path: str | Path | None = None,
    allow_camera_frame: bool = False,
    camera_up_axis: list[float] | None = None,
) -> dict[str, Any]:
    config = load_yaml(config_path)
    metadata = json.loads(Path(metadata_path).read_text(encoding="utf-8"))
    intrinsics_data = metadata.get("intrinsics", metadata)
    if not isinstance(intrinsics_data, Mapping):
        raise ValueError("Frame metadata must contain an 'intrinsics' mapping.")
    intrinsics = _intrinsics_from_mapping(intrinsics_data)
    depth = np.load(Path(depth_path), allow_pickle=False)
    if rgb_path is None:
        rgb = np.zeros((*depth.shape, 3), dtype=np.uint8)
        has_registered_rgb = False
    else:
        rgb = np.load(Path(rgb_path), allow_pickle=False)
        has_registered_rgb = bool(metadata.get("depth_aligned_to_color", True))

    coordinates = _mapping(config.get("coordinates"), "coordinates")
    configured_transform = coordinates.get("target_from_camera_npy")
    transform_path = target_from_camera_path or configured_transform
    if transform_path:
        target_from_camera = np.load(Path(transform_path), allow_pickle=False)
        target_frame_id = str(coordinates.get("target_frame_id", "jaka_base"))
        up_axis = coordinates.get("up_axis", [0.0, 0.0, 1.0])
    elif allow_camera_frame:
        if camera_up_axis is None:
            raise ValueError("--camera-up-axis is required with --allow-camera-frame.")
        target_from_camera = None
        target_frame_id = None
        up_axis = camera_up_axis
    else:
        raise ValueError(
            "Tabletop segmentation requires calibrated target_from_camera. Provide "
            "--target-from-camera, or explicitly use --allow-camera-frame with --camera-up-axis "
            "for diagnostics only."
        )

    frame = RGBDFrame(
        rgb=rgb,
        depth_m=depth,
        intrinsics=intrinsics,
        host_timestamp_s=float(metadata.get("host_timestamp_s", 0.0)),
        color_timestamp_ms=float(metadata.get("color_timestamp_ms", 0.0)),
        depth_timestamp_ms=float(metadata.get("depth_timestamp_ms", 0.0)),
        color_timestamp_domain=str(metadata.get("color_timestamp_domain", "unknown")),
        depth_timestamp_domain=str(metadata.get("depth_timestamp_domain", "unknown")),
        color_frame_number=int(metadata.get("color_frame_number", -1)),
        depth_frame_number=int(metadata.get("depth_frame_number", -1)),
        depth_aligned_to_color=has_registered_rgb,
    )
    workspace = _mapping(config.get("workspace"), "workspace")
    workspace_min = workspace.get("min_xyz_m")
    workspace_max = workspace.get("max_xyz_m")
    result = process_tabletop_frame(
        frame,
        up_axis=up_axis,
        config=_tabletop_config(config),
        target_from_camera=target_from_camera,
        target_frame_id=target_frame_id,
        workspace_min_xyz_m=workspace_min,
        workspace_max_xyz_m=workspace_max,
    )

    out_dir = Path(output_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_path = out_dir / "scene_cloud.npz"
    object_path = out_dir / "object_points_m.npy"
    report_path = out_dir / "tabletop_report.json"
    cloud_payload = {
        "points_m": result.scene.points_m,
        "table_points_m": result.table.points_m,
        "object_points_m": result.objects.points_m,
    }
    if result.scene.colors_rgb is not None:
        cloud_payload["colors_rgb"] = result.scene.colors_rgb
    np.savez_compressed(scene_path, **cloud_payload)
    np.save(object_path, result.objects.points_m)
    report = {
        "frame_id": result.scene.frame_id,
        "diagnostic_camera_frame_only": target_from_camera is None,
        "scene_point_count": len(result.scene),
        "table_point_count": len(result.table),
        "object_point_count": len(result.objects),
        "plane": {
            "normal": result.plane.normal.tolist(),
            "offset_m": result.plane.offset_m,
            "inlier_count": result.plane.inlier_count,
            "rmse_m": result.plane.rmse_m,
        },
        "depth_quality": result.depth_quality,
        "scene_cloud": str(scene_path),
        "object_points_m": str(object_path),
    }
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    report["report"] = str(report_path)
    return report


def _intrinsics_from_mapping(data: Mapping[str, Any]) -> CameraIntrinsics:
    return CameraIntrinsics(
        width=int(data["width"]),
        height=int(data["height"]),
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        frame_id=str(data["frame_id"]),
        distortion_model=str(data.get("distortion_model", "none")),
        distortion_coefficients=[float(value) for value in data.get("distortion_coefficients", [])],
    )


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping.")
    return value


def _tabletop_config(config: Mapping[str, Any]) -> TabletopConfig:
    depth = _mapping(config.get("depth"), "depth")
    point_cloud = _mapping(config.get("point_cloud"), "point_cloud")
    tabletop = _mapping(config.get("tabletop"), "tabletop")
    return TabletopConfig(
        min_depth_m=float(depth.get("min_m", 0.15)),
        max_depth_m=float(depth.get("max_m", 1.5)),
        pixel_stride=int(depth.get("pixel_stride", 1)),
        voxel_size_m=float(point_cloud.get("voxel_size_m", 0.005)),
        plane_distance_threshold_m=float(tabletop.get("plane_distance_threshold_m", 0.008)),
        plane_max_iterations=int(tabletop.get("plane_max_iterations", 300)),
        plane_min_inlier_ratio=float(tabletop.get("plane_min_inlier_ratio", 0.2)),
        plane_max_tilt_deg=float(tabletop.get("plane_max_tilt_deg", 25.0)),
        object_min_height_m=float(tabletop.get("object_min_height_m", 0.01)),
        object_max_height_m=float(tabletop.get("object_max_height_m", 0.30)),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract a table plane and above-table points from a saved RGB-D frame.")
    parser.add_argument("--depth", required=True, help="Depth image .npy in metres.")
    parser.add_argument("--metadata", required=True, help="Frame JSON written by check_realsense_stream.py.")
    parser.add_argument("--rgb", default=None, help="Optional registered RGB .npy.")
    parser.add_argument("--config", default="configs/perception/d435_tabletop.yaml")
    parser.add_argument("--target-from-camera", default=None, help="Calibrated 4x4 target_from_camera .npy.")
    parser.add_argument("--allow-camera-frame", action="store_true", help="Diagnostic mode without robot-base extrinsics.")
    parser.add_argument("--camera-up-axis", nargs=3, type=float, default=None)
    parser.add_argument("--output-dir", default="data/reports/tabletop")
    args = parser.parse_args()
    report = process_saved_frame(
        depth_path=args.depth,
        metadata_path=args.metadata,
        output_dir=args.output_dir,
        config_path=args.config,
        rgb_path=args.rgb,
        target_from_camera_path=args.target_from_camera,
        allow_camera_frame=args.allow_camera_frame,
        camera_up_axis=args.camera_up_axis,
    )
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
