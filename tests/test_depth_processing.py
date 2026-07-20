from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from embodiment_core.types import CameraIntrinsics
from vision_interface.depth_processing import (
    PointCloud,
    TabletopConfig,
    crop_point_cloud,
    depth_quality_summary,
    depth_to_point_cloud,
    fit_plane_ransac,
    guided_depth_completion,
    process_tabletop_frame,
    remove_radius_outliers,
    remove_statistical_outliers,
    transform_point_cloud,
    voxel_downsample,
)
from vision_interface.interfaces import RGBDFrame
from tools.process_rgbd_tabletop import process_saved_frame


def _intrinsics(width: int = 4, height: int = 3) -> CameraIntrinsics:
    return CameraIntrinsics(
        width=width,
        height=height,
        fx=2.0,
        fy=2.0,
        cx=1.0,
        cy=1.0,
        frame_id="camera_color_optical_frame",
    )


def test_depth_to_point_cloud_filters_and_deprojects_pixels() -> None:
    depth = np.array(
        [[1.0, 0.0, np.nan, 3.0], [1.0, 2.0, 1.0, 1.0], [0.1, 1.0, 1.0, 1.0]],
        dtype=np.float32,
    )
    rgb = np.arange(3 * 4 * 3, dtype=np.uint8).reshape(3, 4, 3)

    cloud = depth_to_point_cloud(
        depth,
        _intrinsics(),
        rgb=rgb,
        min_depth_m=0.5,
        max_depth_m=2.0,
    )

    assert len(cloud) == 8
    first = np.flatnonzero(np.all(cloud.pixels_xy == [0, 0], axis=1))[0]
    np.testing.assert_allclose(cloud.points_m[first], [-0.5, -0.5, 1.0])
    np.testing.assert_array_equal(cloud.colors_rgb[first], rgb[0, 0])


def test_depth_to_point_cloud_rejects_unrectified_pixels() -> None:
    intrinsics = _intrinsics()
    intrinsics.distortion_coefficients = [0.1, 0.0, 0.0, 0.0, 0.0]

    with pytest.raises(ValueError, match="rectified"):
        depth_to_point_cloud(np.ones((3, 4), dtype=np.float32), intrinsics)


def test_transform_crop_and_voxel_downsample_preserve_metric_geometry() -> None:
    cloud = PointCloud(
        points_m=np.array([[0.001, 0.0, 0.0], [0.004, 0.0, 0.0], [0.02, 0.0, 0.0]], dtype=np.float32),
        frame_id="camera",
        colors_rgb=np.array([[10, 20, 30], [30, 40, 50], [200, 210, 220]], dtype=np.uint8),
    )
    transform = np.eye(4)
    transform[:3, 3] = [1.0, 2.0, 3.0]

    transformed = transform_point_cloud(cloud, transform, target_frame_id="jaka_base")
    cropped = crop_point_cloud(transformed, [1.0, 1.9, 2.9], [1.01, 2.1, 3.1])
    reduced = voxel_downsample(cropped, 0.01)

    assert transformed.frame_id == "jaka_base"
    assert len(cropped) == 2
    assert len(reduced) == 1
    np.testing.assert_allclose(reduced.points_m[0], [1.0025, 2.0, 3.0], atol=1e-6)
    np.testing.assert_array_equal(reduced.colors_rgb[0], [20, 30, 40])


def test_fit_plane_ransac_uses_up_axis_and_rejects_outliers() -> None:
    rng = np.random.default_rng(4)
    xy = rng.uniform(-0.5, 0.5, size=(500, 2))
    plane_points = np.column_stack((xy, 0.3 + rng.normal(0.0, 0.001, size=500)))
    outliers = rng.uniform(-0.5, 0.5, size=(80, 3))
    points = np.vstack((plane_points, outliers))

    plane, inliers = fit_plane_ransac(
        points,
        distance_threshold_m=0.004,
        min_inliers=400,
        normal_hint=[0.0, 0.0, 1.0],
        max_normal_angle_deg=10.0,
        random_seed=2,
    )

    assert plane.normal @ np.array([0.0, 0.0, 1.0]) > 0.999
    assert abs(plane.offset_m + 0.3) < 0.002
    assert plane.rmse_m < 0.002
    assert np.count_nonzero(inliers[:500]) > 490
    assert np.count_nonzero(inliers[500:]) < 10


def test_process_tabletop_frame_extracts_points_above_plane() -> None:
    height, width = 60, 80
    depth = np.ones((height, width), dtype=np.float32)
    depth[24:36, 34:46] = 0.90
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    rgb[..., 1] = 100
    intrinsics = CameraIntrinsics(
        width=width,
        height=height,
        fx=100.0,
        fy=100.0,
        cx=(width - 1) / 2.0,
        cy=(height - 1) / 2.0,
        frame_id="camera_color_optical_frame",
    )
    frame = RGBDFrame(
        rgb=rgb,
        depth_m=depth,
        intrinsics=intrinsics,
        host_timestamp_s=1.0,
        color_timestamp_ms=1000.0,
        depth_timestamp_ms=1000.2,
        color_timestamp_domain="global_time",
        depth_timestamp_domain="global_time",
        color_frame_number=1,
        depth_frame_number=1,
        depth_aligned_to_color=True,
    )
    config = TabletopConfig(
        min_depth_m=0.5,
        max_depth_m=1.2,
        voxel_size_m=0.004,
        plane_distance_threshold_m=0.005,
        object_min_height_m=0.05,
        object_max_height_m=0.15,
    )

    result = process_tabletop_frame(frame, up_axis=[0.0, 0.0, -1.0], config=config)

    assert result.plane.normal @ np.array([0.0, 0.0, -1.0]) > 0.999
    assert abs(result.plane.offset_m - 1.0) < 0.005
    assert len(result.table) > 1000
    assert len(result.objects) > 20
    assert np.median(result.objects.points_m[:, 2]) == pytest.approx(0.9, abs=0.01)
    assert result.depth_quality["valid_ratio"] == 1.0


def test_depth_quality_summary_counts_only_configured_range() -> None:
    depth = np.array([[0.0, 0.5], [1.0, np.nan]], dtype=np.float32)

    summary = depth_quality_summary(depth, min_depth_m=0.2, max_depth_m=0.8)

    assert summary["total_pixels"] == 4
    assert summary["valid_pixels"] == 1
    assert summary["valid_ratio"] == 0.25
    assert summary["median_m"] == 0.5


def test_guided_depth_completion_fills_small_holes_without_changing_measurements() -> None:
    depth = np.ones((31, 41), dtype=np.float32)
    depth[:, 21:] = 1.4
    depth[14:17, 18:20] = 0.0
    depth[14:17, 22:24] = 0.0
    rgb = np.zeros((31, 41, 3), dtype=np.uint8)
    rgb[:, :21] = [30, 30, 30]
    rgb[:, 21:] = [230, 230, 230]
    valid = depth > 0.0

    completed = guided_depth_completion(depth, rgb, radius_px=4, epsilon=1e-4)

    np.testing.assert_array_equal(completed[valid], depth[valid])
    assert np.all(completed[14:17, 18:20] < 1.1)
    assert np.all(completed[14:17, 22:24] > 1.3)


def test_point_cloud_outlier_filters_remove_isolated_points() -> None:
    xx, yy = np.meshgrid(np.arange(8), np.arange(8))
    cluster = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size))) * 0.003
    points = np.vstack((cluster, [[0.5, 0.5, 0.5], [-0.5, -0.5, 0.2]])).astype(np.float32)
    cloud = PointCloud(points_m=points, frame_id="camera")

    statistical = remove_statistical_outliers(cloud, mean_k=8, std_ratio=1.0)
    radius = remove_radius_outliers(cloud, radius_m=0.005, min_neighbors=2)

    assert len(statistical) == len(cluster)
    assert len(radius) == len(cluster)
    assert np.max(np.linalg.norm(statistical.points_m, axis=1)) < 0.1
    assert np.max(np.linalg.norm(radius.points_m, axis=1)) < 0.1


def test_saved_frame_tool_requires_extrinsics_and_writes_object_cloud(tmp_path: Path) -> None:
    height, width = 40, 50
    depth = np.ones((height, width), dtype=np.float32)
    depth[15:25, 20:30] = 1.10
    rgb = np.zeros((height, width, 3), dtype=np.uint8)
    depth_path = tmp_path / "depth.npy"
    rgb_path = tmp_path / "rgb.npy"
    metadata_path = tmp_path / "frame.json"
    transform_path = tmp_path / "target_from_camera.npy"
    np.save(depth_path, depth)
    np.save(rgb_path, rgb)
    np.save(transform_path, np.eye(4))
    metadata_path.write_text(
        json.dumps(
            {
                "intrinsics": {
                    "width": width,
                    "height": height,
                    "fx": 80.0,
                    "fy": 80.0,
                    "cx": (width - 1) / 2.0,
                    "cy": (height - 1) / 2.0,
                    "frame_id": "camera_color_optical_frame",
                },
                "depth_aligned_to_color": True,
            }
        ),
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="requires calibrated"):
        process_saved_frame(
            depth_path=depth_path,
            rgb_path=rgb_path,
            metadata_path=metadata_path,
            output_dir=tmp_path / "rejected",
            config_path="configs/perception/d435_tabletop.yaml",
        )

    report = process_saved_frame(
        depth_path=depth_path,
        rgb_path=rgb_path,
        metadata_path=metadata_path,
        output_dir=tmp_path / "output",
        config_path="configs/perception/d435_tabletop.yaml",
        target_from_camera_path=transform_path,
    )

    assert report["frame_id"] == "jaka_base"
    assert report["diagnostic_camera_frame_only"] is False
    assert report["object_point_count"] > 20
    assert Path(report["object_points_m"]).exists()
    assert Path(report["report"]).exists()
