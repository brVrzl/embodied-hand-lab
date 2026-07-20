from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from embodiment_core.types import CameraIntrinsics

from .interfaces import RGBDFrame


@dataclass(frozen=True, slots=True)
class PointCloud:
    points_m: np.ndarray
    frame_id: str
    colors_rgb: np.ndarray | None = None
    pixels_xy: np.ndarray | None = None

    def __post_init__(self) -> None:
        points = np.asarray(self.points_m)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"points_m must have shape (N, 3), got {points.shape}.")
        if not np.issubdtype(points.dtype, np.floating):
            raise ValueError("points_m must use a floating dtype.")
        if self.colors_rgb is not None and np.asarray(self.colors_rgb).shape != points.shape:
            raise ValueError("colors_rgb must have shape (N, 3).")
        if self.pixels_xy is not None and np.asarray(self.pixels_xy).shape != (len(points), 2):
            raise ValueError("pixels_xy must have shape (N, 2).")

    def __len__(self) -> int:
        return int(self.points_m.shape[0])

    def subset(self, selection: np.ndarray) -> PointCloud:
        selection = np.asarray(selection)
        return PointCloud(
            points_m=self.points_m[selection].copy(),
            frame_id=self.frame_id,
            colors_rgb=None if self.colors_rgb is None else self.colors_rgb[selection].copy(),
            pixels_xy=None if self.pixels_xy is None else self.pixels_xy[selection].copy(),
        )


@dataclass(frozen=True, slots=True)
class PlaneModel:
    normal: np.ndarray
    offset_m: float
    inlier_count: int
    rmse_m: float

    def signed_distance(self, points_m: np.ndarray) -> np.ndarray:
        points = np.asarray(points_m, dtype=np.float64)
        return points @ self.normal + self.offset_m


@dataclass(frozen=True, slots=True)
class TabletopConfig:
    min_depth_m: float = 0.15
    max_depth_m: float = 1.5
    pixel_stride: int = 1
    voxel_size_m: float = 0.005
    plane_distance_threshold_m: float = 0.008
    plane_max_iterations: int = 300
    plane_min_inlier_ratio: float = 0.2
    plane_max_tilt_deg: float = 25.0
    object_min_height_m: float = 0.01
    object_max_height_m: float = 0.30


@dataclass(frozen=True, slots=True)
class TabletopResult:
    scene: PointCloud
    table: PointCloud
    objects: PointCloud
    plane: PlaneModel
    depth_quality: dict[str, float | int]


def depth_quality_summary(
    depth_m: np.ndarray,
    *,
    min_depth_m: float = 0.15,
    max_depth_m: float = 1.5,
) -> dict[str, float | int]:
    depth = _validate_depth(depth_m)
    valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)
    values = depth[valid]
    summary: dict[str, float | int] = {
        "total_pixels": int(depth.size),
        "valid_pixels": int(values.size),
        "valid_ratio": float(values.size / max(depth.size, 1)),
    }
    for key, value in (
        ("min_m", np.min(values) if values.size else np.nan),
        ("median_m", np.median(values) if values.size else np.nan),
        ("p95_m", np.percentile(values, 95) if values.size else np.nan),
        ("max_m", np.max(values) if values.size else np.nan),
    ):
        summary[key] = float(value)
    return summary


def depth_to_point_cloud(
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    rgb: np.ndarray | None = None,
    mask: np.ndarray | None = None,
    min_depth_m: float = 0.15,
    max_depth_m: float = 1.5,
    stride: int = 1,
) -> PointCloud:
    """Deproject a depth image using the pinhole model.

    Inputs and output use metres. Non-zero distortion coefficients are rejected
    because silently applying a pinhole model to distorted pixels biases metric
    tabletop geometry.
    """

    depth = _validate_depth(depth_m)
    _validate_intrinsics(intrinsics, depth.shape)
    if stride <= 0:
        raise ValueError("stride must be positive.")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("Expected 0 <= min_depth_m < max_depth_m.")
    coefficients = np.asarray(intrinsics.distortion_coefficients, dtype=np.float64)
    if coefficients.size and np.any(np.abs(coefficients) > 1e-12):
        raise ValueError(
            "depth_to_point_cloud requires rectified pixels or zero distortion coefficients."
        )

    sampled_depth = depth[::stride, ::stride]
    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= min_depth_m)
        & (sampled_depth <= max_depth_m)
    )
    if mask is not None:
        mask_array = np.asarray(mask, dtype=bool)
        if mask_array.shape != depth.shape:
            raise ValueError(f"mask shape {mask_array.shape} does not match depth {depth.shape}.")
        valid &= mask_array[::stride, ::stride]

    y_px = np.arange(0, depth.shape[0], stride, dtype=np.int32)
    x_px = np.arange(0, depth.shape[1], stride, dtype=np.int32)
    xx, yy = np.meshgrid(x_px, y_px)
    z = sampled_depth[valid].astype(np.float64, copy=False)
    x = (xx[valid].astype(np.float64) - intrinsics.cx) * z / intrinsics.fx
    y = (yy[valid].astype(np.float64) - intrinsics.cy) * z / intrinsics.fy
    points = np.column_stack((x, y, z)).astype(np.float32, copy=False)
    pixels = np.column_stack((xx[valid], yy[valid])).astype(np.int32, copy=False)

    colors = None
    if rgb is not None:
        rgb_array = np.asarray(rgb)
        expected_shape = (*depth.shape, 3)
        if rgb_array.shape != expected_shape:
            raise ValueError(f"rgb shape {rgb_array.shape} does not match {expected_shape}.")
        colors = rgb_array[::stride, ::stride][valid].copy()
    return PointCloud(points_m=points, frame_id=intrinsics.frame_id, colors_rgb=colors, pixels_xy=pixels)


def transform_point_cloud(
    cloud: PointCloud,
    target_from_source: np.ndarray,
    *,
    target_frame_id: str,
) -> PointCloud:
    transform = np.asarray(target_from_source, dtype=np.float64)
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise ValueError("target_from_source must be a finite 4x4 matrix.")
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("target_from_source must have homogeneous last row [0, 0, 0, 1].")
    points = cloud.points_m.astype(np.float64) @ transform[:3, :3].T + transform[:3, 3]
    return PointCloud(
        points_m=points.astype(np.float32),
        frame_id=target_frame_id,
        colors_rgb=None if cloud.colors_rgb is None else cloud.colors_rgb.copy(),
        pixels_xy=None if cloud.pixels_xy is None else cloud.pixels_xy.copy(),
    )


def crop_point_cloud(
    cloud: PointCloud,
    min_xyz_m: Sequence[float],
    max_xyz_m: Sequence[float],
) -> PointCloud:
    lower = _xyz_vector(min_xyz_m, "min_xyz_m")
    upper = _xyz_vector(max_xyz_m, "max_xyz_m")
    if np.any(upper <= lower):
        raise ValueError("Each max_xyz_m component must be greater than min_xyz_m.")
    keep = np.all((cloud.points_m >= lower) & (cloud.points_m <= upper), axis=1)
    return cloud.subset(keep)


def voxel_downsample(cloud: PointCloud, voxel_size_m: float) -> PointCloud:
    if not np.isfinite(voxel_size_m) or voxel_size_m <= 0.0:
        raise ValueError("voxel_size_m must be finite and positive.")
    if len(cloud) == 0:
        return cloud.subset(np.zeros(0, dtype=bool))

    voxel_keys = np.floor(cloud.points_m.astype(np.float64) / voxel_size_m).astype(np.int64)
    _, inverse = np.unique(voxel_keys, axis=0, return_inverse=True)
    count = np.bincount(inverse).astype(np.float64)
    points = np.column_stack(
        [np.bincount(inverse, weights=cloud.points_m[:, axis]) / count for axis in range(3)]
    ).astype(np.float32)
    colors = None
    if cloud.colors_rgb is not None:
        colors = np.column_stack(
            [np.bincount(inverse, weights=cloud.colors_rgb[:, axis]) / count for axis in range(3)]
        )
        if np.issubdtype(cloud.colors_rgb.dtype, np.integer):
            colors = np.rint(colors).clip(0, 255).astype(cloud.colors_rgb.dtype)
        else:
            colors = colors.astype(cloud.colors_rgb.dtype)
    return PointCloud(points_m=points, frame_id=cloud.frame_id, colors_rgb=colors)


def guided_depth_completion(
    depth_m: np.ndarray,
    rgb: np.ndarray,
    *,
    radius_px: int = 6,
    epsilon: float = 1e-3,
    min_support: float = 0.08,
    min_depth_m: float = 0.15,
    max_depth_m: float = 1.5,
) -> np.ndarray:
    """Fill nearby depth holes with an RGB-guided normalized filter.

    Existing valid measurements are copied unchanged. The filter only proposes
    values for invalid pixels, so RGB texture cannot rescale trusted metric
    depth. Its finite support also prevents filling large unobserved regions.
    """

    depth = _validate_depth(depth_m).astype(np.float32, copy=False)
    color = np.asarray(rgb)
    if color.shape != (*depth.shape, 3):
        raise ValueError(f"rgb shape {color.shape} does not match {(*depth.shape, 3)}.")
    if radius_px <= 0:
        raise ValueError("radius_px must be positive.")
    if not np.isfinite(epsilon) or epsilon <= 0.0:
        raise ValueError("epsilon must be finite and positive.")
    if not 0.0 < min_support <= 1.0:
        raise ValueError("min_support must be in (0, 1].")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("Expected 0 <= min_depth_m < max_depth_m.")

    guide = color.astype(np.float32)
    if np.issubdtype(color.dtype, np.integer):
        guide /= float(np.iinfo(color.dtype).max)
    else:
        finite_max = float(np.nanmax(guide)) if guide.size else 1.0
        if finite_max > 1.0:
            guide /= 255.0
    # Luminance avoids tripling the integral-image working set at 848x480.
    luminance = 0.299 * guide[..., 0] + 0.587 * guide[..., 1] + 0.114 * guide[..., 2]
    valid = np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)
    valid_float = valid.astype(np.float32)
    weighted_depth = np.where(valid, depth, 0.0)
    numerator = _guided_filter(luminance, weighted_depth, radius_px, epsilon)
    support = _guided_filter(luminance, valid_float, radius_px, epsilon)

    candidate = np.zeros_like(depth)
    fillable = (~valid) & np.isfinite(support) & (support >= min_support)
    np.divide(numerator, support, out=candidate, where=fillable)
    fillable &= (
        np.isfinite(candidate) & (candidate >= min_depth_m) & (candidate <= max_depth_m)
    )
    output = depth.copy()
    output[~np.isfinite(output)] = 0.0
    output[fillable] = candidate[fillable]
    return output


def remove_statistical_outliers(
    cloud: PointCloud,
    *,
    mean_k: int = 20,
    std_ratio: float = 1.5,
) -> PointCloud:
    """Remove points whose mean neighbor distance is globally exceptional."""

    if mean_k <= 0:
        raise ValueError("mean_k must be positive.")
    if not np.isfinite(std_ratio) or std_ratio < 0.0:
        raise ValueError("std_ratio must be finite and non-negative.")
    if len(cloud) <= 2:
        return cloud.subset(np.ones(len(cloud), dtype=bool))

    tree = _make_kd_tree(cloud.points_m)
    neighbors = min(mean_k + 1, len(cloud))
    distances, _ = tree.query(cloud.points_m, k=neighbors, workers=-1)
    if distances.ndim == 1:
        distances = distances[:, None]
    mean_distance = np.mean(distances[:, 1:], axis=1)
    threshold = float(np.mean(mean_distance) + std_ratio * np.std(mean_distance))
    return cloud.subset(mean_distance <= threshold)


def remove_radius_outliers(
    cloud: PointCloud,
    *,
    radius_m: float = 0.015,
    min_neighbors: int = 4,
) -> PointCloud:
    """Remove points without enough other points inside a metric radius."""

    if not np.isfinite(radius_m) or radius_m <= 0.0:
        raise ValueError("radius_m must be finite and positive.")
    if min_neighbors < 0:
        raise ValueError("min_neighbors must be non-negative.")
    if len(cloud) == 0:
        return cloud.subset(np.zeros(0, dtype=bool))

    tree = _make_kd_tree(cloud.points_m)
    counts = tree.query_ball_point(
        cloud.points_m,
        r=radius_m,
        return_length=True,
        workers=-1,
    )
    # query_ball_point includes the query point itself.
    return cloud.subset(np.asarray(counts) >= min_neighbors + 1)


def fit_plane_ransac(
    points_m: np.ndarray,
    *,
    distance_threshold_m: float = 0.008,
    max_iterations: int = 300,
    min_inliers: int = 100,
    normal_hint: Sequence[float] | None = None,
    max_normal_angle_deg: float | None = None,
    random_seed: int = 0,
) -> tuple[PlaneModel, np.ndarray]:
    points = np.asarray(points_m, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError("points_m must be a finite array with shape (N, 3).")
    if len(points) < 3:
        raise ValueError("At least three points are required to fit a plane.")
    if distance_threshold_m <= 0.0 or max_iterations <= 0 or min_inliers < 3:
        raise ValueError("Plane thresholds, iterations, and min_inliers must be positive.")
    if min_inliers > len(points):
        raise ValueError("min_inliers exceeds the number of input points.")

    hint = None if normal_hint is None else _unit_vector(normal_hint, "normal_hint")
    min_hint_dot = -1.0
    if max_normal_angle_deg is not None:
        if hint is None:
            raise ValueError("normal_hint is required when max_normal_angle_deg is set.")
        if not 0.0 <= max_normal_angle_deg < 90.0:
            raise ValueError("max_normal_angle_deg must be in [0, 90).")
        min_hint_dot = float(np.cos(np.deg2rad(max_normal_angle_deg)))

    rng = np.random.default_rng(random_seed)
    best_mask: np.ndarray | None = None
    best_count = 0
    best_error = np.inf
    for _ in range(max_iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm <= 1e-12:
            continue
        normal /= norm
        if hint is not None and float(normal @ hint) < 0.0:
            normal = -normal
        if hint is not None and float(normal @ hint) < min_hint_dot:
            continue
        offset = -float(normal @ sample[0])
        distances = np.abs(points @ normal + offset)
        mask = distances <= distance_threshold_m
        count = int(np.count_nonzero(mask))
        error = float(np.mean(distances[mask])) if count else np.inf
        if count > best_count or (count == best_count and error < best_error):
            best_mask, best_count, best_error = mask, count, error

    if best_mask is None or best_count < min_inliers:
        raise RuntimeError(
            f"No plane reached min_inliers={min_inliers}; best inlier count was {best_count}."
        )

    normal, offset = _refine_plane(points[best_mask], hint)
    if hint is not None and float(normal @ hint) < min_hint_dot:
        raise RuntimeError("Refined plane violates the requested normal constraint.")
    distances = np.abs(points @ normal + offset)
    inliers = distances <= distance_threshold_m
    if int(np.count_nonzero(inliers)) < min_inliers:
        raise RuntimeError("Refined plane no longer satisfies min_inliers.")
    normal, offset = _refine_plane(points[inliers], hint)
    distances = np.abs(points @ normal + offset)
    inliers = distances <= distance_threshold_m
    rmse = float(np.sqrt(np.mean(np.square(distances[inliers]))))
    model = PlaneModel(
        normal=normal,
        offset_m=float(offset),
        inlier_count=int(np.count_nonzero(inliers)),
        rmse_m=rmse,
    )
    return model, inliers


def process_tabletop_frame(
    frame: RGBDFrame,
    *,
    up_axis: Sequence[float],
    config: TabletopConfig | None = None,
    target_from_camera: np.ndarray | None = None,
    target_frame_id: str | None = None,
    workspace_min_xyz_m: Sequence[float] | None = None,
    workspace_max_xyz_m: Sequence[float] | None = None,
    random_seed: int = 0,
) -> TabletopResult:
    cfg = config or TabletopConfig()
    rgb = frame.rgb if frame.depth_aligned_to_color else None
    cloud = depth_to_point_cloud(
        frame.depth_m,
        frame.intrinsics,
        rgb=rgb,
        min_depth_m=cfg.min_depth_m,
        max_depth_m=cfg.max_depth_m,
        stride=cfg.pixel_stride,
    )
    if target_from_camera is not None:
        if not target_frame_id:
            raise ValueError("target_frame_id is required with target_from_camera.")
        cloud = transform_point_cloud(cloud, target_from_camera, target_frame_id=target_frame_id)
    elif target_frame_id is not None and target_frame_id != cloud.frame_id:
        raise ValueError("target_frame_id cannot rename a cloud without target_from_camera.")
    if (workspace_min_xyz_m is None) != (workspace_max_xyz_m is None):
        raise ValueError("Both workspace bounds must be provided together.")
    if workspace_min_xyz_m is not None and workspace_max_xyz_m is not None:
        cloud = crop_point_cloud(cloud, workspace_min_xyz_m, workspace_max_xyz_m)
    scene = voxel_downsample(cloud, cfg.voxel_size_m)
    if len(scene) < 3:
        raise RuntimeError("Too few points remain after depth filtering and workspace cropping.")

    min_inliers = max(3, int(np.ceil(cfg.plane_min_inlier_ratio * len(scene))))
    plane, table_mask = fit_plane_ransac(
        scene.points_m,
        distance_threshold_m=cfg.plane_distance_threshold_m,
        max_iterations=cfg.plane_max_iterations,
        min_inliers=min_inliers,
        normal_hint=up_axis,
        max_normal_angle_deg=cfg.plane_max_tilt_deg,
        random_seed=random_seed,
    )
    signed_height = plane.signed_distance(scene.points_m)
    object_mask = (
        (signed_height >= cfg.object_min_height_m)
        & (signed_height <= cfg.object_max_height_m)
        & ~table_mask
    )
    return TabletopResult(
        scene=scene,
        table=scene.subset(table_mask),
        objects=scene.subset(object_mask),
        plane=plane,
        depth_quality=depth_quality_summary(
            frame.depth_m,
            min_depth_m=cfg.min_depth_m,
            max_depth_m=cfg.max_depth_m,
        ),
    )


def _validate_depth(depth_m: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_m)
    if depth.ndim != 2 or not np.issubdtype(depth.dtype, np.number):
        raise ValueError(f"depth_m must be a numeric HxW array, got {depth.shape}.")
    return depth


def _guided_filter(
    guide: np.ndarray,
    source: np.ndarray,
    radius_px: int,
    epsilon: float,
) -> np.ndarray:
    mean_guide = _box_mean(guide, radius_px)
    mean_source = _box_mean(source, radius_px)
    correlation = _box_mean(guide * source, radius_px)
    variance = _box_mean(guide * guide, radius_px) - mean_guide * mean_guide
    covariance = correlation - mean_guide * mean_source
    a = covariance / (variance + epsilon)
    b = mean_source - a * mean_guide
    return _box_mean(a, radius_px) * guide + _box_mean(b, radius_px)


def _box_mean(array: np.ndarray, radius_px: int) -> np.ndarray:
    source = np.asarray(array, dtype=np.float32)
    padded = np.pad(source, radius_px, mode="reflect")
    integral = np.pad(padded, ((1, 0), (1, 0)), mode="constant")
    integral = np.cumsum(np.cumsum(integral, axis=0), axis=1)
    size = 2 * radius_px + 1
    sums = (
        integral[size:, size:]
        - integral[:-size, size:]
        - integral[size:, :-size]
        + integral[:-size, :-size]
    )
    return sums / float(size * size)


def _make_kd_tree(points_m: np.ndarray) -> Any:
    try:
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise ImportError(
            "Point-cloud outlier removal requires scipy. Install the asset-tools extra."
        ) from exc
    return cKDTree(points_m)


def _validate_intrinsics(intrinsics: CameraIntrinsics, shape: tuple[int, int]) -> None:
    if (intrinsics.height, intrinsics.width) != shape:
        raise ValueError(
            f"Depth shape {shape} does not match intrinsics "
            f"{(intrinsics.height, intrinsics.width)}."
        )
    values = [intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy]
    if not np.all(np.isfinite(values)) or intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
        raise ValueError("Camera intrinsics must be finite with positive fx and fy.")


def _xyz_vector(value: Sequence[float], name: str) -> np.ndarray:
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values.")
    return vector


def _unit_vector(value: Sequence[float], name: str) -> np.ndarray:
    vector = _xyz_vector(value, name)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        raise ValueError(f"{name} must be non-zero.")
    return vector / norm


def _refine_plane(points: np.ndarray, hint: np.ndarray | None) -> tuple[np.ndarray, float]:
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    if hint is not None and float(normal @ hint) < 0.0:
        normal = -normal
    elif hint is None:
        dominant = int(np.argmax(np.abs(normal)))
        if normal[dominant] < 0.0:
            normal = -normal
    offset = -float(normal @ centroid)
    return normal, offset
