from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True, slots=True)
class ObjectGeometry:
    centroid_xyz: list[float]
    extents_xyz_m: list[float]
    principal_axes: list[list[float]]
    point_count: int
    frame_id: str = "camera"
    shape_hint: str = "unknown"

    @property
    def min_width_m(self) -> float:
        return float(min(self.extents_xyz_m))

    @property
    def max_width_m(self) -> float:
        return float(max(self.extents_xyz_m))

    @property
    def height_m(self) -> float:
        return float(self.extents_xyz_m[2])

    @property
    def flatness(self) -> float:
        largest = max(self.extents_xyz_m)
        return 0.0 if largest <= 1e-9 else float(min(self.extents_xyz_m) / largest)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def geometry_from_point_cloud(
    points: Sequence[Sequence[float]] | np.ndarray,
    *,
    frame_id: str = "camera",
    percentile: float = 98.0,
    shape_hint: str | None = None,
) -> ObjectGeometry:
    cloud = np.asarray(points, dtype=np.float64)
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        raise ValueError(f"Expected point cloud shape (N, 3), got {cloud.shape}.")
    if cloud.shape[0] < 4:
        raise ValueError("At least four points are required to estimate object geometry.")
    if not np.isfinite(cloud).all():
        raise ValueError("Point cloud contains NaN or infinite values.")

    low_q = (100.0 - percentile) / 2.0
    high_q = 100.0 - low_q
    lo = np.percentile(cloud, low_q, axis=0)
    hi = np.percentile(cloud, high_q, axis=0)
    extents = np.maximum(hi - lo, 1e-6)
    centroid = np.mean(cloud, axis=0)

    centered = cloud - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    axes = vh.astype(np.float64)
    if np.linalg.det(axes) < 0.0:
        axes[-1] *= -1.0

    inferred_shape = shape_hint or _infer_shape_hint(extents)
    return ObjectGeometry(
        centroid_xyz=centroid.astype(float).tolist(),
        extents_xyz_m=extents.astype(float).tolist(),
        principal_axes=axes.astype(float).tolist(),
        point_count=int(cloud.shape[0]),
        frame_id=frame_id,
        shape_hint=inferred_shape,
    )


def _infer_shape_hint(extents: np.ndarray) -> str:
    ordered = np.sort(extents)
    if ordered[0] / max(ordered[-1], 1e-9) < 0.22:
        return "flat"
    if ordered[-1] / max(ordered[0], 1e-9) < 1.35:
        return "round"
    if ordered[-1] / max(ordered[1], 1e-9) > 1.55:
        return "elongated"
    return "box"
