from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from embodiment_core.types import CameraIntrinsics


@dataclass(frozen=True, slots=True)
class DepthLandmark:
    index: int
    pixel_xy: tuple[int, int]
    depth_m: float | None
    camera_xyz_m: tuple[float, float, float] | None
    valid: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "pixel_xy": list(self.pixel_xy),
            "depth_m": self.depth_m,
            "camera_xyz_m": None if self.camera_xyz_m is None else list(self.camera_xyz_m),
            "valid": self.valid,
        }


def sample_depth_median(
    depth_m: np.ndarray,
    x_px: int,
    y_px: int,
    *,
    window: int = 5,
    min_depth_m: float = 0.05,
    max_depth_m: float = 2.0,
) -> float | None:
    if depth_m.ndim != 2:
        raise ValueError("depth_m must be a HxW array.")
    if window <= 0:
        raise ValueError("window must be positive.")
    half = int(window) // 2
    h, w = depth_m.shape
    x0, x1 = max(0, int(x_px) - half), min(w, int(x_px) + half + 1)
    y0, y1 = max(0, int(y_px) - half), min(h, int(y_px) + half + 1)
    patch = np.asarray(depth_m[y0:y1, x0:x1], dtype=np.float32)
    valid = patch[np.isfinite(patch) & (patch >= min_depth_m) & (patch <= max_depth_m)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def deproject_pixel(
    x_px: int,
    y_px: int,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    z = float(depth_m)
    x = (float(x_px) - float(intrinsics.cx)) * z / float(intrinsics.fx)
    y = (float(y_px) - float(intrinsics.cy)) * z / float(intrinsics.fy)
    return (x, y, z)


def enrich_landmarks_with_depth(
    landmarks: Sequence[dict[str, float]],
    depth_m: np.ndarray,
    intrinsics: CameraIntrinsics,
    *,
    window: int = 5,
    min_depth_m: float = 0.05,
    max_depth_m: float = 2.0,
) -> list[DepthLandmark]:
    h, w = depth_m.shape
    enriched: list[DepthLandmark] = []
    for idx, landmark in enumerate(landmarks):
        x_px = int(round(float(landmark.get("x", 0.0)) * max(w - 1, 1)))
        y_px = int(round(float(landmark.get("y", 0.0)) * max(h - 1, 1)))
        x_px = int(np.clip(x_px, 0, w - 1))
        y_px = int(np.clip(y_px, 0, h - 1))
        z = sample_depth_median(
            depth_m,
            x_px,
            y_px,
            window=window,
            min_depth_m=min_depth_m,
            max_depth_m=max_depth_m,
        )
        xyz = None if z is None else deproject_pixel(x_px, y_px, z, intrinsics)
        enriched.append(
            DepthLandmark(
                index=idx,
                pixel_xy=(x_px, y_px),
                depth_m=z,
                camera_xyz_m=xyz,
                valid=z is not None,
            )
        )
    return enriched


def depth_quality_features(enriched: Sequence[DepthLandmark]) -> dict[str, float]:
    valid = [item for item in enriched if item.valid and item.depth_m is not None]
    depths = np.asarray([item.depth_m for item in valid], dtype=np.float64)
    features = {
        "depth_palm_valid_ratio": float(
            sum(1 for item in enriched[:6] if item.valid) / max(len(enriched[:6]), 1)
        ),
        "depth_thumb_valid_ratio": float(
            sum(1 for item in enriched[1:5] if item.valid) / max(len(enriched[1:5]), 1)
        ),
        "depth_index_tip_valid": float(len(enriched) > 8 and enriched[8].valid),
        "depth_m_median": float(np.median(depths)) if depths.size else 0.0,
        "depth_m_min": float(np.min(depths)) if depths.size else 0.0,
        "depth_m_max": float(np.max(depths)) if depths.size else 0.0,
    }
    return features
