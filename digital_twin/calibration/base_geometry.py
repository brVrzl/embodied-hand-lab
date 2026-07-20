from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlaneFit:
    normal: np.ndarray
    offset: float
    inliers: np.ndarray
    residuals: np.ndarray


@dataclass(frozen=True)
class CircleFit:
    center: np.ndarray
    radius: float
    inliers: np.ndarray
    residuals: np.ndarray


def fit_plane_ransac(points: np.ndarray, threshold: float, iterations: int = 3000, seed: int = 0) -> PlaneFit:
    xyz = np.asarray(points, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) < 3 or threshold <= 0:
        raise ValueError("Plane fitting needs at least three 3D points and a positive threshold.")
    rng = np.random.default_rng(seed)
    best = np.zeros(len(xyz), dtype=bool)
    best_error = np.inf
    for _ in range(iterations):
        sample = xyz[rng.choice(len(xyz), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-12:
            continue
        normal /= norm
        offset = -normal @ sample[0]
        residuals = np.abs(xyz @ normal + offset)
        inliers = residuals <= threshold
        if inliers.sum() < 3:
            continue
        error = float(np.mean(residuals[inliers]))
        if inliers.sum() > best.sum() or (inliers.sum() == best.sum() and error < best_error):
            best, best_error = inliers, error
    if best.sum() < 3:
        raise ValueError("Plane RANSAC found no valid model.")
    center = xyz[best].mean(axis=0)
    _, _, vt = np.linalg.svd(xyz[best] - center, full_matrices=False)
    normal = vt[-1]
    offset = -normal @ center
    residuals = np.abs(xyz @ normal + offset)
    inliers = residuals <= threshold
    return PlaneFit(normal, float(offset), inliers, residuals)


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    z = np.asarray(normal, dtype=float)
    z /= np.linalg.norm(z)
    helper = np.asarray([1.0, 0.0, 0.0]) if abs(z[0]) < 0.8 else np.asarray([0.0, 1.0, 0.0])
    x = np.cross(helper, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return x, y


def project_to_plane_coordinates(points: np.ndarray, origin: np.ndarray, axis_x: np.ndarray, axis_y: np.ndarray) -> np.ndarray:
    centered = np.asarray(points, dtype=float) - np.asarray(origin, dtype=float)
    return np.column_stack((centered @ axis_x, centered @ axis_y))


def _circle_from_three(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    first, second, third = points
    matrix = 2.0 * np.asarray([second - first, third - first])
    rhs = np.asarray([second @ second - first @ first, third @ third - first @ first])
    if abs(np.linalg.det(matrix)) < 1e-12:
        return None
    center = np.linalg.solve(matrix, rhs)
    return center, float(np.linalg.norm(first - center))


def fit_circle_ransac(
    points: np.ndarray, threshold: float, radius_bounds: tuple[float, float], iterations: int = 10000, seed: int = 0
) -> CircleFit:
    xy = np.asarray(points, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 3 or threshold <= 0:
        raise ValueError("Circle fitting needs at least three 2D points and a positive threshold.")
    lower, upper = radius_bounds
    if not 0 < lower < upper:
        raise ValueError("Circle radius bounds must be positive and increasing.")
    rng = np.random.default_rng(seed)
    best = np.zeros(len(xy), dtype=bool)
    best_error = np.inf
    for _ in range(iterations):
        candidate = _circle_from_three(xy[rng.choice(len(xy), 3, replace=False)])
        if candidate is None:
            continue
        center, radius = candidate
        if not lower <= radius <= upper:
            continue
        residuals = np.abs(np.linalg.norm(xy - center, axis=1) - radius)
        inliers = residuals <= threshold
        error = float(np.mean(residuals[inliers])) if inliers.any() else np.inf
        if inliers.sum() > best.sum() or (inliers.sum() == best.sum() and error < best_error):
            best, best_error = inliers, error
    if best.sum() < 3:
        raise ValueError("Circle RANSAC found no valid model.")
    selected = xy[best]
    A = np.column_stack((2 * selected[:, 0], 2 * selected[:, 1], np.ones(len(selected))))
    b = np.sum(selected**2, axis=1)
    solution, *_ = np.linalg.lstsq(A, b, rcond=None)
    center = solution[:2]
    radius = float(np.sqrt(max(solution[2] + center @ center, 0.0)))
    residuals = np.abs(np.linalg.norm(xy - center, axis=1) - radius)
    inliers = residuals <= threshold
    return CircleFit(center, radius, inliers, residuals)


def fit_line_pca(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xy = np.asarray(points, dtype=float)
    if xy.ndim != 2 or xy.shape[1] != 2 or len(xy) < 2:
        raise ValueError("Line PCA needs at least two 2D points.")
    center = np.median(xy, axis=0)
    _, _, vt = np.linalg.svd(xy - center, full_matrices=False)
    direction = vt[0]
    normal = np.asarray([-direction[1], direction[0]])
    return center, direction, (xy - center) @ normal
