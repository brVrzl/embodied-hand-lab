from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class SimilarityResult:
    scale: float
    rotation: np.ndarray
    translation: np.ndarray
    residuals: np.ndarray
    rms_error: float
    max_error: float
    inliers: np.ndarray

    @property
    def matrix(self) -> np.ndarray:
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = self.scale * self.rotation
        matrix[:3, 3] = self.translation
        return matrix


def quaternion_xyzw_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    q = np.asarray(list(quaternion), dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("Quaternion must contain four finite xyzw values.")
    norm = float(np.linalg.norm(q))
    if norm <= np.finfo(float).eps:
        raise ValueError("Quaternion norm must be non-zero.")
    x, y, z, w = q / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("Rotation matrix must be 3x3.")
    if not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-7) or not np.isclose(
        np.linalg.det(matrix), 1.0, atol=1e-7
    ):
        raise ValueError("Input is not a proper rotation matrix.")
    trace = float(np.trace(matrix))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        q = np.asarray(
            [
                (matrix[2, 1] - matrix[1, 2]) / s,
                (matrix[0, 2] - matrix[2, 0]) / s,
                (matrix[1, 0] - matrix[0, 1]) / s,
                0.25 * s,
            ]
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            q = np.asarray(
                [0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s,
                 (matrix[2, 1] - matrix[1, 2]) / s]
            )
        elif index == 1:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            q = np.asarray(
                [(matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s,
                 (matrix[0, 2] - matrix[2, 0]) / s]
            )
        else:
            s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            q = np.asarray(
                [(matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s,
                 (matrix[1, 0] - matrix[0, 1]) / s]
            )
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def compose_transforms(T_A_B: np.ndarray, T_B_C: np.ndarray) -> np.ndarray:
    first = validate_rigid_transform(T_A_B)
    second = validate_rigid_transform(T_B_C)
    return first @ second


def invert_transform(T_A_B: np.ndarray) -> np.ndarray:
    transform = validate_rigid_transform(T_A_B)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = transform[:3, :3].T
    inverse[:3, 3] = -inverse[:3, :3] @ transform[:3, 3]
    return inverse


def validate_rigid_transform(transform: np.ndarray, *, atol: float = 1e-7) -> np.ndarray:
    matrix = np.asarray(transform, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("Rigid transform must be a finite 4x4 matrix.")
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=atol):
        raise ValueError("Rigid transform must have homogeneous last row [0, 0, 0, 1].")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=atol):
        raise ValueError("Rigid-transform rotation is not orthogonal.")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=atol):
        raise ValueError("Rigid-transform rotation determinant must be +1.")
    return matrix


def _validate_points(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    src = np.asarray(source, dtype=np.float64)
    dst = np.asarray(target, dtype=np.float64)
    if src.shape != dst.shape or src.ndim != 2 or src.shape[1] != 3:
        raise ValueError("Source and target must have identical shape (N, 3).")
    if len(src) < 3 or not np.all(np.isfinite(src)) or not np.all(np.isfinite(dst)):
        raise ValueError("At least three finite 3D correspondences are required.")
    if np.linalg.matrix_rank(src - src.mean(axis=0)) < 2:
        raise ValueError("Source correspondences are collinear or coincident.")
    return src, dst


def apply_similarity(points: np.ndarray, scale: float, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    return (float(scale) * (np.asarray(rotation) @ values.T)).T + np.asarray(translation)


def umeyama_similarity(
    source: np.ndarray,
    target: np.ndarray,
    *,
    weights: np.ndarray | None = None,
    with_scale: bool = True,
) -> SimilarityResult:
    src, dst = _validate_points(source, target)
    if weights is None:
        weight = np.ones(len(src), dtype=np.float64)
    else:
        weight = np.asarray(weights, dtype=np.float64)
        if weight.shape != (len(src),) or np.any(weight < 0) or not np.all(np.isfinite(weight)):
            raise ValueError("Weights must be a finite non-negative vector of length N.")
    if float(weight.sum()) <= 0:
        raise ValueError("At least one correspondence weight must be positive.")
    weight = weight / weight.sum()
    src_mean = np.sum(src * weight[:, None], axis=0)
    dst_mean = np.sum(dst * weight[:, None], axis=0)
    src_centered = src - src_mean
    dst_centered = dst - dst_mean
    covariance = (dst_centered * weight[:, None]).T @ src_centered
    u, singular, vt = np.linalg.svd(covariance)
    correction = np.eye(3)
    if np.linalg.det(u @ vt) < 0:
        correction[-1, -1] = -1
    rotation = u @ correction @ vt
    variance = float(np.sum(weight * np.sum(src_centered**2, axis=1)))
    if variance <= np.finfo(float).eps:
        raise ValueError("Source correspondences have zero variance.")
    scale = float(np.sum(singular * np.diag(correction)) / variance) if with_scale else 1.0
    if scale <= 0 or not np.isfinite(scale):
        raise ValueError("Estimated similarity scale is not positive and finite.")
    translation = dst_mean - scale * rotation @ src_mean
    residuals = np.linalg.norm(apply_similarity(src, scale, rotation, translation) - dst, axis=1)
    return SimilarityResult(
        scale=scale,
        rotation=rotation,
        translation=translation,
        residuals=residuals,
        rms_error=float(np.sqrt(np.average(residuals**2, weights=weight))),
        max_error=float(residuals.max()),
        inliers=np.ones(len(src), dtype=bool),
    )


def ransac_similarity(
    source: np.ndarray,
    target: np.ndarray,
    *,
    threshold: float,
    iterations: int = 1000,
    seed: int = 0,
    weights: np.ndarray | None = None,
) -> SimilarityResult:
    src, dst = _validate_points(source, target)
    if threshold <= 0 or iterations <= 0:
        raise ValueError("RANSAC threshold and iterations must be positive.")
    rng = np.random.default_rng(seed)
    best: np.ndarray | None = None
    best_error = np.inf
    for _ in range(iterations):
        indices = rng.choice(len(src), size=3, replace=False)
        try:
            candidate = umeyama_similarity(src[indices], dst[indices])
        except ValueError:
            continue
        residuals = np.linalg.norm(
            apply_similarity(src, candidate.scale, candidate.rotation, candidate.translation) - dst,
            axis=1,
        )
        inliers = residuals <= threshold
        if inliers.sum() < 3:
            continue
        error = float(np.mean(residuals[inliers]))
        if best is None or inliers.sum() > best.sum() or (inliers.sum() == best.sum() and error < best_error):
            best, best_error = inliers, error
    if best is None:
        raise ValueError("RANSAC found no valid similarity model.")
    fit_weights = None if weights is None else np.asarray(weights)[best]
    refined = umeyama_similarity(src[best], dst[best], weights=fit_weights)
    all_residuals = np.linalg.norm(
        apply_similarity(src, refined.scale, refined.rotation, refined.translation) - dst,
        axis=1,
    )
    return SimilarityResult(
        scale=refined.scale,
        rotation=refined.rotation,
        translation=refined.translation,
        residuals=all_residuals,
        rms_error=float(np.sqrt(np.mean(all_residuals[best] ** 2))),
        max_error=float(all_residuals[best].max()),
        inliers=best,
    )
