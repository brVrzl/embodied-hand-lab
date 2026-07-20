from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ..contracts import Pose3D


def _vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def normalize_quaternion_xyzw(values: Sequence[float]) -> tuple[float, float, float, float]:
    q = _vector(values, 4, "quaternion")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    q /= norm
    return tuple(float(value) for value in q)  # type: ignore[return-value]


def quaternion_conjugate(values: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(values)
    return (-x, -y, -z, w)


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = normalize_quaternion_xyzw(left)
    bx, by, bz, bw = normalize_quaternion_xyzw(right)
    return normalize_quaternion_xyzw(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def quaternion_to_matrix(values: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(values)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(matrix: Sequence[Sequence[float]]) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-3):
        raise ValueError("rotation matrix is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=2e-3):
        raise ValueError("rotation matrix must have determinant +1")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.asarray(
            [
                (rotation[2, 1] - rotation[1, 2]) / s,
                (rotation[0, 2] - rotation[2, 0]) / s,
                (rotation[1, 0] - rotation[0, 1]) / s,
                0.25 * s,
            ]
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            s = math.sqrt(max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 0.0)) * 2.0
            q = np.asarray(
                [0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s,
                 (rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[2, 1] - rotation[1, 2]) / s]
            )
        elif axis == 1:
            s = math.sqrt(max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 0.0)) * 2.0
            q = np.asarray(
                [(rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s,
                 (rotation[1, 2] + rotation[2, 1]) / s,
                 (rotation[0, 2] - rotation[2, 0]) / s]
            )
        else:
            s = math.sqrt(max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 0.0)) * 2.0
            q = np.asarray(
                [(rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s,
                 (rotation[1, 0] - rotation[0, 1]) / s]
            )
    result = np.asarray(normalize_quaternion_xyzw(q))
    if result[3] < 0.0:
        result *= -1.0
    return tuple(float(value) for value in result)  # type: ignore[return-value]


def quaternion_log(values: Sequence[float]) -> np.ndarray:
    q = np.asarray(normalize_quaternion_xyzw(values))
    if q[3] < 0.0:
        q *= -1.0
    vector_norm = float(np.linalg.norm(q[:3]))
    if vector_norm < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(vector_norm, float(q[3]))
    return q[:3] * (angle / vector_norm)


def quaternion_exp(rotation_vector: Sequence[float]) -> tuple[float, float, float, float]:
    vector = _vector(rotation_vector, 3, "rotation_vector")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    scale = math.sin(0.5 * angle) / angle
    return normalize_quaternion_xyzw((*tuple(vector * scale), math.cos(0.5 * angle)))


def quaternion_angle(left: Sequence[float], right: Sequence[float]) -> float:
    delta = quaternion_multiply(left, quaternion_conjugate(right))
    return float(np.linalg.norm(quaternion_log(delta)))


def slerp(left: Sequence[float], right: Sequence[float], fraction: float) -> tuple[float, float, float, float]:
    if not math.isfinite(fraction):
        raise ValueError("slerp fraction must be finite")
    t = min(1.0, max(0.0, float(fraction)))
    a = np.asarray(normalize_quaternion_xyzw(left))
    b = np.asarray(normalize_quaternion_xyzw(right))
    if float(np.dot(a, b)) < 0.0:
        b *= -1.0
    delta = quaternion_multiply(b, quaternion_conjugate(a))
    step = quaternion_exp(quaternion_log(delta) * t)
    return quaternion_multiply(step, a)


def compose_pose(left: Pose3D, right: Pose3D) -> Pose3D:
    rotation = quaternion_to_matrix(left.quaternion_xyzw)
    position = np.asarray(left.position_m) + rotation @ np.asarray(right.position_m)
    return Pose3D(
        tuple(float(value) for value in position),
        quaternion_multiply(left.quaternion_xyzw, right.quaternion_xyzw),
    )


def inverse_pose(pose: Pose3D) -> Pose3D:
    inverse_q = quaternion_conjugate(pose.quaternion_xyzw)
    inverse_p = -(quaternion_to_matrix(inverse_q) @ np.asarray(pose.position_m))
    return Pose3D(tuple(float(value) for value in inverse_p), inverse_q)
