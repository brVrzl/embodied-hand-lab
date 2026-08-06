"""Small, dependency-free SE(3) and timestamp-aware filtering primitives.

Conventions
-----------
Quaternions are ``(x, y, z, w)`` active rotations. ``Pose6D`` represents
``T_parent_child``: it maps coordinates expressed in the child frame into the
parent frame.  Therefore the reference-relative operator motion is
``inv(T_world_reference) @ T_world_current`` and a robot target is
``T_robot_tcp_reference @ T_tcp_reference_delta``.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Generic, Iterable, TypeVar

import numpy as np

from motion_input import Pose6D


MIN_QUATERNION_NORM = 1e-9


def normalize_quaternion_xyzw(values: Iterable[float]) -> tuple[float, float, float, float]:
    q = np.asarray(tuple(values), dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite xyzw values")
    norm = float(np.linalg.norm(q))
    if norm < MIN_QUATERNION_NORM:
        raise ValueError("quaternion norm is below the minimum")
    return tuple(float(value) for value in q / norm)  # type: ignore[return-value]


def align_quaternion_sign(
    quaternion: Iterable[float], reference: Iterable[float]
) -> tuple[float, float, float, float]:
    q = np.asarray(normalize_quaternion_xyzw(quaternion))
    ref = np.asarray(normalize_quaternion_xyzw(reference))
    if float(np.dot(q, ref)) < 0.0:
        q = -q
    return tuple(float(value) for value in q)  # type: ignore[return-value]


def quaternion_conjugate_xyzw(values: Iterable[float]) -> tuple[float, float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(values)
    return (-x, -y, -z, w)


def quaternion_multiply_xyzw(
    left: Iterable[float], right: Iterable[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = normalize_quaternion_xyzw(left)
    rx, ry, rz, rw = normalize_quaternion_xyzw(right)
    return normalize_quaternion_xyzw(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def quaternion_to_matrix(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(values)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> tuple[float, float, float, float]:
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("rotation matrix must be finite 3x3")
    wxyz = np.empty(4, dtype=np.float64)
    # MuJoCo is intentionally not used here so this module remains easy to test.
    trace = float(np.trace(matrix))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        wxyz[:] = (0.25 * s, (matrix[2, 1] - matrix[1, 2]) / s,
                   (matrix[0, 2] - matrix[2, 0]) / s,
                   (matrix[1, 0] - matrix[0, 1]) / s)
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = math.sqrt(max(0.0, 1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])) * 2.0
            wxyz[:] = ((matrix[2, 1] - matrix[1, 2]) / s, 0.25 * s,
                       (matrix[0, 1] + matrix[1, 0]) / s,
                       (matrix[0, 2] + matrix[2, 0]) / s)
        elif index == 1:
            s = math.sqrt(max(0.0, 1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])) * 2.0
            wxyz[:] = ((matrix[0, 2] - matrix[2, 0]) / s,
                       (matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s,
                       (matrix[1, 2] + matrix[2, 1]) / s)
        else:
            s = math.sqrt(max(0.0, 1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])) * 2.0
            wxyz[:] = ((matrix[1, 0] - matrix[0, 1]) / s,
                       (matrix[0, 2] + matrix[2, 0]) / s,
                       (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s)
    return normalize_quaternion_xyzw((wxyz[1], wxyz[2], wxyz[3], wxyz[0]))


def quaternion_to_rotvec(values: Iterable[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(values)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    vector = np.asarray((x, y, z), dtype=np.float64)
    sine = float(np.linalg.norm(vector))
    if sine < 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * math.atan2(sine, max(-1.0, min(1.0, w)))
    return vector / sine * angle


def swing_twist_about_local_z(
    values: Iterable[float],
) -> tuple[tuple[float, float, float, float], float]:
    """Decompose ``q = swing * twist`` about the source frame's local Z axis.

    The returned scalar is the signed shortest-path twist angle in ``[-pi, pi]``.
    Projection in quaternion space avoids Euler angles and remains continuous for
    small rotations.  At the one genuinely ambiguous case -- an exact 180-degree
    swing whose quaternion has both ``z == 0`` and ``w == 0`` -- twist is defined
    as zero and the complete rotation is returned as swing.
    """

    x, y, z, w = normalize_quaternion_xyzw(values)
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    twist_norm = math.hypot(z, w)
    if twist_norm < 1e-12:
        return (x, y, z, w), 0.0
    twist = (0.0, 0.0, z / twist_norm, w / twist_norm)
    swing = quaternion_multiply_xyzw(
        (x, y, z, w), quaternion_conjugate_xyzw(twist)
    )
    angle = 2.0 * math.atan2(twist[2], twist[3])
    angle = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return swing, angle


def rotvec_to_quaternion_xyzw(values: Iterable[float]) -> tuple[float, float, float, float]:
    vector = np.asarray(tuple(values), dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("rotation vector must contain three finite values")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    xyz = vector / angle * math.sin(angle / 2.0)
    return normalize_quaternion_xyzw((*xyz, math.cos(angle / 2.0)))


def quaternion_angle_rad(left: Iterable[float], right: Iterable[float]) -> float:
    dot = abs(float(np.dot(
        np.asarray(normalize_quaternion_xyzw(left)),
        np.asarray(normalize_quaternion_xyzw(right)),
    )))
    if dot >= 1.0 - 1e-15:
        return 0.0
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def quaternion_slerp_xyzw(
    start: Iterable[float], end: Iterable[float], fraction: float
) -> tuple[float, float, float, float]:
    a = np.asarray(normalize_quaternion_xyzw(start))
    b = np.asarray(align_quaternion_sign(end, a))
    t = max(0.0, min(1.0, float(fraction)))
    dot = max(-1.0, min(1.0, float(np.dot(a, b))))
    if dot > 0.9995:
        return normalize_quaternion_xyzw(a + t * (b - a))
    angle = math.acos(dot)
    sine = math.sin(angle)
    return normalize_quaternion_xyzw(
        math.sin((1.0 - t) * angle) / sine * a
        + math.sin(t * angle) / sine * b
    )


def bounded_pose_step(
    start: Pose6D,
    end: Pose6D,
    *,
    maximum_translation_m: float,
    maximum_rotation_rad: float,
) -> tuple[Pose6D, float]:
    """Advance along one coupled SE(3) segment without dropping pose axes.

    A single fraction is used for both translation and quaternion SLERP.  This
    is intentionally different from clipping Cartesian and rotational
    components independently: the requested six-dimensional path is retained,
    while its progress per control tick is bounded.
    """

    start_position = np.asarray(start.position_m, dtype=np.float64)
    end_position = np.asarray(end.position_m, dtype=np.float64)
    displacement = float(np.linalg.norm(end_position - start_position))
    rotation = quaternion_angle_rad(start.orientation_xyzw, end.orientation_xyzw)
    fraction = 1.0
    if displacement > 0.0:
        fraction = min(
            fraction,
            max(0.0, float(maximum_translation_m)) / displacement,
        )
    if rotation > 0.0:
        fraction = min(
            fraction,
            max(0.0, float(maximum_rotation_rad)) / rotation,
        )
    fraction = max(0.0, min(1.0, fraction))
    position = start_position + fraction * (end_position - start_position)
    return (
        Pose6D(
            tuple(float(value) for value in position),
            quaternion_slerp_xyzw(
                start.orientation_xyzw, end.orientation_xyzw, fraction
            ),
        ),
        fraction,
    )


def relative_pose(reference: Pose6D, current: Pose6D) -> Pose6D:
    """Return ``inv(T_parent_reference) @ T_parent_current``."""

    rotation_reference = quaternion_to_matrix(reference.orientation_xyzw)
    translation = rotation_reference.T @ (
        np.asarray(current.position_m) - np.asarray(reference.position_m)
    )
    orientation = quaternion_multiply_xyzw(
        quaternion_conjugate_xyzw(reference.orientation_xyzw),
        current.orientation_xyzw,
    )
    return Pose6D(tuple(float(value) for value in translation), orientation)


def compose_pose(reference: Pose6D, delta: Pose6D) -> Pose6D:
    """Return ``T_parent_reference @ T_reference_delta``."""

    translation = np.asarray(reference.position_m) + quaternion_to_matrix(
        reference.orientation_xyzw
    ) @ np.asarray(delta.position_m)
    orientation = quaternion_multiply_xyzw(
        reference.orientation_xyzw, delta.orientation_xyzw
    )
    return Pose6D(tuple(float(value) for value in translation), orientation)


def _alpha(cutoff_hz: float, dt_s: float) -> float:
    cutoff = max(float(cutoff_hz), 1e-6)
    tau = 1.0 / (2.0 * math.pi * cutoff)
    return 1.0 / (1.0 + tau / max(dt_s, 1e-9))


class OneEuroVectorFilter:
    """Timestamp-aware One Euro filter for a fixed-size Euclidean vector."""

    def __init__(
        self,
        *,
        min_cutoff_hz: float,
        beta: float,
        derivative_cutoff_hz: float,
        maximum_dt_s: float,
    ) -> None:
        if min_cutoff_hz <= 0 or derivative_cutoff_hz <= 0 or beta < 0 or maximum_dt_s <= 0:
            raise ValueError("invalid One Euro filter parameters")
        self.min_cutoff_hz = float(min_cutoff_hz)
        self.beta = float(beta)
        self.derivative_cutoff_hz = float(derivative_cutoff_hz)
        self.maximum_dt_s = float(maximum_dt_s)
        self.reset()

    def reset(self) -> None:
        self._timestamp_ns: int | None = None
        self._raw: np.ndarray | None = None
        self._filtered: np.ndarray | None = None
        self._derivative: np.ndarray | None = None

    def filter(self, timestamp_ns: int, values: Iterable[float]) -> np.ndarray:
        value = np.asarray(tuple(values), dtype=np.float64)
        if value.ndim != 1 or not np.all(np.isfinite(value)):
            raise ValueError("filter input must be a finite vector")
        if self._timestamp_ns is None:
            self._timestamp_ns = int(timestamp_ns)
            self._raw = value.copy()
            self._filtered = value.copy()
            self._derivative = np.zeros_like(value)
            return value.copy()
        if timestamp_ns <= self._timestamp_ns:
            raise ValueError("filter timestamps must be strictly monotonic")
        assert self._raw is not None and self._filtered is not None and self._derivative is not None
        if value.shape != self._raw.shape:
            raise ValueError("filter vector shape changed")
        dt = min((timestamp_ns - self._timestamp_ns) / 1e9, self.maximum_dt_s)
        derivative_raw = (value - self._raw) / dt
        derivative_alpha = _alpha(self.derivative_cutoff_hz, dt)
        derivative = self._derivative + derivative_alpha * (derivative_raw - self._derivative)
        cutoff = self.min_cutoff_hz + self.beta * float(np.linalg.norm(derivative))
        value_alpha = _alpha(cutoff, dt)
        filtered = self._filtered + value_alpha * (value - self._filtered)
        self._timestamp_ns = int(timestamp_ns)
        self._raw = value.copy()
        self._filtered = filtered
        self._derivative = derivative
        return filtered.copy()


class OneEuroQuaternionFilter:
    """One Euro cutoff selection with shortest-path quaternion SLERP."""

    def __init__(
        self,
        *,
        min_cutoff_hz: float,
        beta: float,
        derivative_cutoff_hz: float,
        maximum_dt_s: float,
    ) -> None:
        self._derivative_filter = OneEuroVectorFilter(
            min_cutoff_hz=derivative_cutoff_hz,
            beta=0.0,
            derivative_cutoff_hz=derivative_cutoff_hz,
            maximum_dt_s=maximum_dt_s,
        )
        self.min_cutoff_hz = float(min_cutoff_hz)
        self.beta = float(beta)
        self.maximum_dt_s = float(maximum_dt_s)
        if self.min_cutoff_hz <= 0 or self.beta < 0:
            raise ValueError("invalid quaternion filter parameters")
        self.reset()

    def reset(self) -> None:
        self._timestamp_ns: int | None = None
        self._raw: tuple[float, float, float, float] | None = None
        self._filtered: tuple[float, float, float, float] | None = None
        self._derivative_filter.reset()

    def filter(
        self, timestamp_ns: int, quaternion: Iterable[float]
    ) -> tuple[float, float, float, float]:
        current = normalize_quaternion_xyzw(quaternion)
        if self._timestamp_ns is None:
            self._timestamp_ns = int(timestamp_ns)
            self._raw = current
            self._filtered = current
            self._derivative_filter.filter(timestamp_ns, (0.0, 0.0, 0.0))
            return current
        if timestamp_ns <= self._timestamp_ns:
            raise ValueError("filter timestamps must be strictly monotonic")
        assert self._raw is not None and self._filtered is not None
        current = align_quaternion_sign(current, self._raw)
        dt = min((timestamp_ns - self._timestamp_ns) / 1e9, self.maximum_dt_s)
        delta = quaternion_multiply_xyzw(quaternion_conjugate_xyzw(self._raw), current)
        angular_velocity = quaternion_to_rotvec(delta) / dt
        derivative = self._derivative_filter.filter(timestamp_ns, angular_velocity)
        cutoff = self.min_cutoff_hz + self.beta * float(np.linalg.norm(derivative))
        filtered = quaternion_slerp_xyzw(self._filtered, current, _alpha(cutoff, dt))
        self._timestamp_ns = int(timestamp_ns)
        self._raw = current
        self._filtered = filtered
        return filtered


T = TypeVar("T")


@dataclass(frozen=True, slots=True)
class TimedPoseSample(Generic[T]):
    timestamp_monotonic_ns: int
    sequence_number: int
    pose: Pose6D
    payload: T


class PoseSampleBuffer(Generic[T]):
    """Small monotonic sample buffer with bounded interpolation and no prediction."""

    def __init__(self, capacity: int = 32) -> None:
        if capacity < 2:
            raise ValueError("sample buffer capacity must be at least two")
        self._samples: deque[TimedPoseSample[T]] = deque(maxlen=capacity)
        self.dropped_out_of_order = 0
        self.repeated_sequences = 0

    def clear(self) -> None:
        self._samples.clear()

    def add(self, sample: TimedPoseSample[T]) -> bool:
        if self._samples and sample.timestamp_monotonic_ns <= self._samples[-1].timestamp_monotonic_ns:
            self.dropped_out_of_order += 1
            return False
        if self._samples and sample.sequence_number == self._samples[-1].sequence_number:
            self.repeated_sequences += 1
            return False
        self._samples.append(sample)
        return True

    @property
    def latest(self) -> TimedPoseSample[T] | None:
        return None if not self._samples else self._samples[-1]

    def sample(self, timestamp_ns: int) -> TimedPoseSample[T] | None:
        if not self._samples:
            return None
        if timestamp_ns <= self._samples[0].timestamp_monotonic_ns:
            return self._samples[0]
        for left, right in zip(self._samples, tuple(self._samples)[1:]):
            if left.timestamp_monotonic_ns <= timestamp_ns <= right.timestamp_monotonic_ns:
                span = right.timestamp_monotonic_ns - left.timestamp_monotonic_ns
                fraction = (timestamp_ns - left.timestamp_monotonic_ns) / span
                position = tuple(
                    float(a + fraction * (b - a))
                    for a, b in zip(left.pose.position_m, right.pose.position_m)
                )
                orientation = quaternion_slerp_xyzw(
                    left.pose.orientation_xyzw, right.pose.orientation_xyzw, fraction
                )
                return TimedPoseSample(
                    timestamp_ns,
                    right.sequence_number,
                    Pose6D(position, orientation),
                    right.payload,
                )
        # Hold the newest sample. The caller owns stale-age enforcement.
        return self._samples[-1]
