"""Uncalibrated operator-to-JAKA-base mapping for simulation only."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np

from motion_input import OfflineOperatorTarget, Pose6D


class MappingRejection(RuntimeError):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True, slots=True)
class ProvisionalMappingConfig:
    calibration_id: str
    calibrated: bool
    operator_to_robot_basis: tuple[tuple[float, float, float], ...]
    translation_scale_per_axis: tuple[float, float, float]
    translation_deadband_m: float
    orientation_enabled: bool
    orientation_scale: float
    orientation_deadband_rad: float
    maximum_operator_displacement_m: float
    maximum_target_displacement_m: float

    def __post_init__(self) -> None:
        if not self.calibration_id.strip():
            raise ValueError("calibration_id is required")
        if self.calibrated:
            raise ValueError("this gate accepts uncalibrated simulation mappings only")
        basis = np.asarray(self.operator_to_robot_basis, dtype=np.float64)
        if basis.shape != (3, 3) or not np.all(np.isfinite(basis)):
            raise ValueError("operator_to_robot_basis must be finite 3x3")
        if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-8):
            raise ValueError("operator_to_robot_basis must be an axis permutation/sign matrix")
        if not math.isclose(abs(float(np.linalg.det(basis))), 1.0, abs_tol=1e-8):
            raise ValueError("operator_to_robot_basis determinant must be +/-1")
        scales = np.asarray(self.translation_scale_per_axis, dtype=np.float64)
        if scales.shape != (3,) or not np.all(np.isfinite(scales)) or np.any(scales < 0):
            raise ValueError("translation_scale_per_axis must be three non-negative values")
        for name in (
            "translation_deadband_m",
            "orientation_scale",
            "orientation_deadband_rad",
            "maximum_operator_displacement_m",
            "maximum_target_displacement_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.maximum_operator_displacement_m <= 0 or self.maximum_target_displacement_m <= 0:
            raise ValueError("mapping displacement limits must be positive")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "ProvisionalMappingConfig":
        return cls(
            calibration_id=str(values["calibration_id"]),
            calibrated=bool(values.get("calibrated", False)),
            operator_to_robot_basis=tuple(
                tuple(float(value) for value in row)
                for row in values["operator_to_robot_basis"]
            ),
            translation_scale_per_axis=_triple(values["translation_scale_per_axis"]),
            translation_deadband_m=float(values.get("translation_deadband_m", 0.0)),
            orientation_enabled=bool(values.get("orientation_enabled", False)),
            orientation_scale=float(values.get("orientation_scale", 0.0)),
            orientation_deadband_rad=math.radians(
                float(values.get("orientation_deadband_deg", 0.0))
            ),
            maximum_operator_displacement_m=float(
                values["maximum_operator_displacement_m"]
            ),
            maximum_target_displacement_m=float(values["maximum_target_displacement_m"]),
        )


class ProvisionalOperatorToRobotMapper:
    """Map a canonical-operator delta onto a captured simulated TCP reference."""

    def __init__(self, config: ProvisionalMappingConfig) -> None:
        self.config = config
        self.robot_tcp_reference: Pose6D | None = None

    @property
    def basis(self) -> np.ndarray:
        return np.asarray(self.config.operator_to_robot_basis, dtype=np.float64)

    def clear_reference(self) -> None:
        self.robot_tcp_reference = None

    def capture_robot_reference(self, simulated_tcp: Pose6D) -> None:
        self.robot_tcp_reference = simulated_tcp

    def map(self, operator: OfflineOperatorTarget) -> Pose6D:
        if not operator.valid_for_mapping:
            raise MappingRejection("DISENGAGED")
        if self.robot_tcp_reference is None:
            raise MappingRejection("DISENGAGED")
        delta = np.asarray(operator.translation_m, dtype=np.float64)
        if float(np.linalg.norm(delta)) > self.config.maximum_operator_displacement_m:
            raise MappingRejection("OUTSIDE_OPERATOR_ENVELOPE")
        delta[np.abs(delta) < self.config.translation_deadband_m] = 0.0
        scaled = delta * np.asarray(self.config.translation_scale_per_axis)
        robot_delta = self.basis @ scaled
        if float(np.linalg.norm(robot_delta)) > self.config.maximum_target_displacement_m:
            raise MappingRejection("OUTSIDE_ROBOT_WORKSPACE")

        reference = self.robot_tcp_reference
        target_position = tuple(
            float(value)
            for value in np.asarray(reference.position_m, dtype=np.float64) + robot_delta
        )
        if not self.config.orientation_enabled:
            target_orientation = reference.orientation_xyzw
        else:
            rotation = _quaternion_to_matrix(operator.orientation_xyzw)
            mapped_rotation = self.basis @ rotation @ self.basis.T
            rotation_vector = _matrix_to_rotation_vector(mapped_rotation)
            angle = float(np.linalg.norm(rotation_vector))
            if angle < self.config.orientation_deadband_rad:
                rotation_vector[:] = 0.0
            else:
                rotation_vector *= self.config.orientation_scale
            mapped_delta_q = _rotation_vector_to_quaternion(rotation_vector)
            target_orientation = _quaternion_multiply(
                mapped_delta_q, reference.orientation_xyzw
            )
        return Pose6D(target_position, target_orientation)


def _triple(values: Any) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError("expected three values")
    return result


def _normalize(q: Any) -> np.ndarray:
    result = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if result.shape != (4,) or norm <= 1e-12:
        raise ValueError("quaternion must contain four finite non-zero values")
    return result / norm


def _quaternion_to_matrix(q: Any) -> np.ndarray:
    x, y, z, w = _normalize(q)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quaternion_multiply(left: Any, right: Any) -> tuple[float, float, float, float]:
    ax, ay, az, aw = _normalize(left)
    bx, by, bz, bw = _normalize(right)
    return tuple(
        float(value)
        for value in _normalize(
            (
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            )
        )
    )  # type: ignore[return-value]


def _matrix_to_rotation_vector(matrix: np.ndarray) -> np.ndarray:
    cosine = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-10:
        return np.zeros(3)
    axis = np.asarray(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
    ) / (2.0 * math.sin(angle))
    return axis * angle


def _rotation_vector_to_quaternion(vector: np.ndarray) -> tuple[float, float, float, float]:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    axis = vector / angle
    sine = math.sin(angle / 2.0)
    return (float(axis[0] * sine), float(axis[1] * sine), float(axis[2] * sine), math.cos(angle / 2.0))
