"""Uncalibrated operator-to-JAKA-base mapping for simulation only."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np

from motion_input import OfflineOperatorTarget, Pose6D
from .se3 import (
    compose_pose,
    quaternion_to_matrix,
    quaternion_to_rotvec,
    rotvec_to_quaternion_xyzw,
)


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
    translation_enabled: bool = True
    rotation_operator_to_robot_basis: tuple[tuple[float, float, float], ...] | None = None
    orientation_scale_per_axis: tuple[float, float, float] | None = None
    maximum_relative_rotation_rad: float = math.pi

    def __post_init__(self) -> None:
        if not self.calibration_id.strip():
            raise ValueError("calibration_id is required")
        if self.calibrated:
            raise ValueError("this gate accepts uncalibrated simulation mappings only")
        _validate_basis(self.operator_to_robot_basis, "operator_to_robot_basis")
        if self.rotation_operator_to_robot_basis is not None:
            _validate_basis(
                self.rotation_operator_to_robot_basis,
                "rotation_operator_to_robot_basis",
            )
        scales = np.asarray(self.translation_scale_per_axis, dtype=np.float64)
        if scales.shape != (3,) or not np.all(np.isfinite(scales)) or np.any(scales < 0):
            raise ValueError("translation_scale_per_axis must be three non-negative values")
        rotation_scales = np.asarray(
            self.orientation_scale_per_axis
            if self.orientation_scale_per_axis is not None
            else (self.orientation_scale,) * 3,
            dtype=np.float64,
        )
        if (
            rotation_scales.shape != (3,)
            or not np.all(np.isfinite(rotation_scales))
            or np.any(rotation_scales < 0)
        ):
            raise ValueError("orientation_scale_per_axis must be three non-negative values")
        for name in (
            "translation_deadband_m",
            "orientation_scale",
            "orientation_deadband_rad",
            "maximum_operator_displacement_m",
            "maximum_target_displacement_m",
            "maximum_relative_rotation_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.maximum_operator_displacement_m <= 0 or self.maximum_target_displacement_m <= 0:
            raise ValueError("mapping displacement limits must be positive")
        if self.maximum_relative_rotation_rad <= 0:
            raise ValueError("maximum_relative_rotation_rad must be positive")

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
            translation_enabled=bool(values.get("translation_enabled", True)),
            rotation_operator_to_robot_basis=tuple(
                tuple(float(value) for value in row)
                for row in values.get(
                    "rotation_operator_to_robot_basis",
                    values["operator_to_robot_basis"],
                )
            ),
            orientation_scale_per_axis=_triple(
                values.get(
                    "orientation_scale_per_axis",
                    (float(values.get("orientation_scale", 0.0)),) * 3,
                )
            ),
            maximum_relative_rotation_rad=math.radians(
                float(values.get("maximum_relative_rotation_deg", 180.0))
            ),
        )


class ProvisionalOperatorToRobotMapper:
    """Map a canonical-operator delta onto a captured simulated TCP reference."""

    def __init__(self, config: ProvisionalMappingConfig) -> None:
        self.config = config
        self.robot_tcp_reference: Pose6D | None = None

    @property
    def basis(self) -> np.ndarray:
        return np.asarray(self.config.operator_to_robot_basis, dtype=np.float64)

    @property
    def rotation_basis(self) -> np.ndarray:
        return np.asarray(
            self.config.rotation_operator_to_robot_basis
            or self.config.operator_to_robot_basis,
            dtype=np.float64,
        )

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
        scaled = (
            delta * np.asarray(self.config.translation_scale_per_axis)
            if self.config.translation_enabled
            else np.zeros(3, dtype=np.float64)
        )
        robot_delta = self.basis @ scaled
        if float(np.linalg.norm(robot_delta)) > self.config.maximum_target_displacement_m:
            raise MappingRejection("OUTSIDE_ROBOT_WORKSPACE")

        reference = self.robot_tcp_reference
        if not self.config.orientation_enabled:
            mapped_delta_q = (0.0, 0.0, 0.0, 1.0)
        else:
            rotation = quaternion_to_matrix(operator.orientation_xyzw)
            mapped_rotation = self.rotation_basis @ rotation @ self.rotation_basis.T
            rotation_vector = quaternion_to_rotvec(
                _matrix_to_quaternion_xyzw(mapped_rotation)
            )
            angle = float(np.linalg.norm(rotation_vector))
            if angle > self.config.maximum_relative_rotation_rad:
                raise MappingRejection("OUTSIDE_OPERATOR_ENVELOPE")
            if angle < self.config.orientation_deadband_rad:
                rotation_vector[:] = 0.0
            else:
                rotation_vector *= np.asarray(
                    self.config.orientation_scale_per_axis
                    or (self.config.orientation_scale,) * 3
                )
            mapped_delta_q = rotvec_to_quaternion_xyzw(rotation_vector)
        return compose_pose(
            reference,
            Pose6D(
                tuple(float(value) for value in robot_delta),
                mapped_delta_q,
            )
        )


def _triple(values: Any) -> tuple[float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 3:
        raise ValueError("expected three values")
    return result


def _validate_basis(values: Any, name: str) -> None:
    basis = np.asarray(values, dtype=np.float64)
    if basis.shape != (3, 3) or not np.all(np.isfinite(basis)):
        raise ValueError(f"{name} must be finite 3x3")
    if not np.allclose(basis @ basis.T, np.eye(3), atol=1e-8):
        raise ValueError(f"{name} must be an axis permutation/sign matrix")
    if not math.isclose(abs(float(np.linalg.det(basis))), 1.0, abs_tol=1e-8):
        raise ValueError(f"{name} determinant must be +/-1")


def _matrix_to_quaternion_xyzw(matrix: np.ndarray) -> tuple[float, float, float, float]:
    # Local import avoids exporting a second matrix conversion API from mapping.
    from .se3 import matrix_to_quaternion_xyzw

    return matrix_to_quaternion_xyzw(matrix)
