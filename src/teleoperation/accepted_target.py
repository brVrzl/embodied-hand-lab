"""MuJoCo-independent accepted arm-target contract shared by output adapters."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True, slots=True)
class AcceptedTcpPose:
    """Immutable, dependency-neutral TCP pose in metres and XYZW order."""

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        position = tuple(float(value) for value in self.position_m)
        orientation = tuple(float(value) for value in self.orientation_xyzw)
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise ValueError("accepted TCP position must contain three finite metres")
        if len(orientation) != 4 or not all(math.isfinite(value) for value in orientation):
            raise ValueError("accepted TCP orientation must contain four finite values")
        norm = math.sqrt(sum(value * value for value in orientation))
        if abs(norm - 1.0) > 1e-6:
            raise ValueError("accepted TCP orientation must be a unit XYZW quaternion")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "orientation_xyzw", orientation)


@dataclass(frozen=True, slots=True)
class AcceptedTargetDiagnostics:
    """Immutable acceptance evidence, including rejected continuation trials."""

    final_reason: str
    attempted_reasons: tuple[str, ...]
    continuation_fraction: float
    continuation_backtracks: int
    ik_position_error_m: float
    ik_orientation_error_rad: float
    jacobian_condition: float
    minimum_jacobian_singular_value: float
    nearest_safe_joint_limit_margin_rad: float

    def __post_init__(self) -> None:
        numeric = (
            self.continuation_fraction,
            self.ik_position_error_m,
            self.ik_orientation_error_rad,
            self.jacobian_condition,
            self.minimum_jacobian_singular_value,
            self.nearest_safe_joint_limit_margin_rad,
        )
        if not self.final_reason or not self.attempted_reasons:
            raise ValueError("accepted target diagnostics require acceptance reasons")
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("accepted target diagnostics must be finite")
        if not 0.0 < self.continuation_fraction <= 1.0:
            raise ValueError("continuation fraction must be in (0, 1]")
        if self.continuation_backtracks < 0:
            raise ValueError("continuation backtracks must be non-negative")


@dataclass(frozen=True, slots=True)
class AcceptedArmTarget:
    """One authoritative post-IK target shared by simulation and hardware."""

    sequence_number: int
    input_sequence_number: int
    source_sequence_number: int | None
    source_timestamp_ns: int | None
    input_receive_monotonic_ns: int
    generated_monotonic_ns: int
    reference_generation: int
    clutch_generation: int
    desired_tcp: AcceptedTcpPose
    filtered_tcp: AcceptedTcpPose
    joint_position_rad: tuple[float, float, float, float, float, float]
    diagnostics: AcceptedTargetDiagnostics

    def __post_init__(self) -> None:
        if self.sequence_number < 0 or self.input_sequence_number < 0:
            raise ValueError("accepted target sequence must be non-negative")
        if self.source_sequence_number is not None and self.source_sequence_number < 0:
            raise ValueError("source sequence must be non-negative")
        if self.source_timestamp_ns is not None and self.source_timestamp_ns < 0:
            raise ValueError("source timestamp must be non-negative")
        if not 0 <= self.input_receive_monotonic_ns <= self.generated_monotonic_ns:
            raise ValueError("accepted target timestamps must be monotonic")
        if self.reference_generation < 1 or self.clutch_generation < 1:
            raise ValueError("accepted target requires a captured reference generation")
        if len(self.joint_position_rad) != 6 or not all(
            math.isfinite(value) for value in self.joint_position_rad
        ):
            raise ValueError("accepted target must contain six finite joint radians")
