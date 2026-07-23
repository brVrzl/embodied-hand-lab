"""Shared feasibility contract for time-resampled JAKA joint targets.

The contract mirrors the transport's causal, latest-segment policy without
depending on MuJoCo or the JAKA SDK.  Accepted-target timestamps are local
``CLOCK_MONOTONIC`` generation times.  A new segment starts at the joint point
that the preceding segment could most recently have emitted on the 8 ms grid;
its duration is the interval between adjacent accepted-target timestamps.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


OUTPUT_FEASIBILITY_NUMERIC_TOLERANCE_RAD_S = 1e-12


@dataclass(frozen=True, slots=True)
class JointOutputContractConfig:
    maximum_velocity_rad_s: float
    servo_period_ns: int
    numeric_tolerance_rad_s: float = OUTPUT_FEASIBILITY_NUMERIC_TOLERANCE_RAD_S

    def __post_init__(self) -> None:
        if not math.isfinite(self.maximum_velocity_rad_s) or self.maximum_velocity_rad_s <= 0.0:
            raise ValueError("output velocity boundary must be finite and positive")
        if not isinstance(self.servo_period_ns, int) or self.servo_period_ns <= 0:
            raise ValueError("servo period must be a positive integer nanosecond count")
        if not math.isfinite(self.numeric_tolerance_rad_s) or self.numeric_tolerance_rad_s < 0.0:
            raise ValueError("output feasibility tolerance must be finite and non-negative")


def continuous_joint_delta(candidate_rad: float, previous_rad: float) -> float:
    """Return the valid delta for branch-continuous absolute JAKA joints.

    JAKA ServoJ consumes absolute multi-turn joint positions.  The shared IK
    already rejects branch changes, so modulo-2-pi wrapping here would turn a
    different absolute joint target into a false short arc.
    """

    candidate = float(candidate_rad)
    previous = float(previous_rad)
    delta = candidate - previous
    if not all(math.isfinite(value) for value in (candidate, previous, delta)):
        raise ValueError("joint output feasibility requires finite radians")
    return delta


@dataclass(frozen=True, slots=True)
class JointOutputFeasibility:
    """Prediction for one prospective AcceptedArmTarget transition."""

    generated_monotonic_ns: int
    interval_ns: int
    segment_start_rad: tuple[float, ...]
    candidate_rad: tuple[float, ...]
    delta_rad: tuple[float, ...]
    predicted_velocity_rad_s: tuple[float, ...]
    violating_joint_indices: tuple[int, ...]
    maximum_velocity_rad_s: float
    boundary_rad_s: float

    @property
    def feasible(self) -> bool:
        return not self.violating_joint_indices


class JointOutputFeasibilityTracker:
    """Bounded virtual copy of the native 8 ms latest-segment resampler.

    The tracker stores one active segment only.  ``preview`` is side-effect
    free so rejected continuation trials cannot become authoritative;
    ``commit`` is called only for a target that will become AcceptedArmTarget.
    """

    def __init__(
        self,
        *,
        maximum_velocity_rad_s: float,
        servo_period_ns: int,
        numeric_tolerance_rad_s: float = OUTPUT_FEASIBILITY_NUMERIC_TOLERANCE_RAD_S,
    ) -> None:
        boundary = float(maximum_velocity_rad_s)
        tolerance = float(numeric_tolerance_rad_s)
        if not math.isfinite(boundary) or boundary <= 0.0:
            raise ValueError("output velocity boundary must be finite and positive")
        if not isinstance(servo_period_ns, int) or servo_period_ns <= 0:
            raise ValueError("servo period must be a positive integer nanosecond count")
        if not math.isfinite(tolerance) or tolerance < 0.0:
            raise ValueError("output feasibility tolerance must be finite and non-negative")
        self.maximum_velocity_rad_s = boundary
        self.servo_period_ns = servo_period_ns
        self.numeric_tolerance_rad_s = tolerance
        self._initial_rad: tuple[float, ...] | None = None
        self._last_accepted_ns: int | None = None
        self._segment_start_rad: tuple[float, ...] | None = None
        self._segment_destination_rad: tuple[float, ...] | None = None
        self._segment_duration_ns = servo_period_ns

    @classmethod
    def from_config(cls, config: JointOutputContractConfig) -> "JointOutputFeasibilityTracker":
        return cls(
            maximum_velocity_rad_s=config.maximum_velocity_rad_s,
            servo_period_ns=config.servo_period_ns,
            numeric_tolerance_rad_s=config.numeric_tolerance_rad_s,
        )

    @property
    def has_accepted_target(self) -> bool:
        return self._last_accepted_ns is not None

    def reset(self, joint_position_rad: Sequence[float]) -> None:
        values = self._validated_joints(joint_position_rad)
        self._initial_rad = values
        self._last_accepted_ns = None
        self._segment_start_rad = values
        self._segment_destination_rad = values
        self._segment_duration_ns = self.servo_period_ns

    def preview(
        self,
        joint_position_rad: Sequence[float],
        *,
        generated_monotonic_ns: int,
    ) -> JointOutputFeasibility:
        candidate = self._validated_joints(joint_position_rad)
        timestamp = int(generated_monotonic_ns)
        if timestamp <= 0:
            raise ValueError("output feasibility timestamp must be positive")
        if self._initial_rad is None:
            raise RuntimeError("output feasibility tracker has no authoritative initial state")
        if self._last_accepted_ns is None:
            interval_ns = self.servo_period_ns
            emitted = self._initial_rad
        else:
            interval_ns = timestamp - self._last_accepted_ns
            if interval_ns <= 0:
                raise ValueError("accepted-target generation timestamps must be strictly monotonic")
            emitted = self._emitted_before_replacement(interval_ns)
        interval_s = interval_ns / 1e9
        delta = tuple(
            continuous_joint_delta(value, start)
            for value, start in zip(candidate, emitted, strict=True)
        )
        velocity = tuple(value / interval_s for value in delta)
        violating = tuple(
            index
            for index, value in enumerate(velocity)
            if abs(value)
            > self.maximum_velocity_rad_s + self.numeric_tolerance_rad_s
        )
        return JointOutputFeasibility(
            generated_monotonic_ns=timestamp,
            interval_ns=interval_ns,
            segment_start_rad=emitted,
            candidate_rad=candidate,
            delta_rad=delta,
            predicted_velocity_rad_s=velocity,
            violating_joint_indices=violating,
            maximum_velocity_rad_s=max(map(abs, velocity), default=0.0),
            boundary_rad_s=self.maximum_velocity_rad_s,
        )

    def commit(self, prediction: JointOutputFeasibility) -> None:
        if not prediction.feasible:
            raise ValueError("an infeasible joint target cannot enter the accepted contract")
        if self._last_accepted_ns is not None and (
            prediction.generated_monotonic_ns <= self._last_accepted_ns
        ):
            raise ValueError("accepted-target generation timestamps must be strictly monotonic")
        self._segment_start_rad = prediction.segment_start_rad
        self._segment_destination_rad = prediction.candidate_rad
        self._segment_duration_ns = prediction.interval_ns
        self._last_accepted_ns = prediction.generated_monotonic_ns

    def _emitted_before_replacement(self, elapsed_ns: int) -> tuple[float, ...]:
        assert self._segment_start_rad is not None
        assert self._segment_destination_rad is not None
        emitted_elapsed_ns = (elapsed_ns // self.servo_period_ns) * self.servo_period_ns
        alpha = min(1.0, emitted_elapsed_ns / self._segment_duration_ns)
        return tuple(
            start + alpha * continuous_joint_delta(destination, start)
            for start, destination in zip(
                self._segment_start_rad,
                self._segment_destination_rad,
                strict=True,
            )
        )

    @staticmethod
    def _validated_joints(values: Sequence[float]) -> tuple[float, ...]:
        joints = tuple(float(value) for value in values)
        if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
            raise ValueError("output feasibility requires six finite joint radians")
        return joints
