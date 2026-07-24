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
    maximum_acceleration_rad_s2: float = math.inf
    numeric_tolerance_rad_s: float = OUTPUT_FEASIBILITY_NUMERIC_TOLERANCE_RAD_S
    maximum_velocity_rad_s_per_joint: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.maximum_velocity_rad_s) or self.maximum_velocity_rad_s <= 0.0:
            raise ValueError("output velocity boundary must be finite and positive")
        if not isinstance(self.servo_period_ns, int) or self.servo_period_ns <= 0:
            raise ValueError("servo period must be a positive integer nanosecond count")
        if self.maximum_acceleration_rad_s2 <= 0.0 or math.isnan(self.maximum_acceleration_rad_s2):
            raise ValueError("output acceleration boundary must be positive")
        if not math.isfinite(self.numeric_tolerance_rad_s) or self.numeric_tolerance_rad_s < 0.0:
            raise ValueError("output feasibility tolerance must be finite and non-negative")
        if self.maximum_velocity_rad_s_per_joint is not None:
            boundaries = tuple(
                float(value) for value in self.maximum_velocity_rad_s_per_joint
            )
            if len(boundaries) != 6 or not all(
                math.isfinite(value)
                and 0.0 < value <= self.maximum_velocity_rad_s
                for value in boundaries
            ):
                raise ValueError(
                    "per-joint output velocity boundaries must contain six "
                    "finite positive values no greater than the shared hard boundary"
                )
            object.__setattr__(
                self, "maximum_velocity_rad_s_per_joint", boundaries
            )

    @property
    def velocity_boundaries_rad_s(self) -> tuple[float, ...]:
        if self.maximum_velocity_rad_s_per_joint is None:
            return (self.maximum_velocity_rad_s,) * 6
        return self.maximum_velocity_rad_s_per_joint


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
    previous_emitted_velocity_rad_s: tuple[float, ...]
    predicted_acceleration_rad_s2: tuple[float, ...]
    violating_joint_indices: tuple[int, ...]
    acceleration_violating_joint_indices: tuple[int, ...]
    maximum_velocity_rad_s: float
    maximum_acceleration_rad_s2: float
    boundary_rad_s: float
    acceleration_boundary_rad_s2: float
    boundary_rad_s_per_joint: tuple[float, ...] = ()

    @property
    def feasible(self) -> bool:
        return not self.violating_joint_indices and not self.acceleration_violating_joint_indices


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
        maximum_acceleration_rad_s2: float = math.inf,
        numeric_tolerance_rad_s: float = OUTPUT_FEASIBILITY_NUMERIC_TOLERANCE_RAD_S,
        maximum_velocity_rad_s_per_joint: Sequence[float] | None = None,
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
        if maximum_velocity_rad_s_per_joint is None:
            self.maximum_velocity_rad_s_per_joint = (boundary,) * 6
        else:
            per_joint = tuple(
                float(value) for value in maximum_velocity_rad_s_per_joint
            )
            if len(per_joint) != 6 or not all(
                math.isfinite(value) and 0.0 < value <= boundary
                for value in per_joint
            ):
                raise ValueError(
                    "per-joint output velocity boundaries must contain six "
                    "finite positive values no greater than the shared hard boundary"
                )
            self.maximum_velocity_rad_s_per_joint = per_joint
        self.maximum_acceleration_rad_s2 = float(maximum_acceleration_rad_s2)
        if self.maximum_acceleration_rad_s2 <= 0.0 or math.isnan(self.maximum_acceleration_rad_s2):
            raise ValueError("output acceleration boundary must be positive")
        self.servo_period_ns = servo_period_ns
        self.numeric_tolerance_rad_s = tolerance
        self._initial_rad: tuple[float, ...] | None = None
        self._last_accepted_ns: int | None = None
        self._segment_start_rad: tuple[float, ...] | None = None
        self._segment_destination_rad: tuple[float, ...] | None = None
        self._segment_duration_ns = servo_period_ns
        self._segment_entry_velocity_rad_s: tuple[float, ...] = (0.0,) * 6

    @classmethod
    def from_config(cls, config: JointOutputContractConfig) -> "JointOutputFeasibilityTracker":
        return cls(
            maximum_velocity_rad_s=config.maximum_velocity_rad_s,
            servo_period_ns=config.servo_period_ns,
            maximum_acceleration_rad_s2=config.maximum_acceleration_rad_s2,
            numeric_tolerance_rad_s=config.numeric_tolerance_rad_s,
            maximum_velocity_rad_s_per_joint=(
                config.maximum_velocity_rad_s_per_joint
            ),
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
        self._segment_entry_velocity_rad_s = (0.0,) * len(values)

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
            previous_velocity = (0.0,) * len(candidate)
        else:
            interval_ns = timestamp - self._last_accepted_ns
            if interval_ns <= 0:
                raise ValueError("accepted-target generation timestamps must be strictly monotonic")
            emitted, previous_velocity = self._emitted_state_before_replacement(interval_ns)
        interval_s = interval_ns / 1e9
        delta = tuple(
            continuous_joint_delta(value, start)
            for value, start in zip(candidate, emitted, strict=True)
        )
        servo_period_s = self.servo_period_ns / 1e9
        first_tick_alpha = min(1.0, self.servo_period_ns / interval_ns)
        velocity = tuple(value * first_tick_alpha / servo_period_s for value in delta)
        acceleration = tuple(
            (value - previous) / servo_period_s
            for value, previous in zip(velocity, previous_velocity, strict=True)
        )
        violating = tuple(
            index
            for index, (value, boundary) in enumerate(
                zip(
                    velocity,
                    self.maximum_velocity_rad_s_per_joint,
                    strict=True,
                )
            )
            if abs(value)
            > boundary + self.numeric_tolerance_rad_s
        )
        acceleration_violating = tuple(
            index
            for index, value in enumerate(acceleration)
            if abs(value) > self.maximum_acceleration_rad_s2 + self.numeric_tolerance_rad_s
        )
        return JointOutputFeasibility(
            generated_monotonic_ns=timestamp,
            interval_ns=interval_ns,
            segment_start_rad=emitted,
            candidate_rad=candidate,
            delta_rad=delta,
            predicted_velocity_rad_s=velocity,
            previous_emitted_velocity_rad_s=previous_velocity,
            predicted_acceleration_rad_s2=acceleration,
            violating_joint_indices=violating,
            acceleration_violating_joint_indices=acceleration_violating,
            maximum_velocity_rad_s=max(map(abs, velocity), default=0.0),
            maximum_acceleration_rad_s2=max(map(abs, acceleration), default=0.0),
            boundary_rad_s=self.maximum_velocity_rad_s,
            acceleration_boundary_rad_s2=self.maximum_acceleration_rad_s2,
            boundary_rad_s_per_joint=self.maximum_velocity_rad_s_per_joint,
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
        self._segment_entry_velocity_rad_s = prediction.previous_emitted_velocity_rad_s
        self._last_accepted_ns = prediction.generated_monotonic_ns

    def _emitted_state_before_replacement(
        self, elapsed_ns: int
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        assert self._segment_start_rad is not None
        assert self._segment_destination_rad is not None
        emitted_elapsed_ns = (elapsed_ns // self.servo_period_ns) * self.servo_period_ns
        alpha = min(1.0, emitted_elapsed_ns / self._segment_duration_ns)
        emitted = tuple(
            start + alpha * continuous_joint_delta(destination, start)
            for start, destination in zip(
                self._segment_start_rad,
                self._segment_destination_rad,
                strict=True,
            )
        )
        if emitted_elapsed_ns == 0:
            return emitted, self._segment_entry_velocity_rad_s
        previous_elapsed_ns = emitted_elapsed_ns - self.servo_period_ns
        previous_alpha = min(1.0, previous_elapsed_ns / self._segment_duration_ns)
        previous_emitted = tuple(
            start + previous_alpha * continuous_joint_delta(destination, start)
            for start, destination in zip(
                self._segment_start_rad,
                self._segment_destination_rad,
                strict=True,
            )
        )
        servo_period_s = self.servo_period_ns / 1e9
        velocity = tuple(
            continuous_joint_delta(value, previous) / servo_period_s
            for value, previous in zip(emitted, previous_emitted, strict=True)
        )
        return emitted, velocity

    @staticmethod
    def _validated_joints(values: Sequence[float]) -> tuple[float, ...]:
        joints = tuple(float(value) for value in values)
        if len(joints) != 6 or not all(math.isfinite(value) for value in joints):
            raise ValueError("output feasibility requires six finite joint radians")
        return joints
