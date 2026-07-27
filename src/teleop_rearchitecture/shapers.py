"""Deterministic 125 Hz, latest-wins, jerk-bounded offline output shapers.

They are deliberately not a hardware adapter.  They model two architecture
candidates after full IK: a resolved-rate velocity servo (B) and a streaming
joint-position servo with target feed-forward (C).  Both provide the same
strict output contract and bounded controlled stop for clutch release only.
Hard failures are reported to the caller and must stop transport immediately.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable


Joint = tuple[float, float, float, float, float, float]


def _joint(values: Iterable[float]) -> Joint:
    result = tuple(float(v) for v in values)
    if len(result) != 6 or not all(math.isfinite(v) for v in result):
        raise ValueError("expected six finite joint values")
    return result  # type: ignore[return-value]


def _clip(value: float, magnitude: float) -> float:
    return max(-magnitude, min(magnitude, value))


@dataclass(frozen=True, slots=True)
class ShaperLimits:
    period_s: float = 0.008
    maximum_velocity_rad_s: float = math.pi
    maximum_acceleration_rad_s2: float = 4.0 * math.pi
    # A prototype policy bound, deliberately not presented as a Mini2 limit.
    # It is near the best offline Ruckig sweep values in the adjacent research
    # worktree while retaining an 8 ms streaming interface.
    maximum_jerk_rad_s3: float = 50.0
    target_horizon_s: float = 0.250

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value) and value > 0.0
            for value in (
                self.period_s,
                self.maximum_velocity_rad_s,
                self.maximum_acceleration_rad_s2,
                self.maximum_jerk_rad_s3,
                self.target_horizon_s,
            )
        ):
            raise ValueError("shaper limits must be finite and positive")


@dataclass(frozen=True, slots=True)
class ShaperPoint:
    position_rad: Joint
    velocity_rad_s: Joint
    acceleration_rad_s2: Joint
    jerk_rad_s3: Joint
    target_error_rad: Joint


class _BaseJerkServo:
    def __init__(self, initial_position_rad: Iterable[float], limits: ShaperLimits) -> None:
        self.limits = limits
        self.position = _joint(initial_position_rad)
        self.velocity: Joint = (0.0,) * 6
        self.acceleration: Joint = (0.0,) * 6
        self.target = self.position
        self.target_timestamp_ns: int | None = None
        self._stopping = False

    def set_target(
        self,
        target_rad: Iterable[float],
        *,
        timestamp_ns: int | None = None,
    ) -> None:
        if timestamp_ns is not None and (
            isinstance(timestamp_ns, bool)
            or not isinstance(timestamp_ns, int)
            or timestamp_ns < 0
        ):
            raise ValueError("target timestamp must be a non-negative integer nanosecond value")
        self.target = _joint(target_rad)
        self.target_timestamp_ns = timestamp_ns
        self._stopping = False

    def request_controlled_stop(self) -> None:
        """Bounded deceleration used only for an explicit clutch release."""

        # A position equal to the current point would cause a jerk-limited
        # servo that already has velocity to brake, reverse, and hunt.  Aim at
        # the bounded constant-deceleration stopping point instead; the normal
        # tracking law then settles there without a discontinuous hold command.
        self.target = _joint(
            position
            + math.copysign(
                velocity * velocity / (2.0 * self.limits.maximum_acceleration_rad_s2),
                velocity,
            )
            for position, velocity in zip(self.position, self.velocity, strict=True)
        )
        self._stopping = False

    def _desired_acceleration(self) -> Joint:
        raise NotImplementedError

    def tick(self) -> ShaperPoint:
        dt = self.limits.period_s
        desired_acceleration = self._desired_acceleration()
        next_acceleration = tuple(
            current + _clip(
                desired - current, self.limits.maximum_jerk_rad_s3 * dt
            )
            for desired, current in zip(desired_acceleration, self.acceleration, strict=True)
        )
        next_velocity = tuple(
            _clip(current + acceleration * dt, self.limits.maximum_velocity_rad_s)
            for current, acceleration in zip(self.velocity, next_acceleration, strict=True)
        )
        next_position = tuple(
            current + velocity * dt
            for current, velocity in zip(self.position, next_velocity, strict=True)
        )
        jerk = tuple(
            (new - old) / dt
            for new, old in zip(next_acceleration, self.acceleration, strict=True)
        )
        self.position = _joint(next_position)
        self.velocity = _joint(next_velocity)
        self.acceleration = _joint(next_acceleration)
        return ShaperPoint(
            position_rad=self.position,
            velocity_rad_s=self.velocity,
            acceleration_rad_s2=self.acceleration,
            jerk_rad_s3=_joint(jerk),
            target_error_rad=_joint(
                target - position for target, position in zip(self.target, self.position, strict=True)
            ),
        )


class ResolvedRateVelocityServo(_BaseJerkServo):
    """Candidate B: velocity servo after a robot-independent differential IK.

    The replay already supplies accepted joints from the shared full IK.  This
    class therefore evaluates the output side of B without claiming to replace
    that IK or to model a physical robot's tracking dynamics.
    """

    def _desired_acceleration(self) -> Joint:
        desired_velocity = tuple(
            math.copysign(
                min(
                    abs(target - position) / self.limits.target_horizon_s,
                    math.sqrt(2.0 * self.limits.maximum_acceleration_rad_s2 * abs(target - position)),
                    self.limits.maximum_velocity_rad_s,
                ),
                target - position,
            )
            for target, position in zip(self.target, self.position, strict=True)
        )
        return _joint(
            _clip(8.0 * (target - current), self.limits.maximum_acceleration_rad_s2)
            for target, current in zip(desired_velocity, self.velocity, strict=True)
        )


class JerkBoundedPositionServo(_BaseJerkServo):
    """Candidate C: full-IK position stream plus a target-velocity feed-forward.

    This is an independently implemented contract model, not copied Ruckig
    code.  It predicts the latest target velocity and blends it with position
    error; acceleration and jerk remain explicitly bounded at every 8 ms tick.
    """

    def __init__(self, initial_position_rad: Iterable[float], limits: ShaperLimits) -> None:
        super().__init__(initial_position_rad, limits)
        self._target_velocity: Joint = (0.0,) * 6

    @property
    def target_velocity_rad_s(self) -> Joint:
        """One-replacement feed-forward velocity derived from source time."""

        return self._target_velocity

    def set_target(
        self,
        target_rad: Iterable[float],
        *,
        timestamp_ns: int | None = None,
    ) -> None:
        target = _joint(target_rad)
        if timestamp_ns is not None and self.target_timestamp_ns is not None:
            delta_ns = timestamp_ns - self.target_timestamp_ns
            if delta_ns <= 0:
                raise ValueError("target timestamps must increase")
            delta_s = delta_ns / 1e9
            self._target_velocity = _joint(
                (new - old) / delta_s
                for new, old in zip(target, self.target, strict=True)
            )
        else:
            self._target_velocity = (0.0,) * 6
        super().set_target(target, timestamp_ns=timestamp_ns)

    def request_controlled_stop(self) -> None:
        # A clutch release supersedes any not-yet-consumed target replacement.
        # Otherwise one stale feed-forward impulse could be applied after the
        # stop request when release lands between set_target() and tick().
        self._target_velocity = (0.0,) * 6
        super().request_controlled_stop()

    def _desired_acceleration(self) -> Joint:
        result = _joint(
            _clip(
                36.0 * (target - position) + 10.0 * (target_velocity - velocity),
                self.limits.maximum_acceleration_rad_s2,
            )
            for target, position, target_velocity, velocity in zip(
                self.target, self.position, self._target_velocity, self.velocity, strict=True
            )
        )
        # Feed-forward applies to the target replacement that created it, not
        # indefinitely while the target remains unchanged.
        self._target_velocity = (0.0,) * 6
        return result
