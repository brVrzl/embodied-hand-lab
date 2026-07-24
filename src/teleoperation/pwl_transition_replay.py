"""Plant-free replay model for the native PWL acceleration transition.

This module is evidence tooling, not a command-path trajectory authority. The
native worker remains authoritative because it alone has the last successful
SDK output timestamp and emitted position/velocity.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True, slots=True)
class MotionSample:
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]
    jerk_rad_s3: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class TransitionReplay:
    current_would_terminate: bool
    naive_position_hold_feasible: bool
    hold_cycles: int
    added_latency_ns: int
    recovered: bool
    selected_samples: tuple[MotionSample, ...]
    maximum_velocity_rad_s: tuple[float, ...]
    maximum_acceleration_rad_s2: tuple[float, ...]
    maximum_jerk_rad_s3: tuple[float, ...]
    maximum_continuity_error_rad: tuple[float, ...]


def motion_from_position(
    *,
    previous_position_rad: Sequence[float],
    previous_velocity_rad_s: Sequence[float],
    previous_acceleration_rad_s2: Sequence[float],
    position_rad: Sequence[float],
    dt_s: float,
) -> MotionSample:
    """Reconstruct controller-visible finite differences for one output."""

    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    arrays = (
        previous_position_rad,
        previous_velocity_rad_s,
        previous_acceleration_rad_s2,
        position_rad,
    )
    if any(len(values) != 6 for values in arrays):
        raise ValueError("PWL replay requires J1 through J6")
    if not all(math.isfinite(value) for values in arrays for value in values):
        raise ValueError("PWL replay values must be finite")
    velocity = tuple(
        (position - previous) / dt_s
        for position, previous in zip(
            position_rad, previous_position_rad, strict=True
        )
    )
    acceleration = tuple(
        (current - previous) / dt_s
        for current, previous in zip(
            velocity, previous_velocity_rad_s, strict=True
        )
    )
    jerk = tuple(
        (current - previous) / dt_s
        for current, previous in zip(
            acceleration, previous_acceleration_rad_s2, strict=True
        )
    )
    return MotionSample(
        tuple(float(value) for value in position_rad),
        velocity,
        acceleration,
        jerk,
    )


def replay_transition_case(
    *,
    previous_position_rad: Sequence[float],
    previous_velocity_rad_s: Sequence[float],
    previous_acceleration_rad_s2: Sequence[float],
    proposed_position_rad: Sequence[float],
    destination_position_rad: Sequence[float],
    first_dt_s: float,
    boundary_rad_s2: float,
    jerk_limit_rad_s3: float | None = None,
    servo_period_s: float = 0.008,
    maximum_cycles: int = 250,
) -> TransitionReplay:
    """Compare terminal PWL, a literal hold, and the adopted velocity slew."""

    if not math.isfinite(boundary_rad_s2) or boundary_rad_s2 <= 0.0:
        raise ValueError("acceleration boundary must be finite and positive")
    if jerk_limit_rad_s3 is not None and (
        not math.isfinite(jerk_limit_rad_s3) or jerk_limit_rad_s3 <= 0.0
    ):
        raise ValueError("jerk limit must be finite and positive")
    current = motion_from_position(
        previous_position_rad=previous_position_rad,
        previous_velocity_rad_s=previous_velocity_rad_s,
        previous_acceleration_rad_s2=previous_acceleration_rad_s2,
        position_rad=proposed_position_rad,
        dt_s=first_dt_s,
    )
    current_would_terminate = any(
        abs(value) > boundary_rad_s2 + 1e-12
        for value in current.acceleration_rad_s2
    )
    literal_hold = motion_from_position(
        previous_position_rad=previous_position_rad,
        previous_velocity_rad_s=previous_velocity_rad_s,
        previous_acceleration_rad_s2=previous_acceleration_rad_s2,
        position_rad=previous_position_rad,
        dt_s=first_dt_s,
    )
    naive_hold_feasible = all(
        abs(value) <= boundary_rad_s2 + 1e-12
        for value in literal_hold.acceleration_rad_s2
    )

    position = tuple(float(value) for value in previous_position_rad)
    velocity = tuple(float(value) for value in previous_velocity_rad_s)
    acceleration = tuple(float(value) for value in previous_acceleration_rad_s2)
    selected: list[MotionSample] = []
    continuity = [0.0] * 6
    for cycle in range(maximum_cycles):
        dt_s = first_dt_s if cycle == 0 else servo_period_s
        raw_position = (
            tuple(float(value) for value in proposed_position_rad)
            if cycle == 0
            else tuple(float(value) for value in destination_position_rad)
        )
        raw = motion_from_position(
            previous_position_rad=position,
            previous_velocity_rad_s=velocity,
            previous_acceleration_rad_s2=acceleration,
            position_rad=raw_position,
            dt_s=dt_s,
        )
        crossing = any(
            abs(value) > boundary_rad_s2 + 1e-12
            for value in raw.acceleration_rad_s2
        )
        if jerk_limit_rad_s3 is not None:
            crossing = crossing or any(
                abs(value) > jerk_limit_rad_s3 + 1e-12
                for value in raw.jerk_rad_s3
            )
        if not crossing:
            selected.append(raw)
            position = raw.position_rad
            velocity = raw.velocity_rad_s
            acceleration = raw.acceleration_rad_s2
            break
        headroom_boundary = boundary_rad_s2 - max(
            1e-9, boundary_rad_s2 * 1e-9
        )
        limited_velocity = tuple(
            max(
                previous - headroom_boundary * dt_s,
                min(
                    previous + headroom_boundary * dt_s,
                    desired,
                ),
            )
            for desired, previous in zip(
                raw.velocity_rad_s, velocity, strict=True
            )
        )
        if jerk_limit_rad_s3 is not None:
            maximum_acceleration_change = jerk_limit_rad_s3 * dt_s
            limited_acceleration = tuple(
                max(
                    previous - maximum_acceleration_change,
                    min(
                        previous + maximum_acceleration_change,
                        (desired - previous_velocity) / dt_s,
                    ),
                )
                for desired, previous_velocity, previous in zip(
                    limited_velocity, velocity, acceleration, strict=True
                )
            )
            limited_velocity = tuple(
                previous_velocity + current_acceleration * dt_s
                for previous_velocity, current_acceleration in zip(
                    velocity, limited_acceleration, strict=True
                )
            )
        limited_position = tuple(
            previous + selected_velocity * dt_s
            for previous, selected_velocity in zip(
                position, limited_velocity, strict=True
            )
        )
        limited = motion_from_position(
            previous_position_rad=position,
            previous_velocity_rad_s=velocity,
            previous_acceleration_rad_s2=acceleration,
            position_rad=limited_position,
            dt_s=dt_s,
        )
        selected.append(limited)
        for joint, (proposed, emitted) in enumerate(
            zip(raw_position, limited_position, strict=True)
        ):
            continuity[joint] = max(
                continuity[joint], abs(proposed - emitted)
            )
        position = limited.position_rad
        velocity = limited.velocity_rad_s
        acceleration = limited.acceleration_rad_s2

    maximum_velocity = tuple(
        max((abs(sample.velocity_rad_s[joint]) for sample in selected), default=0.0)
        for joint in range(6)
    )
    maximum_acceleration = tuple(
        max((abs(sample.acceleration_rad_s2[joint]) for sample in selected), default=0.0)
        for joint in range(6)
    )
    maximum_jerk = tuple(
        max((abs(sample.jerk_rad_s3[joint]) for sample in selected), default=0.0)
        for joint in range(6)
    )
    recovered = bool(selected) and selected[-1].position_rad == tuple(
        float(value) for value in destination_position_rad
    )
    hold_cycles = len(selected) - int(recovered)
    return TransitionReplay(
        current_would_terminate=current_would_terminate,
        naive_position_hold_feasible=naive_hold_feasible,
        hold_cycles=hold_cycles,
        added_latency_ns=hold_cycles * int(round(servo_period_s * 1e9)),
        recovered=recovered,
        selected_samples=tuple(selected),
        maximum_velocity_rad_s=maximum_velocity,
        maximum_acceleration_rad_s2=maximum_acceleration,
        maximum_jerk_rad_s3=maximum_jerk,
        maximum_continuity_error_rad=tuple(continuity),
    )
