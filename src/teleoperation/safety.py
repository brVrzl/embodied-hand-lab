from __future__ import annotations

import math


def validate_zero_motion_target(
    observed_joint_rad: tuple[float, ...],
    target_joint_rad: tuple[float, ...],
    *,
    maximum_initial_delta_rad: float = 1e-4,
) -> None:
    if len(observed_joint_rad) != 6 or len(target_joint_rad) != 6:
        raise ValueError("zero-motion validation requires six joint values")
    values = (*observed_joint_rad, *target_joint_rad, maximum_initial_delta_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("zero-motion inputs must be finite")
    if not 0.0 < maximum_initial_delta_rad <= 0.01:
        raise ValueError("maximum initial delta is outside the conservative range")
    delta = max(abs(a - b) for a, b in zip(observed_joint_rad, target_joint_rad))
    if delta > maximum_initial_delta_rad:
        raise ValueError(f"initial command delta {delta:.9f} rad exceeds {maximum_initial_delta_rad:.9f} rad")
