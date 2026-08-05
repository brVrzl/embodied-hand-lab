"""Robot limits shared by simulation, data validation, and control boundaries."""

from __future__ import annotations

import math
from collections.abc import Sequence

# Conservative JAKA Mini2 joint position limits in radians.  These values are
# checked against the maintained MJCF and native worker constants by tests.
JAKA_MINI2_JOINT_LIMITS_RAD: tuple[tuple[float, float], ...] = (
    (-6.28, 6.28),
    (-2.09, 2.09),
    (-2.27, 2.27),
    (-6.28, 6.28),
    (-2.09, 2.09),
    (-6.28, 6.28),
)

DEFAULT_JOINT_LIMIT_MARGIN_RAD = math.radians(5.0)
PERIODIC_JOINT_INDICES: tuple[int, ...] = (0, 3, 5)


def shortest_equivalent_delta_rad(value_rad: float, reference_rad: float) -> float:
    """Return the signed shortest angular delta between equivalent angles."""

    return math.remainder(float(value_rad) - float(reference_rad), 2.0 * math.pi)


def select_nearest_equivalent_joint_branch(
    candidate_rad: Sequence[float],
    reference_rad: Sequence[float],
    *,
    joint_limits_rad: Sequence[tuple[float, float]] = JAKA_MINI2_JOINT_LIMITS_RAD,
) -> tuple[tuple[float, ...], tuple[int, ...]]:
    """Select periodic joint representations nearest a fresh measured state.

    IK may return an angle equivalent to the measured angle plus or minus one
    revolution.  Only representations inside the project's per-joint limits
    are considered; non-periodic joints are passed through unchanged.
    """

    if len(candidate_rad) != 6 or len(reference_rad) != 6:
        raise ValueError("candidate and reference must contain six joints")
    if len(joint_limits_rad) != 6:
        raise ValueError("joint_limits_rad must contain six limits")
    selected = [float(value) for value in candidate_rad]
    offsets = [0] * 6
    for index in PERIODIC_JOINT_INDICES:
        value = float(candidate_rad[index])
        reference = float(reference_rad[index])
        lower, upper = (float(v) for v in joint_limits_rad[index])
        choices = [
            (value + offset * 2.0 * math.pi, offset)
            for offset in range(-2, 3)
            if lower - 1e-12 <= value + offset * 2.0 * math.pi <= upper + 1e-12
        ]
        if not choices:
            raise ValueError(
                f"joint {index + 1} has no equivalent representation inside limits"
            )
        chosen, offset = min(
            choices,
            key=lambda item: (abs(item[0] - reference), abs(item[1])),
        )
        selected[index] = chosen
        offsets[index] = offset
    return tuple(selected), tuple(offsets)


def safe_jaka_mini2_joint_limits_rad(
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> tuple[tuple[float, float], ...]:
    """Return the conservative limits after applying a nonnegative margin."""

    margin = max(0.0, float(margin_rad))
    return tuple(
        (lower + margin, upper - margin)
        for lower, upper in JAKA_MINI2_JOINT_LIMITS_RAD
    )
