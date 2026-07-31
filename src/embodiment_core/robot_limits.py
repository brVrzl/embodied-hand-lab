"""Robot limits shared by simulation, data validation, and control boundaries."""

from __future__ import annotations

import math

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


def safe_jaka_mini2_joint_limits_rad(
    margin_rad: float = DEFAULT_JOINT_LIMIT_MARGIN_RAD,
) -> tuple[tuple[float, float], ...]:
    """Return the conservative limits after applying a nonnegative margin."""

    margin = max(0.0, float(margin_rad))
    return tuple(
        (lower + margin, upper - margin)
        for lower, upper in JAKA_MINI2_JOINT_LIMITS_RAD
    )
