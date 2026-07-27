"""Offline-only interfaces and replay prototypes for teleoperation rearchitecture.

This package intentionally contains no JAKA SDK imports, socket client, ROS
node, or actuator code.  It is a testbed for the robot-independent boundary
between accepted commands and a future hardware adapter.
"""

from .contracts import CommandState, JointCommand, LatestCommandMailbox, StopReason
from .health import output_must_terminate
from .shapers import (
    JerkBoundedPositionServo,
    ResolvedRateVelocityServo,
    ShaperLimits,
)

__all__ = [
    "CommandState",
    "JointCommand",
    "LatestCommandMailbox",
    "StopReason",
    "output_must_terminate",
    "JerkBoundedPositionServo",
    "ResolvedRateVelocityServo",
    "ShaperLimits",
]
