"""Shared arm-target, safety, supervision, and transport contracts.

This package must not import device-specific input stacks or the RH56 driver.
Quest input and robot transports remain adapters at the package boundary.
"""

from .contracts import (
    ArmPoseSample,
    CommandAcknowledgement,
    ControllerState,
    HealthState,
    PoseTarget,
    RobotState,
    SafetyState,
    TimestampSet,
)
from .accepted_target import (
    AcceptedArmTarget,
    AcceptedTargetDiagnostics,
    AcceptedTcpPose,
    ArmControlHeartbeat,
    ArmControlState,
)

__all__ = [
    "ArmPoseSample",
    "AcceptedArmTarget",
    "AcceptedTargetDiagnostics",
    "AcceptedTcpPose",
    "ArmControlHeartbeat",
    "ArmControlState",
    "CommandAcknowledgement",
    "ControllerState",
    "HealthState",
    "PoseTarget",
    "RobotState",
    "SafetyState",
    "TimestampSet",
]
