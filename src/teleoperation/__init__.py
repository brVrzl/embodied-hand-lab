"""Clean-slate, arm-only teleoperation foundation.

This package must not import :mod:`teleop_tools`, HEBI, TeleDex, or RH56.  Input
devices and robot transports are adapters at the package boundary; the active
Gates 1-2 runtime is synthetic/fake by default.
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
    TimingStatistics,
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
    "TimingStatistics",
]
