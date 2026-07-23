"""Accepted JAKA arm targets and deliberately thin output adapters."""

from __future__ import annotations

from typing import Protocol, Sequence

from teleoperation.accepted_target import (
    AcceptedArmTarget,
    AcceptedTcpPose,
    ArmControlHeartbeat,
)


class ArmTargetOutputAdapter(Protocol):
    """Post-pipeline boundary: adapters may only consume the accepted target."""

    def apply(self, target: AcceptedArmTarget) -> bool: ...

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool: ...


class MujocoArmTargetAdapter:
    """Apply the accepted joint target to the existing MuJoCo plant."""

    def __init__(self, simulation: object) -> None:
        self.simulation = simulation
        self.applied_count = 0

    def apply(self, target: AcceptedArmTarget) -> bool:
        setter = getattr(self.simulation, "set_accepted_arm_joint_target")
        setter(target.joint_position_rad)
        self.applied_count += 1
        return True

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        return True


class CompositeArmTargetAdapter:
    """Fan out one immutable accepted target without recomputing it."""

    def __init__(self, adapters: Sequence[ArmTargetOutputAdapter]) -> None:
        self.adapters = tuple(adapters)

    def apply(self, target: AcceptedArmTarget) -> bool:
        return all(adapter.apply(target) for adapter in self.adapters)

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        return all(adapter.heartbeat(heartbeat) for adapter in self.adapters)


class RecordingArmTargetAdapter:
    """In-memory shadow adapter used by parity and contract tests."""

    def __init__(self) -> None:
        self.targets: list[AcceptedArmTarget] = []
        self.heartbeats: list[ArmControlHeartbeat] = []

    def apply(self, target: AcceptedArmTarget) -> bool:
        self.targets.append(target)
        return True

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        self.heartbeats.append(heartbeat)
        return True
