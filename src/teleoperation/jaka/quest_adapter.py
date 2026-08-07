"""Thin accepted-joint-target adapter for the JAKA native transport."""

from __future__ import annotations

import time
from typing import Protocol

from teleoperation.accepted_target import AcceptedArmTarget, ArmControlHeartbeat

from ..runtime.arm_only import ArmOnlyRuntime
from ..wire import (
    heartbeat_target_packet,
    hold_current_target_packet,
    joint_position_target_packet,
)


JAKA_JOINT_ORDER = tuple(f"jaka_joint_{index}" for index in range(1, 7))
JAKA_COMMAND_MODE = "edg_servo_j_absolute"
JAKA_JOINT_ANGLE_UNIT = "rad"

class _PacketRuntime(Protocol):
    def dispatch_packet(self, packet: object) -> bool: ...
    def dispatch_stop(self, *, sequence: int) -> bool: ...


class JakaAcceptedJointTargetAdapter:
    """Representation-only boundary: SI radians/J1..J6 go out unchanged."""

    def __init__(
        self,
        runtime: ArmOnlyRuntime | _PacketRuntime,
        *,
        allow_motion: bool,
        joint_order: tuple[str, ...] = JAKA_JOINT_ORDER,
        joint_angle_unit: str = JAKA_JOINT_ANGLE_UNIT,
        command_mode: str = JAKA_COMMAND_MODE,
    ) -> None:
        if tuple(joint_order) != JAKA_JOINT_ORDER:
            raise ValueError("JAKA adapter requires canonical J1..J6 ordering")
        if joint_angle_unit != JAKA_JOINT_ANGLE_UNIT:
            raise ValueError("JAKA EDG joint targets must be radians")
        if command_mode != JAKA_COMMAND_MODE:
            raise ValueError("JAKA adapter requires absolute EDG servo-j mode")
        self.runtime = runtime
        self.allow_motion = bool(allow_motion)
        self.applied_count = 0
        self.last_sequence = 0
        self.stopped = False

    def apply(self, target: AcceptedArmTarget) -> bool:
        if self.stopped:
            return False
        dispatch_ns = max(time.monotonic_ns(), target.generated_monotonic_ns)
        self.last_sequence += 1
        packet = joint_position_target_packet(
            sequence=self.last_sequence,
            joint_position_rad=target.joint_position_rad,
            local_receive_ns=target.input_receive_monotonic_ns,
            processing_ns=target.generated_monotonic_ns,
            dispatch_ns=dispatch_ns,
            allow_motion=self.allow_motion,
        )
        sent = self.runtime.dispatch_packet(packet)
        if sent:
            self.applied_count += 1
        return sent

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        if self.stopped:
            return False
        self.last_sequence += 1
        dispatch_ns = max(time.monotonic_ns(), heartbeat.generated_monotonic_ns)
        packet = heartbeat_target_packet(
            sequence=self.last_sequence,
            input_sequence=heartbeat.input_sequence_number,
            local_receive_ns=heartbeat.input_receive_monotonic_ns,
            processing_ns=heartbeat.generated_monotonic_ns,
            dispatch_ns=dispatch_ns,
            last_accepted_target_sequence=heartbeat.last_accepted_target_sequence,
            control_state_code=1,
            allow_motion=self.allow_motion,
        )
        return self.runtime.dispatch_packet(packet)

    def stop(self) -> bool:
        self.stopped = True
        self.last_sequence += 1
        return self.runtime.dispatch_stop(sequence=self.last_sequence)

    def pause(self) -> bool:
        """Request recoverable braking; unlike stop(), future targets remain allowed."""

        if self.stopped:
            return False
        self.last_sequence += 1
        return self.runtime.dispatch_packet(
            hold_current_target_packet(
                sequence=self.last_sequence,
                monotonic_ns=time.monotonic_ns(),
            )
        )
