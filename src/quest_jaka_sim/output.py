"""Accepted JAKA arm targets and deliberately thin output adapters."""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Any, Protocol, Sequence

import numpy as np

from teleoperation.accepted_target import (
    AcceptedArmTarget,
    AcceptedTcpPose,
    ArmControlHeartbeat,
)
from .production_resampler import (
    ProductionJointServoResampler,
    SERVO_PERIOD_NS,
)


class ArmOutputMode(str, Enum):
    SHAPED_500HZ = "shaped-500hz"
    JAKA_EQUIVALENT_125HZ = "jaka-equivalent-125hz"


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
        self.simulation.set_accepted_arm_tcp_pose(target.filtered_tcp)
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


class JakaEquivalent125HzMujocoAdapter:
    """Drive MuJoCo from the production 8 ms latest-destination PWL output."""

    def __init__(self, simulation: Any, *, library_path: Path | None = None) -> None:
        self.simulation = simulation
        self.resampler = ProductionJointServoResampler(library_path)
        self._time_origin_ns = 1
        self._next_emit_simulation_ns = 0
        self._last_emit_resampler_ns = self._time_origin_ns
        self._clock_started = False
        self._accepted: dict[int, AcceptedArmTarget] = {}
        self._last_q = np.asarray(simulation.arm_joints_rad, dtype=float)
        self._clutch_generation: int | None = None
        self._pending_destination_replacement = False
        self._control_state = "DISENGAGED"
        self.records: list[dict[str, object]] = []
        self.applied_count = 0
        hard_acceleration = float(
            simulation.config.raw.get("hardware_adapter", {}).get(
                "native_hard_output_joint_acceleration_rad_s2",
                simulation.config.output_contract.maximum_acceleration_rad_s2,
            )
        )
        self.resampler.configure_transition(
            maximum_velocity_rad_s=simulation.config.output_contract.velocity_boundaries_rad_s,
            recoverable_acceleration_rad_s2=(
                simulation.config.output_contract.maximum_acceleration_rad_s2
            ),
            hard_acceleration_rad_s2=hard_acceleration,
            maximum_jerk_rad_s3=simulation.config.command_limits.maximum_jerk_rad_s3,
        )
        self.resampler.initialize(self._last_q, self._time_origin_ns)
        self.resampler.hold(self._last_q, self._time_origin_ns, 0)
        simulation.enable_direct_125hz_arm_output()

    def apply(self, target: AcceptedArmTarget) -> bool:
        if target.clutch_generation != self._clutch_generation:
            self._reset_from_simulation()
            self._clutch_generation = target.clutch_generation
        self.resampler.accept(
            target.joint_position_rad,
            target.generated_monotonic_ns,
            target.sequence_number,
        )
        self._accepted[target.sequence_number] = target
        self.simulation.set_accepted_arm_tcp_pose(target.filtered_tcp)
        self._pending_destination_replacement = True
        self._control_state = "ACTIVE"
        self.applied_count += 1
        return True

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        self._control_state = heartbeat.state.value
        return True

    def advance_to(self, simulation_time_s: float) -> None:
        now_ns = int(round(float(simulation_time_s) * 1e9))
        if not self._clock_started:
            self._next_emit_simulation_ns = (
                now_ns // SERVO_PERIOD_NS
            ) * SERVO_PERIOD_NS
            self._clock_started = True
        while self._next_emit_simulation_ns <= now_ns:
            point = self.resampler.evaluate_and_commit(
                self._time_origin_ns + self._next_emit_simulation_ns
            )
            q = np.asarray(point.position_rad, dtype=float)
            dq = np.asarray(point.emitted_velocity_rad_s, dtype=float)
            ddq = np.asarray(point.emitted_acceleration_rad_s2, dtype=float)
            self.simulation.set_emitted_arm_joint_target(tuple(q))
            source = self._accepted.get(point.to_sequence)
            actual = self.simulation.arm_joints_rad
            self.records.append(
                {
                    "emitted_sequence": len(self.records) + 1,
                    "emitted_simulation_time_s": self._next_emit_simulation_ns / 1e9,
                    "emitted_monotonic_ns": point.servo_time_ns,
                    "emitted_time_domain": "simulation_monotonic",
                    "source_accepted_sequence": point.to_sequence,
                    "source_accepted_timestamp_ns": None if source is None else source.generated_monotonic_ns,
                    "source_segment_from_sequence": point.from_sequence,
                    "source_segment_from_timestamp_ns": point.from_accepted_ns,
                    "q_emit_rad": q.tolist(),
                    "dq_emit_rad_s": dq.tolist(),
                    "ddq_emit_rad_s2": ddq.tolist(),
                    "segment_velocity_rad_s": list(point.segment_velocity_rad_s),
                    "segment_alpha": point.alpha,
                    "segment_endpoint": point.endpoint,
                    "destination_replacement": self._pending_destination_replacement,
                    "hold_state": self._control_state,
                    "jerk_emit_rad_s3": list(point.emitted_jerk_rad_s3),
                    "transition_limited": point.transition_limited,
                    "recovered_from_transition": point.recovered_from_transition,
                    "actual_joint_position_rad": actual.tolist(),
                    "command_actual_error_rad": (q - actual).tolist(),
                }
            )
            self._pending_destination_replacement = False
            self._last_q = q
            self._last_emit_resampler_ns = point.servo_time_ns
            self._next_emit_simulation_ns += SERVO_PERIOD_NS

    def _reset_from_simulation(self) -> None:
        current = np.asarray(self.simulation.arm_joints_rad, dtype=float)
        self.resampler.initialize(current, self._last_emit_resampler_ns)
        self.resampler.hold(current, self._last_emit_resampler_ns, 0)
        self._accepted.clear()
        self._last_q = current
        self._pending_destination_replacement = False

    def report(self) -> dict[str, object]:
        maximum_velocity = max(
            (max(abs(value) for value in row["dq_emit_rad_s"]) for row in self.records),
            default=0.0,
        )
        maximum_acceleration = max(
            (max(abs(value) for value in row["ddq_emit_rad_s2"]) for row in self.records),
            default=0.0,
        )
        return {
            "arm_output_mode": ArmOutputMode.JAKA_EQUIVALENT_125HZ.value,
            "arm_emitted_rate_hz": 125.0,
            "arm_emitted_time_domain": "simulation_monotonic",
            "arm_emitted_count": len(self.records),
            "maximum_emitted_joint_velocity_rad_s": maximum_velocity,
            "maximum_emitted_joint_acceleration_rad_s2": maximum_acceleration,
            "production_resampler_library": str(self.resampler.library_path.resolve()),
            "production_transition_recovery_enabled": True,
            "transition_limited_emitted_count": sum(
                bool(row["transition_limited"]) for row in self.records
            ),
        }

    def close(self) -> None:
        self.resampler.close()
