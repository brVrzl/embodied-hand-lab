"""Accepted JAKA arm targets and deliberately thin output adapters."""

from __future__ import annotations

from collections import deque
from collections.abc import Iterator
from enum import Enum
from pathlib import Path
from typing import Any, Protocol, Sequence

import numpy as np

from teleoperation.accepted_target import (
    AcceptedArmTarget,
    ArmControlHeartbeat,
    AcceptedTcpPose,
)
from .production_resampler import (
    ProductionJointServoResampler,
)


class ArmOutputMode(str, Enum):
    SHAPED_500HZ = "shaped-500hz"
    JAKA_EQUIVALENT_125HZ = "jaka-equivalent-125hz"


DEFAULT_EMITTED_RECORD_CAPACITY = 8_192


class _BoundedRecordWindow(Sequence[dict[str, object]]):
    """Bounded records with monotonic indices for incremental consumers.

    ``len(window)`` is the total number of records ever appended, so an
    existing ``cursor = len(window)`` remains valid after the retained window
    wraps. Iteration and slices return only records that are still retained.
    """

    def __init__(self, capacity: int) -> None:
        if capacity <= 0:
            raise ValueError("record capacity must be positive")
        self.capacity = int(capacity)
        self._records: deque[dict[str, object]] = deque(maxlen=self.capacity)
        self._total_count = 0

    @property
    def retained_count(self) -> int:
        return len(self._records)

    @property
    def dropped_count(self) -> int:
        return self._total_count - len(self._records)

    @property
    def retained_start_index(self) -> int:
        return self._total_count - len(self._records)

    def append(self, record: dict[str, object]) -> None:
        self._records.append(record)
        self._total_count += 1

    def __len__(self) -> int:
        return self._total_count

    def __iter__(self) -> Iterator[dict[str, object]]:
        return iter(tuple(self._records))

    def __getitem__(
        self, index: int | slice
    ) -> dict[str, object] | list[dict[str, object]]:
        if isinstance(index, slice):
            start, stop, step = index.indices(self._total_count)
            retained_start = self.retained_start_index
            return [
                self._records[global_index - retained_start]
                for global_index in range(start, stop, step)
                if global_index >= retained_start
            ]
        global_index = int(index)
        if global_index < 0:
            global_index += self._total_count
        if not 0 <= global_index < self._total_count:
            raise IndexError("record index out of range")
        retained_offset = global_index - self.retained_start_index
        if retained_offset < 0:
            raise IndexError("record has been evicted from the retained window")
        return self._records[retained_offset]


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
    """Fan out one immutable target and aggregate non-exceptional failures.

    A ``False`` result from one adapter does not short-circuit later adapters.
    Adapter exceptions remain terminal and propagate to the caller.
    """

    def __init__(self, adapters: Sequence[ArmTargetOutputAdapter]) -> None:
        self.adapters = tuple(adapters)

    def apply(self, target: AcceptedArmTarget) -> bool:
        results = tuple(adapter.apply(target) for adapter in self.adapters)
        return all(results)

    def heartbeat(self, heartbeat: ArmControlHeartbeat) -> bool:
        results = tuple(adapter.heartbeat(heartbeat) for adapter in self.adapters)
        return all(results)


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
    """Drive MuJoCo from the configured latest-destination PWL output."""

    def __init__(
        self,
        simulation: Any,
        *,
        library_path: Path | None = None,
        record_capacity: int = DEFAULT_EMITTED_RECORD_CAPACITY,
    ) -> None:
        self.simulation = simulation
        self._servo_period_ns = simulation.config.output_contract.servo_period_ns
        self.resampler = ProductionJointServoResampler(
            library_path, servo_period_ns=self._servo_period_ns
        )
        self._time_origin_ns = 1
        self._next_emit_simulation_ns = 0
        self._last_emit_resampler_ns = self._time_origin_ns
        self._clock_started = False
        self._last_q = np.asarray(simulation.arm_joints_rad, dtype=float)
        self._clutch_generation: int | None = None
        self._pending_destination_replacement = False
        self._control_state = "DISENGAGED"
        self.records = _BoundedRecordWindow(record_capacity)
        self._maximum_emitted_velocity_rad_s = 0.0
        self._maximum_emitted_acceleration_rad_s2 = 0.0
        self._transition_limited_emitted_count = 0
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
                now_ns // self._servo_period_ns
            ) * self._servo_period_ns
            self._clock_started = True
        while self._next_emit_simulation_ns <= now_ns:
            point = self.resampler.evaluate_and_commit(
                self._time_origin_ns + self._next_emit_simulation_ns
            )
            q = np.asarray(point.position_rad, dtype=float)
            dq = np.asarray(point.emitted_velocity_rad_s, dtype=float)
            ddq = np.asarray(point.emitted_acceleration_rad_s2, dtype=float)
            self.simulation.set_emitted_arm_joint_target(tuple(q))
            actual = self.simulation.arm_joints_rad
            record = {
                "emitted_sequence": len(self.records) + 1,
                "emitted_simulation_time_s": self._next_emit_simulation_ns / 1e9,
                "emitted_monotonic_ns": point.servo_time_ns,
                "emitted_time_domain": "simulation_monotonic",
                "source_accepted_sequence": point.to_sequence,
                "source_accepted_timestamp_ns": (
                    point.to_accepted_ns if point.to_sequence > 0 else None
                ),
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
            self.records.append(record)
            self._maximum_emitted_velocity_rad_s = max(
                self._maximum_emitted_velocity_rad_s,
                max(abs(value) for value in point.emitted_velocity_rad_s),
            )
            self._maximum_emitted_acceleration_rad_s2 = max(
                self._maximum_emitted_acceleration_rad_s2,
                max(abs(value) for value in point.emitted_acceleration_rad_s2),
            )
            self._transition_limited_emitted_count += int(
                point.transition_limited
            )
            self._pending_destination_replacement = False
            self._last_q = q
            self._last_emit_resampler_ns = point.servo_time_ns
            self._next_emit_simulation_ns += self._servo_period_ns

    def _reset_from_simulation(self) -> None:
        current = np.asarray(self.simulation.arm_joints_rad, dtype=float)
        self.resampler.initialize(current, self._last_emit_resampler_ns)
        self.resampler.hold(current, self._last_emit_resampler_ns, 0)
        self._last_q = current
        self._pending_destination_replacement = False

    def report(self) -> dict[str, object]:
        transport_hz = 1e9 / self._servo_period_ns
        return {
            "arm_output_mode": ArmOutputMode.JAKA_EQUIVALENT_125HZ.value,
            "arm_emitted_rate_hz": transport_hz,
            "servo_period_ns": self._servo_period_ns,
            "servo_step_num": self._servo_period_ns // 8_000_000,
            "arm_emitted_time_domain": "simulation_monotonic",
            "arm_emitted_count": len(self.records),
            "arm_emitted_record_capacity": self.records.capacity,
            "arm_emitted_record_retained_count": self.records.retained_count,
            "arm_emitted_record_dropped_count": self.records.dropped_count,
            "maximum_emitted_joint_velocity_rad_s": (
                self._maximum_emitted_velocity_rad_s
            ),
            "maximum_emitted_joint_acceleration_rad_s2": (
                self._maximum_emitted_acceleration_rad_s2
            ),
            "production_resampler_library": str(self.resampler.library_path.resolve()),
            "production_transition_recovery_enabled": True,
            "transition_limited_emitted_count": (
                self._transition_limited_emitted_count
            ),
        }

    def close(self) -> None:
        self.resampler.close()
