"""Fixed-rate, dual-clutch Quest-to-JAKA/RH56 MuJoCo session.

The Quest HTS receiver and its validation remain unchanged.  Controller inputs
arrive through the provider-independent ``set_clutch_samples`` boundary because
HTS does not transport controller state.  With no explicit clutch provider both
outputs remain frozen and fail disengaged.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, replace
import math
import time
from typing import Any

import numpy as np

from motion_input import CanonicalQuestState, HtsCanonicalAssembler, Pose6D, SerializationError, parse_hts_datagram
from motion_input.hts_transport import ReceivedHtsDatagram

from .clutch import (
    AnalogClutchSample,
    ArmClutchMachine,
    ArmClutchState,
    ClutchAction,
    HandClutchMachine,
    HandClutchState,
)
from .hand_retarget import HandRetargetCalibration, InspireRetargetResult, ProjectRh56Retargeter, QuestHandSkeleton
from .precision_mapping import LatchedHeadYawArmMapper
from .output import (
    AcceptedArmTarget,
    ArmTargetOutputAdapter,
    MujocoArmTargetAdapter,
)
from .se3 import PoseSampleBuffer, TimedPoseSample, quaternion_angle_rad
from .simulation import FeasibilityReason, FeasibilityResult, JakaMujocoSimulation, ReplayConfig
from .smooth_operator import Se3FilterProfile


@dataclass(frozen=True, slots=True)
class ArmControlTickResult:
    input_sequence: int | None
    validated_wrist: Pose6D | None
    relative_hand_transform: Pose6D | None
    tcp_target: Pose6D | None
    filtered_tcp_target: Pose6D | None
    feasibility: FeasibilityResult | None
    accepted_target: AcceptedArmTarget | None
    output_applied: bool
    reason: str


class SmoothQuestJakaSession:
    """One coherent fixed-rate session with independent arm and hand clutches."""

    def __init__(
        self,
        config: ReplayConfig,
        simulation: JakaMujocoSimulation,
        *,
        arm_output: ArmTargetOutputAdapter | None = None,
    ) -> None:
        filter_values = config.raw.get("filter", {})
        profile_name = str(filter_values.get("selected_profile", "simulation_exploration"))
        profiles = filter_values.get("profiles", {})
        if profile_name not in profiles:
            raise ValueError(f"unknown SE(3) filter profile {profile_name!r}")
        self.profile = Se3FilterProfile.from_mapping(profile_name, profiles[profile_name])
        self.config = config
        self.simulation = simulation
        self.arm_output = arm_output or MujocoArmTargetAdapter(simulation)
        self.assembler = HtsCanonicalAssembler(stale_after_s=config.stale_after_s)
        rates = config.raw.get("rates", {})
        self.interpolation_delay_ns = int(float(rates.get("interpolation_delay_ms", 20.0)) * 1e6)
        self.buffer: PoseSampleBuffer[CanonicalQuestState] = PoseSampleBuffer(
            capacity=int(rates.get("input_buffer_capacity", 16))
        )
        clutch = config.raw.get("clutches", {})
        pressed_at = float(clutch.get("pressed_at", 0.75))
        released_at = float(clutch.get("released_at", 0.55))
        clutch_stale_s = float(clutch.get("stale_after_ms", 150.0)) / 1000.0
        self.arm_clutch = ArmClutchMachine(
            stale_after_s=clutch_stale_s,
            pressed_at=pressed_at,
            released_at=released_at,
        )
        self.hand_clutch = HandClutchMachine(
            stale_after_s=clutch_stale_s,
            reacquisition_duration_s=float(clutch.get("hand_reacquisition_ms", 200.0)) / 1000.0,
            pressed_at=pressed_at,
            released_at=released_at,
        )
        # Compatibility name for older diagnostics; it is the arm sub-state
        # machine, not a combined arm/hand mode.
        self.operator = self.arm_clutch
        self.arm_mapper = LatchedHeadYawArmMapper(config.mapping, self.profile)
        self.latest_state: CanonicalQuestState | None = None
        self.last_input_sequence: int | None = None
        self.last_desired = simulation.current_tcp_pose
        self.last_reason = FeasibilityReason.DISENGAGED.value
        self.rejections: Counter[str] = Counter()
        self.input_timestamps_ns: list[int] = []
        self.head_timestamps_ns: list[int] = []
        self.index_timestamps_ns: list[int] = []
        self.grip_timestamps_ns: list[int] = []
        self.control_timestamps_ns: list[int] = []
        self.ik_timestamps_ns: list[int] = []
        self.hand_timestamps_ns: list[int] = []
        self.arm_capture_durations_ns: list[int] = []
        self.hand_retarget_durations_ns: list[int] = []
        self.arm_engagement_latencies_ns: list[int] = []
        self.hand_engagement_latencies_ns: list[int] = []
        self.event_records: list[dict[str, Any]] = []
        self.accepted_targets = 0
        self._accepted_sequence = 0
        self.consecutive_rejections = 0
        self.isolated_rejection_hold_count = int(
            config.raw.get("simulation", {}).get("isolated_rejection_hold_count", 2)
        )
        hand_values = config.raw.get("hand_retargeting", {})
        self.hand_enabled = bool(hand_values.get("enabled", False))
        self.hand_retargeter: ProjectRh56Retargeter | None = None
        self.last_hand_result: InspireRetargetResult | None = None
        self.hand_valid_results = 0
        if self.hand_enabled:
            backend, calibration = HandRetargetCalibration.load(hand_values["calibration_path"])
            self.hand_retargeter = ProjectRh56Retargeter(calibration, backend=backend)
        self._held_hand_command = simulation.commanded_hand_target.copy()
        self._hand_reacquire_anchor = self._held_hand_command.copy()
        self._hand_press_receive_ns: int | None = None
        self._index_sample = AnalogClutchSample(0.0, 0, 0, valid=False)
        self._grip_sample = AnalogClutchSample(0.0, 0, 0, valid=False)
        self.left_controller_valid = False
        self.clutch_provider = "unavailable"
        self._last_head_receive_ns: int | None = None
        self._last_index_sequence: int | None = None
        self._last_grip_sequence: int | None = None
        self._hand_updated_this_tick = False

    def set_clutch_samples(
        self,
        *,
        index: AnalogClutchSample,
        grip: AnalogClutchSample,
        left_controller_valid: bool,
        provider: str,
    ) -> None:
        """Publish independent left-controller controls into the session.

        ``provider`` is recorded so fake/replay sources cannot be mistaken for a
        live Quest controller.  Controller pose is intentionally absent.
        """

        if not provider.strip():
            raise ValueError("clutch provider name is required")
        self._index_sample = index
        self._grip_sample = grip
        if index.sequence_number != self._last_index_sequence:
            self.index_timestamps_ns.append(index.host_receive_monotonic_ns)
            self._last_index_sequence = index.sequence_number
        if grip.sequence_number != self._last_grip_sequence:
            self.grip_timestamps_ns.append(grip.host_receive_monotonic_ns)
            self._last_grip_sequence = grip.sequence_number
        self.left_controller_valid = bool(left_controller_valid)
        self.clutch_provider = provider

    def request_toggle(self) -> None:
        raise RuntimeError(
            "high-level toggle removed; inject independent index/grip samples through set_clutch_samples"
        )

    def set_mode(self, mode: str) -> None:
        raise RuntimeError("arm/hand/both mode selection is not part of dual-clutch control")

    @property
    def right_hand_valid(self) -> bool:
        return bool(self.latest_state is not None and self.latest_state.right.tracking_valid)

    def ingest(self, datagram: ReceivedHtsDatagram) -> bool:
        """Validate/cache one HTS datagram; packet arrival never runs IK."""

        try:
            state = self.assembler.ingest(
                parse_hts_datagram(datagram.payload),
                receive_monotonic_ns=datagram.receive_monotonic_ns,
                source_endpoint=datagram.source_endpoint,
                datagram_size=len(datagram.payload),
            )
        except SerializationError:
            # HTS wrist and skeleton share one inseparable hand datagram, so a
            # malformed right-hand packet conservatively faults both channels.
            self.arm_clutch.fault(datagram.receive_monotonic_ns, "MALFORMED_SHARED_RIGHT_HAND_DATA")
            self.hand_clutch.fault(datagram.receive_monotonic_ns, "MALFORMED_SHARED_RIGHT_HAND_DATA")
            self.arm_mapper.clear()
            self.rejections[FeasibilityReason.INPUT_INVALID.value] += 1
            return False
        self.latest_state = state
        if (
            state.head is not None
            and state.head.host_receive_monotonic_ns != self._last_head_receive_ns
        ):
            self.head_timestamps_ns.append(state.head.host_receive_monotonic_ns)
            self._last_head_receive_ns = state.head.host_receive_monotonic_ns
        hand = state.right
        if (
            hand.tracking_valid
            and hand.wrist_pose is not None
            and len(hand.joints) == 21
            and hand.host_sequence_number is not None
            and hand.host_sequence_number != self.last_input_sequence
        ):
            self.last_input_sequence = hand.host_sequence_number
            receive_ns = int(hand.host_receive_monotonic_ns or datagram.receive_monotonic_ns)
            self.input_timestamps_ns.append(receive_ns)
            self.buffer.add(TimedPoseSample(receive_ns, hand.host_sequence_number, hand.wrist_pose, state))
            return True
        return False

    def control_tick(self, now_ns: int) -> ArmControlTickResult:
        self.control_timestamps_ns.append(now_ns)
        state = self.assembler.state(now_monotonic_ns=now_ns)
        interpolated = self.buffer.sample(now_ns - self.interpolation_delay_ns)
        if interpolated is not None and state.right.tracking_valid:
            state = replace(state, right=replace(state.right, wrist_pose=interpolated.pose))
        right = state.right
        wrist_valid = bool(right.tracking_valid and right.wrist_pose is not None)
        skeleton_valid = bool(right.tracking_valid and len(right.joints) == 21)
        head_valid = bool(state.head is not None and state.head.tracking_valid)

        if not self.left_controller_valid:
            if self.arm_clutch.state is not ArmClutchState.TRACKING_FAULT:
                self.arm_clutch.fault(now_ns, "LEFT_CONTROLLER_STALE_OR_INVALID")
                self.arm_mapper.clear()
            if self.hand_clutch.state is not HandClutchState.TRACKING_FAULT:
                self.hand_clutch.fault(now_ns, "LEFT_CONTROLLER_STALE_OR_INVALID")

        # Current HTS validity is coupled: if its shared hand observation is
        # lost, both channels fault.  Provider-independent tests can exercise
        # independent faults directly on the sub-state machines.
        if not right.tracking_valid and (
            self.arm_clutch.state is ArmClutchState.ENGAGED
            or self.hand_clutch.state in {HandClutchState.ENGAGED, HandClutchState.REACQUIRE}
        ):
            if self.arm_clutch.state is ArmClutchState.ENGAGED:
                self.arm_clutch.fault(now_ns, "SHARED_RIGHT_HAND_TRACKING_LOST")
                self.arm_mapper.clear()
            if self.hand_clutch.state in {HandClutchState.ENGAGED, HandClutchState.REACQUIRE}:
                self.hand_clutch.fault(now_ns, "SHARED_RIGHT_HAND_TRACKING_LOST")

        arm_action = self.arm_clutch.step(
            self._index_sample,
            now_ns=now_ns,
            controller_valid=self.left_controller_valid,
            continuous_inputs_valid=wrist_valid,
            capture_inputs_valid=wrist_valid and head_valid,
        )
        hand_state_before = self.hand_clutch.state
        hand_action = self.hand_clutch.step(
            self._grip_sample,
            now_ns=now_ns,
            controller_valid=self.left_controller_valid,
            skeleton_valid=skeleton_valid,
        )
        if (
            hand_state_before is HandClutchState.REACQUIRE
            and self.hand_clutch.state is HandClutchState.ENGAGED
            and self._hand_press_receive_ns is not None
        ):
            self.hand_engagement_latencies_ns.append(max(0, now_ns - self._hand_press_receive_ns))

        self._hand_updated_this_tick = False
        self._update_hand(state, hand_action, now_ns)
        desired = self._arm_target(state, arm_action, now_ns)
        record = self._base_record(state, now_ns)
        if desired is None:
            record.update(accepted=False, reason=FeasibilityReason.DISENGAGED.value)
            self.event_records.append(record)
            return ArmControlTickResult(
                state.right.host_sequence_number,
                state.right.wrist_pose if wrist_valid else None,
                None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
                None,
                None,
                None,
                None,
                False,
                FeasibilityReason.DISENGAGED.value,
            )
        self.ik_timestamps_ns.append(now_ns)
        started = time.perf_counter_ns()
        result = self.simulation.evaluate(
            desired,
            dt_s=(
                self.simulation.model.opt.timestep
                if len(self.ik_timestamps_ns) < 2
                else (self.ik_timestamps_ns[-1] - self.ik_timestamps_ns[-2]) / 1e9
            ),
        )
        record.update(
            desired_tcp=_pose_dict(desired),
            mapped_tcp_target=_pose_dict(desired),
            filtered_tcp_target=_pose_dict(desired),
            metrics=asdict(result.metrics),
            ik_solution_rad=result.joint_target_rad,
            ik_computation_ms=(time.perf_counter_ns() - started) / 1e6,
        )
        if not result.accepted:
            self._handle_rejection(now_ns, result.reason.value)
            record.update(accepted=False, reason=result.reason.value)
            self.event_records.append(record)
            return ArmControlTickResult(
                state.right.host_sequence_number,
                state.right.wrist_pose,
                None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
                desired,
                desired,
                result,
                None,
                False,
                result.reason.value,
            )
        assert result.joint_target_rad is not None
        assert state.right.host_sequence_number is not None
        input_receive_ns = int(state.right.host_receive_monotonic_ns or now_ns)
        self._accepted_sequence += 1
        accepted_target = AcceptedArmTarget(
            sequence_number=self._accepted_sequence,
            input_sequence_number=state.right.host_sequence_number,
            input_receive_monotonic_ns=min(input_receive_ns, now_ns),
            generated_monotonic_ns=now_ns,
            desired_tcp=desired,
            filtered_tcp=desired,
            joint_position_rad=result.joint_target_rad,
        )
        output_applied = self.arm_output.apply(accepted_target)
        self.accepted_targets += 1
        self.consecutive_rejections = 0
        self.last_reason = FeasibilityReason.ACCEPTED.value
        self.last_desired = desired
        current = self.simulation.current_tcp_pose
        record.update(
            accepted=True,
            reason=FeasibilityReason.ACCEPTED.value,
            position_error_m=float(np.linalg.norm(np.asarray(desired.position_m) - np.asarray(current.position_m))),
            orientation_error_deg=math.degrees(quaternion_angle_rad(desired.orientation_xyzw, current.orientation_xyzw)),
            accepted_joint_target_rad=list(accepted_target.joint_position_rad),
            output_applied=output_applied,
        )
        self.event_records.append(record)
        return ArmControlTickResult(
            state.right.host_sequence_number,
            state.right.wrist_pose,
            None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
            desired,
            desired,
            result,
            accepted_target,
            output_applied,
            FeasibilityReason.ACCEPTED.value,
        )

    def _arm_target(self, state: CanonicalQuestState, action: ClutchAction, now_ns: int):
        if action is ClutchAction.CAPTURE_ARM_REFERENCE:
            assert state.right.wrist_pose is not None and state.head is not None
            started = time.perf_counter_ns()
            authoritative_tcp = self.simulation.capture_reference()
            desired = self.arm_mapper.capture(
                wrist=state.right.wrist_pose,
                robot_tcp=authoritative_tcp,
                head=state.head.pose,
                timestamp_ns=now_ns,
            )
            self.arm_clutch.reference_captured(now_ns)
            duration = time.perf_counter_ns() - started
            self.arm_capture_durations_ns.append(duration)
            self.arm_engagement_latencies_ns.append(max(0, now_ns - self._index_sample.host_receive_monotonic_ns))
            self.consecutive_rejections = 0
            return desired
        if action is ClutchAction.UPDATE:
            assert state.right.wrist_pose is not None
            return self.arm_mapper.target(state.right.wrist_pose, timestamp_ns=now_ns)
        if self.arm_clutch.state is not ArmClutchState.ENGAGED:
            self.arm_mapper.clear()
        self.last_reason = FeasibilityReason.DISENGAGED.value
        return None

    def _update_hand(self, state: CanonicalQuestState, action: ClutchAction, now_ns: int) -> None:
        if self.hand_retargeter is None or action is ClutchAction.FREEZE:
            return
        started = time.perf_counter_ns()
        result = self.hand_retargeter.retarget(QuestHandSkeleton.from_observation(state.right))
        elapsed = time.perf_counter_ns() - started
        self.hand_retarget_durations_ns.append(elapsed)
        self.hand_timestamps_ns.append(now_ns)
        self.last_hand_result = result
        if not result.valid:
            self.hand_clutch.fault(now_ns, result.rejection_reason or "HAND_RETARGET_FAILED")
            return
        self.hand_valid_results += 1
        order = ("thumb_lateral", "thumb_close", "index", "middle", "ring", "pinky")
        target = np.asarray([result.actuator_targets[name] for name in order], dtype=float)
        if action is ClutchAction.START_HAND_REACQUISITION:
            self._hand_reacquire_anchor = self._held_hand_command.copy()
            self._hand_press_receive_ns = self._grip_sample.host_receive_monotonic_ns
        fraction = self.hand_clutch.reacquisition_fraction(now_ns)
        if self.hand_clutch.state is HandClutchState.REACQUIRE:
            target = self._hand_reacquire_anchor + fraction * (target - self._hand_reacquire_anchor)
        mapping = dict(zip(order, target.tolist(), strict=True))
        self.simulation.set_hand_actuator_target(mapping)
        self._held_hand_command = target.copy()
        self._hand_updated_this_tick = True

    def _base_record(self, state: CanonicalQuestState, now_ns: int) -> dict[str, Any]:
        mapping = self.arm_mapper.last_telemetry
        return {
            "control_monotonic_ns": now_ns,
            "mujoco_time_s": float(self.simulation.data.time),
            "input_sequence": state.right.host_sequence_number,
            "right_wrist_valid": bool(state.right.tracking_valid and state.right.wrist_pose is not None),
            "right_wrist_age_s": state.right.stream_age_s,
            "hand_skeleton_valid": bool(state.right.tracking_valid and len(state.right.joints) == 21),
            "hand_skeleton_age_s": state.right.stream_age_s,
            "index_trigger_value": self._index_sample.value,
            "index_trigger_age_s": max(0.0, (now_ns - self._index_sample.host_receive_monotonic_ns) / 1e9),
            "grip_trigger_value": self._grip_sample.value,
            "grip_trigger_age_s": max(0.0, (now_ns - self._grip_sample.host_receive_monotonic_ns) / 1e9),
            "clutch_provider": self.clutch_provider,
            "arm_clutch_state": self.arm_clutch.state.value,
            "hand_clutch_state": self.hand_clutch.state.value,
            "captured_head_yaw_rad": self.arm_mapper.latched_head_yaw_rad,
            "arm_reference_pose": _pose_dict(self.arm_mapper.robot_reference),
            "current_arm_target": _pose_dict(self.simulation.last_safe_target),
            "operator_delta": None if mapping is None else {
                "translation_m": mapping.horizontal_delta.position_m,
                "orientation_xyzw": mapping.horizontal_delta.orientation_xyzw,
            },
            "comfort_translation_warning": bool(mapping and mapping.comfort_translation_warning),
            "comfort_rotation_warning": bool(mapping and mapping.comfort_rotation_warning),
            "ik_status": self.last_reason,
            "hand_retarget_status": None if self.last_hand_result is None else self.last_hand_result.rejection_reason or "VALID",
            "hand_command_updated": self._hand_updated_this_tick,
            "hand_reacquisition_fraction": self.hand_clutch.reacquisition_fraction(now_ns),
            "active_arm_fault": None if self.arm_clutch.active_fault is None else self.arm_clutch.active_fault.reason,
            "active_hand_fault": None if self.hand_clutch.active_fault is None else self.hand_clutch.active_fault.reason,
            "arm_clutch_cycle_count": self.arm_clutch.cycle_count,
            "hand_clutch_cycle_count": self.hand_clutch.cycle_count,
            "raw_wrist": _pose_dict(self.arm_mapper.raw_wrist),
            "filtered_wrist": _pose_dict(self.arm_mapper.filtered_wrist),
            "actual_tcp": _pose_dict(self.simulation.current_tcp_pose),
            "actual_joint_position_rad": self.simulation.arm_joints_rad.tolist(),
            "simulated_joint_target_rad": self.simulation.commanded_joint_target.tolist(),
            "commanded_hand_target_rad": self.simulation.commanded_hand_target.tolist(),
            "actual_hand_actuator_position_rad": self.simulation.data.qpos[
                self.simulation.model.jnt_qposadr[
                    self.simulation.model.actuator_trnid[self.simulation.hand_actuator_ids, 0]
                ]
            ].tolist(),
        }

    def _handle_rejection(self, timestamp_ns: int, reason: str) -> None:
        self.rejections[reason] += 1
        self.last_reason = reason
        self.consecutive_rejections += 1
        if self.consecutive_rejections > self.isolated_rejection_hold_count:
            self.arm_clutch.fault(timestamp_ns, reason)
            self.arm_mapper.clear()

    def report(self, replay_source: str) -> dict[str, Any]:
        return {
            "schema_version": "quest_jaka_rh56_dual_clutch_precision.v1",
            "replay_source": replay_source,
            "input_frame_count": len(self.input_timestamps_ns),
            "control_tick_count": len(self.control_timestamps_ns),
            "ik_attempt_count": len(self.ik_timestamps_ns),
            "accepted_target_count": self.accepted_targets,
            "rejections": dict(sorted(self.rejections.items())),
            "input_rate_hz": _rate(self.input_timestamps_ns),
            "head_source_rate_hz": _rate(self.head_timestamps_ns),
            "index_trigger_source_rate_hz": _rate(self.index_timestamps_ns),
            "grip_trigger_source_rate_hz": _rate(self.grip_timestamps_ns),
            "arm_control_rate_hz": _rate(self.control_timestamps_ns),
            "hand_retarget_rate_hz": _rate(self.hand_timestamps_ns),
            "ik_rate_hz": _rate(self.ik_timestamps_ns),
            "filter_profile": self.profile.name,
            "clutch_provider": self.clutch_provider,
            "hand_backend": None if self.hand_retargeter is None else self.hand_retargeter.backend,
            "hand_valid_result_count": self.hand_valid_results,
            "arm_final_state": self.arm_clutch.state.value,
            "hand_final_state": self.hand_clutch.state.value,
            "final_state": f"arm={self.arm_clutch.state.value},hand={self.hand_clutch.state.value}",
            "arm_clutch_cycle_count": self.arm_clutch.cycle_count,
            "hand_clutch_cycle_count": self.hand_clutch.cycle_count,
            "arm_reference_capture_ms": _distribution_ms(self.arm_capture_durations_ns),
            "hand_reacquisition_configured_ms": self.hand_clutch.reacquisition_duration_ns / 1e6,
            "ik_computation_ms": _event_metric(self.event_records, "ik_computation_ms"),
            "hand_retarget_computation_ms": _distribution_ms(self.hand_retarget_durations_ns),
            "arm_trigger_to_engagement_ms": _distribution_ms(self.arm_engagement_latencies_ns),
            "hand_trigger_to_engagement_ms": _distribution_ms(self.hand_engagement_latencies_ns),
            "right_wrist_sample_age_ms": _event_metric_scaled(self.event_records, "right_wrist_age_s", 1000.0),
            "index_trigger_sample_age_ms": _event_metric_scaled(self.event_records, "index_trigger_age_s", 1000.0),
            "grip_trigger_sample_age_ms": _event_metric_scaled(self.event_records, "grip_trigger_age_s", 1000.0),
            "arm_target_latency_ms": _accepted_event_metric_scaled(self.event_records, "right_wrist_age_s", 1000.0),
            "hand_target_latency_ms": _event_metric_scaled(
                [record for record in self.event_records if record.get("hand_command_updated")],
                "hand_skeleton_age_s",
                1000.0,
            ),
            "arm_transitions": [asdict(item) for item in self.arm_clutch.transitions],
            "hand_transitions": [asdict(item) for item in self.hand_clutch.transitions],
            "hardware_connections": False,
            "hardware_commands": False,
            **self.simulation.metrics_report(),
        }


def _pose_dict(pose: Any) -> dict[str, Any] | None:
    if pose is None:
        return None
    return {"position_m": list(pose.position_m), "orientation_xyzw": list(pose.orientation_xyzw)}


def _rate(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2:
        return None
    duration = (timestamps_ns[-1] - timestamps_ns[0]) / 1e9
    return None if duration <= 0 else (len(timestamps_ns) - 1) / duration


def _distribution_ms(values_ns: list[int]) -> dict[str, float] | None:
    if not values_ns:
        return None
    values = np.asarray(values_ns, dtype=float) / 1e6
    return {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95)), "max": float(np.max(values))}


def _event_metric(records: list[dict[str, Any]], name: str) -> dict[str, float] | None:
    values = [float(record[name]) for record in records if name in record]
    if not values:
        return None
    array = np.asarray(values)
    return {"mean": float(np.mean(array)), "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}


def _event_metric_scaled(records: list[dict[str, Any]], name: str, scale: float) -> dict[str, float] | None:
    values = [float(record[name]) * scale for record in records if record.get(name) is not None]
    if not values:
        return None
    array = np.asarray(values)
    return {"mean": float(np.mean(array)), "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}


def _accepted_event_metric_scaled(records: list[dict[str, Any]], name: str, scale: float) -> dict[str, float] | None:
    return _event_metric_scaled([record for record in records if record.get("accepted")], name, scale)
