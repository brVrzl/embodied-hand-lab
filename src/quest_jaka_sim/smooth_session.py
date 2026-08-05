"""Fixed-rate dual-clutch Quest-to-JAKA/RH56 MuJoCo session.

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
from typing import Any, Protocol, Sequence

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
from .control_timing import CONTROL_TIMING_FIELDS, ControlTimingRecorder
from .hand_retarget import (
    HandRetargetCalibration,
    InspireRetargetResult,
    PinchPoseBlender,
    ProjectRh56Retargeter,
    QuestHandSkeleton,
    RH56_MUJOCO_ACTUATOR_MAX_RAD,
    ThumbFirstPinchSequencer,
)
from .precision_mapping import LatchedHeadYawArmMapper
from .output import (
    AcceptedArmTarget,
    ArmTargetOutputAdapter,
    MujocoArmTargetAdapter,
)
from teleoperation.accepted_target import (
    AcceptedTargetDiagnostics,
    AcceptedTcpPose,
    ArmControlHeartbeat,
    ArmControlState,
)
from .se3 import (
    PoseSampleBuffer,
    TimedPoseSample,
    bounded_pose_step,
    quaternion_angle_rad,
)
from .simulation import (
    FeasibilityReason,
    FeasibilityResult,
    JakaMujocoSimulation,
    ReplayConfig,
    SharedJakaTargetGenerator,
)
from .smooth_operator import Se3FilterProfile


INPUT_RECOVERY_HOLD_REASON = "QUEST_INPUT_RECOVERY_HOLD"
INPUT_RECOVERY_RECLUTCH_REASON = "QUEST_INPUT_RECOVERED_RECLUTCH_REQUIRED"
INPUT_RECOVERY_TIMEOUT_REASON = "QUEST_INPUT_RECOVERY_TIMEOUT"
_CONTROL_TIMING_INDEX = {
    name: index for index, name in enumerate(CONTROL_TIMING_FIELDS)
}


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


class NormalizedHandOutput(Protocol):
    max_target_normalized: float

    def activate_from_measured(self, monotonic_ns: int) -> tuple[float, ...]: ...

    def submit_target(self, target: Sequence[float], monotonic_ns: int) -> None: ...

    def hold(self, reason: str) -> None: ...


class SmoothQuestJakaSession:
    """One fixed-rate session with independent index and grip clutches.

    Index controls only the arm reference lifecycle.  Grip controls only the
    RH56 simulation reference lifecycle.  Their references are intentionally
    independent so either channel can freeze while the other continues.
    """

    def __init__(
        self,
        config: ReplayConfig,
        target_generator: SharedJakaTargetGenerator,
        *,
        arm_output: ArmTargetOutputAdapter | None = None,
        mujoco_plant: JakaMujocoSimulation | None = None,
        control_compute_budget_ms: float | None = None,
        normalized_hand_output: NormalizedHandOutput | None = None,
        arm_input_enabled: bool = True,
    ) -> None:
        filter_values = config.raw.get("filter", {})
        profile_name = str(filter_values.get("selected_profile", "simulation_exploration"))
        profiles = filter_values.get("profiles", {})
        if profile_name not in profiles:
            raise ValueError(f"unknown SE(3) filter profile {profile_name!r}")
        self.profile = Se3FilterProfile.from_mapping(profile_name, profiles[profile_name])
        self.config = config
        self.target_generator = target_generator
        # Compatibility alias for existing simulation diagnostics/tests. The
        # physical runner passes a plant-free target generator here.
        self.simulation = target_generator
        self.mujoco_plant = (
            target_generator
            if isinstance(target_generator, JakaMujocoSimulation)
            else mujoco_plant
        )
        if arm_output is None:
            if self.mujoco_plant is None:
                raise ValueError("a plant-free target generator requires an explicit output adapter")
            arm_output = MujocoArmTargetAdapter(self.mujoco_plant)
        self.arm_output = arm_output
        self.normalized_hand_output = normalized_hand_output
        self.arm_input_enabled = bool(arm_input_enabled)
        self.input_recovery_timeout_ns = int(
            config.input_recovery_timeout_s * 1e9
        )
        self._input_recovery_started_ns: int | None = None
        self._input_recovery_hard_stop_reason: str | None = None
        self.input_recovery_count = 0
        self.input_recovery_success_count = 0
        self.input_recovery_timeout_count = 0
        shared_policy = config.raw.get("shared_target_generation", {})
        self.continuation_enabled = bool(shared_policy.get("continuation_enabled", True))
        self.maximum_continuation_backtracks = int(shared_policy.get("maximum_backtracks", 5))
        self.control_compute_budget_ms = (
            None
            if control_compute_budget_ms is None
            else float(control_compute_budget_ms)
        )
        self.control_compute_budget_ns = (
            None
            if self.control_compute_budget_ms is None
            else int(self.control_compute_budget_ms * 1e6)
        )
        self.minimum_continuation_fraction = float(
            shared_policy.get("minimum_continuation_fraction", 1.0 / 32.0)
        )
        self.rejection_policy = str(
            shared_policy.get(
                "rejection_policy",
                "hold_last_accepted_and_allow_operator_retreat",
            )
        )
        if self.maximum_continuation_backtracks < 0:
            raise ValueError("maximum continuation backtracks must be non-negative")
        if self.control_compute_budget_ms is not None and (
            not math.isfinite(self.control_compute_budget_ms) or not (
                0.0 < self.control_compute_budget_ms
                < float(
                    config.raw.get("hardware_adapter", {}).get(
                        "command_stream_timeout_ms", 100.0
                    )
                )
            )
        ):
            raise ValueError(
                "control compute budget must be positive and below command-stream timeout"
            )
        if not 0.0 < self.minimum_continuation_fraction < 1.0:
            raise ValueError("minimum continuation fraction must be in (0, 1)")
        if self.rejection_policy != "hold_last_accepted_and_allow_operator_retreat":
            raise ValueError(f"unsupported shared rejection policy {self.rejection_policy!r}")
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
        self.operator = self.arm_clutch
        self.arm_mapper = LatchedHeadYawArmMapper(config.mapping, self.profile)
        self.latest_state: CanonicalQuestState | None = None
        self.last_input_sequence: int | None = None
        self.last_input_receive_ns: int | None = None
        self.last_desired = target_generator.current_tcp_pose
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
        self._last_accepted_generated_ns: int | None = None
        self.last_accepted_target: AcceptedArmTarget | None = None
        self._hold_rejected_started_ns: int | None = None
        self.reference_generation = 0
        self.consecutive_rejections = 0
        self.continuation_intervention_count = 0
        self.continuation_backtrack_count = 0
        self.control_compute_budget_exhausted_count = 0
        self._control_timing = ControlTimingRecorder(capacity=512)
        self._active_rh56_command_duration_ns = 0
        self.singularity_warning_count = 0
        self.maximum_requested_backlog_m = 0.0
        self.maximum_requested_backlog_rad = 0.0
        self.isolated_rejection_hold_count = int(
            config.raw.get("simulation", {}).get("isolated_rejection_hold_count", 2)
        )
        hand_values = config.raw.get("hand_retargeting", {})
        # Physical hand-only teleoperation may opt into an absolute pose
        # re-alignment on each grip press.  The command still starts with the
        # measured RH56 activation target; the worker's normal delta/contact
        # gates perform the bounded transition to the current Quest pose.
        self.hand_align_on_grip = bool(hand_values.get("align_on_grip", False))
        self.hand_align_index_pinch_to_validated_pose = bool(
            hand_values.get("align_index_pinch_to_validated_pose", False)
        )
        self.hand_enabled = (
            bool(hand_values.get("enabled", False))
            and (
                normalized_hand_output is not None
                or (
                    self.mujoco_plant is not None
                    and bool(getattr(self.mujoco_plant, "hand_available", True))
                )
            )
        )
        self.hand_retargeter: ProjectRh56Retargeter | None = None
        self.last_hand_result: InspireRetargetResult | None = None
        self.hand_valid_results = 0
        if self.hand_enabled:
            backend, calibration = HandRetargetCalibration.load(hand_values["calibration_path"])
            self.hand_retargeter = ProjectRh56Retargeter(calibration, backend=backend)
            validated_poses = (
                calibration.validated_pinch_poses
                if calibration.pinch_pose_blending_enabled
                else {}
            )
            self._pinch_pose_blender = PinchPoseBlender(
                validated_poses,
                maximum_weight_step=calibration.pinch_pose_maximum_weight_step,
            )
        else:
            self._pinch_pose_blender = PinchPoseBlender(
                {}, maximum_weight_step=0.05
            )
        if self.hand_retargeter is not None:
            calibration = self.hand_retargeter.calibration
            self._thumb_first_pinch = ThumbFirstPinchSequencer(
                enabled=calibration.thumb_first_pinch_enabled,
                lateral_target=calibration.thumb_first_lateral_target,
                lateral_tolerance=calibration.thumb_first_lateral_tolerance,
                index_guard=calibration.thumb_first_index_guard,
                thumb_close_guard=calibration.thumb_first_thumb_close_guard,
                index_activation=calibration.thumb_first_index_activation,
                thumb_close_activation=calibration.thumb_first_thumb_close_activation,
                lateral_activation=calibration.thumb_first_lateral_activation,
            )
        else:
            self._thumb_first_pinch = ThumbFirstPinchSequencer(
                enabled=False,
                lateral_target=0.0,
                lateral_tolerance=0.03,
                index_guard=0.15,
                thumb_close_guard=0.25,
            )
        self._thumb_first_pinch_stage = "idle"
        self._pinch_blend_mode = "none"
        self._pinch_blend_weight = 0.0
        self._pinch_continuous_target: np.ndarray | None = None
        relative_values = hand_values.get("four_finger_relative", {})
        self.four_finger_gain = float(relative_values.get("gain", 1.0))
        self.four_finger_dead_zone_rad = float(relative_values.get("dead_zone_rad", 0.015))
        self.four_finger_max_step_rad = float(relative_values.get("maximum_target_step_rad", 0.04))
        if (
            not math.isfinite(self.four_finger_gain)
            or self.four_finger_gain <= 0.0
            or not math.isfinite(self.four_finger_dead_zone_rad)
            or self.four_finger_dead_zone_rad < 0.0
            or not math.isfinite(self.four_finger_max_step_rad)
            or self.four_finger_max_step_rad <= 0.0
        ):
            raise ValueError("invalid H1 four-finger relative hand policy")
        thumb_values = hand_values.get("thumb_close_relative", {})
        self.thumb_close_gain = float(thumb_values.get("gain", 1.0))
        self.thumb_close_dead_zone_rad = float(thumb_values.get("dead_zone_rad", 0.008))
        self.thumb_close_max_step_rad = float(thumb_values.get("maximum_target_step_rad", 0.025))
        if (
            not math.isfinite(self.thumb_close_gain)
            or self.thumb_close_gain <= 0.0
            or not math.isfinite(self.thumb_close_dead_zone_rad)
            or self.thumb_close_dead_zone_rad < 0.0
            or not math.isfinite(self.thumb_close_max_step_rad)
            or self.thumb_close_max_step_rad <= 0.0
        ):
            raise ValueError("invalid H2 thumb-close relative hand policy")
        lateral_values = hand_values.get("thumb_lateral_relative", {})
        self.thumb_lateral_gain = float(lateral_values.get("gain", 1.0))
        self.thumb_lateral_dead_zone = float(
            lateral_values.get("dead_zone", 0.015)
        )
        self.thumb_lateral_max_step_rad = float(
            lateral_values.get("maximum_target_step_rad", 0.025)
        )
        if (
            not math.isfinite(self.thumb_lateral_gain)
            or self.thumb_lateral_gain <= 0.0
            or not math.isfinite(self.thumb_lateral_dead_zone)
            or self.thumb_lateral_dead_zone < 0.0
            or not math.isfinite(self.thumb_lateral_max_step_rad)
            or self.thumb_lateral_max_step_rad <= 0.0
        ):
            raise ValueError("invalid thumb-lateral relative hand policy")
        self._held_hand_command = (
            self.mujoco_plant.commanded_hand_target.copy()
            if self.mujoco_plant is not None
            else np.zeros(6, dtype=np.float64)
        )
        self._hand_target_reference: np.ndarray | None = None
        self._hand_activation_reference: np.ndarray | None = None
        self._hand_alignment_target: np.ndarray | None = None
        self._four_finger_feature_reference: np.ndarray | None = None
        self._four_finger_features: np.ndarray | None = None
        self._four_finger_feature_delta: np.ndarray | None = None
        self._four_finger_requested_target: np.ndarray | None = None
        self._four_finger_clipped_target: np.ndarray | None = None
        self._four_finger_saturated: np.ndarray | None = None
        self._thumb_close_feature_reference: float | None = None
        self._thumb_lateral_feature_reference: float | None = None
        self._thumb_close_feature_delta: float | None = None
        self._thumb_close_requested_target: float | None = None
        self._thumb_close_clipped_target: float | None = None
        self._thumb_close_saturated = False
        self._thumb_lateral_feature_delta: float | None = None
        self._thumb_lateral_requested_target: float | None = None
        self._thumb_lateral_clipped_target: float | None = None
        self._thumb_lateral_saturated = False
        self._requested_hand_target: np.ndarray | None = None
        self._clipped_hand_target: np.ndarray | None = None
        self._hand_press_receive_ns: int | None = None
        self._index_sample = AnalogClutchSample(0.0, 0, 0, valid=False)
        self._grip_sample = AnalogClutchSample(0.0, 0, 0, valid=False)
        self.left_controller_valid = False
        self.clutch_provider = "unavailable"
        self._last_head_receive_ns: int | None = None
        self._last_index_sequence: int | None = None
        self._last_grip_sequence: int | None = None
        self._hand_updated_this_tick = False
        self._shared_input_mode = False

    def set_clutch_samples(
        self,
        *,
        index: AnalogClutchSample,
        grip: AnalogClutchSample,
        left_controller_valid: bool,
        provider: str,
    ) -> None:
        """Publish controller samples into the session.

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
            # malformed right-hand packet conservatively faults both independent
            # channels; each still requires its own later release before press.
            self.arm_clutch.fault(datagram.receive_monotonic_ns, "MALFORMED_SHARED_RIGHT_HAND_DATA")
            self.hand_clutch.fault(datagram.receive_monotonic_ns, "MALFORMED_SHARED_RIGHT_HAND_DATA")
            self.arm_mapper.clear()
            self._clear_hand_reference()
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
            self.last_input_receive_ns = receive_ns
            self.input_timestamps_ns.append(receive_ns)
            self.buffer.add(TimedPoseSample(receive_ns, hand.host_sequence_number, hand.wrist_pose, state))
            return True
        return False

    def ingest_shared_state(self, state: CanonicalQuestState) -> bool:
        """Broadcast one already-canonical Quest sample into this policy state."""

        self._shared_input_mode = True
        self.latest_state = state
        hand = state.right
        if (
            hand.tracking_valid
            and hand.wrist_pose is not None
            and len(hand.joints) == 21
            and hand.host_sequence_number is not None
            and hand.host_sequence_number != self.last_input_sequence
        ):
            self.last_input_sequence = hand.host_sequence_number
            receive_ns = int(hand.host_receive_monotonic_ns or state.host_monotonic_ns)
            self.last_input_receive_ns = receive_ns
            self.input_timestamps_ns.append(receive_ns)
            self.buffer.add(
                TimedPoseSample(receive_ns, hand.host_sequence_number, hand.wrist_pose, state)
            )
            return True
        return False

    def _shared_state_at(self, now_ns: int) -> CanonicalQuestState:
        """Refresh freshness without reparsing the broadcast Quest sample."""

        assert self.latest_state is not None
        state = self.latest_state
        stale_ns = int(self.config.stale_after_s * 1e9)

        def refresh_hand(hand: Any) -> Any:
            age_ns = (
                None
                if hand.host_receive_monotonic_ns is None
                else max(0, now_ns - hand.host_receive_monotonic_ns)
            )
            valid = bool(hand.tracking_valid and age_ns is not None and age_ns <= stale_ns)
            return replace(hand, tracking_valid=valid, stream_age_s=None if age_ns is None else age_ns / 1e9)

        right = refresh_hand(state.right)
        left = refresh_hand(state.left)
        head = state.head
        if head is not None:
            head_age_ns = max(0, now_ns - head.host_receive_monotonic_ns)
            head = replace(
                head,
                tracking_valid=head.tracking_valid and head_age_ns <= stale_ns,
                stream_age_s=head_age_ns / 1e9,
            )
        return replace(state, host_monotonic_ns=now_ns, right=right, left=left, head=head)

    def control_tick(self, now_ns: int) -> ArmControlTickResult:
        tick_started_ns = time.perf_counter_ns()
        timing_values = [0] * len(CONTROL_TIMING_FIELDS)
        self._active_rh56_command_duration_ns = 0
        self.control_timestamps_ns.append(now_ns)
        state = (
            self._shared_state_at(now_ns)
            if self._shared_input_mode
            else self.assembler.state(now_monotonic_ns=now_ns)
        )
        raw_quest_wrist = state.right.wrist_pose
        interpolated = self.buffer.sample(now_ns - self.interpolation_delay_ns)
        # Never substitute an old buffered wrist for a frame that explicitly
        # lacks a wrist pose.  That would let an index-held arm extrapolate
        # through wrist loss and would violate arm/hand fault isolation.
        if interpolated is not None and state.right.tracking_valid and state.right.wrist_pose is not None:
            state = replace(state, right=replace(state.right, wrist_pose=interpolated.pose))
        right = state.right
        wrist_valid = bool(right.tracking_valid and right.wrist_pose is not None)
        skeleton_valid = bool(right.tracking_valid and len(right.joints) == 21)
        head_valid = bool(state.head is not None and state.head.tracking_valid)
        arm_stream_valid = bool(self.left_controller_valid and wrist_valid)
        if (
            self.arm_input_enabled
            and self.reference_generation > 0
            and self._input_recovery_hard_stop_reason is None
        ):
            if not arm_stream_valid:
                if self._input_recovery_started_ns is None:
                    self._input_recovery_started_ns = now_ns
                    self.input_recovery_count += 1
            elif self._input_recovery_started_ns is not None:
                self._input_recovery_started_ns = None
                self.input_recovery_success_count += 1

        quest_input_finished_ns = time.perf_counter_ns()

        if not self.left_controller_valid:
            if self.arm_clutch.state is not ArmClutchState.TRACKING_FAULT:
                self.arm_clutch.fault(now_ns, "LEFT_CONTROLLER_STALE_OR_INVALID")
                self.arm_mapper.clear()
            if self.hand_clutch.state is not HandClutchState.TRACKING_FAULT:
                self.hand_clutch.fault(now_ns, "LEFT_CONTROLLER_STALE_OR_INVALID")
                self._clear_hand_reference()

        # Wrist loss belongs to the arm channel. Landmark-only loss belongs to
        # neither clutch state: H2 holds the last hand command without
        # extrapolation while a valid index-held arm may continue.
        if not right.tracking_valid and self.arm_clutch.state is ArmClutchState.ENGAGED:
            self.arm_clutch.fault(now_ns, "RIGHT_WRIST_TRACKING_LOST")
            self.arm_mapper.clear()

        clutch_started_ns = quest_input_finished_ns
        arm_action = (
            self.arm_clutch.step(
                self._index_sample,
                now_ns=now_ns,
                controller_valid=self.left_controller_valid,
                continuous_inputs_valid=wrist_valid,
                capture_inputs_valid=wrist_valid and head_valid,
            )
            if (
                self.arm_input_enabled
                and self._input_recovery_hard_stop_reason is None
            )
            else ClutchAction.FREEZE
        )
        # The hand state machine receives grip only. Skeleton validity is
        # checked at capture below; during hold a transient landmark loss must
        # freeze the hand target rather than fault arm or hand state.
        hand_action = self.hand_clutch.step(
            self._grip_sample,
            now_ns=now_ns,
            controller_valid=self.left_controller_valid,
            skeleton_valid=True,
        )
        clutch_finished_ns = time.perf_counter_ns()
        self._hand_updated_this_tick = False
        if hand_action is ClutchAction.START_HAND_REACQUISITION:
            if not self._capture_hand_reference(state, now_ns):
                self.hand_clutch.fault(now_ns, "HAND_REFERENCE_INPUT_INVALID")
                self._clear_hand_reference()
        else:
            self._update_hand(state, hand_action, now_ns, skeleton_valid=skeleton_valid)
            if self.hand_clutch.state not in {HandClutchState.REACQUIRE, HandClutchState.ENGAGED}:
                if self.normalized_hand_output is not None:
                    self.normalized_hand_output.hold("grip_not_active")
                self._clear_hand_reference()
        mapping_started_ns = time.perf_counter_ns()
        timing_values[_CONTROL_TIMING_INDEX["quest_input_duration_ns"]] = (
            quest_input_finished_ns - tick_started_ns
        )
        timing_values[_CONTROL_TIMING_INDEX["clutch_state_duration_ns"]] = (
            clutch_finished_ns - clutch_started_ns
        )
        timing_values[_CONTROL_TIMING_INDEX["smooth_session_duration_ns"]] = (
            mapping_started_ns - clutch_finished_ns
        )
        desired = self._arm_target(state, arm_action, now_ns)
        mapping_finished_ns = time.perf_counter_ns()
        timing_values[_CONTROL_TIMING_INDEX["mapping_duration_ns"]] = (
            mapping_finished_ns - mapping_started_ns
        )
        record_started_ns = time.perf_counter_ns()
        record = self._base_record(state, now_ns)
        record_finished_ns = time.perf_counter_ns()
        event_diagnostic_duration_ns = record_finished_ns - record_started_ns
        record["arm_clutch_action"] = arm_action.value
        record["hand_clutch_action"] = hand_action.value
        record["arm_reference_capture"] = arm_action is ClutchAction.CAPTURE_ARM_REFERENCE
        record["hand_reference_capture"] = hand_action is ClutchAction.START_HAND_REACQUISITION
        record["raw_quest_wrist"] = _pose_dict(raw_quest_wrist)
        record["interpolated_wrist"] = _pose_dict(right.wrist_pose)
        record["filtered_mapped_tcp"] = _pose_dict(self.arm_mapper.filtered_mapped_target)
        record["control_stage_timing_ms"] = {
            "quest_input_clutch_and_hand": (mapping_started_ns - tick_started_ns) / 1e6,
            "target_mapping": (mapping_finished_ns - mapping_started_ns) / 1e6,
            "event_record_allocation": (record_finished_ns - record_started_ns) / 1e6,
        }
        if desired is None:
            self._hold_rejected_started_ns = None
            heartbeat_applied = False
            recovery_elapsed_ns = (
                None
                if self._input_recovery_started_ns is None
                else max(0, now_ns - self._input_recovery_started_ns)
            )
            recovery_timed_out = bool(
                recovery_elapsed_ns is not None
                and self.input_recovery_timeout_ns > 0
                and recovery_elapsed_ns > self.input_recovery_timeout_ns
            )
            if (
                recovery_timed_out
                and self._input_recovery_hard_stop_reason is None
            ):
                self._input_recovery_hard_stop_reason = (
                    INPUT_RECOVERY_TIMEOUT_REASON
                )
                self.input_recovery_timeout_count += 1
                self.arm_clutch.fault(now_ns, INPUT_RECOVERY_TIMEOUT_REASON)
                self.arm_mapper.clear()
            recovery_hold = bool(
                self.arm_input_enabled
                and self.arm_clutch.state is ArmClutchState.TRACKING_FAULT
                and self.reference_generation > 0
                and self.arm_clutch.cycle_count > 0
                and self.last_input_sequence is not None
                and self.last_input_receive_ns is not None
                and self.input_recovery_timeout_ns > 0
                and self._input_recovery_hard_stop_reason is None
            )
            recovery_reason = (
                INPUT_RECOVERY_HOLD_REASON
                if self._input_recovery_started_ns is not None
                else INPUT_RECOVERY_RECLUTCH_REASON
            )
            if recovery_hold:
                assert self.last_input_sequence is not None
                assert self.last_input_receive_ns is not None
                heartbeat = ArmControlHeartbeat(
                    input_sequence_number=self.last_input_sequence,
                    input_receive_monotonic_ns=min(
                        self.last_input_receive_ns, now_ns
                    ),
                    generated_monotonic_ns=now_ns,
                    reference_generation=self.reference_generation,
                    clutch_generation=self.arm_clutch.cycle_count,
                    state=ArmControlState.DISENGAGED,
                    reason=recovery_reason,
                    last_accepted_target_sequence=self._accepted_sequence,
                )
                heartbeat_applied = self.arm_output.heartbeat(heartbeat)
            elif (
                self.arm_input_enabled
                and self.arm_clutch.state is ArmClutchState.DISENGAGED
                and self.left_controller_valid
                and wrist_valid
                and state.right.host_sequence_number is not None
                and state.right.host_receive_monotonic_ns is not None
                and self.reference_generation > 0
                and self.arm_clutch.cycle_count > 0
            ):
                heartbeat = ArmControlHeartbeat(
                    input_sequence_number=state.right.host_sequence_number,
                    input_receive_monotonic_ns=min(
                        state.right.host_receive_monotonic_ns, now_ns
                    ),
                    generated_monotonic_ns=now_ns,
                    reference_generation=self.reference_generation,
                    clutch_generation=self.arm_clutch.cycle_count,
                    state=ArmControlState.DISENGAGED,
                    reason=FeasibilityReason.DISENGAGED.value,
                    last_accepted_target_sequence=self._accepted_sequence,
                )
                heartbeat_applied = self.arm_output.heartbeat(heartbeat)
            hard_stop = self._input_recovery_hard_stop_reason is not None
            reason = (
                self._input_recovery_hard_stop_reason
                if hard_stop
                else recovery_reason
                if recovery_hold
                else FeasibilityReason.DISENGAGED.value
            )
            diagnostic_started_ns = time.perf_counter_ns()
            record.update(
                accepted=False,
                reason=reason,
                control_state=(
                    ArmControlState.HARD_STOP.value
                    if hard_stop
                    else ArmControlState.DISENGAGED.value
                ),
                heartbeat_applied=heartbeat_applied,
                heartbeat_generated_monotonic_ns=(
                    now_ns if heartbeat_applied else None
                ),
                control_compute_budget_exhausted=False,
                control_tick_wall_ms=(time.perf_counter_ns() - tick_started_ns) / 1e6,
            )
            self.event_records.append(record)
            event_diagnostic_duration_ns += time.perf_counter_ns() - diagnostic_started_ns
            self._finish_control_timing(
                now_ns,
                timing_values,
                tick_started_ns,
                event_diagnostic_duration_ns,
                state=state,
            )
            return ArmControlTickResult(
                state.right.host_sequence_number,
                state.right.wrist_pose if wrist_valid else None,
                None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
                None,
                None,
                None,
                None,
                heartbeat_applied,
                reason,
            )
        self.ik_timestamps_ns.append(now_ns)
        started = time.perf_counter_ns()
        compute_deadline_ns = (
            None
            if self.control_compute_budget_ns is None
            else tick_started_ns + self.control_compute_budget_ns
        )
        dt_s = (
            1.0 / float(self.config.raw.get("rates", {}).get("target_generation_hz", 60.0))
            if len(self.ik_timestamps_ns) < 2
            else (self.ik_timestamps_ns[-1] - self.ik_timestamps_ns[-2]) / 1e9
        )
        evaluated_target = desired
        continuation_fraction = 1.0
        continuation_backtracks = 0
        attempted_reasons: list[str] = []
        attempted_continuation_fractions: list[float] = []
        output_feasibility_attempts: list[dict[str, Any]] = []
        continuation_attempt_timing_ms: list[dict[str, Any]] = []
        candidate_timing_totals_ms = [0.0] * 9

        def accumulate_candidate_timing(timing: Any) -> None:
            candidate_timing_totals_ms[0] += timing.seed_fk_ms
            candidate_timing_totals_ms[1] += timing.pre_ik_jacobian_ms
            candidate_timing_totals_ms[2] += timing.ik_iterations_ms
            candidate_timing_totals_ms[3] += timing.ik_final_fk_ms
            candidate_timing_totals_ms[4] += timing.post_ik_jacobian_ms
            candidate_timing_totals_ms[5] += timing.workspace_check_ms
            candidate_timing_totals_ms[6] += timing.collision_check_ms
            candidate_timing_totals_ms[7] += timing.output_feasibility_ms
            candidate_timing_totals_ms[8] += timing.remaining_checks_ms

        if self.continuation_enabled:
            limits = self.config.feasibility
            # Stay just inside strict ``>`` gates without introducing a new
            # tuning knob.  nextafter only changes the last representable bit.
            maximum_translation = min(
                limits.maximum_target_jump_m,
                limits.maximum_tcp_velocity_m_s * max(dt_s, 1e-6),
            )
            maximum_rotation = min(
                limits.maximum_target_rotation_jump_rad,
                limits.maximum_tcp_angular_velocity_rad_s * max(dt_s, 1e-6),
            )
            evaluated_target, continuation_fraction = bounded_pose_step(
                self.target_generator.last_safe_target,
                desired,
                maximum_translation_m=float(np.nextafter(maximum_translation, 0.0)),
                maximum_rotation_rad=float(np.nextafter(maximum_rotation, 0.0)),
            )
            if continuation_fraction < 1.0:
                self.continuation_intervention_count += 1
        result = self.target_generator.evaluate(
            evaluated_target,
            dt_s=dt_s,
            generated_monotonic_ns=now_ns,
            compute_deadline_ns=compute_deadline_ns,
        )
        attempted_reasons.append(result.reason.value)
        attempted_continuation_fractions.append(continuation_fraction)
        output_feasibility_attempts.append(
            _output_feasibility_attempt(result, continuation_fraction)
        )
        continuation_attempt_timing_ms.append(asdict(result.timing))
        accumulate_candidate_timing(result.timing)
        # A rejected trial never becomes authoritative.  Retry smaller points
        # on the same full-pose segment; all hard feasibility gates are run on
        # every trial and remain unchanged.
        while (
            self.continuation_enabled
            and not result.accepted
            and result.reason is not FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED
            and continuation_fraction > self.minimum_continuation_fraction
            and continuation_backtracks < self.maximum_continuation_backtracks
        ):
            continuation_fraction *= 0.5
            evaluated_target, _ = bounded_pose_step(
                self.target_generator.last_safe_target,
                desired,
                maximum_translation_m=(
                    np.linalg.norm(
                        np.asarray(desired.position_m)
                        - np.asarray(self.target_generator.last_safe_target.position_m)
                    )
                    * continuation_fraction
                ),
                maximum_rotation_rad=(
                    quaternion_angle_rad(
                        self.target_generator.last_safe_target.orientation_xyzw,
                        desired.orientation_xyzw,
                    )
                    * continuation_fraction
                ),
            )
            continuation_backtracks += 1
            self.continuation_backtrack_count += 1
            result = self.target_generator.evaluate(
                evaluated_target,
                dt_s=dt_s,
                generated_monotonic_ns=now_ns,
                compute_deadline_ns=compute_deadline_ns,
            )
            attempted_reasons.append(result.reason.value)
            attempted_continuation_fractions.append(continuation_fraction)
            output_feasibility_attempts.append(
                _output_feasibility_attempt(result, continuation_fraction)
            )
            continuation_attempt_timing_ms.append(asdict(result.timing))
            accumulate_candidate_timing(result.timing)
        ik_duration_ns = time.perf_counter_ns() - started
        timing_values[_CONTROL_TIMING_INDEX["ik_duration_ns"]] = int(
            round(
                1e6
                * sum(candidate_timing_totals_ms[:5])
            )
        )
        timing_values[
            _CONTROL_TIMING_INDEX["collision_singularity_duration_ns"]
        ] = int(
            round(
                1e6
                * (
                    sum(candidate_timing_totals_ms[5:7])
                    + candidate_timing_totals_ms[8]
                )
            )
        )
        timing_values[
            _CONTROL_TIMING_INDEX["output_feasibility_duration_ns"]
        ] = int(round(candidate_timing_totals_ms[7] * 1e6))

        def attach_control_timing() -> None:
            record["control_timing_ns"] = {
                "quest_input_duration_ns": timing_values[
                    _CONTROL_TIMING_INDEX["quest_input_duration_ns"]
                ],
                "target_mapping_duration_ns": timing_values[
                    _CONTROL_TIMING_INDEX["mapping_duration_ns"]
                ],
                "ik_duration_ns": ik_duration_ns,
                "collision_safety_duration_ns": timing_values[
                    _CONTROL_TIMING_INDEX["collision_singularity_duration_ns"]
                ],
                "output_feasibility_duration_ns": timing_values[
                    _CONTROL_TIMING_INDEX["output_feasibility_duration_ns"]
                ],
                "control_total_duration_ns": time.perf_counter_ns() - tick_started_ns,
            }
        backlog_m = float(
            np.linalg.norm(
                np.asarray(desired.position_m)
                - np.asarray(evaluated_target.position_m)
            )
        )
        backlog_rad = quaternion_angle_rad(
            desired.orientation_xyzw, evaluated_target.orientation_xyzw
        )
        self.maximum_requested_backlog_m = max(
            self.maximum_requested_backlog_m, backlog_m
        )
        self.maximum_requested_backlog_rad = max(
            self.maximum_requested_backlog_rad, backlog_rad
        )
        limits = self.config.feasibility
        singularity_warning = bool(
            result.metrics.jacobian_condition >= limits.jacobian_slowdown_condition
            or result.metrics.minimum_jacobian_singular_value
            <= limits.minimum_singular_value_slowdown
            or (
                limits.wrist_proximity_warning_rad > 0.0
                and result.metrics.wrist_bend_from_singularity_rad
                <= limits.wrist_proximity_warning_rad
            )
        )
        if singularity_warning:
            self.singularity_warning_count += 1
        record.update(
            desired_tcp=_pose_dict(desired),
            mapped_tcp_target=_pose_dict(desired),
            filtered_tcp_target=_pose_dict(evaluated_target),
            continuation_enabled=self.continuation_enabled,
            continuation_fraction=continuation_fraction,
            continuation_backtracks=continuation_backtracks,
            continuation_attempt_reasons=attempted_reasons,
            continuation_attempt_fractions=attempted_continuation_fractions,
            output_feasibility_attempts=output_feasibility_attempts,
            continuation_attempt_timing_ms=continuation_attempt_timing_ms,
            requested_backlog_m=backlog_m,
            requested_backlog_deg=math.degrees(backlog_rad),
            singularity_warning=singularity_warning,
            metrics=asdict(result.metrics),
            previous_accepted_target_sequence=self._accepted_sequence,
            candidate_source_sequence=state.right.source_sequence_number,
            output_velocity_boundary_rad_s=(
                self.config.output_contract.maximum_velocity_rad_s
            ),
            output_velocity_boundary_rad_s_per_joint=list(
                self.config.output_contract.velocity_boundaries_rad_s
            ),
            output_acceleration_boundary_rad_s2=(
                self.config.output_contract.maximum_acceleration_rad_s2
            ),
            ik_solution_rad=result.joint_target_rad,
            ik_rejection_reason=None if result.accepted else result.reason.value,
            hold_last=not result.accepted,
            ik_computation_ms=(time.perf_counter_ns() - started) / 1e6,
            control_compute_budget_ms=self.control_compute_budget_ms,
            control_compute_budget_exhausted=(
                result.reason is FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED
            ),
        )
        if result.reason is FeasibilityReason.CONTROL_COMPUTE_BUDGET_EXHAUSTED:
            self.control_compute_budget_exhausted_count += 1
        if not result.accepted:
            self._handle_rejection(now_ns, result.reason.value)
            if result.metrics.hard_stop_required:
                self.arm_clutch.fault(now_ns, "HARD_SINGULARITY_AT_ACCEPTED_STATE")
                self.arm_mapper.clear()
                attach_control_timing()
                diagnostic_started_ns = time.perf_counter_ns()
                record.update(
                    accepted=False,
                    reason=result.reason.value,
                    control_state=ArmControlState.HARD_STOP.value,
                    heartbeat_applied=False,
                    hard_stop_reason="HARD_SINGULARITY_AT_ACCEPTED_STATE",
                    adapter_dispatch_ms=0.0,
                    control_tick_wall_ms=(time.perf_counter_ns() - tick_started_ns) / 1e6,
                )
                self.event_records.append(record)
                event_diagnostic_duration_ns += time.perf_counter_ns() - diagnostic_started_ns
                self._finish_control_timing(
                    now_ns,
                    timing_values,
                    tick_started_ns,
                    event_diagnostic_duration_ns,
                    state=state,
                )
                return ArmControlTickResult(
                    state.right.host_sequence_number,
                    state.right.wrist_pose,
                    None
                    if self.arm_mapper.last_telemetry is None
                    else self.arm_mapper.last_telemetry.hand_local_delta,
                    desired,
                    evaluated_target,
                    result,
                    None,
                    False,
                    result.reason.value,
                )
            if self._hold_rejected_started_ns is None:
                self._hold_rejected_started_ns = now_ns
            assert state.right.host_sequence_number is not None
            input_receive_ns = int(state.right.host_receive_monotonic_ns or now_ns)
            heartbeat = ArmControlHeartbeat(
                input_sequence_number=state.right.host_sequence_number,
                input_receive_monotonic_ns=min(input_receive_ns, now_ns),
                generated_monotonic_ns=now_ns,
                reference_generation=self.reference_generation,
                clutch_generation=self.arm_clutch.cycle_count,
                state=ArmControlState.HOLD_REJECTED,
                reason=result.reason.value,
                last_accepted_target_sequence=self._accepted_sequence,
            )
            dispatch_started_ns = time.perf_counter_ns()
            heartbeat_applied = self.arm_output.heartbeat(heartbeat)
            adapter_dispatch_ms = (time.perf_counter_ns() - dispatch_started_ns) / 1e6
            timing_values[_CONTROL_TIMING_INDEX["target_encode_publish_duration_ns"]] = (
                int(round(adapter_dispatch_ms * 1e6))
            )
            attach_control_timing()
            diagnostic_started_ns = time.perf_counter_ns()
            record.update(
                accepted=False,
                reason=result.reason.value,
                control_state=ArmControlState.HOLD_REJECTED.value,
                heartbeat_applied=heartbeat_applied,
                heartbeat_generated_monotonic_ns=now_ns,
                hold_duration_s=(now_ns - self._hold_rejected_started_ns) / 1e9,
                last_accepted_target_age_s=(
                    None
                    if self._last_accepted_generated_ns is None
                    else (now_ns - self._last_accepted_generated_ns) / 1e9
                ),
                adapter_dispatch_ms=adapter_dispatch_ms,
                control_tick_wall_ms=(time.perf_counter_ns() - tick_started_ns) / 1e6,
            )
            self.event_records.append(record)
            event_diagnostic_duration_ns += time.perf_counter_ns() - diagnostic_started_ns
            self._finish_control_timing(
                now_ns,
                timing_values,
                tick_started_ns,
                event_diagnostic_duration_ns,
                state=state,
            )
            return ArmControlTickResult(
                state.right.host_sequence_number,
                state.right.wrist_pose,
                None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
                desired,
                evaluated_target,
                result,
                None,
                heartbeat_applied,
                result.reason.value,
            )
        assert result.joint_target_rad is not None
        assert state.right.host_sequence_number is not None
        input_receive_ns = int(state.right.host_receive_monotonic_ns or now_ns)
        target_encode_started_ns = time.perf_counter_ns()
        self._accepted_sequence += 1
        accepted_target = AcceptedArmTarget(
            sequence_number=self._accepted_sequence,
            input_sequence_number=state.right.host_sequence_number,
            source_sequence_number=state.right.source_sequence_number,
            source_timestamp_ns=state.right.source_timestamp_ns,
            input_receive_monotonic_ns=min(input_receive_ns, now_ns),
            generated_monotonic_ns=now_ns,
            reference_generation=self.reference_generation,
            clutch_generation=self.arm_clutch.cycle_count,
            desired_tcp=AcceptedTcpPose(
                position_m=desired.position_m,
                orientation_xyzw=desired.orientation_xyzw,
            ),
            filtered_tcp=AcceptedTcpPose(
                position_m=evaluated_target.position_m,
                orientation_xyzw=evaluated_target.orientation_xyzw,
            ),
            joint_position_rad=result.joint_target_rad,
            diagnostics=AcceptedTargetDiagnostics(
                final_reason=result.reason.value,
                attempted_reasons=tuple(attempted_reasons),
                continuation_fraction=continuation_fraction,
                continuation_backtracks=continuation_backtracks,
                ik_position_error_m=result.metrics.ik_error_m,
                ik_orientation_error_rad=result.metrics.ik_orientation_error_rad,
                jacobian_condition=result.metrics.jacobian_condition,
                minimum_jacobian_singular_value=result.metrics.minimum_jacobian_singular_value,
                nearest_safe_joint_limit_margin_rad=result.metrics.nearest_safe_joint_limit_margin_rad,
            ),
        )
        dispatch_started_ns = time.perf_counter_ns()
        output_applied = self.arm_output.apply(accepted_target)
        self.last_accepted_target = accepted_target
        adapter_dispatch_ms = (time.perf_counter_ns() - dispatch_started_ns) / 1e6
        timing_values[_CONTROL_TIMING_INDEX["target_encode_publish_duration_ns"]] = (
            time.perf_counter_ns() - target_encode_started_ns
        )
        recovery_event = self._hold_rejected_started_ns is not None
        self._hold_rejected_started_ns = None
        self._last_accepted_generated_ns = now_ns
        self.accepted_targets += 1
        self.consecutive_rejections = 0
        self.last_reason = FeasibilityReason.ACCEPTED.value
        self.last_desired = evaluated_target
        current = (
            self.mujoco_plant.current_tcp_pose
            if self.mujoco_plant is not None
            else self.target_generator.current_tcp_pose
        )
        attach_control_timing()
        diagnostic_started_ns = time.perf_counter_ns()
        record.update(
            accepted=True,
            reason=FeasibilityReason.ACCEPTED.value,
            position_error_m=float(np.linalg.norm(np.asarray(evaluated_target.position_m) - np.asarray(current.position_m))),
            orientation_error_deg=math.degrees(quaternion_angle_rad(evaluated_target.orientation_xyzw, current.orientation_xyzw)),
            accepted_joint_target_rad=list(accepted_target.joint_position_rad),
            accepted_target_sequence=accepted_target.sequence_number,
            accepted_source_sequence=accepted_target.source_sequence_number,
            accepted_source_timestamp_ns=accepted_target.source_timestamp_ns,
            accepted_reference_generation=accepted_target.reference_generation,
            accepted_clutch_generation=accepted_target.clutch_generation,
            accepted_diagnostics=asdict(accepted_target.diagnostics),
            output_applied=output_applied,
            control_state=ArmControlState.ACTIVE.value,
            recovery_event=recovery_event,
            heartbeat_applied=False,
            adapter_dispatch_ms=adapter_dispatch_ms,
            control_tick_wall_ms=(time.perf_counter_ns() - tick_started_ns) / 1e6,
        )
        self.event_records.append(record)
        event_diagnostic_duration_ns += time.perf_counter_ns() - diagnostic_started_ns
        self._finish_control_timing(
            now_ns,
            timing_values,
            tick_started_ns,
            event_diagnostic_duration_ns,
            state=state,
        )
        return ArmControlTickResult(
            state.right.host_sequence_number,
            state.right.wrist_pose,
            None if self.arm_mapper.last_telemetry is None else self.arm_mapper.last_telemetry.hand_local_delta,
            desired,
            evaluated_target,
            result,
            accepted_target,
            output_applied,
            FeasibilityReason.ACCEPTED.value,
        )

    def _finish_control_timing(
        self,
        now_ns: int,
        timing_values: list[int],
        tick_started_ns: int,
        event_diagnostic_duration_ns: int,
        *,
        state: CanonicalQuestState,
    ) -> None:
        timing_values[_CONTROL_TIMING_INDEX["rh56_command_duration_ns"]] = (
            self._active_rh56_command_duration_ns
        )
        timing_values[_CONTROL_TIMING_INDEX["event_diagnostic_duration_ns"]] = (
            max(0, int(event_diagnostic_duration_ns))
        )
        timing_values[_CONTROL_TIMING_INDEX["control_total_duration_ns"]] = (
            time.perf_counter_ns() - tick_started_ns
        )
        over_budget = bool(
            self.control_compute_budget_ns is not None
            and timing_values[_CONTROL_TIMING_INDEX["control_total_duration_ns"]]
            >= self.control_compute_budget_ns
        )
        context = {
            "quest_age_ns": (
                None
                if state.right.stream_age_s is None
                else int(max(0.0, state.right.stream_age_s) * 1e9)
            ),
            "hand_state": self.hand_clutch.state.value,
            "arm_clutch_state": self.arm_clutch.state.value,
            "rh56_command_write": bool(
                timing_values[_CONTROL_TIMING_INDEX["rh56_command_duration_ns"]]
            ),
            "rh56_feedback_read": False,
            "episode_metadata_published": False,
            "event_log_enqueued": False,
            "camera_health": "unknown",
            "recorder_health": "unknown",
        }
        self._control_timing.record(
            timestamp_ns=now_ns,
            durations_ns=timing_values,
            over_budget=over_budget,
            context=context,
        )

    def add_control_timing(self, field: str, duration_ns: int) -> None:
        """Add an outer-producer phase measured after ``control_tick``."""

        self._control_timing.add_to_last(field, duration_ns)

    def set_control_timing(self, field: str, duration_ns: int) -> None:
        self._control_timing.set_last(field, duration_ns)

    def update_control_timing_context(self, values: dict[str, Any]) -> None:
        self._control_timing.update_last_context(values)

    def finalize_control_timing(self, total_duration_ns: int) -> None:
        self._control_timing.set_last("control_total_duration_ns", total_duration_ns)
        if (
            self.control_compute_budget_ns is not None
            and total_duration_ns >= self.control_compute_budget_ns
        ):
            self._control_timing.mark_last_over_budget()

    def control_timing_report(self) -> dict[str, Any]:
        return self._control_timing.report()

    def _arm_target(self, state: CanonicalQuestState, action: ClutchAction, now_ns: int):
        if action is ClutchAction.CAPTURE_ARM_REFERENCE:
            assert state.right.wrist_pose is not None and state.head is not None
            started = time.perf_counter_ns()
            authoritative_tcp = self.target_generator.capture_reference()
            desired = self.arm_mapper.capture(
                wrist=state.right.wrist_pose,
                robot_tcp=authoritative_tcp,
                head=state.head.pose,
                timestamp_ns=now_ns,
            )
            self.arm_clutch.reference_captured(now_ns)
            self.reference_generation += 1
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

    def _capture_hand_reference(self, state: CanonicalQuestState, now_ns: int) -> bool:
        """Capture all enabled hand references on a grip edge without a jump."""

        if self.hand_retargeter is None:
            return True
        # A new clutch cycle must reference the current pose, not a feature
        # slew state carried from the previous engagement.
        self.hand_retargeter.reset()
        self._pinch_pose_blender.reset()
        self._thumb_first_pinch.reset()
        self._thumb_first_pinch_stage = "idle"
        self._pinch_blend_mode = "none"
        self._pinch_blend_weight = 0.0
        self._pinch_continuous_target = None
        self._hand_alignment_target = None
        features = self._hand_features(state, now_ns)
        if features is None:
            return False
        self._four_finger_feature_reference = features[:4].copy()
        self._four_finger_features = features[:4].copy()
        self._four_finger_feature_delta = np.zeros(4, dtype=float)
        self._thumb_close_feature_reference = float(features[4])
        self._thumb_lateral_feature_reference = float(features[5])
        if self.normalized_hand_output is not None:
            command_started_ns = time.perf_counter_ns()
            measured = self.normalized_hand_output.activate_from_measured(now_ns)
            self._active_rh56_command_duration_ns += (
                time.perf_counter_ns() - command_started_ns
            )
            # Session-internal order is lateral, close, index, middle, ring, pinky.
            measured_reference = np.asarray(
                [measured[5], measured[4], *measured[:4]], dtype=np.float64
            )
            self._hand_activation_reference = measured_reference.copy()
            if self.hand_align_on_grip:
                # ``features`` is canonical Quest order: index, middle, ring,
                # pinky, thumb-close, thumb-lateral.  Use this absolute pose
                # as the new target reference while retaining measured
                # activation in ``_held_hand_command`` so no command jumps.
                self._hand_target_reference = np.asarray(
                    [features[5], features[4], *features[:4]], dtype=np.float64
                )
                self._hand_alignment_target = self._hand_target_reference.copy()
                pinch_diagnostics = self.last_hand_result.pinch_diagnostics
                pinch_confidence = pinch_diagnostics.get("pinch_confidence", 0.0)
                validated_index = (
                    self.hand_retargeter.calibration.validated_pinch_poses.get(
                        "index"
                    )
                    if self.hand_align_index_pinch_to_validated_pose
                    and str(pinch_diagnostics.get("pinch_mode", "none")) == "index"
                    and isinstance(pinch_confidence, (int, float))
                    and math.isfinite(float(pinch_confidence))
                    and float(pinch_confidence) > 0.0
                    else None
                )
                if validated_index is not None:
                    # Keep middle/ring/pinky at the current Quest absolute
                    # target.  Only the three coupled index-pinch channels
                    # are replaced by the validated contact relationship.
                    aligned = self._hand_target_reference.copy()
                    aligned[0] = validated_index[5]
                    aligned[1] = validated_index[4]
                    aligned[2] = validated_index[0]
                    self._hand_target_reference = aligned
                    self._hand_alignment_target = self._hand_target_reference.copy()
            else:
                self._hand_target_reference = measured_reference
                self._hand_alignment_target = None
        else:
            assert self.mujoco_plant is not None
            self._hand_target_reference = self.mujoco_plant.commanded_hand_target.copy()
            self._hand_activation_reference = self._hand_target_reference.copy()
            self._hand_alignment_target = None
        self._held_hand_command = self._hand_activation_reference.copy()
        self._four_finger_requested_target = self._hand_target_reference[2:].copy()
        self._four_finger_clipped_target = self._hand_target_reference[2:].copy()
        self._four_finger_saturated = np.zeros(4, dtype=bool)
        self._thumb_close_feature_delta = 0.0
        self._thumb_close_requested_target = float(self._hand_target_reference[1])
        self._thumb_close_clipped_target = float(self._hand_target_reference[1])
        self._thumb_close_saturated = False
        self._thumb_lateral_feature_delta = 0.0
        self._thumb_lateral_requested_target = float(self._hand_target_reference[0])
        self._thumb_lateral_clipped_target = float(self._hand_target_reference[0])
        self._thumb_lateral_saturated = False
        self._requested_hand_target = self._hand_target_reference.copy()
        self._clipped_hand_target = self._hand_target_reference.copy()
        self._hand_press_receive_ns = self._grip_sample.host_receive_monotonic_ns
        self.hand_engagement_latencies_ns.append(max(0, now_ns - self._hand_press_receive_ns))
        return True

    def _update_hand(
        self,
        state: CanonicalQuestState,
        action: ClutchAction,
        now_ns: int,
        *,
        skeleton_valid: bool,
    ) -> None:
        if (
            self.hand_retargeter is None
            or action is not ClutchAction.UPDATE
            or not skeleton_valid
            or self._four_finger_feature_reference is None
            or self._thumb_close_feature_reference is None
            or self._thumb_lateral_feature_reference is None
            or self._hand_target_reference is None
        ):
            return
        features = self._hand_features(state, now_ns)
        if features is None:
            return
        finger_delta = features[:4] - self._four_finger_feature_reference
        finger_dead_zone = self.four_finger_dead_zone_rad
        thumb_dead_zone = self.thumb_close_dead_zone_rad
        if self.normalized_hand_output is not None:
            finger_dead_zone = max(
                self.four_finger_dead_zone_rad
                / RH56_MUJOCO_ACTUATOR_MAX_RAD[name]
                for name in ("index", "middle", "ring", "pinky")
            )
            thumb_dead_zone = (
                self.thumb_close_dead_zone_rad
                / RH56_MUJOCO_ACTUATOR_MAX_RAD["thumb_close"]
            )
        finger_delta[np.abs(finger_delta) <= finger_dead_zone] = 0.0
        thumb_delta = float(features[4] - self._thumb_close_feature_reference)
        if abs(thumb_delta) <= thumb_dead_zone:
            thumb_delta = 0.0
        lateral_delta = float(features[5] - self._thumb_lateral_feature_reference)
        if abs(lateral_delta) <= self.thumb_lateral_dead_zone:
            lateral_delta = 0.0
        requested_fingers = self._hand_target_reference[2:] + self.four_finger_gain * finger_delta
        requested_thumb_close = self._hand_target_reference[1] + self.thumb_close_gain * thumb_delta
        _joint_range, _ctrl_range, valid_range = self._hand_channel_model_ranges(1)
        clipped_thumb_close = float(
            np.clip(requested_thumb_close, valid_range[0], valid_range[1])
        )
        _lateral_joint_range, _lateral_ctrl_range, lateral_valid_range = (
            self._hand_channel_model_ranges(0)
        )
        lateral_span = lateral_valid_range[1] - lateral_valid_range[0]
        requested_thumb_lateral = (
            self._hand_target_reference[0]
            + self.thumb_lateral_gain * lateral_delta * lateral_span
        )
        clipped_thumb_lateral = float(
            np.clip(
                requested_thumb_lateral,
                lateral_valid_range[0],
                lateral_valid_range[1],
            )
        )
        self._thumb_close_feature_delta = thumb_delta
        self._thumb_close_requested_target = float(requested_thumb_close)
        self._thumb_close_clipped_target = clipped_thumb_close
        self._thumb_close_saturated = not math.isclose(
            requested_thumb_close,
            clipped_thumb_close,
            abs_tol=1e-12,
        )
        self._thumb_lateral_feature_delta = lateral_delta
        self._thumb_lateral_requested_target = float(requested_thumb_lateral)
        self._thumb_lateral_clipped_target = clipped_thumb_lateral
        self._thumb_lateral_saturated = not math.isclose(
            requested_thumb_lateral,
            clipped_thumb_lateral,
            abs_tol=1e-12,
        )
        target = self._held_hand_command.copy()
        target[0] = clipped_thumb_lateral
        target[1] = clipped_thumb_close
        target[2:] = requested_fingers
        if self.normalized_hand_output is not None:
            continuous_canonical = np.asarray(
                [*target[2:].tolist(), float(target[1]), float(target[0])],
                dtype=np.float64,
            )
            self._pinch_continuous_target = continuous_canonical.copy()
            diagnostics = (
                {}
                if self.last_hand_result is None
                else self.last_hand_result.pinch_diagnostics
            )
            pinch_mode = str(diagnostics.get("pinch_mode", "none"))
            pinch_confidence = float(diagnostics.get("pinch_confidence", 0.0))
            validated_index = (
                self.hand_retargeter.calibration.validated_pinch_poses.get("index")
                if self.hand_align_index_pinch_to_validated_pose
                and pinch_mode == "index"
                and math.isfinite(pinch_confidence)
                and pinch_confidence > 0.0
                else None
            )
            if validated_index is not None:
                # The physical calibration established a real index/thumb
                # contact relationship. A detected index pinch may otherwise
                # map to insufficient lateral opposition (observed around
                # .75). Replace only index, thumb-close, and thumb-lateral;
                # middle/ring/pinky remain the continuous Quest mapping.
                blended_canonical = continuous_canonical.copy()
                blended_canonical[0] = validated_index[0]
                blended_canonical[4] = validated_index[4]
                blended_canonical[5] = validated_index[5]
                self._pinch_blend_mode = "validated_index"
                self._pinch_blend_weight = 1.0
            else:
                blended_canonical, self._pinch_blend_mode, self._pinch_blend_weight = (
                    self._pinch_pose_blender.update(
                        continuous_canonical,
                        detected_mode=pinch_mode,
                        confidence=pinch_confidence,
                        tracking_valid=skeleton_valid,
                    )
                )
            target = np.asarray(
                [
                    blended_canonical[5],
                    blended_canonical[4],
                    *blended_canonical[:4],
                ],
                dtype=np.float64,
            )
            measured_canonical = self._latest_hand_measured_canonical(now_ns)
            target, self._thumb_first_pinch_stage = self._thumb_first_pinch.update(
                np.asarray(
                    [*target[2:].tolist(), float(target[1]), float(target[0])],
                    dtype=np.float64,
                ),
                detected_mode=str(diagnostics.get("pinch_mode", "none")),
                confidence=float(diagnostics.get("pinch_confidence", 0.0)),
                tracking_valid=skeleton_valid,
                measured_canonical=measured_canonical,
            )
            target = np.asarray(
                [target[5], target[4], *target[:4]],
                dtype=np.float64,
            )
        channel_ranges = np.asarray(
            [self._hand_channel_model_ranges(index)[2] for index in range(6)],
            dtype=float,
        )
        requested_target = target.copy()
        target = np.clip(target, channel_ranges[:, 0], channel_ranges[:, 1])
        self._four_finger_features = features[:4].copy()
        self._four_finger_feature_delta = finger_delta.copy()
        self._four_finger_requested_target = requested_fingers.copy()
        self._four_finger_clipped_target = target[2:].copy()
        self._four_finger_saturated = ~np.isclose(
            requested_fingers, target[2:], atol=1e-12, rtol=0.0
        )
        self._requested_hand_target = requested_target
        self._clipped_hand_target = target.copy()
        if self.normalized_hand_output is None:
            step = np.clip(
                target[2:] - self._held_hand_command[2:],
                -self.four_finger_max_step_rad,
                self.four_finger_max_step_rad,
            )
            target[2:] = self._held_hand_command[2:] + step
            thumb_step = float(np.clip(
                target[1] - self._held_hand_command[1],
                -self.thumb_close_max_step_rad,
                self.thumb_close_max_step_rad,
            ))
            target[1] = self._held_hand_command[1] + thumb_step
            lateral_step = float(np.clip(
                target[0] - self._held_hand_command[0],
                -self.thumb_lateral_max_step_rad,
                self.thumb_lateral_max_step_rad,
            ))
            target[0] = self._held_hand_command[0] + lateral_step
        if not np.all(np.isfinite(target)):
            return
        if self.normalized_hand_output is not None:
            canonical_target = [*target[2:].tolist(), float(target[1]), float(target[0])]
            command_started_ns = time.perf_counter_ns()
            self.normalized_hand_output.submit_target(canonical_target, now_ns)
            self._active_rh56_command_duration_ns += (
                time.perf_counter_ns() - command_started_ns
            )
        else:
            order = ("thumb_lateral", "thumb_close", "index", "middle", "ring", "pinky")
            mapping = dict(zip(order, target.tolist(), strict=True))
            assert self.mujoco_plant is not None
            self.mujoco_plant.set_hand_actuator_target(mapping)
        self._held_hand_command = target.copy()
        self._hand_updated_this_tick = True

    def _hand_features(self, state: CanonicalQuestState, now_ns: int) -> np.ndarray | None:
        assert self.hand_retargeter is not None
        started = time.perf_counter_ns()
        result = self.hand_retargeter.retarget(QuestHandSkeleton.from_observation(state.right))
        self.hand_retarget_durations_ns.append(time.perf_counter_ns() - started)
        self.hand_timestamps_ns.append(now_ns)
        self.last_hand_result = result
        if not result.valid:
            return None
        lateral_feature = result.pinch_diagnostics.get(
            "thumb_lateral_effective_feature"
        )
        source = (
            result.normalized_targets
            if self.normalized_hand_output is not None
            else result.actuator_targets
        )
        features = np.asarray(
            [source[name] for name in ("index", "middle", "ring", "pinky", "thumb_close")]
            + [float(lateral_feature) if lateral_feature is not None else math.nan],
            dtype=float,
        )
        if features.shape != (6,) or not np.all(np.isfinite(features)):
            return None
        self.hand_valid_results += 1
        return features

    def _clear_hand_reference(self) -> None:
        self._pinch_pose_blender.reset()
        self._thumb_first_pinch.reset()
        self._thumb_first_pinch_stage = "idle"
        self._pinch_blend_mode = "none"
        self._pinch_blend_weight = 0.0
        self._pinch_continuous_target = None
        self._four_finger_feature_reference = None
        self._four_finger_features = None
        self._four_finger_feature_delta = None
        self._four_finger_requested_target = None
        self._four_finger_clipped_target = None
        self._four_finger_saturated = None
        self._thumb_close_feature_reference = None
        self._thumb_lateral_feature_reference = None
        self._hand_target_reference = None
        self._hand_activation_reference = None
        self._hand_alignment_target = None
        self._requested_hand_target = None
        self._clipped_hand_target = None
        self._thumb_close_feature_delta = None
        self._thumb_close_requested_target = None
        self._thumb_close_clipped_target = None
        self._thumb_close_saturated = False
        self._thumb_lateral_feature_delta = None
        self._thumb_lateral_requested_target = None
        self._thumb_lateral_clipped_target = None
        self._thumb_lateral_saturated = False
        self._hand_press_receive_ns = None

    def _latest_hand_measured_canonical(self, now_ns: int) -> np.ndarray | None:
        """Read worker feedback without coupling the session to RH56 classes."""

        output = self.normalized_hand_output
        feedback = None if output is None else getattr(output, "latest_feedback", None)
        if feedback is None:
            return None
        feedback_ns = getattr(feedback, "monotonic_ns", None)
        if feedback_ns is not None and (
            not isinstance(feedback_ns, int)
            or now_ns < feedback_ns
            or now_ns - feedback_ns > 250_000_000
        ):
            return None
        values = getattr(feedback, "position_normalized", None)
        if values is None:
            return None
        measured = np.asarray(values, dtype=np.float64)
        if measured.shape != (6,) or not np.all(np.isfinite(measured)):
            return None
        return measured

    def _hand_channel_model_ranges(
        self,
        actuator_order_index: int,
    ) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        if self.normalized_hand_output is not None:
            return (0.0, 1.0), (0.0, 1.0), (
                0.0,
                float(self.normalized_hand_output.max_target_normalized),
            )
        assert self.mujoco_plant is not None
        actuator_id = int(
            self.mujoco_plant.hand_actuator_ids[actuator_order_index]
        )
        joint_id = int(self.mujoco_plant.model.actuator_trnid[actuator_id, 0])
        joint_range = tuple(
            float(value) for value in self.mujoco_plant.model.jnt_range[joint_id]
        )
        ctrl_range = tuple(
            float(value)
            for value in self.mujoco_plant.model.actuator_ctrlrange[actuator_id]
        )
        valid_range = (
            max(joint_range[0], ctrl_range[0]),
            min(joint_range[1], ctrl_range[1]),
        )
        if valid_range[0] > valid_range[1]:
            raise ValueError("hand joint and actuator ranges do not overlap")
        return joint_range, ctrl_range, valid_range

    def _base_record(self, state: CanonicalQuestState, now_ns: int) -> dict[str, Any]:
        mapping = self.arm_mapper.last_telemetry
        plant = self.mujoco_plant
        hand_positions = None
        if plant is not None:
            hand_positions = plant.data.qpos[
                plant.model.jnt_qposadr[
                    plant.model.actuator_trnid[plant.hand_actuator_ids, 0]
                ]
            ].tolist()
        thumb_diagnostics = (
            {}
            if self.last_hand_result is None
            else self.last_hand_result.pinch_diagnostics
        )
        thumb_joint_range = thumb_ctrl_range = thumb_valid_range = None
        lateral_joint_range = lateral_ctrl_range = lateral_valid_range = None
        if plant is not None and plant.hand_available:
            (
                thumb_joint_range,
                thumb_ctrl_range,
                thumb_valid_range,
            ) = self._hand_channel_model_ranges(1)
            (
                lateral_joint_range,
                lateral_ctrl_range,
                lateral_valid_range,
            ) = self._hand_channel_model_ranges(0)
        thumb_captured_target = (
            None
            if self._hand_activation_reference is None
            else float(self._hand_activation_reference[1])
        )
        lateral_captured_target = (
            None
            if self._hand_activation_reference is None
            else float(self._hand_activation_reference[0])
        )
        actual_joint_position = None if plant is None else plant.arm_joints_rad
        actual_tcp = None if plant is None else plant.current_tcp_pose
        desired_tcp = self.target_generator.last_safe_target
        tracking_error_m = (
            None
            if actual_tcp is None
            else float(
                np.linalg.norm(
                    np.asarray(desired_tcp.position_m)
                    - np.asarray(actual_tcp.position_m)
                )
            )
        )
        contact_summary = None
        if plant is not None:
            contact_summary = {
                "count": int(plant.data.ncon),
                "minimum_distance_m": min(
                    (float(plant.data.contact[index].dist) for index in range(plant.data.ncon)),
                    default=None,
                ),
            }
        return {
            "control_monotonic_ns": now_ns,
            "mujoco_time_s": None if plant is None else float(plant.data.time),
            "input_sequence": state.right.host_sequence_number,
            "source_sequence": state.right.source_sequence_number,
            "source_timestamp_ns": state.right.source_timestamp_ns,
            "raw_quest_wrist_timestamp_ns": state.right.host_receive_monotonic_ns,
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
            "operator_reference_wrist": _pose_dict(self.arm_mapper.hand_reference),
            "current_arm_target": _pose_dict(self.target_generator.last_safe_target),
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
            "hand_reference_captured": self._four_finger_feature_reference is not None,
            "hand_align_on_grip": self.hand_align_on_grip,
            "hand_alignment_target_rad": (
                None
                if self._hand_alignment_target is None
                else self._hand_alignment_target.tolist()
            ),
            "active_arm_fault": None if self.arm_clutch.active_fault is None else self.arm_clutch.active_fault.reason,
            "active_hand_fault": None if self.hand_clutch.active_fault is None else self.hand_clutch.active_fault.reason,
            "input_recovery_active": self._input_recovery_started_ns is not None,
            "input_recovery_elapsed_s": (
                None
                if self._input_recovery_started_ns is None
                else max(0.0, (now_ns - self._input_recovery_started_ns) / 1e9)
            ),
            "input_recovery_timeout_s": self.config.input_recovery_timeout_s,
            "input_recovery_hard_stop_reason": (
                self._input_recovery_hard_stop_reason
            ),
            "arm_clutch_cycle_count": self.arm_clutch.cycle_count,
            "reference_generation": self.reference_generation,
            "hand_clutch_cycle_count": self.hand_clutch.cycle_count,
            "raw_wrist": _pose_dict(self.arm_mapper.raw_wrist),
            "filtered_wrist": _pose_dict(self.arm_mapper.filtered_wrist),
            "shared_model_tcp": _pose_dict(self.target_generator.current_tcp_pose),
            "desired_tcp": _pose_dict(desired_tcp),
            "actual_tcp": _pose_dict(actual_tcp),
            "tracking_error_m": tracking_error_m,
            "actual_joint_position_rad": None if actual_joint_position is None else actual_joint_position.tolist(),
            "actual_joint_velocity_rad_s": None if plant is None else plant.data.qvel[plant.arm_dof_ids].tolist(),
            "simulated_joint_target_rad": None if plant is None else plant.commanded_joint_target.tolist(),
            "commanded_hand_target_rad": None if plant is None else plant.commanded_hand_target.tolist(),
            "actual_hand_actuator_position_rad": hand_positions,
            "contact_summary": contact_summary,
            "hand_target_reference_rad": None if self._hand_target_reference is None else self._hand_target_reference.tolist(),
            "hand_activation_reference_rad": (
                None
                if self._hand_activation_reference is None
                else self._hand_activation_reference.tolist()
            ),
            "hand_requested_target_rad": None if self._requested_hand_target is None else self._requested_hand_target.tolist(),
            "hand_clipped_target_rad": None if self._clipped_hand_target is None else self._clipped_hand_target.tolist(),
            "hand_target_valid": bool(self.last_hand_result is not None and self.last_hand_result.valid),
            "hand_target_saturation": bool(
                self._thumb_close_saturated
                or self._thumb_lateral_saturated
                or (
                    self._four_finger_saturated is not None
                    and np.any(self._four_finger_saturated)
                )
            ),
            "pinch_intent_debug": {
                "mode": thumb_diagnostics.get("pinch_mode"),
                "confidence": thumb_diagnostics.get("pinch_confidence"),
                "thumb_index_distance_palm": thumb_diagnostics.get(
                    "thumb_index_distance_palm"
                ),
                "thumb_middle_distance_palm": thumb_diagnostics.get(
                    "thumb_middle_distance_palm"
                ),
                "index_middle_distance_palm": thumb_diagnostics.get(
                    "index_middle_distance_palm"
                ),
                "blend_mode": self._pinch_blend_mode,
                "blend_weight": self._pinch_blend_weight,
                "thumb_first_stage": self._thumb_first_pinch_stage,
                "continuous_target_canonical": (
                    None
                    if self._pinch_continuous_target is None
                    else self._pinch_continuous_target.tolist()
                ),
            },
            "four_finger_debug": {
                "feature_rad": None if self._four_finger_features is None else self._four_finger_features.tolist(),
                "captured_feature_reference_rad": None if self._four_finger_feature_reference is None else self._four_finger_feature_reference.tolist(),
                "feature_delta_rad": None if self._four_finger_feature_delta is None else self._four_finger_feature_delta.tolist(),
                "requested_target_rad": None if self._four_finger_requested_target is None else self._four_finger_requested_target.tolist(),
                "clipped_target_rad": None if self._four_finger_clipped_target is None else self._four_finger_clipped_target.tolist(),
                "saturation": None if self._four_finger_saturated is None else self._four_finger_saturated.tolist(),
            },
            "thumb_close_debug": {
                "raw_thumb_bend_rad": thumb_diagnostics.get(
                    "thumb_raw_bend_rad"
                ),
                "normalized_thumb_bend": thumb_diagnostics.get(
                    "thumb_normalized_bend"
                ),
                "raw_pinch_distance_m": thumb_diagnostics.get(
                    "thumb_raw_pinch_distance_m"
                ),
                "raw_pinch_distance_palm": thumb_diagnostics.get(
                    "thumb_raw_pinch_distance_palm"
                ),
                "normalized_pinch": thumb_diagnostics.get(
                    "thumb_normalized_pinch"
                ),
                "base_bend_contribution": thumb_diagnostics.get(
                    "thumb_base_bend_contribution"
                ),
                "pinch_assist_contribution": thumb_diagnostics.get(
                    "thumb_pinch_assist_contribution"
                ),
                "combined_feature_normalized": thumb_diagnostics.get(
                    "thumb_close_feature"
                ),
                "effective_feature_normalized": thumb_diagnostics.get(
                    "thumb_effective_feature"
                ),
                "captured_feature_reference_rad": self._thumb_close_feature_reference,
                "feature_delta_rad": self._thumb_close_feature_delta,
                "captured_rh56_reference_rad": thumb_captured_target,
                "requested_target_rad": self._thumb_close_requested_target,
                "clipped_target_rad": self._thumb_close_clipped_target,
                "slew_limited_target_rad": float(self._held_hand_command[1]),
                "actual_mujoco_joint_rad": (
                    None if hand_positions is None else float(hand_positions[1])
                ),
                "saturation": self._thumb_close_saturated,
                "joint_range_rad": thumb_joint_range,
                "ctrl_range_rad": thumb_ctrl_range,
                "valid_range_rad": thumb_valid_range,
            },
            "thumb_lateral_debug": {
                "raw_across_palm": thumb_diagnostics.get(
                    "thumb_lateral_raw_across_palm"
                ),
                "feature_normalized": thumb_diagnostics.get(
                    "thumb_lateral_feature"
                ),
                "effective_feature_normalized": thumb_diagnostics.get(
                    "thumb_lateral_effective_feature"
                ),
                "captured_feature_reference": self._thumb_lateral_feature_reference,
                "feature_delta": self._thumb_lateral_feature_delta,
                "captured_rh56_reference_rad": lateral_captured_target,
                "requested_target_rad": self._thumb_lateral_requested_target,
                "clipped_target_rad": self._thumb_lateral_clipped_target,
                "slew_limited_target_rad": float(self._held_hand_command[0]),
                "actual_mujoco_joint_rad": (
                    None if hand_positions is None else float(hand_positions[0])
                ),
                "saturation": self._thumb_lateral_saturated,
                "joint_range_rad": lateral_joint_range,
                "ctrl_range_rad": lateral_ctrl_range,
                "valid_range_rad": lateral_valid_range,
                "palm_width_m": thumb_diagnostics.get("palm_width_m"),
                "across_axis": [
                    thumb_diagnostics.get("palm_across_x"),
                    thumb_diagnostics.get("palm_across_y"),
                    thumb_diagnostics.get("palm_across_z"),
                ],
                "forward_axis": [
                    thumb_diagnostics.get("palm_forward_x"),
                    thumb_diagnostics.get("palm_forward_y"),
                    thumb_diagnostics.get("palm_forward_z"),
                ],
                "normal_axis": [
                    thumb_diagnostics.get("palm_normal_x"),
                    thumb_diagnostics.get("palm_normal_y"),
                    thumb_diagnostics.get("palm_normal_z"),
                ],
            },
        }

    def _handle_rejection(self, timestamp_ns: int, reason: str) -> None:
        self.rejections[reason] += 1
        self.last_reason = reason
        self.consecutive_rejections += 1
        if self.continuation_enabled:
            # Candidate rejection already holds the last safe joint target.
            # Keeping the clutch engaged lets the same absolute relative-pose
            # mapping recover as soon as the operator retreats.  Tracking and
            # controller faults are handled earlier and still fault instantly.
            return
        if self.consecutive_rejections > self.isolated_rejection_hold_count:
            self.arm_clutch.fault(timestamp_ns, reason)
            self.arm_mapper.clear()

    def report(self, replay_source: str) -> dict[str, Any]:
        metrics = self.target_generator.metrics_report()
        if self.mujoco_plant is not None and self.mujoco_plant is not self.target_generator:
            plant_metrics = self.mujoco_plant.metrics_report()
            for name in (
                "maximum_desired_to_simulated_tcp_error_m",
                "peak_actual_joint_velocity_rad_s",
                "simulated_velocity_limit_hits_per_joint",
                "simulated_acceleration_limit_hits_per_joint",
                "simulated_jerk_limit_hits_per_joint",
            ):
                metrics[name] = plant_metrics[name]
        return {
            "schema_version": "quest_jaka_rh56_full_hand_grip.v1",
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
            "shared_continuation_enabled": self.continuation_enabled,
            "shared_rejection_policy": self.rejection_policy,
            "continuation_intervention_count": self.continuation_intervention_count,
            "continuation_backtrack_count": self.continuation_backtrack_count,
            "control_compute_budget_ms": self.control_compute_budget_ms,
            "control_compute_budget_exhausted_count": (
                self.control_compute_budget_exhausted_count
            ),
            "control_timing": self.control_timing_report(),
            "input_recovery_timeout_s": self.config.input_recovery_timeout_s,
            "input_recovery_count": self.input_recovery_count,
            "input_recovery_success_count": self.input_recovery_success_count,
            "input_recovery_timeout_count": self.input_recovery_timeout_count,
            "singularity_warning_count": self.singularity_warning_count,
            "maximum_requested_backlog_m": self.maximum_requested_backlog_m,
            "maximum_requested_backlog_deg": math.degrees(
                self.maximum_requested_backlog_rad
            ),
            "arm_reference_capture_ms": _distribution_ms(self.arm_capture_durations_ns),
            "four_finger_relative_gain": self.four_finger_gain,
            "four_finger_relative_dead_zone_rad": self.four_finger_dead_zone_rad,
            "four_finger_relative_maximum_step_rad": self.four_finger_max_step_rad,
            "thumb_close_relative_gain": self.thumb_close_gain,
            "thumb_close_relative_dead_zone_rad": self.thumb_close_dead_zone_rad,
            "thumb_close_relative_maximum_step_rad": self.thumb_close_max_step_rad,
            "thumb_close_bend_gain": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_close_bend_gain
            ),
            "thumb_close_pinch_assist_gain": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_close_pinch_assist_gain
            ),
            "thumb_lateral_relative_gain": self.thumb_lateral_gain,
            "thumb_lateral_relative_dead_zone": self.thumb_lateral_dead_zone,
            "thumb_lateral_relative_maximum_step_rad": self.thumb_lateral_max_step_rad,
            "thumb_lateral_open_across_palm": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_lateral_open_across_palm
            ),
            "thumb_lateral_pregrasp_across_palm": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_lateral_pregrasp_across_palm
            ),
            "thumb_lateral_pregrasp_normalized": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_lateral_pregrasp_normalized
            ),
            "thumb_lateral_opposed_across_palm": (
                None
                if self.hand_retargeter is None
                else self.hand_retargeter.calibration.thumb_lateral_opposed_across_palm
            ),
            "ik_computation_ms": _event_metric(self.event_records, "ik_computation_ms"),
            "control_tick_wall_ms": _event_metric(
                self.event_records, "control_tick_wall_ms"
            ),
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
            **self._motion_statistics(),
            **metrics,
        }

    def _motion_statistics(self) -> dict[str, Any]:
        """Small aggregate diagnostics over the existing per-tick event log."""

        predicted_peaks = np.zeros(6, dtype=float)
        velocity_hits = np.zeros(6, dtype=int)
        acceleration_hits = np.zeros(6, dtype=int)
        for record in self.event_records:
            metrics = record.get("metrics") or {}
            predicted = metrics.get("predicted_output_joint_velocity_rad_s") or ()
            if len(predicted) == 6:
                predicted_peaks = np.maximum(predicted_peaks, np.abs(predicted))
            for attempt in record.get("output_feasibility_attempts") or ():
                for index in attempt.get("violating_joint_indices_zero_based") or ():
                    velocity_hits[int(index)] += 1
                for index in attempt.get("acceleration_violating_joint_indices_zero_based") or ():
                    acceleration_hits[int(index)] += 1
        fractions = [
            float(record["continuation_fraction"])
            for record in self.event_records
            if record.get("continuation_fraction") is not None
        ]
        backlogs = [
            (int(record["control_monotonic_ns"]), float(record["orientation_backlog_deg"]))
            for record in self.event_records
            if record.get("orientation_backlog_deg") is not None
        ]
        recovery_ms = None
        for current_index, (previous, current) in enumerate(
            zip(self.event_records, self.event_records[1:]),
            start=1,
        ):
            if previous.get("arm_clutch_state") == "engaged" and current.get("arm_clutch_state") != "engaged":
                release_ns = current.get("control_monotonic_ns")
                if release_ns is None:
                    continue
                for candidate in self.event_records[current_index:]:
                    if candidate.get("orientation_backlog_deg", float("inf")) <= 1.0:
                        recovery_ms = (candidate["control_monotonic_ns"] - release_ns) / 1e6
                        break
                if recovery_ms is not None:
                    break
        return {
            "predicted_joint_peak_velocity_rad_s_per_joint": predicted_peaks.tolist(),
            "predicted_velocity_limit_hit_count_per_joint": velocity_hits.tolist(),
            "output_acceleration_limit_hit_count_per_joint": acceleration_hits.tolist(),
            "continuation_fraction_min": min(fractions, default=None),
            "continuation_fraction_below_one_ratio": (
                sum(value < 1.0 for value in fractions) / len(fractions)
                if fractions else None
            ),
            "maximum_tcp_orientation_backlog_deg": max(
                (value for _, value in backlogs), default=0.0
            ),
            "orientation_backlog_recovery_to_1deg_ms_after_release": recovery_ms,
        }


def _output_feasibility_attempt(
    result: FeasibilityResult, fraction: float
) -> dict[str, Any]:
    metrics = result.metrics
    return {
        "continuation_fraction": fraction,
        "reason": result.reason.value,
        "candidate_interval_s": metrics.output_feasibility_interval_s,
        "joint_delta_rad": list(metrics.output_feasibility_delta_rad),
        "predicted_joint_velocity_rad_s": list(
            metrics.predicted_output_joint_velocity_rad_s
        ),
        "violating_joint_indices_zero_based": list(
            metrics.output_velocity_violating_joint_indices
        ),
        "maximum_predicted_joint_velocity_rad_s": (
            metrics.predicted_output_maximum_joint_velocity_rad_s
        ),
        "previous_emitted_joint_velocity_rad_s": list(
            metrics.previous_emitted_output_joint_velocity_rad_s
        ),
        "predicted_joint_acceleration_rad_s2": list(
            metrics.predicted_output_joint_acceleration_rad_s2
        ),
        "acceleration_violating_joint_indices_zero_based": list(
            metrics.output_acceleration_violating_joint_indices
        ),
        "maximum_predicted_joint_acceleration_rad_s2": (
            metrics.predicted_output_maximum_joint_acceleration_rad_s2
        ),
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
