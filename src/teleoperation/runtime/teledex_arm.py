from __future__ import annotations

from dataclasses import dataclass, replace

from ..contracts import PoseTarget, SafetyAction, SafetyState
from ..input.interface import AdapterSnapshot
from ..processing.clutch import ClutchController, ClutchState, ClutchUpdate
from ..processing.one_euro_se3 import OneEuroSE3Filter
from ..processing.pose_validator import PoseValidator, ValidationAction, ValidationResult
from ..processing.target_shaper import JerkLimitedPoseShaper
from ..supervision import ArmSafetySupervisor


@dataclass(frozen=True, slots=True)
class PipelineResult:
    generation: int
    validation: ValidationResult
    clutch: ClutchUpdate | None
    target: PoseTarget | None
    safety: SafetyState
    reason: str


class BoundedArmTeleoperationPipeline:
    """Device-neutral, no-SDK production pose-processing composition."""

    def __init__(
        self,
        *,
        validator: PoseValidator,
        clutch: ClutchController,
        measurement_filter: OneEuroSE3Filter,
        shaper: JerkLimitedPoseShaper,
        safety: ArmSafetySupervisor,
        startup_tcp_relative_output: bool = False,
    ) -> None:
        if not clutch.poses_are_operator_frame:
            raise ValueError("production pipeline requires normalized operator-frame clutch input")
        self.validator = validator
        self.clutch = clutch
        self.measurement_filter = measurement_filter
        self.shaper = shaper
        self.safety = safety
        self.startup_tcp_relative_output = startup_tcp_relative_output
        self._last_clutch_state = clutch.state

    @staticmethod
    def _hold(now_ns: int, reason: str, *, fault: bool = False) -> SafetyState:
        return SafetyState(
            SafetyAction.ABORT if fault else SafetyAction.HOLD,
            now_ns,
            (reason,),
            fault_latched=fault,
        )

    def process(
        self,
        snapshot: AdapterSnapshot,
        *,
        robot_tcp_pose: object,
        now_ns: int,
    ) -> PipelineResult:
        from ..contracts import Pose3D

        if not isinstance(robot_tcp_pose, Pose3D):
            raise TypeError("robot_tcp_pose must be Pose3D")
        if not snapshot.connected and snapshot.pose is None:
            self.measurement_filter.reset()
            self.clutch.require_recenter(snapshot.reason or "transport_disconnected")
            validation = ValidationResult(
                ValidationAction.CONTROLLED_STOP,
                snapshot.reason or "transport_disconnected",
                None,
            )
            safety = SafetyState(
                SafetyAction.CONTROLLED_STOP,
                now_ns,
                (validation.reason,),
                fault_latched=False,
            )
            return PipelineResult(
                snapshot.generation,
                validation,
                None,
                None,
                safety,
                validation.reason,
            )
        validation = self.validator.validate(snapshot.pose, now_ns=now_ns)
        if validation.action in {
            ValidationAction.RECLUTCH_REQUIRED,
            ValidationAction.CONTROLLED_STOP,
            ValidationAction.ABORT,
        }:
            self.measurement_filter.reset()
            self.clutch.require_recenter(validation.reason)
            if validation.action == ValidationAction.ABORT:
                self.clutch.fault()
            action = {
                ValidationAction.RECLUTCH_REQUIRED: SafetyAction.HOLD,
                ValidationAction.CONTROLLED_STOP: SafetyAction.CONTROLLED_STOP,
                ValidationAction.ABORT: SafetyAction.ABORT,
            }[validation.action]
            safety = SafetyState(
                action,
                now_ns,
                (validation.reason,),
                fault_latched=validation.action == ValidationAction.ABORT,
            )
            return PipelineResult(snapshot.generation, validation, None, None, safety, validation.reason)
        if validation.action in {ValidationAction.HOLD, ValidationAction.REJECT} or validation.sample is None:
            return PipelineResult(
                snapshot.generation,
                validation,
                None,
                None,
                self._hold(now_ns, validation.reason),
                validation.reason,
            )

        sample = validation.sample
        operator_pose = self.clutch.mapper.frames.source_pose_to_operator(sample.pose)
        operator_sample = replace(
            sample,
            frame_id=self.clutch.mapper.frames.normalized_frame_id,
            pose=operator_pose,
        )
        filtered = self.measurement_filter.filter(operator_sample)
        previous_state = self.clutch.state
        clutch_update = self.clutch.update(
            filtered,
            snapshot.run_gate,
            robot_tcp_pose=robot_tcp_pose,
        )
        current_state = clutch_update.state
        if current_state != previous_state:
            self.measurement_filter.reset()
        if current_state != ClutchState.ACTIVE or clutch_update.target_pose is None:
            if previous_state == ClutchState.ACTIVE:
                self.safety.stop()
            self._last_clutch_state = current_state
            return PipelineResult(
                snapshot.generation,
                validation,
                clutch_update,
                None,
                self._hold(now_ns, clutch_update.reason),
                clutch_update.reason,
            )

        if previous_state != ClutchState.ACTIVE:
            self.shaper.reset(robot_tcp_pose, timestamp_ns=now_ns)
            self.safety.start(now_ns)
            target = PoseTarget(
                source_id=sample.source_id,
                sequence=sample.sequence,
                target_frame_id=(
                    "startup_tcp_relative"
                    if self.startup_tcp_relative_output
                    else self.clutch.mapper.frames.robot_base_frame_id
                ),
                pose=robot_tcp_pose,
                timestamps=sample.timestamps.with_stage(processing_ns=now_ns),
                linear_velocity_m_s=(0.0, 0.0, 0.0),
                angular_velocity_rad_s=(0.0, 0.0, 0.0),
            )
        else:
            target = self.shaper.update(
                clutch_update.target_pose,
                source_id=sample.source_id,
                sequence=sample.sequence,
                source_timestamps=sample.timestamps,
                now_ns=now_ns,
            )
            if self.startup_tcp_relative_output:
                target = replace(target, target_frame_id="startup_tcp_relative")
        safety = self.safety.evaluate_cartesian(target, now_ns=now_ns)
        if safety.action != SafetyAction.ALLOW:
            if safety.action == SafetyAction.ABORT:
                self.clutch.fault()
            elif safety.action == SafetyAction.CONTROLLED_STOP:
                self.clutch.stop()
            target = None
        self._last_clutch_state = self.clutch.state
        return PipelineResult(
            snapshot.generation,
            validation,
            clutch_update,
            target,
            safety,
            "target_ready" if target is not None else safety.reasons[0],
        )

    def stop(self) -> None:
        self.clutch.stop()
        self.measurement_filter.reset()
        self.safety.stop()

    def reset_fault(self, *, gate_released: bool, safe: bool) -> None:
        self.clutch.reset_fault(gate_released=gate_released, safe=safe)
        self.safety.reset_fault(safe=safe)
        self.measurement_filter.reset()
