from __future__ import annotations

import enum
from dataclasses import dataclass

from ..contracts import ArmPoseSample, Pose3D, RunGateSample
from ..transforms.frame_mapping import RelativePoseMapper


class ClutchState(str, enum.Enum):
    WAITING_FOR_RELEASE = "waiting_for_release"
    DISENGAGED = "disengaged"
    ACTIVE = "active"
    RECENTER_REQUIRED = "recenter_required"
    STOPPED = "stopped"
    FAULT = "fault"


@dataclass(frozen=True, slots=True)
class ClutchUpdate:
    state: ClutchState
    target_pose: Pose3D | None
    reason: str
    anchor_id: int


class ClutchController:
    """Explicit clutch/recenter state independent of any input device."""

    def __init__(self, mapper: RelativePoseMapper, *, poses_are_operator_frame: bool = False) -> None:
        self.mapper = mapper
        self.poses_are_operator_frame = poses_are_operator_frame
        self.state = ClutchState.WAITING_FOR_RELEASE
        self._last_gate_engaged = False

    def update(
        self,
        sample: ArmPoseSample,
        gate: RunGateSample,
        *,
        robot_tcp_pose: Pose3D,
    ) -> ClutchUpdate:
        if self.state in {ClutchState.STOPPED, ClutchState.FAULT}:
            return ClutchUpdate(self.state, None, "latched", self.mapper.anchor_id)
        if not gate.valid or not sample.tracking_valid:
            self.require_recenter(gate.reason or sample.validity_reason or "invalid_input")
            self._last_gate_engaged = gate.engaged
            return ClutchUpdate(self.state, None, "invalid_input", self.mapper.anchor_id)
        if not gate.engaged:
            self._last_gate_engaged = False
            if self.state == ClutchState.WAITING_FOR_RELEASE:
                self.state = ClutchState.DISENGAGED
            elif self.state == ClutchState.RECENTER_REQUIRED:
                self.state = ClutchState.DISENGAGED
            elif self.state == ClutchState.ACTIVE:
                self.mapper.clear()
                self.state = ClutchState.DISENGAGED
            return ClutchUpdate(self.state, None, "clutch_released", self.mapper.anchor_id)

        rising_edge = gate.engaged and not self._last_gate_engaged
        self._last_gate_engaged = gate.engaged
        if self.state == ClutchState.WAITING_FOR_RELEASE:
            return ClutchUpdate(self.state, None, "startup_release_required", self.mapper.anchor_id)
        if self.state == ClutchState.RECENTER_REQUIRED:
            return ClutchUpdate(self.state, None, "explicit_release_and_reclutch_required", self.mapper.anchor_id)
        if self.state == ClutchState.DISENGAGED:
            if not rising_edge:
                return ClutchUpdate(self.state, None, "fresh_clutch_edge_required", self.mapper.anchor_id)
            if self.poses_are_operator_frame:
                self.mapper.anchor_operator(sample.pose, robot_tcp_pose)
            else:
                self.mapper.anchor(sample.pose, robot_tcp_pose)
            self.state = ClutchState.ACTIVE
        target = (
            self.mapper.map_operator(sample.pose)
            if self.poses_are_operator_frame
            else self.mapper.map(sample.pose)
        )
        return ClutchUpdate(self.state, target, "following", self.mapper.anchor_id)

    def recenter(self, sample: ArmPoseSample, *, robot_tcp_pose: Pose3D, gate_released: bool) -> int:
        if not gate_released:
            raise RuntimeError("recenter requires the clutch to be released")
        if not sample.tracking_valid:
            raise RuntimeError("recenter requires valid tracking")
        anchor_id = (
            self.mapper.anchor_operator(sample.pose, robot_tcp_pose)
            if self.poses_are_operator_frame
            else self.mapper.anchor(sample.pose, robot_tcp_pose)
        )
        self.mapper.clear()
        self.state = ClutchState.DISENGAGED
        self._last_gate_engaged = False
        return anchor_id

    def require_recenter(self, reason: str) -> None:
        del reason
        self.mapper.clear()
        if self.state not in {ClutchState.STOPPED, ClutchState.FAULT}:
            self.state = ClutchState.RECENTER_REQUIRED

    def release_after_discontinuity(self) -> None:
        if self.state == ClutchState.RECENTER_REQUIRED:
            self.state = ClutchState.DISENGAGED
            self._last_gate_engaged = False

    def stop(self) -> None:
        self.mapper.clear()
        self.state = ClutchState.STOPPED

    def fault(self) -> None:
        self.mapper.clear()
        self.state = ClutchState.FAULT

    def reset_fault(self, *, gate_released: bool, safe: bool) -> None:
        if self.state not in {ClutchState.FAULT, ClutchState.STOPPED}:
            raise RuntimeError("fault reset is only valid from a latched state")
        if not gate_released or not safe:
            raise RuntimeError("fault reset requires released clutch and verified safe state")
        self.state = ClutchState.WAITING_FOR_RELEASE
        self._last_gate_engaged = False
