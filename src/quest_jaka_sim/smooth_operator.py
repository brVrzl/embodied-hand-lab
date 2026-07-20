"""Fixed-rate, fail-disengaged right-wrist SE(3) preparation for simulation.

This module consumes the validated canonical Quest contract.  It has no robot
or hardware dependency; its output remains an operator-relative transform.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np

from motion_input import (
    CanonicalQuestState,
    OfflineOperatorTarget,
    OperatorInputState,
    OperatorStateTransition,
    Pose6D,
)
from motion_input.hts_canonical import CANONICAL_OPERATOR_FRAME

from .se3 import (
    OneEuroQuaternionFilter,
    OneEuroVectorFilter,
    quaternion_angle_rad,
    relative_pose,
)


@dataclass(frozen=True, slots=True)
class Se3FilterProfile:
    name: str
    translation_min_cutoff: float
    translation_beta: float
    translation_derivative_cutoff: float
    rotation_min_cutoff: float
    rotation_beta: float
    rotation_derivative_cutoff: float
    maximum_filter_dt: float

    @classmethod
    def from_mapping(cls, name: str, values: Mapping[str, Any]) -> "Se3FilterProfile":
        return cls(
            name=name,
            translation_min_cutoff=float(values["translation_min_cutoff"]),
            translation_beta=float(values["translation_beta"]),
            translation_derivative_cutoff=float(values["translation_derivative_cutoff"]),
            rotation_min_cutoff=float(values["rotation_min_cutoff"]),
            rotation_beta=float(values["rotation_beta"]),
            rotation_derivative_cutoff=float(values["rotation_derivative_cutoff"]),
            maximum_filter_dt=float(values["maximum_filter_dt"]),
        )


class SmoothRightHandOperator:
    """Reference-relative SE(3), One Euro filtering, and explicit engagement."""

    def __init__(
        self,
        *,
        profile: Se3FilterProfile,
        stale_after_s: float = 0.25,
        required_joint_count: int = 21,
        jump_reject_translation_m: float = 0.25,
        jump_reject_rotation_rad: float = math.radians(45.0),
        maximum_relative_translation_m: float = 0.30,
        maximum_relative_rotation_rad: float = math.radians(120.0),
    ) -> None:
        if stale_after_s <= 0 or required_joint_count != 21:
            raise ValueError("right-hand input requires positive stale timeout and 21 joints")
        self.profile = profile
        self.stale_after_s = float(stale_after_s)
        self.required_joint_count = required_joint_count
        self.jump_reject_translation_m = float(jump_reject_translation_m)
        self.jump_reject_rotation_rad = float(jump_reject_rotation_rad)
        self.maximum_relative_translation_m = float(maximum_relative_translation_m)
        self.maximum_relative_rotation_rad = float(maximum_relative_rotation_rad)
        self.state = OperatorInputState.DISENGAGED
        self.reference_pose: Pose6D | None = None
        self.reference_host_sequence: int | None = None
        self.transitions: list[OperatorStateTransition] = []
        self.raw_pose: Pose6D | None = None
        self.filtered_pose: Pose6D | None = None
        self._last_new_pose: Pose6D | None = None
        self._last_sequence: int | None = None
        self._armed_sequence: int | None = None
        self._position_filter = OneEuroVectorFilter(
            min_cutoff_hz=profile.translation_min_cutoff,
            beta=profile.translation_beta,
            derivative_cutoff_hz=profile.translation_derivative_cutoff,
            maximum_dt_s=profile.maximum_filter_dt,
        )
        self._rotation_filter = OneEuroQuaternionFilter(
            min_cutoff_hz=profile.rotation_min_cutoff,
            beta=profile.rotation_beta,
            derivative_cutoff_hz=profile.rotation_derivative_cutoff,
            maximum_dt_s=profile.maximum_filter_dt,
        )

    def step(
        self,
        quest: CanonicalQuestState,
        *,
        now_monotonic_ns: int,
        engage_request: bool = False,
        capture_reference_request: bool = False,
        disengage_request: bool = False,
        malformed_data: bool = False,
    ) -> OfflineOperatorTarget:
        hand = quest.right
        reason = self._unsafe_reason(quest, now_monotonic_ns, malformed_data)
        if reason is not None:
            self._force_disengaged(now_monotonic_ns, reason)
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, reason)
        assert hand.wrist_pose is not None
        self.raw_pose = hand.wrist_pose
        new_sample = hand.host_sequence_number != self._last_sequence
        if new_sample:
            if self._last_new_pose is not None and self.state is not OperatorInputState.DISENGAGED:
                if np.linalg.norm(
                    np.asarray(hand.wrist_pose.position_m)
                    - np.asarray(self._last_new_pose.position_m)
                ) > self.jump_reject_translation_m:
                    self._remember(hand.wrist_pose, hand.host_sequence_number)
                    self._force_disengaged(now_monotonic_ns, "excessive_translation_jump")
                    return self._neutral(now_monotonic_ns, hand.host_sequence_number, "excessive_translation_jump")
                if quaternion_angle_rad(
                    hand.wrist_pose.orientation_xyzw,
                    self._last_new_pose.orientation_xyzw,
                ) > self.jump_reject_rotation_rad:
                    self._remember(hand.wrist_pose, hand.host_sequence_number)
                    self._force_disengaged(now_monotonic_ns, "excessive_orientation_jump")
                    return self._neutral(now_monotonic_ns, hand.host_sequence_number, "excessive_orientation_jump")
            self._remember(hand.wrist_pose, hand.host_sequence_number)

        if disengage_request:
            self._force_disengaged(now_monotonic_ns, "explicit_disengage_request")
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, "explicit_disengage_request")
        if self.state is OperatorInputState.DISENGAGED:
            if engage_request and new_sample:
                self._armed_sequence = hand.host_sequence_number
                self._transition(now_monotonic_ns, OperatorInputState.ARMED_REFERENCE_CAPTURE, "explicit_engage_request_with_fresh_right_hand")
                return self._neutral(now_monotonic_ns, hand.host_sequence_number, "awaiting_reference_capture")
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, "disengaged")
        if self.state is OperatorInputState.ARMED_REFERENCE_CAPTURE:
            if (
                capture_reference_request
                and new_sample
                and hand.host_sequence_number != self._armed_sequence
            ):
                self.reference_pose = hand.wrist_pose
                self.reference_host_sequence = hand.host_sequence_number
                self._reset_filters()
                self.filtered_pose = hand.wrist_pose
                self._position_filter.filter(now_monotonic_ns, hand.wrist_pose.position_m)
                self._rotation_filter.filter(now_monotonic_ns, hand.wrist_pose.orientation_xyzw)
                self._transition(now_monotonic_ns, OperatorInputState.ENGAGED, "fresh_right_hand_reference_captured")
                return self._target(now_monotonic_ns, hand.host_sequence_number, Pose6D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), "reference_captured")
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, "awaiting_reference_capture")

        assert self.reference_pose is not None
        filtered_position = self._position_filter.filter(
            now_monotonic_ns, hand.wrist_pose.position_m
        )
        filtered_orientation = self._rotation_filter.filter(
            now_monotonic_ns, hand.wrist_pose.orientation_xyzw
        )
        self.filtered_pose = Pose6D(
            tuple(float(value) for value in filtered_position), filtered_orientation
        )
        delta = relative_pose(self.reference_pose, self.filtered_pose)
        if np.linalg.norm(delta.position_m) > self.maximum_relative_translation_m:
            self._force_disengaged(now_monotonic_ns, "operator_translation_envelope_violation")
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, "operator_translation_envelope_violation")
        if quaternion_angle_rad(delta.orientation_xyzw, (0.0, 0.0, 0.0, 1.0)) > self.maximum_relative_rotation_rad:
            self._force_disengaged(now_monotonic_ns, "operator_rotation_envelope_violation")
            return self._neutral(now_monotonic_ns, hand.host_sequence_number, "operator_rotation_envelope_violation")
        return self._target(now_monotonic_ns, hand.host_sequence_number, delta, "engaged_filtered_relative_operator_transform")

    def force_fault(self, timestamp_monotonic_ns: int, reason: str) -> None:
        if not reason.strip():
            raise ValueError("fault reason must not be empty")
        self._force_disengaged(timestamp_monotonic_ns, reason)

    def _unsafe_reason(self, quest: CanonicalQuestState, now_ns: int, malformed: bool) -> str | None:
        hand = quest.right
        if malformed:
            return "malformed_right_hand_data"
        if not hand.tracking_valid or hand.wrist_pose is None:
            return "right_hand_not_tracking"
        if hand.host_receive_monotonic_ns is None:
            return "right_hand_stale"
        if (now_ns - hand.host_receive_monotonic_ns) / 1e9 > self.stale_after_s:
            return "right_hand_stale"
        if len(hand.joints) != self.required_joint_count:
            return "right_hand_joint_count_invalid"
        return None

    def _remember(self, pose: Pose6D, sequence: int | None) -> None:
        self._last_new_pose = pose
        self._last_sequence = sequence

    def _reset_filters(self) -> None:
        self._position_filter.reset()
        self._rotation_filter.reset()

    def _transition(self, now_ns: int, state: OperatorInputState, reason: str) -> None:
        if state is self.state:
            return
        previous = self.state
        self.state = state
        self.transitions.append(OperatorStateTransition(now_ns, previous, state, reason))

    def _force_disengaged(self, now_ns: int, reason: str) -> None:
        self.reference_pose = None
        self.reference_host_sequence = None
        self.filtered_pose = None
        self._armed_sequence = None
        self._reset_filters()
        self._transition(now_ns, OperatorInputState.DISENGAGED, reason)

    def _neutral(self, now_ns: int, sequence: int | None, reason: str) -> OfflineOperatorTarget:
        return OfflineOperatorTarget(now_ns, self.state, False, True, CANONICAL_OPERATOR_FRAME, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), self.reference_host_sequence, sequence, reason)

    def _target(self, now_ns: int, sequence: int | None, delta: Pose6D, reason: str) -> OfflineOperatorTarget:
        return OfflineOperatorTarget(now_ns, self.state, True, False, CANONICAL_OPERATOR_FRAME, delta.position_m, delta.orientation_xyzw, self.reference_host_sequence, sequence, reason)
