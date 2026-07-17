"""Replay evaluation for the required-right-hand HTS safety gate."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .errors import SerializationError
from .hts_canonical import HtsCanonicalAssembler
from .hts_operator import OperatorInputState, RightHandOperatorConfig, RightHandOperatorPipeline
from .hts_protocol import parse_hts_datagram
from .hts_transport import replay_datagrams


@dataclass(slots=True)
class _Interruption:
    last_valid_receive_ns: int
    stale_observed_ns: int
    head_valid_when_stale: bool
    pipeline_was_engaged_before_loss: bool
    pipeline_disengaged: bool
    reference_invalidated: bool
    disengagement_reason: str | None
    recovery_receive_ns: int | None = None
    recovery_joint_count: int | None = None
    recovery_output_neutral: bool | None = None

    def report(self) -> dict[str, Any]:
        return {
            "publication_gap_s": None
            if self.recovery_receive_ns is None
            else (self.recovery_receive_ns - self.last_valid_receive_ns) / 1e9,
            "stale_observation_delay_s": (
                self.stale_observed_ns - self.last_valid_receive_ns
            )
            / 1e9,
            "head_valid_when_right_became_stale": self.head_valid_when_stale,
            "pipeline_was_engaged_before_loss": self.pipeline_was_engaged_before_loss,
            "pipeline_disengaged_on_loss": self.pipeline_disengaged,
            "reference_invalidated_on_loss": self.reference_invalidated,
            "disengagement_reason": self.disengagement_reason,
            "recovery_joint_count": self.recovery_joint_count,
            "recovery_output_remained_neutral": self.recovery_output_neutral,
        }


def evaluate_required_right_hand_recording(
    path: str | Path,
    *,
    stale_after_s: float = 0.25,
) -> dict[str, Any]:
    """Replay a raw HTS capture and evaluate right-hand loss/recovery semantics.

    The evaluator automatically exercises the *offline* engage/reference states
    before the first interruption.  It never creates a hardware target.
    """

    config = RightHandOperatorConfig(stale_after_s=stale_after_s)
    assembler = HtsCanonicalAssembler(stale_after_s=stale_after_s)
    pipeline = RightHandOperatorPipeline(config)
    previous_valid = False
    last_valid_receive_ns: int | None = None
    last_right_sequence: int | None = None
    arm_sequence: int | None = None
    active_interruption: _Interruption | None = None
    interruptions: list[_Interruption] = []
    tracked_frames_with_21_joints = 0
    invalid_states_retaining_pose = 0
    malformed_datagrams = 0
    auto_engagement_completed = False

    for datagram in replay_datagrams(path):
        stale_boundary_ns = (
            None
            if last_valid_receive_ns is None
            else last_valid_receive_ns + int(stale_after_s * 1_000_000_000) + 1
        )
        if (
            previous_valid
            and stale_boundary_ns is not None
            and datagram.receive_monotonic_ns >= stale_boundary_ns
        ):
            boundary_state = assembler.state(now_monotonic_ns=stale_boundary_ns)
            was_engaged = pipeline.state is OperatorInputState.ENGAGED
            transition_count = len(pipeline.transitions)
            pipeline.step(boundary_state)
            transition = (
                pipeline.transitions[-1]
                if len(pipeline.transitions) > transition_count
                else None
            )
            active_interruption = _Interruption(
                last_valid_receive_ns=last_valid_receive_ns,
                stale_observed_ns=stale_boundary_ns,
                head_valid_when_stale=boundary_state.head is not None,
                pipeline_was_engaged_before_loss=was_engaged,
                pipeline_disengaged=pipeline.state is OperatorInputState.DISENGAGED,
                reference_invalidated=pipeline.reference_pose is None,
                disengagement_reason=None if transition is None else transition.reason,
            )
            interruptions.append(active_interruption)
            previous_valid = False

        try:
            packets = parse_hts_datagram(datagram.payload)
            state = assembler.ingest(
                packets,
                receive_monotonic_ns=datagram.receive_monotonic_ns,
                source_endpoint=datagram.source_endpoint,
                datagram_size=len(datagram.payload),
            )
        except SerializationError:
            malformed_datagrams += 1
            pipeline.force_fault(
                timestamp_monotonic_ns=datagram.receive_monotonic_ns,
                reason="malformed_recorded_datagram",
            )
            continue

        right = state.right
        if right.tracking_valid and len(right.joints) == config.required_joint_count:
            if right.host_sequence_number != last_right_sequence:
                tracked_frames_with_21_joints += 1
                last_right_sequence = right.host_sequence_number

        if not right.tracking_valid and (right.wrist_pose is not None or right.joints):
            invalid_states_retaining_pose += 1

        engage = False
        capture = False
        if right.tracking_valid and pipeline.state is OperatorInputState.DISENGAGED:
            if not auto_engagement_completed and active_interruption is None:
                engage = True
                arm_sequence = right.host_sequence_number
        elif (
            right.tracking_valid
            and pipeline.state is OperatorInputState.ARMED_REFERENCE_CAPTURE
            and right.host_sequence_number != arm_sequence
        ):
            capture = True
            auto_engagement_completed = True

        was_engaged_before_step = pipeline.state is OperatorInputState.ENGAGED
        transition_count_before_step = len(pipeline.transitions)
        output = pipeline.step(
            state,
            engage_request=engage,
            capture_reference_request=capture,
        )

        if previous_valid and not right.tracking_valid and last_valid_receive_ns is not None:
            transition = (
                pipeline.transitions[-1]
                if len(pipeline.transitions) > transition_count_before_step
                else None
            )
            active_interruption = _Interruption(
                last_valid_receive_ns=last_valid_receive_ns,
                stale_observed_ns=state.host_monotonic_ns,
                head_valid_when_stale=state.head is not None,
                pipeline_was_engaged_before_loss=was_engaged_before_step,
                pipeline_disengaged=pipeline.state is OperatorInputState.DISENGAGED,
                reference_invalidated=pipeline.reference_pose is None,
                disengagement_reason=None if transition is None else transition.reason,
            )
            interruptions.append(active_interruption)
        elif not previous_valid and right.tracking_valid and active_interruption is not None:
            active_interruption.recovery_receive_ns = right.host_receive_monotonic_ns
            active_interruption.recovery_joint_count = len(right.joints)
            active_interruption.recovery_output_neutral = (
                pipeline.state is OperatorInputState.DISENGAGED
                and not output.valid_for_mapping
                and output.emergency_neutral
            )
            active_interruption = None

        if right.tracking_valid:
            last_valid_receive_ns = right.host_receive_monotonic_ns
        previous_valid = right.tracking_valid

    reports = [interruption.report() for interruption in interruptions]
    qualifying = [
        report
        for report in reports
        if report["publication_gap_s"] is not None
        and report["stale_observation_delay_s"] >= stale_after_s
        and report["head_valid_when_right_became_stale"]
        and report["pipeline_was_engaged_before_loss"]
        and report["pipeline_disengaged_on_loss"]
        and report["reference_invalidated_on_loss"]
        and report["disengagement_reason"] == "right_hand_stale"
        and report["recovery_joint_count"] == config.required_joint_count
        and report["recovery_output_remained_neutral"]
    ]
    passed = (
        tracked_frames_with_21_joints > 0
        and invalid_states_retaining_pose == 0
        and malformed_datagrams == 0
        and bool(qualifying)
    )
    return {
        "status": "PASS" if passed else "FAIL",
        "required_hand": "right",
        "left_hand_required": False,
        "head_pose_required": False,
        "stale_threshold_s": stale_after_s,
        "tracked_right_frames_with_21_joints": tracked_frames_with_21_joints,
        "invalid_right_states_retaining_pose": invalid_states_retaining_pose,
        "malformed_datagrams": malformed_datagrams,
        "interruptions": reports,
        "qualifying_loss_and_recovery_events": len(qualifying),
        "recovery_requires_explicit_reengagement": True,
        "recovery_requires_new_reference_capture": True,
        "offline_state_transitions": [
            {
                "timestamp_monotonic_ns": transition.timestamp_monotonic_ns,
                "previous": transition.previous.value,
                "current": transition.current.value,
                "reason": transition.reason,
            }
            for transition in pipeline.transitions
        ],
        "hardware_command_path": "absent",
    }
