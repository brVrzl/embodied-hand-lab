from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import pytest

from motion_input import (
    HtsCanonicalAssembler,
    HtsRawRecordingWriter,
    OfflineOperatorTarget,
    OperatorInputState,
    ReceivedHtsDatagram,
    RightHandOperatorConfig,
    RightHandOperatorPipeline,
    TrackingState,
    evaluate_required_right_hand_recording,
    parse_hts_datagram,
)


def _hand_payload(
    sequence: int,
    *,
    side: str = "Right",
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    wrist = ",".join(str(value) for value in (*position, *quaternion))
    points = ",".join(str(index / 1000.0) for index in range(63))
    return (
        f"{side} wrist | f = {sequence} | t = {sequence * 1000}:, {wrist}\n"
        f"{side} landmarks | f = {sequence} | t = {sequence * 1000}:, {points}"
    ).encode()


def _head_payload(sequence: int) -> bytes:
    return (
        f"Head pose | f = {sequence} | t = {sequence * 1000}:, "
        "0,0,0,0,0,0,1"
    ).encode()


def _ingest(assembler: HtsCanonicalAssembler, payload: bytes, receive_ns: int):
    return assembler.ingest(
        parse_hts_datagram(payload),
        receive_monotonic_ns=receive_ns,
        source_endpoint="10.24.0.78:50000",
        datagram_size=len(payload),
    )


def _engaged_pipeline(
    config: RightHandOperatorConfig | None = None,
) -> tuple[HtsCanonicalAssembler, RightHandOperatorPipeline]:
    assembler = HtsCanonicalAssembler(stale_after_s=0.25)
    pipeline = RightHandOperatorPipeline(config)
    first = _ingest(assembler, _hand_payload(1), 1_000_000_000)
    armed = pipeline.step(first, engage_request=True)
    assert armed.state is OperatorInputState.ARMED_REFERENCE_CAPTURE
    second = _ingest(assembler, _hand_payload(2), 1_010_000_000)
    captured = pipeline.step(second, capture_reference_request=True)
    assert captured.state is OperatorInputState.ENGAGED
    assert captured.valid_for_mapping
    return assembler, pipeline


def test_stale_loss_disengages_invalidates_reference_and_recovery_stays_neutral() -> None:
    assembler, pipeline = _engaged_pipeline()

    stale = assembler.state(now_monotonic_ns=1_260_000_001)
    output = pipeline.step(stale)
    assert output.state is OperatorInputState.DISENGAGED
    assert not output.valid_for_mapping
    assert output.emergency_neutral
    assert pipeline.reference_pose is None
    assert pipeline.transitions[-1].reason == "right_hand_stale"

    recovered = _ingest(assembler, _hand_payload(3), 1_300_000_000)
    recovered_output = pipeline.step(recovered)
    assert recovered.right.tracking_valid and len(recovered.right.joints) == 21
    assert recovered_output.state is OperatorInputState.DISENGAGED
    assert not recovered_output.valid_for_mapping
    assert pipeline.reference_pose is None

    armed = pipeline.step(recovered, engage_request=True)
    assert armed.state is OperatorInputState.ARMED_REFERENCE_CAPTURE
    fresh = _ingest(assembler, _hand_payload(4), 1_310_000_000)
    recaptured = pipeline.step(fresh, capture_reference_request=True)
    assert recaptured.state is OperatorInputState.ENGAGED
    assert recaptured.valid_for_mapping


@pytest.mark.parametrize(
    ("kwargs", "reason"),
    [
        ({"frozen_stream": True}, "frozen_right_hand_stream"),
        ({"malformed_data": True}, "malformed_right_hand_data"),
    ],
)
def test_frozen_or_malformed_input_forces_disengaged(
    kwargs: dict[str, bool], reason: str
) -> None:
    assembler, pipeline = _engaged_pipeline()
    current = assembler.state(now_monotonic_ns=1_020_000_000)
    output = pipeline.step(current, **kwargs)
    assert output.state is OperatorInputState.DISENGAGED
    assert output.reason == reason
    assert pipeline.reference_pose is None
    assert pipeline.transitions[-1].reason == reason


def test_explicit_tracking_loss_immediately_forces_disengaged() -> None:
    assembler, pipeline = _engaged_pipeline()
    current = assembler.state(now_monotonic_ns=1_020_000_000)
    lost_right = replace(
        current.right,
        tracking_state=TrackingState.NOT_TRACKING,
        tracking_valid=False,
        wrist_pose=None,
        joints=(),
    )
    lost = replace(current, right=lost_right)
    output = pipeline.step(lost)
    assert output.state is OperatorInputState.DISENGAGED
    assert output.reason == "right_hand_not_tracking"
    assert not output.valid_for_mapping
    assert pipeline.reference_pose is None


def test_optional_left_and_head_loss_do_not_disengage_right_hand() -> None:
    assembler, pipeline = _engaged_pipeline(
        RightHandOperatorConfig(filter_time_constant_s=0)
    )
    right_only = _ingest(
        assembler,
        _hand_payload(3, position=(0.01, 0.0, 0.0)),
        1_020_000_000,
    )
    assert right_only.left.tracking_valid is False
    assert right_only.head is None
    output = pipeline.step(right_only)
    assert output.state is OperatorInputState.ENGAGED
    assert output.valid_for_mapping


def test_relative_translation_orientation_scaling_and_filtering() -> None:
    config = RightHandOperatorConfig(
        translation_scale=(0.5, 1.0, 2.0),
        orientation_scale=0.5,
        filter_time_constant_s=0,
        jump_reject_translation_m=1.0,
        jump_reject_rotation_rad=math.pi,
        workspace_min_m=(-1.0, -1.0, -1.0),
        workspace_max_m=(1.0, 1.0, 1.0),
    )
    assembler, pipeline = _engaged_pipeline(config)
    root = math.sqrt(0.5)
    moved = _ingest(
        assembler,
        _hand_payload(3, position=(0.10, -0.20, -0.15), quaternion=(0, 0, root, root)),
        1_020_000_000,
    )
    output = pipeline.step(moved)
    # Unity Z=-0.15 becomes canonical Z=+0.15 before scaling.
    assert output.translation_m == pytest.approx((0.05, -0.20, 0.30))
    # 90-degree relative Z rotation with orientation scale 0.5 -> 45 degrees.
    expected = math.sin(math.radians(22.5))
    assert output.orientation_xyzw == pytest.approx(
        (0.0, 0.0, expected, math.cos(math.radians(22.5)))
    )


def test_low_pass_filter_is_bounded_between_previous_and_raw_delta() -> None:
    config = RightHandOperatorConfig(
        translation_scale=(1.0, 1.0, 1.0),
        filter_time_constant_s=0.1,
        jump_reject_translation_m=1.0,
        workspace_min_m=(-1.0, -1.0, -1.0),
        workspace_max_m=(1.0, 1.0, 1.0),
    )
    assembler, pipeline = _engaged_pipeline(config)
    moved = _ingest(
        assembler, _hand_payload(3, position=(0.10, 0.0, 0.0)), 1_110_000_000
    )
    output = pipeline.step(moved)
    assert 0.0 < output.translation_m[0] < 0.10
    assert output.translation_m[1:] == pytest.approx((0.0, 0.0))


def test_jump_and_workspace_violation_never_produce_valid_relative_target() -> None:
    assembler, pipeline = _engaged_pipeline()
    jumped = _ingest(
        assembler, _hand_payload(3, position=(0.30, 0.0, 0.0)), 1_020_000_000
    )
    rejected = pipeline.step(jumped)
    assert rejected.reason == "excessive_translation_jump"
    assert not rejected.valid_for_mapping
    assert rejected.translation_m == (0.0, 0.0, 0.0)

    config = RightHandOperatorConfig(
        translation_scale=(1.0, 1.0, 1.0),
        filter_time_constant_s=0,
        jump_reject_translation_m=1.0,
        workspace_min_m=(-0.10, -0.10, -0.10),
        workspace_max_m=(0.10, 0.10, 0.10),
    )
    assembler, pipeline = _engaged_pipeline(config)
    outside = _ingest(
        assembler, _hand_payload(3, position=(0.11, 0.0, 0.0)), 1_020_000_000
    )
    rejected = pipeline.step(outside)
    assert rejected.reason == "operator_workspace_envelope_violation"
    assert rejected.state is OperatorInputState.DISENGAGED
    assert not rejected.valid_for_mapping


def test_transition_log_has_monotonic_timestamps_and_reasons() -> None:
    assembler, pipeline = _engaged_pipeline()
    pipeline.step(assembler.state(now_monotonic_ns=1_260_000_001))
    assert [transition.current for transition in pipeline.transitions] == [
        OperatorInputState.ARMED_REFERENCE_CAPTURE,
        OperatorInputState.ENGAGED,
        OperatorInputState.DISENGAGED,
    ]
    timestamps = [transition.timestamp_monotonic_ns for transition in pipeline.transitions]
    assert timestamps == sorted(timestamps)
    assert all(transition.reason for transition in pipeline.transitions)


def test_recorded_gate_proves_stale_neutral_and_21_joint_recovery(tmp_path: Path) -> None:
    path = tmp_path / "right-interruption.hts.jsonl"
    datagrams = [
        ReceivedHtsDatagram(_hand_payload(1), "10.24.0.78", 50001, 0, 100),
        ReceivedHtsDatagram(_hand_payload(2), "10.24.0.78", 50001, 10_000_000, 101),
        ReceivedHtsDatagram(_head_payload(0), "10.24.0.78", 50002, 200_000_000, 102),
        ReceivedHtsDatagram(_head_payload(1), "10.24.0.78", 50002, 261_000_000, 103),
        ReceivedHtsDatagram(_head_payload(2), "10.24.0.78", 50002, 500_000_000, 104),
        ReceivedHtsDatagram(_hand_payload(3), "10.24.0.78", 50001, 2_530_000_000, 105),
    ]
    with HtsRawRecordingWriter(path) as writer:
        for datagram in datagrams:
            writer.write(datagram)

    report = evaluate_required_right_hand_recording(path)
    assert report["status"] == "PASS"
    assert report["invalid_right_states_retaining_pose"] == 0
    assert report["qualifying_loss_and_recovery_events"] == 1
    assert [row["current"] for row in report["offline_state_transitions"]] == [
        "armed_reference_capture",
        "engaged",
        "disengaged",
    ]
    interruption = report["interruptions"][0]
    assert interruption["publication_gap_s"] == pytest.approx(2.52)
    assert interruption["head_valid_when_right_became_stale"] is True
    assert interruption["pipeline_was_engaged_before_loss"] is True
    assert interruption["pipeline_disengaged_on_loss"] is True
    assert interruption["reference_invalidated_on_loss"] is True
    assert interruption["disengagement_reason"] == "right_hand_stale"
    assert interruption["recovery_joint_count"] == 21
    assert interruption["recovery_output_remained_neutral"] is True
