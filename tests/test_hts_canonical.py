from __future__ import annotations

import math

import pytest

from motion_input import (
    ClutchState,
    HtsCanonicalAssembler,
    HtsTelemetry,
    ReceivedHtsDatagram,
    Side,
    SourceSequenceTracker,
    SerializationError,
    parse_hts_datagram,
    prepare_inactive_future_input,
)


def _hand_payload(
    side: str,
    sequence: int,
    *,
    position: tuple[float, float, float] = (1.0, 2.0, 3.0),
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> bytes:
    wrist = ",".join(str(value) for value in (*position, *quaternion))
    points = ",".join(str(index / 1000.0) for index in range(63))
    return (
        f"{side} wrist | f = {sequence} | t = {sequence * 1000}:, {wrist}\n"
        f"{side} landmarks | f = {sequence} | t = {sequence * 1000}:, {points}"
    ).encode()


def _ingest(
    assembler: HtsCanonicalAssembler,
    payload: bytes,
    receive_ns: int,
):
    return assembler.ingest(
        parse_hts_datagram(payload),
        receive_monotonic_ns=receive_ns,
        source_endpoint="10.24.2.3:50000",
        datagram_size=len(payload),
    )


def test_axis_units_identity_and_left_right_preservation() -> None:
    assembler = HtsCanonicalAssembler(stale_after_s=0.25)
    left = _ingest(assembler, _hand_payload("Left", 1), 1_000_000_000)
    state = _ingest(assembler, _hand_payload("Right", 1), 1_010_000_000)

    assert left.left.tracking_valid
    assert state.left.side is Side.LEFT
    assert state.right.side is Side.RIGHT
    assert state.right.wrist_pose is not None
    # Unity metres (+X right, +Y up, +Z forward) -> explicit canonical
    # OpenXR-style metres (+X right, +Y up, -Z forward).
    assert state.right.wrist_pose.position_m == (1.0, 2.0, -3.0)
    assert state.right.wrist_pose.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert state.right.joints[0].position_m == (0.0, 0.001, -0.002)


def test_90_degree_rotation_and_quaternion_normalization() -> None:
    root = math.sqrt(0.5)
    assembler = HtsCanonicalAssembler()
    state = _ingest(
        assembler,
        _hand_payload("Right", 1, quaternion=(root, 0.0, 0.0, root)),
        1_000_000_000,
    )
    assert state.right.wrist_pose is not None
    qx, qy, qz, qw = state.right.wrist_pose.orientation_xyzw
    assert (qx, qy, qz, qw) == pytest.approx((-root, 0.0, 0.0, root))
    assert math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) == pytest.approx(1.0)


def test_stale_stream_removes_pose_and_recovers() -> None:
    assembler = HtsCanonicalAssembler(stale_after_s=0.1)
    first = _ingest(assembler, _hand_payload("Right", 1), 1_000_000_000)
    assert first.right.tracking_valid and first.right.wrist_pose is not None

    stale = assembler.state(now_monotonic_ns=1_100_000_001)
    assert not stale.right.tracking_valid
    assert stale.right.wrist_pose is None
    assert stale.right.joints == ()

    recovered = _ingest(assembler, _hand_payload("Right", 2), 1_110_000_000)
    assert recovered.right.tracking_valid


def test_truncated_hand_datagram_is_not_combined_with_an_older_frame() -> None:
    assembler = HtsCanonicalAssembler()
    _ingest(assembler, _hand_payload("Right", 1), 1_000_000_000)
    truncated = b"Right wrist | f = 2 | t = 2000:, 4,5,6,0,0,0,1"
    with pytest.raises(SerializationError, match="one wrist and one landmarks"):
        _ingest(assembler, truncated, 1_010_000_000)


def test_sequence_gap_and_out_of_order_detection() -> None:
    tracker = SourceSequenceTracker()
    assert tracker.observe("right", 10) == 0
    assert tracker.observe("right", 13) == 2
    assert tracker.gaps == {"right": 2}
    assert tracker.observe("right", 12) == 0
    assert tracker.out_of_order == {"right": 1}


def test_inactive_future_boundary_can_never_engage() -> None:
    assembler = HtsCanonicalAssembler()
    state = _ingest(assembler, _hand_payload("Right", 1), 1_000_000_000)
    future = prepare_inactive_future_input(state, Side.RIGHT)
    assert future.operator_wrist_pose == state.right.wrist_pose
    assert future.clutch_state is ClutchState.DISENGAGED
    assert future.tracking_valid
    assert future.emergency_neutral is True


def test_tracking_loss_transition_and_frozen_detection() -> None:
    assembler = HtsCanonicalAssembler(stale_after_s=0.05)
    telemetry = HtsTelemetry(frozen_after_s=0.01)
    for sequence, receive_ns in ((1, 1_000_000_000), (2, 1_020_000_000)):
        payload = _hand_payload("Right", sequence)
        datagram = ReceivedHtsDatagram(payload, "10.24.2.3", 50000, receive_ns, receive_ns)
        packets = parse_hts_datagram(payload)
        state = _ingest(assembler, payload, receive_ns)
        telemetry.observe(datagram, packets, state)
    stale = assembler.state(now_monotonic_ns=1_100_000_000)
    telemetry.observe_tracking_state(stale)
    payload = _hand_payload("Right", 3, position=(1.1, 2.0, 3.0))
    datagram = ReceivedHtsDatagram(payload, "10.24.2.3", 50000, 1_110_000_000, 1)
    packets = parse_hts_datagram(payload)
    recovered = _ingest(assembler, payload, 1_110_000_000)
    telemetry.observe(datagram, packets, recovered)
    report = telemetry.report(now_monotonic_ns=1_100_000_000, assembler=assembler)
    assert report["right"]["tracking_losses"] == 1
    assert report["right"]["tracking_recoveries"] == 1
    assert report["right"]["potential_frozen_events"] == 1
