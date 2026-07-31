from __future__ import annotations

import socket
from pathlib import Path
import tempfile
import time

import pytest

from teleoperation.wire import (
    FrameId,
    LatestTargetPublisher,
    TARGET_STRUCT,
    TargetFlags,
    TargetKind,
    TargetPacket,
    StatusFlags,
    WorkerStatusPacket,
    decode_target,
    encode_target,
    heartbeat_target_packet,
    status_acknowledgement,
)


def packet(sequence: int) -> TargetPacket:
    now = time.monotonic_ns()
    return TargetPacket(TargetKind.CARTESIAN_POSE, TargetFlags.NONE, FrameId.ROBOT_BASE,
                        sequence, now, now, now, now, (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0))


@pytest.fixture
def short_socket_dir() -> Path:
    # AF_UNIX path limits are as small as 104 bytes on macOS; pytest's
    # descriptive per-test directory may exceed that limit.
    with tempfile.TemporaryDirectory(prefix="wire-") as directory:
        yield Path(directory)


def test_wire_round_trip_is_fixed_size_and_crc_protected() -> None:
    encoded = encode_target(packet(9))
    assert len(encoded) == TARGET_STRUCT.size == 124
    assert decode_target(encoded).sequence == 9
    damaged = encoded[:-5] + bytes([encoded[-5] ^ 1]) + encoded[-4:]
    with pytest.raises(ValueError, match="CRC"):
        decode_target(damaged)


def test_startup_tcp_relative_frame_round_trips_explicitly() -> None:
    value = packet(10)
    relative = TargetPacket(
        value.kind,
        value.flags,
        FrameId.STARTUP_TCP_RELATIVE,
        value.sequence,
        value.source_capture_ns,
        value.local_receive_ns,
        value.processing_ns,
        value.dispatch_ns,
        value.payload,
    )
    assert decode_target(encode_target(relative)).frame_id == FrameId.STARTUP_TCP_RELATIVE


def test_hold_rejected_heartbeat_has_no_joint_target_payload() -> None:
    heartbeat = heartbeat_target_packet(
        sequence=11,
        input_sequence=42,
        local_receive_ns=100,
        processing_ns=110,
        dispatch_ns=120,
        last_accepted_target_sequence=7,
        control_state_code=1,
        allow_motion=True,
    )
    decoded = decode_target(encode_target(heartbeat))
    assert decoded.kind is TargetKind.HEARTBEAT
    assert decoded.frame_id is FrameId.NONE
    assert decoded.payload[:3] == (42.0, 7.0, 1.0)
    assert decoded.payload[3:] == (0.0,) * 5


def test_publisher_has_no_application_queue_and_drops_absent_consumer(
    short_socket_dir: Path,
) -> None:
    publisher = LatestTargetPublisher(short_socket_dir / "absent.sock")
    assert not publisher.publish(packet(1))
    assert publisher.dropped == 1
    assert not hasattr(publisher, "queue")
    publisher.close()


def test_latest_datagram_can_be_drained_without_retaining_fifo(
    short_socket_dir: Path,
) -> None:
    path = short_socket_dir / "target.sock"
    receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    receiver.setblocking(False)
    receiver.bind(str(path))
    with LatestTargetPublisher(path) as publisher:
        for sequence in range(5):
            assert publisher.publish(packet(sequence))
    newest = None
    while True:
        try:
            newest = decode_target(receiver.recv(TARGET_STRUCT.size))
        except BlockingIOError:
            break
    receiver.close()
    assert newest is not None and newest.sequence == 4


def test_status_maps_to_typed_command_acknowledgement() -> None:
    status = WorkerStatusPacket(
        state=5,
        flags=int(StatusFlags.HAS_TARGET | StatusFlags.ACCEPTED_SINCE_STATUS),
        last_sequence=12,
        loop_sequence=20,
        worker_monotonic_ns=100,
        command_monotonic_ns=99,
        observation_monotonic_ns=98,
        joint_position_rad=(0.0,) * 6,
        error_code=0,
    )
    acknowledgement = status_acknowledgement(status)
    assert acknowledgement is not None
    assert acknowledgement.sequence == 12
    assert acknowledgement.accepted
