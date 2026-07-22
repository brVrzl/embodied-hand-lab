from __future__ import annotations

import math

import pytest

from motion_input import (
    HtsHeadPosePacket,
    HtsLandmarksPacket,
    HtsWristPacket,
    SerializationError,
    Side,
    normalize_quaternion,
    parse_hts_datagram,
    parse_hts_line,
)


def _landmarks(side: str = "Right", *, debug: str = "") -> str:
    values = ", ".join(str(index / 1000.0) for index in range(63))
    return f"{side} landmarks{debug}:, {values}"


def test_parse_legacy_wrist_and_debug_landmarks_datagram() -> None:
    payload = (
        "Right wrist:, 0.25, 1.0, 0.5, 0, 0, 0, 1\n"
        + _landmarks(debug=" | f = 42 | t = 123456")
    ).encode()
    wrist, landmarks = parse_hts_datagram(payload)

    assert isinstance(wrist, HtsWristPacket)
    assert wrist.header.side is Side.RIGHT
    assert wrist.position_m == (0.25, 1.0, 0.5)
    assert wrist.orientation_xyzw == (0.0, 0.0, 0.0, 1.0)
    assert isinstance(landmarks, HtsLandmarksPacket)
    assert landmarks.header.source_sequence == 42
    assert landmarks.header.source_timestamp_ns == 123456
    assert len(landmarks.positions_wrist_m) == 21
    assert landmarks.positions_wrist_m[0] == (0.0, 0.001, 0.002)


def test_parse_head_pose_and_preserve_unknown_integer_header_metadata() -> None:
    packet = parse_hts_line(
        "Head pose | f = 3 | t = 99 | vendor = 7:, 1, 2, 3, 0, 0, 0, 1"
    )
    assert isinstance(packet, HtsHeadPosePacket)
    assert packet.header.side is Side.NONE
    assert packet.header.extra_fields == {"vendor": 7}


@pytest.mark.parametrize(
    "line",
    [
        "",
        "Right wrist, 1,2,3,0,0,0,1",
        "Middle wrist:, 1,2,3,0,0,0,1",
        "Right wrist:, 1,2,3,0,0,0",
        "Right wrist:, 1,2,3,0,0,0,1,99",
        "Right wrist:, 1,2,,3,0,0,0,1",
        "Right wrist:, 1,2,3,0,0,0,nan",
        "Right wrist:, 1,2,3,0,0,0,inf",
        "Right wrist:, 1,2,3,0,0,0,0.5",
        "Right wrist | f = nope:, 1,2,3,0,0,0,1",
        "Left landmarks:, 1,2,3",
        _landmarks() + ", 99",
    ],
)
def test_malformed_truncated_extra_and_invalid_float_packets_are_rejected(line: str) -> None:
    with pytest.raises(SerializationError):
        parse_hts_line(line)


def test_datagram_length_and_utf8_validation() -> None:
    with pytest.raises(SerializationError, match="exceeds"):
        parse_hts_datagram(b"1234", max_bytes=3)
    with pytest.raises(SerializationError, match="UTF-8"):
        parse_hts_datagram(b"\xff\xfe")


def test_rounded_quaternion_is_checked_then_normalized() -> None:
    packet = parse_hts_line("Left wrist:, 0,0,0, 0.707,0,0,0.707")
    assert isinstance(packet, HtsWristPacket)
    assert packet.quaternion_norm == pytest.approx(math.sqrt(2 * 0.707**2))
    normalized = normalize_quaternion(packet.orientation_xyzw)
    assert math.sqrt(sum(value * value for value in normalized)) == pytest.approx(1.0)


def test_left_and_right_identity_are_preserved() -> None:
    left = parse_hts_line("Left wrist:, 0,0,0,0,0,0,1")
    right = parse_hts_line("Right wrist:, 0,0,0,0,0,0,1")
    assert isinstance(left, HtsWristPacket) and left.header.side is Side.LEFT
    assert isinstance(right, HtsWristPacket) and right.header.side is Side.RIGHT
