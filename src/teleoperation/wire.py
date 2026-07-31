from __future__ import annotations

import enum
import math
import os
import socket
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

from .contracts import CommandAcknowledgement, ControllerState, PoseTarget


TARGET_MAGIC = 0x4A544754  # JTGT
STATUS_MAGIC = 0x4A535441  # JSTA
WIRE_VERSION = 1
_TARGET_WITHOUT_CRC = struct.Struct("<IHHIIQQQQQ8d")
TARGET_STRUCT = struct.Struct("<IHHIIQQQQQ8dI")
STATUS_STRUCT = struct.Struct("<IHHIQQQQQ6diI")


class TargetKind(enum.IntEnum):
    HEARTBEAT = 0
    HOLD_CURRENT = 1
    JOINT_POSITION = 2
    CARTESIAN_POSE = 3
    STOP = 4


class TargetFlags(enum.IntFlag):
    NONE = 0
    ALLOW_MOTION = 1 << 0


class StatusFlags(enum.IntFlag):
    CONNECTED = 1 << 0
    EDG_ACTIVE = 1 << 1
    HOLDING = 1 << 2
    HAS_TARGET = 1 << 3
    ACCEPTED_SINCE_STATUS = 1 << 4
    REJECTED_SINCE_STATUS = 1 << 5
    TARGET_AGE_WARNING = 1 << 6
    OUTPUT_ACCELERATION_HOLD = 1 << 7
    OUTPUT_ACCELERATION_RECOVERED = 1 << 8
    CONTROLLED_BRAKING = 1 << 9
    STOPPED_READY = 1 << 10
    MEASURED_STATE_REFRESH = 1 << 11


class FrameId(enum.IntEnum):
    NONE = 0
    ROBOT_BASE = 1
    STARTUP_TCP_RELATIVE = 2


@dataclass(frozen=True, slots=True)
class TargetPacket:
    kind: TargetKind
    flags: TargetFlags
    frame_id: FrameId
    sequence: int
    source_capture_ns: int
    local_receive_ns: int
    processing_ns: int
    dispatch_ns: int
    payload: tuple[float, float, float, float, float, float, float, float]


@dataclass(frozen=True, slots=True)
class WorkerStatusPacket:
    state: int
    flags: int
    last_sequence: int
    loop_sequence: int
    worker_monotonic_ns: int
    command_monotonic_ns: int
    observation_monotonic_ns: int
    joint_position_rad: tuple[float, float, float, float, float, float]
    error_code: int


def encode_target(packet: TargetPacket) -> bytes:
    body = _TARGET_WITHOUT_CRC.pack(
        TARGET_MAGIC, WIRE_VERSION, int(packet.kind), int(packet.flags), int(packet.frame_id),
        packet.sequence, packet.source_capture_ns, packet.local_receive_ns,
        packet.processing_ns, packet.dispatch_ns, *packet.payload,
    )
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def decode_target(data: bytes) -> TargetPacket:
    if len(data) != TARGET_STRUCT.size:
        raise ValueError(f"target packet must be {TARGET_STRUCT.size} bytes")
    values = TARGET_STRUCT.unpack(data)
    if values[0] != TARGET_MAGIC or values[1] != WIRE_VERSION:
        raise ValueError("invalid target magic or version")
    if zlib.crc32(data[:-4]) & 0xFFFFFFFF != values[-1]:
        raise ValueError("target CRC mismatch")
    return TargetPacket(TargetKind(values[2]), TargetFlags(values[3]), FrameId(values[4]),
                        values[5], values[6], values[7], values[8], values[9], tuple(values[10:18]))


def pose_target_packet(target: PoseTarget, *, allow_motion: bool = False) -> TargetPacket:
    ts = target.timestamps
    frame = {
        "robot_base": FrameId.ROBOT_BASE,
        "startup_tcp_relative": FrameId.STARTUP_TCP_RELATIVE,
    }.get(target.target_frame_id)
    if frame is None:
        raise ValueError(f"unsupported target frame: {target.target_frame_id!r}")
    return TargetPacket(
        TargetKind.CARTESIAN_POSE,
        TargetFlags.ALLOW_MOTION if allow_motion else TargetFlags.NONE,
        frame,
        target.sequence,
        ts.source_capture_ns or 0,
        ts.local_receive_ns,
        ts.processing_ns or 0,
        ts.dispatch_ns or 0,
        (*target.pose.position_m, *target.pose.quaternion_xyzw, 0.0),
    )


def joint_position_target_packet(
    *,
    sequence: int,
    joint_position_rad: tuple[float, float, float, float, float, float],
    local_receive_ns: int,
    processing_ns: int,
    dispatch_ns: int,
    source_capture_ns: int = 0,
    allow_motion: bool = False,
) -> TargetPacket:
    """Encode an already accepted J1..J6 radian target without transforming it."""

    if sequence < 0:
        raise ValueError("joint target sequence must be non-negative")
    if not 0 < local_receive_ns <= processing_ns <= dispatch_ns:
        raise ValueError("joint target host timestamps must be positive and monotonic")
    if source_capture_ns < 0:
        raise ValueError("source timestamp must be non-negative")
    if len(joint_position_rad) != 6:
        raise ValueError("joint target must contain J1 through J6")
    if not all(math.isfinite(value) for value in joint_position_rad):
        raise ValueError("joint target must contain finite radians")
    return TargetPacket(
        TargetKind.JOINT_POSITION,
        TargetFlags.ALLOW_MOTION if allow_motion else TargetFlags.NONE,
        FrameId.NONE,
        sequence,
        source_capture_ns,
        local_receive_ns,
        processing_ns,
        dispatch_ns,
        (*joint_position_rad, 0.0, 0.0),
    )


def heartbeat_target_packet(
    *,
    sequence: int,
    input_sequence: int,
    local_receive_ns: int,
    processing_ns: int,
    dispatch_ns: int,
    last_accepted_target_sequence: int,
    control_state_code: int,
    allow_motion: bool = False,
) -> TargetPacket:
    """Encode producer liveness without changing the current joint target."""

    if sequence < 0 or input_sequence < 0 or last_accepted_target_sequence < 0:
        raise ValueError("heartbeat sequences must be non-negative")
    if not 0 < local_receive_ns <= processing_ns <= dispatch_ns:
        raise ValueError("heartbeat host timestamps must be positive and monotonic")
    return TargetPacket(
        TargetKind.HEARTBEAT,
        TargetFlags.ALLOW_MOTION if allow_motion else TargetFlags.NONE,
        FrameId.NONE,
        sequence,
        0,
        local_receive_ns,
        processing_ns,
        dispatch_ns,
        (
            float(input_sequence),
            float(last_accepted_target_sequence),
            float(control_state_code),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
    )


def stop_target_packet(*, sequence: int, monotonic_ns: int) -> TargetPacket:
    if sequence < 0 or monotonic_ns < 0:
        raise ValueError("stop sequence and timestamp must be non-negative")
    return TargetPacket(
        TargetKind.STOP,
        TargetFlags.NONE,
        FrameId.NONE,
        sequence,
        0,
        monotonic_ns,
        monotonic_ns,
        monotonic_ns,
        (0.0,) * 8,
    )


def hold_current_target_packet(*, sequence: int, monotonic_ns: int) -> TargetPacket:
    """Request a recoverable controlled stop without terminating the session."""

    if sequence < 0 or monotonic_ns < 0:
        raise ValueError("hold sequence and timestamp must be non-negative")
    return TargetPacket(
        TargetKind.HOLD_CURRENT,
        TargetFlags.NONE,
        FrameId.NONE,
        sequence,
        0,
        monotonic_ns,
        monotonic_ns,
        monotonic_ns,
        (0.0,) * 8,
    )


def decode_status(data: bytes) -> WorkerStatusPacket:
    if len(data) != STATUS_STRUCT.size:
        raise ValueError(f"status packet must be {STATUS_STRUCT.size} bytes")
    values = STATUS_STRUCT.unpack(data)
    if values[0] != STATUS_MAGIC or values[1] != WIRE_VERSION:
        raise ValueError("invalid status magic or version")
    if zlib.crc32(data[:-4]) & 0xFFFFFFFF != values[-1]:
        raise ValueError("status CRC mismatch")
    return WorkerStatusPacket(values[2], values[3], values[4], values[5], values[6], values[7],
                              values[8], tuple(values[9:15]), values[15])


def status_acknowledgement(status: WorkerStatusPacket) -> CommandAcknowledgement | None:
    flags = StatusFlags(status.flags)
    if not flags & StatusFlags.HAS_TARGET:
        return None
    states = tuple(ControllerState)
    if status.state >= len(states):
        raise ValueError("worker reported an unknown controller state")
    accepted = bool(flags & StatusFlags.ACCEPTED_SINCE_STATUS)
    return CommandAcknowledgement(
        status.last_sequence,
        accepted,
        states[status.state],
        status.worker_monotonic_ns,
        status.command_monotonic_ns or None,
        "accepted" if accepted else "latest target retained; no new acceptance in this status interval",
    )


class LatestTargetPublisher:
    """Non-blocking, bounded Unix-datagram publisher.

    The kernel send buffer is finite.  A full or absent consumer drops the new
    datagram and increments ``dropped``; it can never grow a Python queue.
    """

    def __init__(self, path: str | Path, *, send_buffer_bytes: int = TARGET_STRUCT.size * 8) -> None:
        self._path = str(path)
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, send_buffer_bytes)
        self.sent = 0
        self.dropped = 0

    def publish(self, packet: TargetPacket) -> bool:
        try:
            self._socket.sendto(encode_target(packet), self._path)
        except (BlockingIOError, FileNotFoundError, ConnectionRefusedError):
            self.dropped += 1
            return False
        self.sent += 1
        return True

    def close(self) -> None:
        self._socket.close()

    def __enter__(self) -> "LatestTargetPublisher":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


class WorkerStatusReceiver:
    """Bounded non-blocking status receiver; callers always drain to latest."""

    def __init__(self, path: str | Path, *, receive_buffer_bytes: int = STATUS_STRUCT.size * 8) -> None:
        self.path = str(path)
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass
        self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._socket.setblocking(False)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer_bytes)
        self._socket.bind(self.path)

    def latest(self) -> WorkerStatusPacket | None:
        result = None
        while True:
            try:
                data = self._socket.recv(STATUS_STRUCT.size + 1)
            except BlockingIOError:
                return result
            try:
                result = decode_status(data)
            except ValueError:
                continue

    def close(self) -> None:
        self._socket.close()
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass

    def __enter__(self) -> "WorkerStatusReceiver":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
