"""Meta Quest ingress isolated behind the UMIP provider boundary.

The headset bridge emits versioned Quest frames. This module validates those
frames, performs the single documented Unity/OpenXR basis conversion, and emits
UMIP. It contains no robot, teleoperation, filtering, or trajectory behavior.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
import json
import socket
import time
from typing import Any, Callable, Mapping
import uuid

from .errors import SerializationError, SourceDisconnected
from .frames import unity_to_openxr_pose
from .model import (
    DeviceDescriptor,
    GestureSample,
    HandArticulation,
    JointSample,
    MotionInputSample,
    Pose6D,
    Side,
    Timestamp,
    TrackingState,
)
from .provider import MotionInputProvider


QUEST_WIRE_SCHEMA = "quest-hand-frame"
QUEST_WIRE_VERSION = "1.0"
MAX_QUEST_DATAGRAM_BYTES = 1_048_576


@dataclass(frozen=True, slots=True)
class QuestFrame:
    session_id: str
    stream_id: str
    sequence_number: int
    side: Side
    reference_space: str
    basis: str
    capture_timestamp: Timestamp
    device_timestamp: Timestamp | None
    tracking_state: TrackingState
    tracking_confidence: float | None
    wrist_pose: Pose6D | None
    palm_pose: Pose6D | None
    articulation: HandArticulation | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)


class QuestFrameSource(ABC):
    @abstractmethod
    def open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def read(self, timeout_s: float | None) -> QuestFrame | None:
        raise NotImplementedError


class InMemoryQuestSource(QuestFrameSource):
    """Deterministic hardware-free source for tests and integration development."""

    def __init__(self, frames: list[QuestFrame | BaseException | None]) -> None:
        self._initial = list(frames)
        self._frames: deque[QuestFrame | BaseException | None] = deque()
        self.is_open = False

    def open(self) -> None:
        self._frames = deque(self._initial)
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read(self, timeout_s: float | None) -> QuestFrame | None:
        if not self.is_open:
            raise SourceDisconnected("Quest source is closed")
        if not self._frames:
            return None
        value = self._frames.popleft()
        if value is None:
            return None
        if isinstance(value, BaseException):
            raise value
        return value


class UdpQuestSource(QuestFrameSource):
    """Receives one UTF-8 JSON Quest frame per UDP datagram."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 7060,
        *,
        allowed_sender: str | None = None,
        max_datagram_bytes: int = MAX_QUEST_DATAGRAM_BYTES,
    ) -> None:
        if port < 1 or port > 65535:
            raise ValueError("UDP port must be in [1, 65535]")
        if max_datagram_bytes < 1:
            raise ValueError("max_datagram_bytes must be positive")
        self.host = host
        self.port = port
        self.allowed_sender = allowed_sender
        self.max_datagram_bytes = max_datagram_bytes
        self._socket: socket.socket | None = None

    def open(self) -> None:
        if self._socket is not None:
            raise RuntimeError("Quest UDP source is already open")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        self._socket = sock

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None

    def read(self, timeout_s: float | None) -> QuestFrame | None:
        if self._socket is None:
            raise SourceDisconnected("Quest UDP source is closed")
        self._socket.settimeout(timeout_s)
        try:
            payload, sender = self._socket.recvfrom(self.max_datagram_bytes + 1)
        except socket.timeout:
            return None
        except OSError as exc:
            raise SourceDisconnected(f"Quest UDP receive failed: {exc}") from exc
        if self.allowed_sender is not None and sender[0] != self.allowed_sender:
            raise SerializationError(f"datagram from unapproved sender {sender[0]!r}")
        if len(payload) > self.max_datagram_bytes:
            raise SerializationError("Quest datagram exceeds configured size limit")
        frame = parse_quest_frame(payload)
        metadata = dict(frame.metadata)
        metadata["source_endpoint"] = f"{sender[0]}:{sender[1]}"
        return QuestFrame(
            session_id=frame.session_id,
            stream_id=frame.stream_id,
            sequence_number=frame.sequence_number,
            side=frame.side,
            reference_space=frame.reference_space,
            basis=frame.basis,
            capture_timestamp=frame.capture_timestamp,
            device_timestamp=frame.device_timestamp,
            tracking_state=frame.tracking_state,
            tracking_confidence=frame.tracking_confidence,
            wrist_pose=frame.wrist_pose,
            palm_pose=frame.palm_pose,
            articulation=frame.articulation,
            metadata=metadata,
        )


class QuestMotionProvider(MotionInputProvider):
    """Quest SDK frame -> UMIP sample, and nothing else."""

    def __init__(
        self,
        source: QuestFrameSource,
        *,
        device: DeviceDescriptor,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        disconnect_timeout_s: float = 1.0,
    ) -> None:
        super().__init__()
        if device.device_type != "xr_headset":
            raise ValueError("Quest provider device_type must be 'xr_headset'")
        if disconnect_timeout_s <= 0:
            raise ValueError("disconnect_timeout_s must be positive")
        self.source = source
        self._descriptor = device
        self._monotonic_ns = monotonic_ns
        self._disconnect_timeout_ns = int(disconnect_timeout_s * 1_000_000_000)
        self._pending: deque[MotionInputSample] = deque()
        self._last_sequence = {Side.LEFT: -1, Side.RIGHT: -1}
        self._last_stream_id = {
            Side.LEFT: f"{device.device_id}/left",
            Side.RIGHT: f"{device.device_id}/right",
        }
        self._disconnected = False
        self._last_activity_ns = 0

    @property
    def descriptor(self) -> DeviceDescriptor:
        return self._descriptor

    def _open(self) -> None:
        self.source.open()
        self._pending.clear()
        self._last_sequence = {Side.LEFT: -1, Side.RIGHT: -1}
        self._disconnected = False
        self._last_activity_ns = self._monotonic_ns()

    def _close(self) -> None:
        self.source.close()
        self._pending.clear()

    def _read(self, timeout_s: float | None) -> MotionInputSample | None:
        if self._pending:
            return self._pending.popleft()
        now_ns = self._monotonic_ns()
        remaining_s = max(0, self._disconnect_timeout_ns - (now_ns - self._last_activity_ns)) / 1e9
        source_timeout_s = remaining_s if timeout_s is None else min(timeout_s, remaining_s)
        try:
            frame = self.source.read(source_timeout_s)
        except SourceDisconnected as exc:
            if self._disconnected:
                return None
            self._disconnected = True
            self._queue_disconnect(str(exc), self._monotonic_ns())
            return self._pending.popleft()
        if frame is None:
            now_ns = self._monotonic_ns()
            if not self._disconnected and now_ns - self._last_activity_ns >= self._disconnect_timeout_ns:
                self._disconnected = True
                self._queue_disconnect("Quest stream timeout", now_ns)
                return self._pending.popleft()
            return None
        self._disconnected = False
        receive_ns = self._monotonic_ns()
        self._last_activity_ns = receive_ns
        self._last_sequence[frame.side] = max(
            self._last_sequence.get(frame.side, -1), frame.sequence_number
        )
        return self._translate(frame, receive_ns)

    def _translate(self, frame: QuestFrame, receive_ns: int) -> MotionInputSample:
        if frame.basis == "unity":
            convert = lambda pose: None if pose is None else unity_to_openxr_pose(pose)
        elif frame.basis == "openxr":
            convert = lambda pose: pose
        else:
            raise SerializationError(f"unsupported Quest pose basis {frame.basis!r}")
        processing_ns = self._monotonic_ns()
        frame_id = (
            f"quest/{self._descriptor.device_id}/{frame.session_id}/"
            f"{frame.reference_space}:openxr"
        )
        self._last_stream_id[frame.side] = frame.stream_id
        return MotionInputSample(
            sample_id=str(
                uuid.uuid5(
                    uuid.NAMESPACE_URL,
                    f"umip:{self._descriptor.device_id}:{frame.session_id}:"
                    f"{frame.side.value}:{frame.sequence_number}",
                )
            ),
            stream_id=frame.stream_id,
            sequence_number=frame.sequence_number,
            capture_timestamp=frame.capture_timestamp,
            receive_timestamp=Timestamp(receive_ns, "host:monotonic"),
            device_timestamp=frame.device_timestamp,
            processing_timestamp=Timestamp(processing_ns, "host:monotonic"),
            tracking_state=frame.tracking_state,
            tracking_confidence=frame.tracking_confidence,
            coordinate_frame=frame_id,
            device=self._descriptor,
            side=frame.side,
            wrist_pose=convert(frame.wrist_pose),
            palm_pose=convert(frame.palm_pose),
            articulation=_convert_articulation(frame.articulation, convert),
            metadata={
                **dict(frame.metadata),
                "provider": "quest",
                "source_basis": frame.basis,
                "reference_space": frame.reference_space,
            },
            extensions={},
        )

    def _queue_disconnect(self, reason: str, timestamp_ns: int) -> None:
        for side in (Side.LEFT, Side.RIGHT):
            sequence = self._last_sequence[side] + 1
            self._last_sequence[side] = sequence
            timestamp = Timestamp(timestamp_ns, "host:monotonic")
            self._pending.append(
                MotionInputSample(
                    sample_id=str(uuid.uuid4()),
                    stream_id=self._last_stream_id[side],
                    sequence_number=sequence,
                    capture_timestamp=timestamp,
                    receive_timestamp=timestamp,
                    device_timestamp=None,
                    processing_timestamp=timestamp,
                    tracking_state=TrackingState.DISCONNECTED,
                    tracking_confidence=None,
                    coordinate_frame=f"quest/{self._descriptor.device_id}/disconnected:openxr",
                    device=self._descriptor,
                    side=side,
                    wrist_pose=None,
                    metadata={
                        "provider": "quest",
                        "disconnect_reason": reason,
                        "event_timestamp_basis": "host_disconnect_detection",
                    },
                )
            )


def parse_quest_frame(payload: str | bytes) -> QuestFrame:
    try:
        value = json.loads(payload)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise SerializationError(f"invalid Quest frame JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise SerializationError("Quest frame must be a JSON object")
    try:
        if value["schema"] != QUEST_WIRE_SCHEMA:
            raise SerializationError(f"unexpected Quest schema {value['schema']!r}")
        version = str(value["version"])
        if version.split(".", 1)[0] != QUEST_WIRE_VERSION.split(".", 1)[0]:
            raise SerializationError(f"unsupported Quest wire major version {version!r}")
        state = TrackingState(str(value["tracking_state"]))
        wrist = _wire_pose(value.get("wrist_pose"))
        palm = _wire_pose(value.get("palm_pose"))
        articulation = _wire_articulation(value.get("articulation"))
        side = Side(str(value["side"]))
        if side not in (Side.LEFT, Side.RIGHT):
            raise SerializationError("Quest frame side must be left or right")
        metadata = value.get("metadata", {})
        if not isinstance(metadata, dict):
            raise SerializationError("Quest frame metadata must be an object")
        return QuestFrame(
            session_id=str(value["session_id"]),
            stream_id=str(value["stream_id"]),
            sequence_number=int(value["sequence_number"]),
            side=side,
            reference_space=str(value["reference_space"]),
            basis=str(value["basis"]),
            capture_timestamp=_wire_timestamp(value["capture_timestamp"]),
            device_timestamp=(
                None if value.get("device_timestamp") is None else _wire_timestamp(value["device_timestamp"])
            ),
            tracking_state=state,
            tracking_confidence=value.get("tracking_confidence"),
            wrist_pose=wrist,
            palm_pose=palm,
            articulation=articulation,
            metadata=dict(metadata),
        )
    except SerializationError:
        raise
    except (KeyError, TypeError, ValueError) as exc:
        raise SerializationError(f"invalid Quest frame: {exc}") from exc


def _wire_timestamp(value: Any) -> Timestamp:
    if not isinstance(value, dict):
        raise SerializationError("Quest timestamp must be an object")
    return Timestamp(
        nanoseconds=int(value["nanoseconds"]),
        clock_id=str(value["clock_id"]),
        uncertainty_ns=None if value.get("uncertainty_ns") is None else int(value["uncertainty_ns"]),
    )


def _wire_pose(value: Any) -> Pose6D | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise SerializationError("Quest pose must be an object")
    return Pose6D(
        position_m=tuple(float(item) for item in value["position_m"]),
        orientation_xyzw=tuple(float(item) for item in value["orientation_xyzw"]),
    )


def _wire_articulation(value: Any) -> HandArticulation | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise SerializationError("Quest articulation must be an object")
    return HandArticulation(
        joints=tuple(
            JointSample(
                name=str(joint["name"]),
                pose=_wire_pose(joint.get("pose")),
                tracking_state=TrackingState(str(joint["tracking_state"])),
                confidence=joint.get("confidence"),
                radius_m=joint.get("radius_m"),
            )
            for joint in value.get("joints", [])
        ),
        gestures=tuple(
            GestureSample(
                name=str(gesture["name"]),
                active=bool(gesture["active"]),
                confidence=gesture.get("confidence"),
                value=gesture.get("value"),
            )
            for gesture in value.get("gestures", [])
        ),
        pinch_strength=value.get("pinch_strength"),
        grasp_strength=value.get("grasp_strength"),
        confidence=value.get("confidence"),
    )


def _convert_articulation(
    articulation: HandArticulation | None,
    convert: Callable[[Pose6D | None], Pose6D | None],
) -> HandArticulation | None:
    if articulation is None:
        return None
    return HandArticulation(
        joints=tuple(
            JointSample(
                name=joint.name,
                pose=convert(joint.pose),
                tracking_state=joint.tracking_state,
                confidence=joint.confidence,
                radius_m=joint.radius_m,
            )
            for joint in articulation.joints
        ),
        gestures=articulation.gestures,
        pinch_strength=articulation.pinch_strength,
        grasp_strength=articulation.grasp_strength,
        confidence=articulation.confidence,
    )
