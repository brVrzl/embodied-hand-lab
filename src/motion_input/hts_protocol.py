"""Validated wire model for the Hand Tracking Streamer (HTS) v1.1 text protocol.

This module owns only UTF-8/CSV decoding and source-schema validation.  It does
not open sockets, convert coordinate frames, infer tracking state, or import
robot-control code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
import re
from types import MappingProxyType
from typing import Mapping, TypeAlias

from .errors import SerializationError
from .model import Side


HTS_PROTOCOL_VERSION = "1.1"
HTS_DEFAULT_UDP_PORT = 9000
HTS_MAX_DATAGRAM_BYTES = 65_535
HTS_LANDMARK_COUNT = 21
HTS_QUATERNION_NORM_TOLERANCE = 0.02

# The order is documented by HTS CONNECTIONS.md and matches the v1.1.0
# HandLandmarkStreamer._streamedJoints array.  Positions are wrist-local.
HTS_JOINT_NAMES = (
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)


class HtsPacketKind(str, Enum):
    WRIST = "wrist"
    LANDMARKS = "landmarks"
    HEAD_POSE = "head_pose"


@dataclass(frozen=True, slots=True)
class HtsHeader:
    kind: HtsPacketKind
    side: Side
    source_sequence: int | None = None
    source_timestamp_ns: int | None = None
    extra_fields: Mapping[str, int] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.kind is HtsPacketKind.HEAD_POSE:
            if self.side is not Side.NONE:
                raise SerializationError("HTS head pose must use side=none")
        elif self.side not in (Side.LEFT, Side.RIGHT):
            raise SerializationError("HTS hand packet must be left or right")
        for name, value in (
            ("source_sequence", self.source_sequence),
            ("source_timestamp_ns", self.source_timestamp_ns),
        ):
            if value is not None and value < 0:
                raise SerializationError(f"HTS {name} must be non-negative")
        object.__setattr__(self, "extra_fields", MappingProxyType(dict(self.extra_fields)))


@dataclass(frozen=True, slots=True)
class HtsWristPacket:
    header: HtsHeader
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    quaternion_norm: float


@dataclass(frozen=True, slots=True)
class HtsLandmarksPacket:
    header: HtsHeader
    positions_wrist_m: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class HtsHeadPosePacket:
    header: HtsHeader
    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    quaternion_norm: float


HtsPacket: TypeAlias = HtsWristPacket | HtsLandmarksPacket | HtsHeadPosePacket


_DEBUG_FIELD = re.compile(r"^([A-Za-z_]+)\s*=\s*([0-9]+)$")


def parse_hts_datagram(payload: bytes, *, max_bytes: int = HTS_MAX_DATAGRAM_BYTES) -> tuple[HtsPacket, ...]:
    """Decode one UDP datagram into one or more validated HTS lines."""

    if not payload:
        raise SerializationError("HTS datagram is empty")
    if len(payload) > max_bytes:
        raise SerializationError(
            f"HTS datagram exceeds {max_bytes} byte limit (got {len(payload)})"
        )
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise SerializationError(f"HTS datagram is not valid UTF-8: {exc}") from exc
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines:
        raise SerializationError("HTS datagram contains no non-empty lines")
    return tuple(parse_hts_line(line) for line in lines)


def parse_hts_line(line: str) -> HtsPacket:
    """Parse one HTS v1.1 CSV line, including optional debug metadata."""

    stripped = line.strip()
    if not stripped:
        raise SerializationError("HTS line is empty")
    label_with_metadata, separator, payload = stripped.partition(":")
    if not separator:
        raise SerializationError("HTS line is missing ':' separator")
    header = _parse_header(label_with_metadata)
    values = _parse_finite_floats(payload)

    if header.kind in (HtsPacketKind.WRIST, HtsPacketKind.HEAD_POSE):
        if len(values) != 7:
            name = "wrist" if header.kind is HtsPacketKind.WRIST else "head pose"
            raise SerializationError(
                f"HTS {name} packet requires exactly 7 floats, got {len(values)}"
            )
        position = (values[0], values[1], values[2])
        quaternion = (values[3], values[4], values[5], values[6])
        norm = quaternion_norm(quaternion)
        if norm <= 1e-12 or abs(norm - 1.0) > HTS_QUATERNION_NORM_TOLERANCE:
            raise SerializationError(
                "HTS quaternion norm outside accepted rounded-source tolerance "
                f"({norm:.8f}, tolerance={HTS_QUATERNION_NORM_TOLERANCE})"
            )
        if header.kind is HtsPacketKind.WRIST:
            return HtsWristPacket(header, position, quaternion, norm)
        return HtsHeadPosePacket(header, position, quaternion, norm)

    expected = HTS_LANDMARK_COUNT * 3
    if len(values) != expected:
        raise SerializationError(
            f"HTS landmarks packet requires exactly {expected} floats, got {len(values)}"
        )
    points = tuple(
        (values[index], values[index + 1], values[index + 2])
        for index in range(0, expected, 3)
    )
    return HtsLandmarksPacket(header, points)


def normalize_quaternion(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    """Normalize a previously validated source quaternion."""

    norm = quaternion_norm(quaternion)
    if norm <= 1e-12 or not math.isfinite(norm):
        raise SerializationError("cannot normalize a zero or non-finite quaternion")
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def quaternion_norm(quaternion: tuple[float, float, float, float]) -> float:
    if len(quaternion) != 4 or not all(math.isfinite(value) for value in quaternion):
        raise SerializationError("HTS quaternion must contain four finite floats")
    return math.sqrt(sum(value * value for value in quaternion))


def _parse_header(value: str) -> HtsHeader:
    parts = [part.strip() for part in value.split("|")]
    if not parts or not parts[0]:
        raise SerializationError("HTS label is empty")
    label = parts[0]
    if label == "Left wrist":
        kind, side = HtsPacketKind.WRIST, Side.LEFT
    elif label == "Right wrist":
        kind, side = HtsPacketKind.WRIST, Side.RIGHT
    elif label == "Left landmarks":
        kind, side = HtsPacketKind.LANDMARKS, Side.LEFT
    elif label == "Right landmarks":
        kind, side = HtsPacketKind.LANDMARKS, Side.RIGHT
    elif label == "Head pose":
        kind, side = HtsPacketKind.HEAD_POSE, Side.NONE
    else:
        raise SerializationError(f"unsupported HTS label {label!r}")

    source_sequence = None
    source_timestamp_ns = None
    extras: dict[str, int] = {}
    for item in parts[1:]:
        match = _DEBUG_FIELD.fullmatch(item)
        if match is None:
            raise SerializationError(f"malformed HTS debug header field {item!r}")
        key = match.group(1).lower()
        number = int(match.group(2))
        if key in {"f", "frame", "frame_id"}:
            if source_sequence is not None:
                raise SerializationError("duplicate HTS source sequence field")
            source_sequence = number
        elif key in {"t", "ts", "timestamp"}:
            if source_timestamp_ns is not None:
                raise SerializationError("duplicate HTS source timestamp field")
            source_timestamp_ns = number
        else:
            extras[key] = number
    return HtsHeader(kind, side, source_sequence, source_timestamp_ns, extras)


def _parse_finite_floats(payload: str) -> list[float]:
    chunks = [chunk.strip() for chunk in payload.split(",")]
    # The source emits a comma immediately after the colon.  Empty leading and
    # trailing CSV cells are framing, but empty cells between values are errors.
    while chunks and not chunks[0]:
        chunks.pop(0)
    while chunks and not chunks[-1]:
        chunks.pop()
    if not chunks or any(not chunk for chunk in chunks):
        raise SerializationError("HTS CSV payload contains an empty value")
    try:
        values = [float(chunk) for chunk in chunks]
    except ValueError as exc:
        raise SerializationError("HTS CSV payload contains a non-float value") from exc
    if not all(math.isfinite(value) for value in values):
        raise SerializationError("HTS CSV payload contains NaN or infinity")
    return values
