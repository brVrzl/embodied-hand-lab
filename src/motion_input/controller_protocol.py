"""Strict Quest left-controller ``CTRL`` wire protocol.

The Quest application emits one fixed-order UTF-8 line per UDP datagram.  This
module validates only wire facts.  It deliberately contains no hysteresis,
clutch state, keyboard fallback, simulation, or robot semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re

from .errors import SerializationError
from .hts_protocol import HTS_MAX_DATAGRAM_BYTES, HtsPacket, parse_hts_datagram


CTRL_PROTOCOL_VERSION = 1
CTRL_PACKET_PREFIX = "CTRL"
CTRL_FIELD_NAMES = (
    "v",
    "session",
    "seq",
    "t_ns",
    "connected",
    "active",
    "tracked",
    "index",
    "grip",
)
UINT64_MAX = (1 << 64) - 1
_ASCII_UINT = re.compile(r"^[0-9]+$")
_ASCII_FLOAT = re.compile(
    r"^(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?$"
)


class ControllerPacketError(SerializationError):
    """A ``CTRL`` packet is malformed or violates its v1 contract."""


@dataclass(frozen=True, slots=True)
class ControllerPacket:
    version: int
    session_id: int
    sequence_number: int
    source_timestamp_ns: int
    connected: bool
    active: bool
    tracked: bool
    index_trigger: float
    grip_trigger: float

    @property
    def facts_valid(self) -> bool:
        return self.connected and self.active and self.tracked


class QuestTransportKind(str, Enum):
    CONTROLLER = "controller"
    HAND_HEAD = "hand_head"


@dataclass(frozen=True, slots=True)
class QuestTransportPacket:
    kind: QuestTransportKind
    controller: ControllerPacket | None = None
    hand_head: tuple[HtsPacket, ...] = ()


def parse_controller_datagram(
    payload: bytes, *, max_bytes: int = HTS_MAX_DATAGRAM_BYTES
) -> ControllerPacket:
    """Decode exactly one CTRL line, allowing only empty line terminators."""

    if not payload:
        raise ControllerPacketError("CTRL datagram is empty")
    if len(payload) > max_bytes:
        raise ControllerPacketError(
            f"CTRL datagram exceeds {max_bytes} byte limit (got {len(payload)})"
        )
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ControllerPacketError(
            f"CTRL datagram is not valid UTF-8: {exc}"
        ) from exc
    lines = text.splitlines()
    nonempty = [line for line in lines if line != ""]
    if len(nonempty) != 1:
        raise ControllerPacketError("CTRL datagram must contain exactly one non-empty line")
    if any(line != "" for line in lines[lines.index(nonempty[0]) + 1 :]):
        raise ControllerPacketError("CTRL datagram has non-empty trailing content")
    return parse_controller_line(nonempty[0])


def parse_controller_line(line: str) -> ControllerPacket:
    """Parse the fixed-order v1 line without normalizing whitespace."""

    if "\n" in line or "\r" in line:
        raise ControllerPacketError("CTRL line contains an embedded line break")
    parts = line.split(",")
    if not parts or parts[0] != CTRL_PACKET_PREFIX:
        raise ControllerPacketError("packet is not CTRL")
    if len(parts) != len(CTRL_FIELD_NAMES) + 1:
        raise ControllerPacketError(
            "CTRL v1 requires exactly nine ordered fields"
        )

    raw_values: dict[str, str] = {}
    seen: set[str] = set()
    for expected_name, field in zip(CTRL_FIELD_NAMES, parts[1:]):
        name, separator, value = field.partition("=")
        if separator != "=" or not name or not value:
            raise ControllerPacketError(f"malformed CTRL field {field!r}")
        if name in seen:
            raise ControllerPacketError(f"duplicate CTRL field {name!r}")
        seen.add(name)
        if name != expected_name:
            if name not in CTRL_FIELD_NAMES:
                raise ControllerPacketError(f"unknown CTRL field {name!r}")
            raise ControllerPacketError(
                f"CTRL field order mismatch: expected {expected_name!r}, got {name!r}"
            )
        raw_values[name] = value

    version = _parse_uint(raw_values["v"], "v", maximum=UINT64_MAX)
    if version != CTRL_PROTOCOL_VERSION:
        raise ControllerPacketError(f"unsupported CTRL version {version}")
    return ControllerPacket(
        version=version,
        session_id=_parse_uint(raw_values["session"], "session", maximum=UINT64_MAX),
        sequence_number=_parse_uint(raw_values["seq"], "seq", maximum=UINT64_MAX),
        source_timestamp_ns=_parse_uint(raw_values["t_ns"], "t_ns", maximum=UINT64_MAX),
        connected=_parse_bool(raw_values["connected"], "connected"),
        active=_parse_bool(raw_values["active"], "active"),
        tracked=_parse_bool(raw_values["tracked"], "tracked"),
        index_trigger=_parse_unit_float(raw_values["index"], "index"),
        grip_trigger=_parse_unit_float(raw_values["grip"], "grip"),
    )


def parse_quest_transport_datagram(payload: bytes) -> QuestTransportPacket:
    """Dispatch CTRL independently while preserving legacy HTS parsing."""

    if payload.startswith(b"CTRL,"):
        return QuestTransportPacket(
            kind=QuestTransportKind.CONTROLLER,
            controller=parse_controller_datagram(payload),
        )
    packets = parse_hts_datagram(payload)
    return QuestTransportPacket(
        kind=QuestTransportKind.HAND_HEAD,
        hand_head=packets,
    )


def _parse_uint(raw: str, field: str, *, maximum: int) -> int:
    if _ASCII_UINT.fullmatch(raw) is None:
        raise ControllerPacketError(f"CTRL {field} must be an ASCII unsigned integer")
    value = int(raw)
    if value > maximum:
        raise ControllerPacketError(f"CTRL {field} exceeds uint64 range")
    return value


def _parse_bool(raw: str, field: str) -> bool:
    if raw == "0":
        return False
    if raw == "1":
        return True
    raise ControllerPacketError(f"CTRL {field} must be 0 or 1")


def _parse_unit_float(raw: str, field: str) -> float:
    if _ASCII_FLOAT.fullmatch(raw) is None:
        raise ControllerPacketError(f"CTRL {field} is not an ASCII finite float")
    value = float(raw)
    if not math.isfinite(value):
        raise ControllerPacketError(f"CTRL {field} must be finite")
    if not 0.0 <= value <= 1.0:
        raise ControllerPacketError(f"CTRL {field} is outside [0, 1]")
    return value
