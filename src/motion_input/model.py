"""Strongly typed Unified Motion Input Protocol (UMIP) data model.

This module intentionally has no device SDK, robot, control, or numerical-stack
dependencies. Providers translate device data into these immutable values.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from types import MappingProxyType
from typing import Any, Mapping, TypeAlias

from .errors import ProtocolValidationError


UMIP_VERSION = "1.0"
JSONScalar: TypeAlias = None | bool | int | float | str
JSONValue: TypeAlias = JSONScalar | list["JSONValue"] | dict[str, "JSONValue"]


class TrackingState(str, Enum):
    TRACKING = "tracking"
    LIMITED = "limited"
    NOT_TRACKING = "not_tracking"
    DISCONNECTED = "disconnected"


class Side(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    NONE = "none"


class MotionKind(str, Enum):
    ABSOLUTE = "absolute_pose"
    RELATIVE = "relative_pose_delta"


def _finite_tuple(values: tuple[float, ...], *, length: int, name: str) -> tuple[float, ...]:
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ProtocolValidationError(f"{name} must contain {length} finite values")
    return tuple(float(value) for value in values)


def _json_mapping(value: Mapping[str, Any], *, name: str) -> Mapping[str, JSONValue]:
    copied = dict(value)

    def validate(item: Any, path: str) -> None:
        if item is None or isinstance(item, (bool, int, str)):
            return
        if isinstance(item, float):
            if not math.isfinite(item):
                raise ProtocolValidationError(f"{path} contains a non-finite float")
            return
        if isinstance(item, list):
            for index, child in enumerate(item):
                validate(child, f"{path}[{index}]")
            return
        if isinstance(item, dict) and all(isinstance(key, str) for key in item):
            for key, child in item.items():
                validate(child, f"{path}.{key}")
            return
        raise ProtocolValidationError(f"{path} is not JSON-compatible")

    validate(copied, name)
    return MappingProxyType(copied)


@dataclass(frozen=True, slots=True)
class Timestamp:
    """Nanosecond timestamp in an explicitly named clock domain."""

    nanoseconds: int
    clock_id: str
    uncertainty_ns: int | None = None

    def __post_init__(self) -> None:
        if self.nanoseconds < 0:
            raise ProtocolValidationError("timestamp nanoseconds must be non-negative")
        if not self.clock_id.strip():
            raise ProtocolValidationError("timestamp clock_id must not be empty")
        if self.uncertainty_ns is not None and self.uncertainty_ns < 0:
            raise ProtocolValidationError("timestamp uncertainty_ns must be non-negative")

    def comparable_to(self, other: "Timestamp") -> bool:
        return self.clock_id == other.clock_id

    def difference_ns(self, other: "Timestamp") -> int:
        if not self.comparable_to(other):
            raise ProtocolValidationError(
                f"timestamps use different clocks: {self.clock_id!r} and {other.clock_id!r}"
            )
        return self.nanoseconds - other.nanoseconds


@dataclass(frozen=True, slots=True)
class Pose6D:
    """Rigid pose: meters and a unit quaternion in x, y, z, w order."""

    position_m: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        position = _finite_tuple(self.position_m, length=3, name="position_m")
        orientation = _finite_tuple(self.orientation_xyzw, length=4, name="orientation_xyzw")
        norm = math.sqrt(sum(value * value for value in orientation))
        if abs(norm - 1.0) > 1e-3:
            raise ProtocolValidationError(
                f"orientation_xyzw must be a unit quaternion (norm={norm:.8f})"
            )
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "orientation_xyzw", orientation)


@dataclass(frozen=True, slots=True)
class DeviceDescriptor:
    device_id: str
    device_type: str
    manufacturer: str
    model: str
    serial_number: str | None = None
    firmware_version: str | None = None
    software_version: str | None = None
    metadata: Mapping[str, JSONValue] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("device_id", "device_type", "manufacturer", "model"):
            if not getattr(self, name).strip():
                raise ProtocolValidationError(f"{name} must not be empty")
        object.__setattr__(self, "metadata", _json_mapping(self.metadata, name="device metadata"))


@dataclass(frozen=True, slots=True)
class JointSample:
    """Optional articulated joint, named by a provider-neutral semantic string."""

    name: str
    pose: Pose6D | None
    tracking_state: TrackingState
    confidence: float | None = None
    radius_m: float | None = None

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ProtocolValidationError("joint name must not be empty")
        _validate_confidence(self.confidence, "joint confidence")
        if self.radius_m is not None and (not math.isfinite(self.radius_m) or self.radius_m <= 0):
            raise ProtocolValidationError("joint radius_m must be finite and positive")
        if self.tracking_state is TrackingState.TRACKING and self.pose is None:
            raise ProtocolValidationError("a tracking joint requires a pose")


@dataclass(frozen=True, slots=True)
class GestureSample:
    name: str
    active: bool
    confidence: float | None = None
    value: float | None = None

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ProtocolValidationError("gesture name must not be empty")
        _validate_confidence(self.confidence, "gesture confidence")
        if self.value is not None and not math.isfinite(self.value):
            raise ProtocolValidationError("gesture value must be finite")


@dataclass(frozen=True, slots=True)
class HandArticulation:
    """Backward-compatible extension point for joints and hand semantics."""

    joints: tuple[JointSample, ...] = ()
    gestures: tuple[GestureSample, ...] = ()
    pinch_strength: float | None = None
    grasp_strength: float | None = None
    confidence: float | None = None

    def __post_init__(self) -> None:
        names = [joint.name for joint in self.joints]
        if len(names) != len(set(names)):
            raise ProtocolValidationError("joint names must be unique within a sample")
        _validate_unit_interval(self.pinch_strength, "pinch_strength")
        _validate_unit_interval(self.grasp_strength, "grasp_strength")
        _validate_confidence(self.confidence, "articulation confidence")


def _validate_unit_interval(value: float | None, name: str) -> None:
    if value is not None and (not math.isfinite(value) or value < 0.0 or value > 1.0):
        raise ProtocolValidationError(f"{name} must be in [0, 1]")


def _validate_confidence(value: float | None, name: str) -> None:
    _validate_unit_interval(value, name)


@dataclass(frozen=True, slots=True)
class MotionInputSample:
    """One immutable, device-independent UMIP sample.

    A tracking sample must contain a wrist pose. Loss and disconnect events must
    not fabricate one. ``motion_kind`` reserves relative 6-DoF devices while the
    same provider contract remains unchanged.
    """

    sample_id: str
    stream_id: str
    sequence_number: int
    capture_timestamp: Timestamp
    receive_timestamp: Timestamp
    device_timestamp: Timestamp | None
    processing_timestamp: Timestamp | None
    tracking_state: TrackingState
    tracking_confidence: float | None
    coordinate_frame: str
    device: DeviceDescriptor
    side: Side
    wrist_pose: Pose6D | None
    palm_pose: Pose6D | None = None
    motion_kind: MotionKind = MotionKind.ABSOLUTE
    articulation: HandArticulation | None = None
    metadata: Mapping[str, JSONValue] = field(default_factory=dict)
    extensions: Mapping[str, JSONValue] = field(default_factory=dict)
    protocol_version: str = UMIP_VERSION

    def __post_init__(self) -> None:
        for name in ("sample_id", "stream_id", "coordinate_frame"):
            if not getattr(self, name).strip():
                raise ProtocolValidationError(f"{name} must not be empty")
        if self.sequence_number < 0:
            raise ProtocolValidationError("sequence_number must be non-negative")
        major_minor = self.protocol_version.split(".")
        if len(major_minor) != 2 or not all(part.isdigit() for part in major_minor):
            raise ProtocolValidationError("protocol_version must be MAJOR.MINOR")
        _validate_confidence(self.tracking_confidence, "tracking_confidence")
        if self.tracking_state is TrackingState.TRACKING and self.wrist_pose is None:
            raise ProtocolValidationError("a tracking sample requires a wrist_pose")
        if self.tracking_state in (TrackingState.NOT_TRACKING, TrackingState.DISCONNECTED):
            if self.wrist_pose is not None or self.palm_pose is not None:
                raise ProtocolValidationError(
                    "not-tracking and disconnected samples must not contain poses"
                )
        object.__setattr__(self, "metadata", _json_mapping(self.metadata, name="sample metadata"))
        extensions = _json_mapping(self.extensions, name="sample extensions")
        for key in extensions:
            if "." not in key:
                raise ProtocolValidationError(
                    f"extension key {key!r} must be namespaced (for example vendor.feature)"
                )
        object.__setattr__(self, "extensions", extensions)

    @property
    def device_type(self) -> str:
        return self.device.device_type
