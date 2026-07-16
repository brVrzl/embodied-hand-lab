from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, fields
from enum import Enum
from typing import Any, Mapping


SCHEMA_VERSION = "arm_teleoperation.v1"
NANOSECONDS_PER_SECOND = 1_000_000_000


class ControllerState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ARMED = "armed"
    EDG_READY = "edg_ready"
    HOLDING = "holding"
    RUNNING = "running"
    CONTROLLED_STOP = "controlled_stop"
    FAULT = "fault"
    SHUTDOWN = "shutdown"


class HealthLevel(str, Enum):
    OK = "ok"
    WARNING = "warning"
    DEGRADED = "degraded"
    ERROR = "error"
    FATAL = "fatal"


class SafetyAction(str, Enum):
    HOLD = "hold"
    ALLOW = "allow"
    CONTROLLED_STOP = "controlled_stop"
    ABORT = "abort"


def _finite(values: tuple[float, ...], *, field_name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field_name} must contain only finite values")
    return result


def _nonnegative_ns(value: int | None, *, field_name: str) -> int | None:
    if value is None:
        return None
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return value


@dataclass(frozen=True, slots=True)
class TimestampSet:
    """Timestamps for one sample as it crosses the control pipeline.

    ``source_capture_ns`` belongs to the source clock domain and is optional.
    Every other field is a host monotonic-clock timestamp and can therefore be
    compared locally.  A missing stage is represented by ``None``, never zero.
    """

    local_receive_ns: int
    source_capture_ns: int | None = None
    processing_ns: int | None = None
    dispatch_ns: int | None = None
    robot_command_ns: int | None = None
    robot_state_observation_ns: int | None = None

    def __post_init__(self) -> None:
        local = _nonnegative_ns(self.local_receive_ns, field_name="local_receive_ns")
        assert local is not None
        object.__setattr__(self, "local_receive_ns", local)
        for field_name in (
            "source_capture_ns",
            "processing_ns",
            "dispatch_ns",
            "robot_command_ns",
            "robot_state_observation_ns",
        ):
            object.__setattr__(
                self,
                field_name,
                _nonnegative_ns(getattr(self, field_name), field_name=field_name),
            )
        local_stages = [
            value
            for value in (
                self.local_receive_ns,
                self.processing_ns,
                self.dispatch_ns,
                self.robot_command_ns,
            )
            if value is not None
        ]
        if any(after < before for before, after in zip(local_stages, local_stages[1:])):
            raise ValueError("local pipeline timestamps must be monotonic by stage")

    def with_stage(self, **updates: int | None) -> "TimestampSet":
        values = asdict(self)
        unknown = set(updates) - set(values)
        if unknown:
            raise ValueError(f"unknown timestamp stages: {sorted(unknown)}")
        values.update(updates)
        return TimestampSet(**values)

    def to_dict(self) -> dict[str, int | None]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class Pose3D:
    """Rigid pose in SI units with canonical quaternion order ``xyzw``."""

    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if len(self.position_m) != 3 or len(self.quaternion_xyzw) != 4:
            raise ValueError("pose requires 3 position and 4 quaternion values")
        position = _finite(self.position_m, field_name="position_m")
        quaternion = _finite(self.quaternion_xyzw, field_name="quaternion_xyzw")
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-12:
            raise ValueError("quaternion norm is zero")
        if abs(norm - 1.0) > 1e-3:
            raise ValueError(f"quaternion must be unit length, got {norm:.9f}")
        normalized = tuple(value / norm for value in quaternion)
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "quaternion_xyzw", normalized)

    def to_dict(self) -> dict[str, list[float]]:
        return {
            "position_m": list(self.position_m),
            "quaternion_xyzw": list(self.quaternion_xyzw),
        }


@dataclass(frozen=True, slots=True)
class ArmPoseSample:
    source_id: str
    sequence: int
    frame_id: str
    pose: Pose3D
    timestamps: TimestampSet
    tracking_valid: bool = True
    tracking_quality: float | None = None

    def __post_init__(self) -> None:
        _validate_identity(self.source_id, "source_id")
        _validate_identity(self.frame_id, "frame_id")
        _validate_sequence(self.sequence)
        if self.tracking_quality is not None:
            quality = float(self.tracking_quality)
            if not math.isfinite(quality) or not 0.0 <= quality <= 1.0:
                raise ValueError("tracking_quality must be within [0, 1]")
            object.__setattr__(self, "tracking_quality", quality)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": SCHEMA_VERSION,
            "source_id": self.source_id,
            "sequence": self.sequence,
            "frame_id": self.frame_id,
            "pose": self.pose.to_dict(),
            "timestamps": self.timestamps.to_dict(),
            "tracking_valid": self.tracking_valid,
            "tracking_quality": self.tracking_quality,
        }


@dataclass(frozen=True, slots=True)
class PoseTarget:
    source_id: str
    sequence: int
    target_frame_id: str
    pose: Pose3D
    timestamps: TimestampSet
    linear_velocity_m_s: tuple[float, float, float] | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        _validate_identity(self.source_id, "source_id")
        _validate_identity(self.target_frame_id, "target_frame_id")
        _validate_sequence(self.sequence)
        for name in ("linear_velocity_m_s", "angular_velocity_rad_s"):
            value = getattr(self, name)
            if value is not None:
                if len(value) != 3:
                    raise ValueError(f"{name} must contain 3 values")
                object.__setattr__(self, name, _finite(value, field_name=name))

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["schema_version"] = SCHEMA_VERSION
        return payload


@dataclass(frozen=True, slots=True)
class RobotState:
    sequence: int
    observed_monotonic_ns: int
    joint_position_rad: tuple[float, float, float, float, float, float]
    joint_velocity_rad_s: tuple[float, float, float, float, float, float]
    tcp_pose: Pose3D | None
    powered: bool
    enabled: bool
    in_servo_mode: bool
    controller_timestamp_ns: int | None = None
    sdk_call_duration_ns: int | None = None

    def __post_init__(self) -> None:
        _validate_sequence(self.sequence)
        observed = _nonnegative_ns(self.observed_monotonic_ns, field_name="observed_monotonic_ns")
        assert observed is not None
        object.__setattr__(self, "observed_monotonic_ns", observed)
        for name in ("joint_position_rad", "joint_velocity_rad_s"):
            value = getattr(self, name)
            if len(value) != 6:
                raise ValueError(f"{name} must contain 6 values")
            object.__setattr__(self, name, _finite(value, field_name=name))
        for name in ("controller_timestamp_ns", "sdk_call_duration_ns"):
            object.__setattr__(self, name, _nonnegative_ns(getattr(self, name), field_name=name))

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["schema_version"] = SCHEMA_VERSION
        return payload


@dataclass(frozen=True, slots=True)
class ControllerStatus:
    state: ControllerState
    monotonic_ns: int
    last_command_sequence: int | None
    owner_pid: int | None
    reason: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "monotonic_ns", int(self.monotonic_ns))
        if self.monotonic_ns < 0:
            raise ValueError("monotonic_ns must be non-negative")
        if self.last_command_sequence is not None:
            _validate_sequence(self.last_command_sequence)


@dataclass(frozen=True, slots=True)
class HealthState:
    level: HealthLevel
    monotonic_ns: int
    connected: bool
    detail: str = ""
    sample_age_ns: int | None = None
    consecutive_failures: int = 0

    def __post_init__(self) -> None:
        if self.monotonic_ns < 0 or self.consecutive_failures < 0:
            raise ValueError("health timestamps and counters must be non-negative")
        object.__setattr__(
            self, "sample_age_ns", _nonnegative_ns(self.sample_age_ns, field_name="sample_age_ns")
        )


@dataclass(frozen=True, slots=True)
class SafetyState:
    action: SafetyAction
    monotonic_ns: int
    reasons: tuple[str, ...] = ()
    fault_latched: bool = False

    def __post_init__(self) -> None:
        if self.monotonic_ns < 0:
            raise ValueError("monotonic_ns must be non-negative")
        if any(not reason for reason in self.reasons):
            raise ValueError("safety reasons may not be empty")


@dataclass(frozen=True, slots=True)
class CommandAcknowledgement:
    sequence: int
    accepted: bool
    state: ControllerState
    received_monotonic_ns: int
    robot_command_monotonic_ns: int | None = None
    reason: str = ""

    def __post_init__(self) -> None:
        _validate_sequence(self.sequence)
        if self.received_monotonic_ns < 0:
            raise ValueError("received_monotonic_ns must be non-negative")
        object.__setattr__(
            self,
            "robot_command_monotonic_ns",
            _nonnegative_ns(
                self.robot_command_monotonic_ns,
                field_name="robot_command_monotonic_ns",
            ),
        )


@dataclass(frozen=True, slots=True)
class TimingStatistics:
    name: str
    unit: str
    count: int
    requested_period_ns: int | None
    mean: float
    median: float
    stddev: float
    minimum: float
    maximum: float
    p95: float
    p99: float
    p999: float | None
    missed_deadlines: int = 0
    max_consecutive_missed_deadlines: int = 0

    def __post_init__(self) -> None:
        _validate_identity(self.name, "name")
        _validate_identity(self.unit, "unit")
        if self.count < 0 or self.missed_deadlines < 0 or self.max_consecutive_missed_deadlines < 0:
            raise ValueError("timing counts must be non-negative")
        object.__setattr__(
            self,
            "requested_period_ns",
            _nonnegative_ns(self.requested_period_ns, field_name="requested_period_ns"),
        )
        numeric = (
            self.mean,
            self.median,
            self.stddev,
            self.minimum,
            self.maximum,
            self.p95,
            self.p99,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("timing statistics must be finite")
        if self.p999 is not None and not math.isfinite(self.p999):
            raise ValueError("p999 must be finite when provided")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _validate_identity(value: str, field_name: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field_name} must be a non-empty string")


def _validate_sequence(value: int) -> None:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise ValueError("sequence must be a non-negative integer")


def contract_json_dumps(value: Any) -> str:
    if hasattr(value, "to_dict"):
        payload = value.to_dict()
    elif hasattr(value, "__dataclass_fields__"):
        payload = asdict(value)
    else:
        raise TypeError(f"unsupported contract type: {type(value)!r}")
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)


def arm_pose_sample_from_dict(payload: Mapping[str, Any]) -> ArmPoseSample:
    version = payload.get("schema_version")
    if version != SCHEMA_VERSION:
        raise ValueError(f"unsupported schema_version: {version!r}")
    pose = payload["pose"]
    timestamps = payload["timestamps"]
    return ArmPoseSample(
        source_id=str(payload["source_id"]),
        sequence=int(payload["sequence"]),
        frame_id=str(payload["frame_id"]),
        pose=Pose3D(
            tuple(float(value) for value in pose["position_m"]),
            tuple(float(value) for value in pose["quaternion_xyzw"]),
        ),
        timestamps=TimestampSet(**{field.name: timestamps.get(field.name) for field in fields(TimestampSet)}),
        tracking_valid=bool(payload.get("tracking_valid", True)),
        tracking_quality=(
            None if payload.get("tracking_quality") is None else float(payload["tracking_quality"])
        ),
    )
