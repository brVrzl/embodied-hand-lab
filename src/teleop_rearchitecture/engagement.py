"""Robot-independent recoverable clutch/reference coordination.

The coordinator owns no transport, robot model, IK, or XR receiver.  It keeps
only the latest observed input pose and makes reference-capture/epoch behavior
explicit for offline tests and future adapters.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Iterable


class EngagementMode(Enum):
    UNINITIALIZED = auto()
    DISENGAGED = auto()
    ENGAGING = auto()
    ACTIVE_TRACKING = auto()
    HOLD_REJECTED = auto()
    CONTROLLED_BRAKING = auto()
    STOPPED_READY = auto()
    HARD_STOPPED = auto()


class EngagementResult(Enum):
    OK = auto()
    ALREADY_APPLIED = auto()
    WAIT_FOR_STOPPED = auto()
    INVALID_STATE = auto()
    INVALID_MEASUREMENT = auto()
    NO_INPUT_POSE = auto()
    OLD_EPOCH = auto()
    OLD_SEQUENCE = auto()


@dataclass(frozen=True, slots=True)
class SpatialPose:
    position_m: tuple[float, float, float]
    orientation_wxyz: tuple[float, float, float, float]

    @classmethod
    def checked(cls, position_m: Iterable[float], orientation_wxyz: Iterable[float]) -> "SpatialPose":
        position = tuple(float(value) for value in position_m)
        orientation = tuple(float(value) for value in orientation_wxyz)
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError("pose must contain xyz and wxyz")
        if not all(math.isfinite(value) for value in (*position, *orientation)):
            raise ValueError("pose must be finite")
        norm = math.sqrt(sum(value * value for value in orientation))
        if norm <= 1e-12:
            raise ValueError("quaternion must be nonzero")
        return cls(position, tuple(value / norm for value in orientation))  # type: ignore[arg-type]


@dataclass(frozen=True, slots=True)
class MeasuredJointState:
    sequence: int
    monotonic_ns: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]
    valid: bool = True

    def is_valid(self) -> bool:
        values = (*self.position_rad, *self.velocity_rad_s, *self.acceleration_rad_s2)
        return (
            self.valid
            and self.sequence > 0
            and self.monotonic_ns >= 0
            and 0 < len(self.position_rad) <= 8
            and len(self.velocity_rad_s) == len(self.position_rad)
            and len(self.acceleration_rad_s2) == len(self.position_rad)
            and all(math.isfinite(value) for value in values)
        )


@dataclass(frozen=True, slots=True)
class RelativePose:
    translation_m: tuple[float, float, float]
    rotation_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class EngagementCapture:
    safety_epoch: int
    robot_reference: MeasuredJointState
    input_reference: SpatialPose
    input_sequence: int
    captured_monotonic_ns: int


@dataclass(frozen=True, slots=True)
class CoordinatorSnapshot:
    mode: EngagementMode
    safety_epoch: int
    last_target_sequence: int
    frozen_source_sequence: int
    latest_input_sequence: int
    old_target_rejection_count: int
    input_replacement_count: int
    history_clear_count: int


def _quaternion_product(
    left: tuple[float, float, float, float], right: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


class EngagementCoordinator:
    """Depth-one input/reference state machine with explicit recovery gates."""

    def __init__(self) -> None:
        self.mode = EngagementMode.UNINITIALIZED
        self.safety_epoch = 0
        self.last_target_sequence = 0
        self.frozen_source_sequence = 0
        self.latest_input_sequence = 0
        self.latest_input_pose: SpatialPose | None = None
        self.capture: EngagementCapture | None = None
        self.old_target_rejection_count = 0
        self.input_replacement_count = 0
        self.history_clear_count = 0

    def initialize_disengaged(self, measured: MeasuredJointState) -> EngagementResult:
        if self.mode is not EngagementMode.UNINITIALIZED:
            return EngagementResult.INVALID_STATE
        if not measured.is_valid():
            return EngagementResult.INVALID_MEASUREMENT
        self.mode = EngagementMode.DISENGAGED
        return EngagementResult.OK

    def observe_input(self, sequence: int, pose: SpatialPose) -> EngagementResult:
        if sequence <= self.latest_input_sequence:
            return EngagementResult.OLD_SEQUENCE
        if self.latest_input_pose is not None:
            self.input_replacement_count += 1
        self.latest_input_sequence = sequence
        self.latest_input_pose = pose
        return EngagementResult.OK

    def begin_engagement(
        self, measured: MeasuredJointState, now_ns: int
    ) -> tuple[EngagementResult, EngagementCapture | None]:
        if self.mode is EngagementMode.CONTROLLED_BRAKING:
            return EngagementResult.WAIT_FOR_STOPPED, None
        if self.mode is EngagementMode.ENGAGING:
            return EngagementResult.ALREADY_APPLIED, self.capture
        if self.mode not in (EngagementMode.DISENGAGED, EngagementMode.STOPPED_READY):
            return EngagementResult.INVALID_STATE, None
        if not measured.is_valid():
            return EngagementResult.INVALID_MEASUREMENT, None
        if self.latest_input_pose is None:
            return EngagementResult.NO_INPUT_POSE, None
        self.safety_epoch += 1
        self.last_target_sequence = 0
        self.frozen_source_sequence = 0
        self.history_clear_count += 1
        self.capture = EngagementCapture(
            safety_epoch=self.safety_epoch,
            robot_reference=measured,
            input_reference=self.latest_input_pose,
            input_sequence=self.latest_input_sequence,
            captured_monotonic_ns=now_ns,
        )
        self.mode = EngagementMode.ENGAGING
        return EngagementResult.OK, self.capture

    def complete_engagement(self) -> EngagementResult:
        if self.mode is EngagementMode.ACTIVE_TRACKING:
            return EngagementResult.ALREADY_APPLIED
        if self.mode is not EngagementMode.ENGAGING or self.capture is None:
            return EngagementResult.INVALID_STATE
        self.mode = EngagementMode.ACTIVE_TRACKING
        return EngagementResult.OK

    def relative_pose(self) -> RelativePose | None:
        if self.mode not in (EngagementMode.ACTIVE_TRACKING, EngagementMode.HOLD_REJECTED):
            return None
        if self.capture is None or self.latest_input_pose is None:
            return None
        reference = self.capture.input_reference
        current = self.latest_input_pose
        inverse_reference = (
            reference.orientation_wxyz[0],
            -reference.orientation_wxyz[1],
            -reference.orientation_wxyz[2],
            -reference.orientation_wxyz[3],
        )
        rotation = _quaternion_product(inverse_reference, current.orientation_wxyz)
        if rotation[0] < 0.0:
            rotation = tuple(-value for value in rotation)  # type: ignore[assignment]
        return RelativePose(
            tuple(current.position_m[i] - reference.position_m[i] for i in range(3)),
            rotation,
        )

    def note_target(self, sequence: int, epoch: int, accepted: bool) -> EngagementResult:
        if epoch != self.safety_epoch:
            self.old_target_rejection_count += 1
            return EngagementResult.OLD_EPOCH
        if self.mode not in (EngagementMode.ACTIVE_TRACKING, EngagementMode.HOLD_REJECTED):
            return EngagementResult.INVALID_STATE
        if sequence <= self.last_target_sequence:
            return EngagementResult.OLD_SEQUENCE
        self.last_target_sequence = sequence
        self.mode = EngagementMode.ACTIVE_TRACKING if accepted else EngagementMode.HOLD_REJECTED
        return EngagementResult.OK

    def request_release(self) -> EngagementResult:
        if self.mode is EngagementMode.CONTROLLED_BRAKING:
            return EngagementResult.ALREADY_APPLIED
        if self.mode not in (EngagementMode.ACTIVE_TRACKING, EngagementMode.HOLD_REJECTED):
            return EngagementResult.INVALID_STATE
        self.frozen_source_sequence = self.last_target_sequence
        self.mode = EngagementMode.CONTROLLED_BRAKING
        return EngagementResult.OK

    def braking_complete(self) -> EngagementResult:
        if self.mode is EngagementMode.STOPPED_READY:
            return EngagementResult.ALREADY_APPLIED
        if self.mode is not EngagementMode.CONTROLLED_BRAKING:
            return EngagementResult.INVALID_STATE
        self.mode = EngagementMode.STOPPED_READY
        return EngagementResult.OK

    def hard_stop(self) -> None:
        self.mode = EngagementMode.HARD_STOPPED

    def reset_hard_stop(self, measured: MeasuredJointState) -> EngagementResult:
        if self.mode is not EngagementMode.HARD_STOPPED:
            return EngagementResult.INVALID_STATE
        if not measured.is_valid():
            return EngagementResult.INVALID_MEASUREMENT
        self.capture = None
        self.last_target_sequence = 0
        self.frozen_source_sequence = 0
        self.safety_epoch += 1
        self.history_clear_count += 1
        self.mode = EngagementMode.DISENGAGED
        return EngagementResult.OK

    def snapshot(self) -> CoordinatorSnapshot:
        return CoordinatorSnapshot(
            self.mode,
            self.safety_epoch,
            self.last_target_sequence,
            self.frozen_source_sequence,
            self.latest_input_sequence,
            self.old_target_rejection_count,
            self.input_replacement_count,
            self.history_clear_count,
        )
