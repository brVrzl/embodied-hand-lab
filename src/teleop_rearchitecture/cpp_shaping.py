"""ctypes conformance bridge for the offline C++ reference shaper.

This module is not a transport and does not import a vendor SDK.  It mirrors
the ABI v1 POD layout so Python tests and the offline evaluator can exercise
the independently compiled C++ implementation.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from enum import IntEnum
import math
from pathlib import Path
from typing import Iterable


MAX_DOF = 8
ABI_MAGIC = 0x54434D44
ABI_VERSION = 1
LITTLE_ENDIAN = 1
REFERENCE_PERIOD_NS = 8_000_000


class EngagementState(IntEnum):
    DISENGAGED = 0
    ENGAGED = 1


class TargetValidity(IntEnum):
    NO_TARGET = 0
    ACCEPTED = 1
    REJECTED_KEEP_PREVIOUS = 2


class OutputMode(IntEnum):
    INACTIVE = 0
    ACTIVE_TRACKING = 1
    CONTROLLED_BRAKING = 2
    STOPPED = 3
    HARD_STOPPED = 4


class StopReason(IntEnum):
    NONE = 0
    CLUTCH_RELEASE = 1
    STALE_INPUT = 2
    TIMING_FAULT = 3
    CONTROLLER_ALARM = 4
    SDK_FAILURE = 5
    ESTOP = 6
    COLLISION = 7
    PRODUCER_FAILURE = 8
    EPOCH_MISMATCH = 9
    INVALID_COMMAND = 10


class OperationCode(IntEnum):
    OK = 0
    COMPLETED = 1
    ALREADY_REQUESTED = 2
    INVALID_ARGUMENT = 3
    INVALID_STATE = 4
    PLANNING_FAILED = 5
    TERMINAL_NO_OUTPUT = 6


class BrakePlanningFailure(IntEnum):
    NONE = 0
    POSITION_LIMIT = 1
    VELOCITY_LIMIT = 2
    NUMERICAL = 3
    INVALID_DYNAMIC_STATE = 4


class AbiHeader(ctypes.Structure):
    _fields_ = [
        ("magic", ctypes.c_uint32),
        ("schema_version", ctypes.c_uint16),
        ("struct_size", ctypes.c_uint16),
        ("host_endianness", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 7),
    ]


class AcceptedJointTargetV1(ctypes.Structure):
    _fields_ = [
        ("header", AbiHeader),
        ("sequence", ctypes.c_uint64),
        ("safety_epoch", ctypes.c_uint64),
        ("source_monotonic_ns", ctypes.c_int64),
        ("accepted_monotonic_ns", ctypes.c_int64),
        ("valid_until_monotonic_ns", ctypes.c_int64),
        ("dof", ctypes.c_uint8),
        ("engagement", ctypes.c_uint8),
        ("validity", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("reason_code", ctypes.c_uint32),
        ("position_rad", ctypes.c_double * MAX_DOF),
    ]


class MeasuredJointStateV1(ctypes.Structure):
    _fields_ = [
        ("header", AbiHeader),
        ("state_sequence", ctypes.c_uint64),
        ("safety_epoch", ctypes.c_uint64),
        ("measured_monotonic_ns", ctypes.c_int64),
        ("dof", ctypes.c_uint8),
        ("validity", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 6),
        ("position_rad", ctypes.c_double * MAX_DOF),
        ("velocity_rad_s", ctypes.c_double * MAX_DOF),
        ("acceleration_rad_s2", ctypes.c_double * MAX_DOF),
    ]


class JointDynamicLimitsV1(ctypes.Structure):
    _fields_ = [
        ("header", AbiHeader),
        ("dof", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 7),
        ("minimum_position_rad", ctypes.c_double * MAX_DOF),
        ("maximum_position_rad", ctypes.c_double * MAX_DOF),
        ("maximum_velocity_rad_s", ctypes.c_double * MAX_DOF),
        ("maximum_acceleration_rad_s2", ctypes.c_double * MAX_DOF),
        ("maximum_jerk_rad_s3", ctypes.c_double * MAX_DOF),
    ]


class ShapedJointCommandV1(ctypes.Structure):
    _fields_ = [
        ("header", AbiHeader),
        ("output_sequence", ctypes.c_uint64),
        ("source_sequence", ctypes.c_uint64),
        ("safety_epoch", ctypes.c_uint64),
        ("generated_monotonic_ns", ctypes.c_int64),
        ("valid_until_monotonic_ns", ctypes.c_int64),
        ("dof", ctypes.c_uint8),
        ("output_mode", ctypes.c_uint8),
        ("stop_class", ctypes.c_uint8),
        ("stop_reason", ctypes.c_uint8),
        ("reason_code", ctypes.c_uint32),
        ("position_rad", ctypes.c_double * MAX_DOF),
        ("velocity_rad_s", ctypes.c_double * MAX_DOF),
        ("acceleration_rad_s2", ctypes.c_double * MAX_DOF),
    ]


class TransportHealthV1(ctypes.Structure):
    _fields_ = [
        ("header", AbiHeader),
        ("health_sequence", ctypes.c_uint64),
        ("last_consumed_output_sequence", ctypes.c_uint64),
        ("safety_epoch", ctypes.c_uint64),
        ("sampled_monotonic_ns", ctypes.c_int64),
        ("transport_state", ctypes.c_uint8),
        ("controller_state", ctypes.c_uint8),
        ("producer_stale", ctypes.c_uint8),
        ("command_stale", ctypes.c_uint8),
        ("deadline_missed", ctypes.c_uint8),
        ("alarm", ctypes.c_uint8),
        ("estop", ctypes.c_uint8),
        ("collision", ctypes.c_uint8),
        ("servo_enabled", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 3),
        ("vendor_status_category", ctypes.c_int32),
    ]


class ValidationContext(ctypes.Structure):
    _fields_ = [
        ("now_ns", ctypes.c_int64),
        ("expected_dof", ctypes.c_uint8),
        ("expected_epoch", ctypes.c_uint64),
        ("previous_sequence", ctypes.c_uint64),
    ]


class ValidationResult(ctypes.Structure):
    _fields_ = [
        ("ok", ctypes.c_bool),
        ("error", ctypes.c_uint16),
        ("field", ctypes.c_uint16),
        ("index", ctypes.c_uint8),
    ]


class ShaperSnapshot(ctypes.Structure):
    _fields_ = [
        ("mode", ctypes.c_uint8),
        ("dof", ctypes.c_uint8),
        ("safety_epoch", ctypes.c_uint64),
        ("last_input_sequence", ctypes.c_uint64),
        ("source_sequence", ctypes.c_uint64),
        ("output_sequence", ctypes.c_uint64),
        ("release_sequence", ctypes.c_uint64),
        ("last_tick_ns", ctypes.c_int64),
        ("liveness_monotonic_ns", ctypes.c_int64),
        ("stop_reason", ctypes.c_uint8),
        ("brake_planning_failure", ctypes.c_uint8),
        ("brake_planning_failure_axis", ctypes.c_uint8),
        ("acceleration_neutralization_axis_count", ctypes.c_uint8),
        ("position_rad", ctypes.c_double * MAX_DOF),
        ("velocity_rad_s", ctypes.c_double * MAX_DOF),
        ("acceleration_rad_s2", ctypes.c_double * MAX_DOF),
    ]


def _header(struct_type: type[ctypes.Structure]) -> AbiHeader:
    header = AbiHeader()
    header.magic = ABI_MAGIC
    header.schema_version = ABI_VERSION
    header.struct_size = ctypes.sizeof(struct_type)
    header.host_endianness = LITTLE_ENDIAN
    return header


def _values(values: Iterable[float], dof: int) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != dof or not all(math.isfinite(value) for value in result):
        raise ValueError(f"expected {dof} finite values")
    return result


@dataclass(frozen=True, slots=True)
class CppShaperPoint:
    output_sequence: int
    source_sequence: int
    output_mode: OutputMode
    stop_reason: StopReason
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class CppShaperSnapshot:
    mode: int
    safety_epoch: int
    last_input_sequence: int
    source_sequence: int
    output_sequence: int
    release_sequence: int
    stop_reason: StopReason
    brake_planning_failure: BrakePlanningFailure
    brake_planning_failure_axis: int
    acceleration_neutralization_axis_count: int
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]


class CppReferenceShaper:
    """Owned C++ shaper instance; allocation occurs only at construction."""

    def __init__(self, library_path: Path) -> None:
        self.library_path = Path(library_path)
        if not self.library_path.is_file():
            raise FileNotFoundError(f"C++ reference library not built: {self.library_path}")
        self.library = ctypes.CDLL(str(self.library_path))
        self._configure_functions()
        self.handle = self.library.teleop_reference_shaper_create()
        if not self.handle:
            raise MemoryError("unable to allocate C++ reference shaper")
        self.dof = 0
        self.epoch = 0

    def _configure_functions(self) -> None:
        lib = self.library
        lib.teleop_reference_shaper_create.restype = ctypes.c_void_p
        lib.teleop_reference_shaper_destroy.argtypes = [ctypes.c_void_p]
        lib.teleop_reference_shaper_initialize.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(MeasuredJointStateV1),
            ctypes.POINTER(JointDynamicLimitsV1),
            ctypes.c_int64,
            ctypes.POINTER(ValidationResult),
        ]
        lib.teleop_reference_shaper_replace_target.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(AcceptedJointTargetV1),
            ctypes.c_int64,
            ctypes.POINTER(ValidationResult),
        ]
        lib.teleop_reference_shaper_tick.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int64,
            ctypes.POINTER(ShapedJointCommandV1),
            ctypes.POINTER(ValidationResult),
        ]
        lib.teleop_reference_shaper_request_stop.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint64,
            ctypes.c_uint8,
            ctypes.c_int64,
            ctypes.POINTER(ValidationResult),
        ]
        lib.teleop_reference_shaper_hard_stop.argtypes = [
            ctypes.c_void_p, ctypes.c_uint8, ctypes.c_int64
        ]
        lib.teleop_reference_shaper_snapshot.argtypes = [
            ctypes.c_void_p, ctypes.POINTER(ShaperSnapshot)
        ]

    def close(self) -> None:
        if self.handle:
            self.library.teleop_reference_shaper_destroy(self.handle)
            self.handle = None

    def __enter__(self) -> CppReferenceShaper:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def __del__(self) -> None:
        handle = getattr(self, "handle", None)
        if handle:
            self.library.teleop_reference_shaper_destroy(handle)
            self.handle = None

    @staticmethod
    def _check(code: int, validation: ValidationResult, operation: str) -> OperationCode:
        result = OperationCode(code)
        if result not in (OperationCode.OK, OperationCode.COMPLETED, OperationCode.ALREADY_REQUESTED):
            raise RuntimeError(
                f"C++ {operation} failed: operation={result.name} "
                f"validation_error={validation.error} field={validation.field} index={validation.index}"
            )
        return result

    def initialize(
        self,
        *,
        position_rad: Iterable[float],
        velocity_rad_s: Iterable[float],
        acceleration_rad_s2: Iterable[float],
        minimum_position_rad: Iterable[float],
        maximum_position_rad: Iterable[float],
        maximum_velocity_rad_s: Iterable[float],
        maximum_acceleration_rad_s2: Iterable[float],
        maximum_jerk_rad_s3: Iterable[float],
        now_ns: int,
        safety_epoch: int = 1,
    ) -> None:
        arrays = [tuple(float(value) for value in values) for values in (
            position_rad, velocity_rad_s, acceleration_rad_s2, minimum_position_rad,
            maximum_position_rad, maximum_velocity_rad_s, maximum_acceleration_rad_s2,
            maximum_jerk_rad_s3,
        )]
        dof = len(arrays[0])
        if not 1 <= dof <= MAX_DOF or any(len(values) != dof for values in arrays):
            raise ValueError("all shaper arrays must use the same supported DOF")
        if any(not all(math.isfinite(value) for value in values) for values in arrays):
            raise ValueError("all shaper values must be finite")
        measured = MeasuredJointStateV1()
        measured.header = _header(MeasuredJointStateV1)
        measured.state_sequence = 1
        measured.safety_epoch = safety_epoch
        measured.measured_monotonic_ns = now_ns
        measured.dof = dof
        measured.validity = 1
        limits = JointDynamicLimitsV1()
        limits.header = _header(JointDynamicLimitsV1)
        limits.dof = dof
        for index in range(dof):
            measured.position_rad[index] = arrays[0][index]
            measured.velocity_rad_s[index] = arrays[1][index]
            measured.acceleration_rad_s2[index] = arrays[2][index]
            limits.minimum_position_rad[index] = arrays[3][index]
            limits.maximum_position_rad[index] = arrays[4][index]
            limits.maximum_velocity_rad_s[index] = arrays[5][index]
            limits.maximum_acceleration_rad_s2[index] = arrays[6][index]
            limits.maximum_jerk_rad_s3[index] = arrays[7][index]
        validation = ValidationResult()
        code = self.library.teleop_reference_shaper_initialize(
            self.handle, ctypes.byref(measured), ctypes.byref(limits), now_ns,
            ctypes.byref(validation),
        )
        self._check(code, validation, "initialize")
        self.dof = dof
        self.epoch = safety_epoch

    def replace_target(
        self,
        position_rad: Iterable[float],
        *,
        sequence: int,
        source_monotonic_ns: int,
        accepted_monotonic_ns: int,
        valid_until_monotonic_ns: int,
        validity: TargetValidity = TargetValidity.ACCEPTED,
        engagement: EngagementState = EngagementState.ENGAGED,
    ) -> OperationCode:
        values = _values(position_rad, self.dof)
        target = AcceptedJointTargetV1()
        target.header = _header(AcceptedJointTargetV1)
        target.sequence = sequence
        target.safety_epoch = self.epoch
        target.source_monotonic_ns = source_monotonic_ns
        target.accepted_monotonic_ns = accepted_monotonic_ns
        target.valid_until_monotonic_ns = valid_until_monotonic_ns
        target.dof = self.dof
        target.engagement = engagement
        target.validity = validity
        if validity == TargetValidity.ACCEPTED:
            for index, value in enumerate(values):
                target.position_rad[index] = value
        validation = ValidationResult()
        code = self.library.teleop_reference_shaper_replace_target(
            self.handle, ctypes.byref(target), accepted_monotonic_ns,
            ctypes.byref(validation),
        )
        return self._check(code, validation, "replace_target")

    def tick(self, now_ns: int) -> CppShaperPoint:
        output = ShapedJointCommandV1()
        validation = ValidationResult()
        code = self.library.teleop_reference_shaper_tick(
            self.handle, now_ns, ctypes.byref(output), ctypes.byref(validation)
        )
        self._check(code, validation, "tick")
        return CppShaperPoint(
            output_sequence=output.output_sequence,
            source_sequence=output.source_sequence,
            output_mode=OutputMode(output.output_mode),
            stop_reason=StopReason(output.stop_reason),
            position_rad=tuple(output.position_rad[: self.dof]),
            velocity_rad_s=tuple(output.velocity_rad_s[: self.dof]),
            acceleration_rad_s2=tuple(output.acceleration_rad_s2[: self.dof]),
        )

    def request_controlled_stop(
        self, *, release_sequence: int, now_ns: int,
        reason: StopReason = StopReason.CLUTCH_RELEASE,
    ) -> OperationCode:
        validation = ValidationResult()
        code = self.library.teleop_reference_shaper_request_stop(
            self.handle, release_sequence, reason, now_ns, ctypes.byref(validation)
        )
        return self._check(code, validation, "request_controlled_stop")

    def hard_stop(self, reason: StopReason, now_ns: int) -> None:
        self.library.teleop_reference_shaper_hard_stop(self.handle, reason, now_ns)

    def snapshot(self) -> CppShaperSnapshot:
        value = ShaperSnapshot()
        if not self.library.teleop_reference_shaper_snapshot(
            self.handle, ctypes.byref(value)
        ):
            raise RuntimeError("unable to read C++ shaper snapshot")
        return CppShaperSnapshot(
            mode=value.mode,
            safety_epoch=value.safety_epoch,
            last_input_sequence=value.last_input_sequence,
            source_sequence=value.source_sequence,
            output_sequence=value.output_sequence,
            release_sequence=value.release_sequence,
            stop_reason=StopReason(value.stop_reason),
            brake_planning_failure=BrakePlanningFailure(value.brake_planning_failure),
            brake_planning_failure_axis=value.brake_planning_failure_axis,
            acceleration_neutralization_axis_count=(
                value.acceleration_neutralization_axis_count
            ),
            position_rad=tuple(value.position_rad[: self.dof]),
            velocity_rad_s=tuple(value.velocity_rad_s[: self.dof]),
            acceleration_rad_s2=tuple(value.acceleration_rad_s2[: self.dof]),
        )


def default_cpp_library(repository_root: Path) -> Path:
    return repository_root / "build/teleop_shaping/libteleop_shaping_c_api.so"
