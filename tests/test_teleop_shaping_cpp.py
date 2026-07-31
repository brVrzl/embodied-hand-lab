from __future__ import annotations

import ctypes
import math
from pathlib import Path

import pytest

from teleop_rearchitecture.cpp_shaping import (
    AcceptedJointTargetV1,
    AbiHeader,
    CppReferenceShaper,
    JointDynamicLimitsV1,
    MeasuredJointStateV1,
    OutputMode,
    ShapedJointCommandV1,
    StopReason,
    TransportHealthV1,
    default_cpp_library,
)


ROOT = Path(__file__).resolve().parents[1]
LIBRARY = default_cpp_library(ROOT)
PERIOD_NS = 8_000_000


@pytest.fixture(scope="module", autouse=True)
def build_cpp_reference(teleop_shaping_library: Path) -> None:
    assert teleop_shaping_library == LIBRARY


def _configure_validation(library: ctypes.CDLL) -> None:
    library.teleop_abi_sizeof.argtypes = [ctypes.c_int]
    library.teleop_abi_sizeof.restype = ctypes.c_size_t
    library.teleop_abi_alignof.argtypes = [ctypes.c_int]
    library.teleop_abi_alignof.restype = ctypes.c_size_t


def _initialize(shaper: CppReferenceShaper, now_ns: int, *, velocity: tuple[float, ...] = (0.0,) * 6,
                acceleration: tuple[float, ...] = (0.0,) * 6) -> None:
    shaper.initialize(
        position_rad=(0.0,) * 6,
        velocity_rad_s=velocity,
        acceleration_rad_s2=acceleration,
        minimum_position_rad=(-3.0,) * 6,
        maximum_position_rad=(3.0,) * 6,
        maximum_velocity_rad_s=(math.pi,) * 6,
        maximum_acceleration_rad_s2=(4.0 * math.pi,) * 6,
        maximum_jerk_rad_s3=(50.0,) * 6,
        now_ns=now_ns,
        safety_epoch=7,
    )


def test_python_layout_matches_compiled_abi() -> None:
    library = ctypes.CDLL(str(LIBRARY))
    _configure_validation(library)
    assert library.teleop_abi_is_little_endian_host() == 1
    types = (
        AbiHeader,
        AcceptedJointTargetV1,
        MeasuredJointStateV1,
        JointDynamicLimitsV1,
        ShapedJointCommandV1,
        TransportHealthV1,
    )
    for kind, structure in enumerate(types):
        assert ctypes.sizeof(structure) == library.teleop_abi_sizeof(kind)
        assert ctypes.alignment(structure) == library.teleop_abi_alignof(kind)
    assert AcceptedJointTargetV1.sequence.offset == 16
    assert AcceptedJointTargetV1.position_rad.offset == 64
    assert ShapedJointCommandV1.position_rad.offset == 64


@pytest.mark.parametrize("speed", [0.02, 0.25, 1.00])
def test_cpp_explicit_braking_completes_without_reversal(speed: float) -> None:
    start = 3_000_000_000
    velocity = (0.0, speed, 0.0, 0.0, 0.0, 0.0)
    with CppReferenceShaper(LIBRARY) as cpp:
        _initialize(cpp, start, velocity=velocity)
        cpp.replace_target(
            (0.0,) * 6,
            sequence=1,
            source_monotonic_ns=start,
            accepted_monotonic_ns=start,
            valid_until_monotonic_ns=start + 2_000_000_000,
        )
        assert cpp.request_controlled_stop(
            release_sequence=2, now_ns=start, reason=StopReason.CLUTCH_RELEASE
        ).name == "OK"
        points = [cpp.tick(start + index * PERIOD_NS) for index in range(200)]
        stopped = next((index for index, point in enumerate(points)
                        if point.output_mode == OutputMode.STOPPED), None)
        assert stopped is not None
        active = points[: stopped + 1]
        assert all(point.velocity_rad_s[1] >= -1e-10 for point in active)
        assert max(abs(point.acceleration_rad_s2[1]) for point in active) <= 4 * math.pi + 1e-10
        accelerations = [0.0, *(point.acceleration_rad_s2[1] for point in active)]
        assert max(abs(right - left) / 0.008 for left, right in zip(
            accelerations, accelerations[1:]
        )) <= 50.0 + 1e-8


def test_cpp_hard_stop_preempts_and_emits_no_further_command() -> None:
    start = 4_000_000_000
    with CppReferenceShaper(LIBRARY) as cpp:
        _initialize(cpp, start)
        cpp.replace_target(
            (0.01,) * 6,
            sequence=1,
            source_monotonic_ns=start,
            accepted_monotonic_ns=start,
            valid_until_monotonic_ns=start + 1_000_000_000,
        )
        cpp.tick(start)
        cpp.hard_stop(StopReason.CONTROLLER_ALARM, start + 1)
        with pytest.raises(RuntimeError, match="TERMINAL_NO_OUTPUT"):
            cpp.tick(start + PERIOD_NS)
