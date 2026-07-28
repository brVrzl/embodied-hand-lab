from __future__ import annotations

import ctypes
import math
from pathlib import Path
import random
import subprocess

import pytest

from teleop_rearchitecture.cpp_shaping import (
    ABI_MAGIC,
    ABI_VERSION,
    AcceptedJointTargetV1,
    AbiHeader,
    CppReferenceShaper,
    JointDynamicLimitsV1,
    MeasuredJointStateV1,
    OutputMode,
    ShapedJointCommandV1,
    StopReason,
    TargetValidity,
    TransportHealthV1,
    ValidationContext,
    ValidationResult,
    default_cpp_library,
)
from teleop_rearchitecture.shapers import JerkBoundedPositionServo, ShaperLimits
from teleop_rearchitecture.stop_sweep import run_controlled_stop_sweep
from teleop_rearchitecture.unified_evaluator import (
    evaluate_unified_fixture,
    load_accepted_joint_targets,
)


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "build/teleop_shaping"
LIBRARY = default_cpp_library(ROOT)
PERIOD_NS = 8_000_000
MODEL = ROOT / "data/sim_assets/jaka_rh56.xml"
FIXTURE = ROOT / (
    "docs/history/incidents/quest_jaka_20260722_23/measurements/"
    "jaka_edg_sim_initial_accepted_targets_20260722.jsonl"
)


@pytest.fixture(scope="module", autouse=True)
def build_cpp_reference() -> None:
    subprocess.run(
        ["cmake", "-S", str(ROOT / "native/teleop_shaping"), "-B", str(BUILD),
         "-DCMAKE_BUILD_TYPE=Release"],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(BUILD), "-j2"], check=True)


def _configure_validation(library: ctypes.CDLL) -> None:
    library.teleop_abi_sizeof.argtypes = [ctypes.c_int]
    library.teleop_abi_sizeof.restype = ctypes.c_size_t
    library.teleop_abi_alignof.argtypes = [ctypes.c_int]
    library.teleop_abi_alignof.restype = ctypes.c_size_t
    library.teleop_validate_target.argtypes = [
        ctypes.POINTER(AcceptedJointTargetV1), ctypes.POINTER(ValidationContext)
    ]
    library.teleop_validate_target.restype = ValidationResult


def _valid_target(now_ns: int = 1_000_000_000) -> AcceptedJointTargetV1:
    target = AcceptedJointTargetV1()
    target.header = AbiHeader(ABI_MAGIC, ABI_VERSION, ctypes.sizeof(AcceptedJointTargetV1), 1)
    target.sequence = 1
    target.safety_epoch = 7
    target.source_monotonic_ns = now_ns
    target.accepted_monotonic_ns = now_ns
    target.valid_until_monotonic_ns = now_ns + 1_000_000_000
    target.dof = 6
    target.engagement = 1
    target.validity = TargetValidity.ACCEPTED
    target.position_rad[1] = 0.1
    return target


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


def test_python_driven_validator_fuzz_is_fail_closed() -> None:
    seed = 0xC0FFEE
    generator = random.Random(seed)
    library = ctypes.CDLL(str(LIBRARY))
    _configure_validation(library)
    now = 1_000_000_000
    context = ValidationContext(now, 6, 7, 0)
    for _ in range(2_000):
        target = _valid_target(now)
        target.header.schema_version = generator.randrange(0, 2**16)
        result = library.teleop_validate_target(ctypes.byref(target), ctypes.byref(context))
        if target.header.schema_version != ABI_VERSION:
            assert not result.ok, f"seed={seed} version={target.header.schema_version}"
        target = _valid_target(now)
        target.dof = generator.randrange(0, 256)
        result = library.teleop_validate_target(ctypes.byref(target), ctypes.byref(context))
        if target.dof != 6:
            assert not result.ok, f"seed={seed} dof={target.dof}"
        target = _valid_target(now)
        target.validity = generator.randrange(0, 256)
        result = library.teleop_validate_target(ctypes.byref(target), ctypes.byref(context))
        if target.validity > TargetValidity.REJECTED_KEEP_PREVIOUS:
            assert not result.ok, f"seed={seed} validity={target.validity}"
    for non_finite in (math.nan, math.inf, -math.inf):
        target = _valid_target(now)
        target.position_rad[3] = non_finite
        assert not library.teleop_validate_target(
            ctypes.byref(target), ctypes.byref(context)
        ).ok


def test_python_and_cpp_active_reference_conform_tick_by_tick() -> None:
    start = 2_000_000_000
    limits = ShaperLimits()
    python = JerkBoundedPositionServo((0.0,) * 6, limits)
    with CppReferenceShaper(LIBRARY) as cpp:
        _initialize(cpp, start)
        sequence = 0
        for tick in range(240):
            now = start + tick * PERIOD_NS
            if tick % 7 == 0:
                sequence += 1
                target = tuple(
                    0.02 * math.sin(0.13 * sequence + joint * 0.2)
                    for joint in range(6)
                )
                python.set_target(target, timestamp_ns=now)
                cpp.replace_target(
                    target,
                    sequence=sequence,
                    source_monotonic_ns=now,
                    accepted_monotonic_ns=now,
                    valid_until_monotonic_ns=now + 2_000_000_000,
                )
            python_point = python.tick()
            cpp_point = cpp.tick(now)
            assert cpp_point.output_mode == OutputMode.ACTIVE_TRACKING
            assert cpp_point.position_rad == pytest.approx(python_point.position_rad, abs=2e-15)
            assert cpp_point.velocity_rad_s == pytest.approx(python_point.velocity_rad_s, abs=2e-15)
            assert cpp_point.acceleration_rad_s2 == pytest.approx(
                python_point.acceleration_rad_s2, abs=2e-14
            )


@pytest.mark.parametrize("speed", [0.02, 0.05, 0.10, 0.25, 0.50, 1.00])
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


def test_unified_evaluator_cpp_active_reference_conforms_to_python() -> None:
    report = evaluate_unified_fixture(
        load_accepted_joint_targets(FIXTURE),
        model_path=MODEL,
        repository_commit="test-commit",
        working_tree_dirty=True,
        settling_duration_s=0.08,
        backends=("candidate_c_position_reference", "candidate_c_cpp_reference"),
        cpp_library_path=LIBRARY,
    )
    python, cpp = report["benchmarks"]
    assert cpp["backend"] == "candidate_c_cpp_reference"
    assert cpp["tracking"] == python["tracking"]
    assert cpp["interpolated_tracking"] == python["interpolated_tracking"]
    assert cpp["settling"] == python["settling"]
    for window in ("active", "settling"):
        for source in ("backend_reported", "output_position_finite_difference"):
            for metric in (
                "joint_velocity_rad_s_peak",
                "joint_acceleration_rad_s2_peak",
                "joint_jerk_rad_s3_peak",
            ):
                assert cpp["dynamics"][window][source][metric] == pytest.approx(
                    python["dynamics"][window][source][metric], abs=1e-10
                )
    assert cpp["mailbox"]["backlog"] == 0
    assert cpp["stop"]["completed"] is True


def test_full_cpp_python_controlled_stop_envelope_conforms() -> None:
    report = run_controlled_stop_sweep(
        model_path=MODEL,
        repository_commit="test-commit",
        working_tree_dirty=True,
        cpp_library_path=LIBRARY,
    )
    conformance = report["cpp_python_explicit_braking_conformance"]
    assert conformance["case_count"] == 60
    assert conformance["completion_mismatch_count"] == 0
    assert conformance["direction_mismatch_count"] == 0
    assert conformance["within_tolerance"] is True
    cpp_summary = report["policy_summary"]["cpp_explicit_jerk_limited_zero_velocity"]
    assert cpp_summary["strict_completion_count"] == 60
    assert cpp_summary["limit_violation_count"] == 0
