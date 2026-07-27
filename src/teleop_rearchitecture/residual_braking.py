"""Deterministic residual-acceleration braking sweep for the C++ reference core."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from .cpp_shaping import CppReferenceShaper, OutputMode, StopReason
from .unified_evaluator import PalmModel


PERIOD_NS = 8_000_000
PERIOD_S = PERIOD_NS / 1e9


def _sign(value: float, tolerance: float = 1e-10) -> int:
    return 1 if value > tolerance else -1 if value < -tolerance else 0


def _reversals(values: list[float]) -> int:
    signs = [sign for value in values if (sign := _sign(value))]
    return sum(left != right for left, right in zip(signs, signs[1:]))


def _case(
    case_id: str,
    velocity: tuple[float, ...],
    acceleration: tuple[float, ...],
    *,
    position: tuple[float, ...] = (0.0,) * 6,
    jerk: tuple[float, ...] = (50.0,) * 6,
    expected_failure: str | None = None,
) -> dict[str, Any]:
    return {
        "case_id": case_id,
        "position_rad": list(position),
        "velocity_rad_s": list(velocity),
        "acceleration_rad_s2": list(acceleration),
        "maximum_jerk_rad_s3": list(jerk),
        "expected_failure": expected_failure,
    }


def build_residual_acceleration_cases() -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    velocities = (0.0, 1e-6, -1e-6, 1e-4, -1e-4, 1e-3, -1e-3, 1e-2, -1e-2)
    accelerations = (0.0, 0.1, -0.1, 0.5, -0.5, 1.0, -1.0, 4.0, -4.0, 12.0, -12.0)
    for velocity in velocities:
        for acceleration in accelerations:
            dq = [0.0] * 6
            ddq = [0.0] * 6
            dq[1], ddq[1] = velocity, acceleration
            cases.append(_case(f"j2_v{velocity:+g}_a{acceleration:+g}", tuple(dq), tuple(ddq)))

    for jerk, acceleration in ((20.0, 0.159), (20.0, 0.16), (20.0, 0.161),
                               (50.0, 0.395), (50.0, 0.4), (50.0, 0.405),
                               (100.0, 0.795), (100.0, 0.8), (100.0, 0.805)):
        ddq = [0.0] * 6
        bounds = [50.0] * 6
        ddq[1], bounds[1] = acceleration, jerk
        cases.append(_case(f"quantized_j{jerk:g}_a{acceleration:g}", (0.0,) * 6,
                           tuple(ddq), jerk=tuple(bounds)))

    cases.extend([
        _case("mixed_six_axis", (0.01, -0.008, 0.004, -0.002, 1e-4, -1e-6),
              (1.0, -0.5, -1.0, 4.0, -0.1, 0.1)),
        _case("dominant_shoulder", (0.01, 0, 0, 0, 0, 0), (4.0, 0, 0, 0, 0, 0)),
        _case("dominant_wrist", (0, 0, 0, 0, 0, -0.01), (0, 0, 0, 0, 0, -4.0)),
        _case("upper_limit_outward_unplannable", (0,) * 6, (1, 0, 0, 0, 0, 0),
              position=(2.9999, 0, 0, 0, 0, 0), expected_failure="POSITION_LIMIT"),
        _case("lower_limit_outward_unplannable", (0,) * 6, (-1, 0, 0, 0, 0, 0),
              position=(-2.9999, 0, 0, 0, 0, 0), expected_failure="POSITION_LIMIT"),
        _case("upper_limit_inward", (-1e-3, 0, 0, 0, 0, 0), (-0.1, 0, 0, 0, 0, 0),
              position=(2.9999, 0, 0, 0, 0, 0)),
        _case("lower_limit_inward", (1e-3, 0, 0, 0, 0, 0), (0.1, 0, 0, 0, 0, 0),
              position=(-2.9999, 0, 0, 0, 0, 0)),
    ])
    return cases


def _run_case(case: dict[str, Any], library_path: Path, palm: PalmModel) -> dict[str, Any]:
    start_ns = 10_000_000_000
    q0 = tuple(float(value) for value in case["position_rad"])
    v0 = tuple(float(value) for value in case["velocity_rad_s"])
    a0 = tuple(float(value) for value in case["acceleration_rad_s2"])
    jerk_limits = tuple(float(value) for value in case["maximum_jerk_rad_s3"])
    result: dict[str, Any] = dict(case)
    with CppReferenceShaper(library_path) as shaper:
        shaper.initialize(
            position_rad=q0,
            velocity_rad_s=v0,
            acceleration_rad_s2=a0,
            minimum_position_rad=(-3.0,) * 6,
            maximum_position_rad=(3.0,) * 6,
            maximum_velocity_rad_s=(math.pi,) * 6,
            maximum_acceleration_rad_s2=(4 * math.pi,) * 6,
            maximum_jerk_rad_s3=jerk_limits,
            now_ns=start_ns,
            safety_epoch=1,
        )
        shaper.replace_target(
            q0,
            sequence=1,
            source_monotonic_ns=start_ns,
            accepted_monotonic_ns=start_ns,
            valid_until_monotonic_ns=start_ns + 10_000_000_000,
        )
        try:
            shaper.request_controlled_stop(
                release_sequence=2, now_ns=start_ns, reason=StopReason.CLUTCH_RELEASE
            )
        except RuntimeError:
            snapshot = shaper.snapshot()
            result.update({
                "completion": False,
                "planning_failure_reason": snapshot.brake_planning_failure.name,
                "planning_failure_axis": snapshot.brake_planning_failure_axis,
                "acceleration_neutralization_axis_count": 0,
                "stop_time_ms": None,
                "maximum_velocity_excursion_rad_s": None,
                "maximum_joint_displacement_rad": None,
                "maximum_palm_displacement_m": None,
                "direction_consistency": None,
                "reversals": None,
                "peak_velocity_rad_s": None,
                "peak_acceleration_rad_s2": None,
                "peak_jerk_rad_s3": None,
            })
            return result
        neutralized = shaper.snapshot().acceleration_neutralization_axis_count
        positions = [q0]
        velocities = [v0]
        accelerations = [a0]
        stopped_tick: int | None = None
        for tick in range(1_250):
            point = shaper.tick(start_ns + tick * PERIOD_NS)
            positions.append(point.position_rad)
            velocities.append(point.velocity_rad_s)
            accelerations.append(point.acceleration_rad_s2)
            if point.output_mode is OutputMode.STOPPED:
                stopped_tick = tick
                break
        if stopped_tick is None:
            result.update({"completion": False, "planning_failure_reason": "TIMEOUT"})
            return result

    palm0, _ = palm.pose(q0)
    palm_displacements = [float(np.linalg.norm(palm.pose(q)[0] - palm0)) for q in positions]
    per_axis_reversals = [_reversals([row[axis] for row in velocities]) for axis in range(6)]
    expected_reversals = []
    for axis in range(6):
        neutral_time = abs(a0[axis]) / jerk_limits[axis]
        residual = v0[axis] + 0.5 * a0[axis] * neutral_time
        initial_sign = _sign(v0[axis])
        residual_sign = _sign(residual)
        expected_reversals.append(1 if initial_sign and residual_sign and initial_sign != residual_sign else 0)
    finite_difference_jerk = [
        tuple((right[axis] - left[axis]) / PERIOD_S for axis in range(6))
        for left, right in zip(accelerations, accelerations[1:])
    ]
    result.update({
        "completion": True,
        "planning_failure_reason": "NONE",
        "planning_failure_axis": None,
        "acceleration_neutralization_axis_count": neutralized,
        "stop_time_ms": stopped_tick * PERIOD_NS / 1e6,
        "maximum_velocity_excursion_rad_s": max(
            abs(velocities[index][axis] - v0[axis])
            for index in range(len(velocities)) for axis in range(6)
        ),
        "maximum_joint_displacement_rad": max(
            abs(positions[index][axis] - q0[axis])
            for index in range(len(positions)) for axis in range(6)
        ),
        "maximum_palm_displacement_m": max(palm_displacements),
        "direction_consistency": all(
            actual <= expected for actual, expected in zip(per_axis_reversals, expected_reversals)
        ),
        "reversals": per_axis_reversals,
        "expected_reversals": expected_reversals,
        "peak_velocity_rad_s": max(abs(value) for row in velocities for value in row),
        "peak_acceleration_rad_s2": max(abs(value) for row in accelerations for value in row),
        "peak_jerk_rad_s3": max(abs(value) for row in finite_difference_jerk for value in row),
        "final_velocity_rad_s": list(velocities[-1]),
        "final_acceleration_rad_s2": list(accelerations[-1]),
    })
    return result


def run_residual_acceleration_sweep(library_path: Path, model_path: Path) -> dict[str, Any]:
    cases = build_residual_acceleration_cases()
    palm = PalmModel(model_path)
    results = [_run_case(case, library_path, palm) for case in cases]
    unexpected_failures = [
        row["case_id"] for row in results
        if not row["completion"] and row["planning_failure_reason"] != row["expected_failure"]
    ]
    expected_failures = [row for row in results if row["expected_failure"] is not None]
    completed = [row for row in results if row["completion"]]
    canonical_cases = json.dumps(cases, sort_keys=True, separators=(",", ":")).encode()
    return {
        "schema_version": "teleop_residual_acceleration_stop_sweep.v1",
        "period_s": PERIOD_S,
        "input_case_sha256": hashlib.sha256(canonical_cases).hexdigest(),
        "case_count": len(results),
        "summary": {
            "completed_count": len(completed),
            "expected_unplannable_count": sum(not row["completion"] for row in expected_failures),
            "unexpected_failure_count": len(unexpected_failures),
            "unexpected_failure_cases": unexpected_failures,
            "direction_consistent_count": sum(row["direction_consistency"] is True for row in completed),
            "maximum_stop_time_ms": max(row["stop_time_ms"] for row in completed),
            "maximum_velocity_excursion_rad_s": max(
                row["maximum_velocity_excursion_rad_s"] for row in completed
            ),
            "maximum_joint_displacement_rad": max(
                row["maximum_joint_displacement_rad"] for row in completed
            ),
            "maximum_palm_displacement_m": max(
                row["maximum_palm_displacement_m"] for row in completed
            ),
            "maximum_peak_velocity_rad_s": max(row["peak_velocity_rad_s"] for row in completed),
            "maximum_peak_acceleration_rad_s2": max(
                row["peak_acceleration_rad_s2"] for row in completed
            ),
            "maximum_peak_jerk_rad_s3": max(row["peak_jerk_rad_s3"] for row in completed),
        },
        "limitations": [
            "Offline C++ reference core only; no scheduler, IPC, SDK, controller, or hardware.",
            "Palm displacement uses the checked-in rh56_R_hand_base_link MuJoCo model frame.",
            "Expected position-limit failures are fail-closed evidence, not completed stops.",
        ],
        "cases": results,
    }
