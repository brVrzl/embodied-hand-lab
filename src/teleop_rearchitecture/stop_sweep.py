"""Deterministic offline controlled-stop policy sweep.

The sweep starts from synthetic joint position/velocity/acceleration states and
never imports a robot adapter or hardware SDK.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Literal, Sequence

import numpy as np
import ruckig

from .cpp_shaping import CppReferenceShaper, OutputMode, StopReason
from .unified_evaluator import Joint, PalmModel, PERIOD_S, _joint, _percentiles


StopPolicy = Literal[
    "stopping_point_tracking",
    "explicit_jerk_limited_zero_velocity",
    "cpp_explicit_jerk_limited_zero_velocity",
    "adaptive_critically_damped",
]
POLICIES: tuple[StopPolicy, ...] = (
    "stopping_point_tracking",
    "explicit_jerk_limited_zero_velocity",
    "adaptive_critically_damped",
)


@dataclass(frozen=True, slots=True)
class StopLimits:
    period_s: float = PERIOD_S
    maximum_velocity_rad_s: float = math.pi
    maximum_acceleration_rad_s2: float = 4.0 * math.pi
    maximum_jerk_rad_s3: float = 50.0
    maximum_duration_s: float = 8.0


@dataclass(frozen=True, slots=True)
class ReleaseState:
    state_id: str
    nominal_peak_velocity_rad_s: float
    state_class: str
    position_rad: Joint
    velocity_rad_s: Joint
    acceleration_rad_s2: Joint
    previous_acceleration_rad_s2: Joint
    dominant_joint: str
    provenance: str = "deterministic synthetic release state; not Quest data"


@dataclass(frozen=True, slots=True)
class _StopPoint:
    position: Joint
    velocity: Joint
    acceleration: Joint
    jerk: Joint


BASE_POSITION: Joint = (-1.2, -0.6, -1.4, 0.2, 1.0, -0.25)


def _axis_state(
    speed: float,
    state_class: str,
    *,
    axis: int,
    direction: float = 1.0,
    acceleration_scale: float = 0.0,
    previous_delta: float = 0.0,
    dominant_joint: str,
) -> ReleaseState:
    velocity = [0.0] * 6
    acceleration = [0.0] * 6
    previous = [0.0] * 6
    velocity[axis] = direction * speed
    acceleration_magnitude = min(1.0, max(0.1, speed * 2.0)) * acceleration_scale
    acceleration[axis] = direction * acceleration_magnitude
    previous[axis] = acceleration[axis] - direction * previous_delta
    return ReleaseState(
        state_id=f"v{speed:.2f}_{state_class}",
        nominal_peak_velocity_rad_s=speed,
        state_class=state_class,
        position_rad=BASE_POSITION,
        velocity_rad_s=_joint(velocity),
        acceleration_rad_s2=_joint(acceleration),
        previous_acceleration_rad_s2=_joint(previous),
        dominant_joint=dominant_joint,
    )


def build_release_state_matrix() -> tuple[ReleaseState, ...]:
    states: list[ReleaseState] = []
    for speed in (0.02, 0.05, 0.10, 0.25, 0.50, 1.00):
        states.extend(
            [
                _axis_state(speed, "zero_acceleration", axis=1, dominant_joint="J2"),
                _axis_state(speed, "positive_acceleration", axis=1, acceleration_scale=1.0, dominant_joint="J2"),
                _axis_state(speed, "negative_acceleration", axis=1, acceleration_scale=-1.0, dominant_joint="J2"),
                _axis_state(
                    speed, "jerk_ramp_up", axis=1, acceleration_scale=0.5,
                    previous_delta=50.0 * PERIOD_S, dominant_joint="J2",
                ),
                _axis_state(
                    speed, "jerk_ramp_down", axis=1, acceleration_scale=-0.5,
                    previous_delta=-50.0 * PERIOD_S, dominant_joint="J2",
                ),
                _axis_state(speed, "after_target_replacement", axis=1, acceleration_scale=0.25, dominant_joint="J2"),
                _axis_state(
                    speed, "after_direction_reversal", axis=1, direction=-1.0,
                    acceleration_scale=-0.5, dominant_joint="J2",
                ),
                ReleaseState(
                    state_id=f"v{speed:.2f}_mixed_six_axis",
                    nominal_peak_velocity_rad_s=speed,
                    state_class="mixed_six_axis",
                    position_rad=BASE_POSITION,
                    velocity_rad_s=_joint((speed, -0.8 * speed, 0.6 * speed, -0.4 * speed, 0.3 * speed, -0.9 * speed)),
                    acceleration_rad_s2=_joint((0.0, 0.2, -0.2, 0.1, -0.1, 0.0)),
                    previous_acceleration_rad_s2=_joint((-0.4, 0.6, -0.6, 0.5, -0.5, 0.4)),
                    dominant_joint="mixed",
                ),
                _axis_state(speed, "dominant_wrist", axis=5, dominant_joint="J6"),
                _axis_state(speed, "dominant_shoulder", axis=0, dominant_joint="J1"),
            ]
        )
    return tuple(states)


def _clip(value: float, magnitude: float) -> float:
    return max(-magnitude, min(magnitude, value))


def _tracking_policy_points(
    state: ReleaseState,
    *,
    limits: StopLimits,
    adaptive: bool,
) -> list[_StopPoint]:
    position = state.position_rad
    velocity = state.velocity_rad_s
    acceleration = state.acceleration_rad_s2
    target = _joint(
        q + math.copysign(v * v / (2.0 * limits.maximum_acceleration_rad_s2), v)
        if abs(v) > 0.0 else q
        for q, v in zip(position, velocity, strict=True)
    )
    points: list[_StopPoint] = []
    maximum_steps = int(limits.maximum_duration_s / limits.period_s)
    stable = 0
    tail_steps = int(round(0.25 / limits.period_s))
    for _ in range(maximum_steps):
        desired: list[float] = []
        for q, v, target_q, release_v in zip(position, velocity, target, state.velocity_rad_s, strict=True):
            if adaptive:
                speed_fraction = min(1.0, abs(release_v) / 0.5)
                natural_frequency = 10.0 + 6.0 * speed_fraction
                value = natural_frequency**2 * (target_q - q) - 2.0 * natural_frequency * v
            else:
                value = 36.0 * (target_q - q) - 10.0 * v
            desired.append(_clip(value, limits.maximum_acceleration_rad_s2))
        next_acceleration = _joint(
            current + _clip(
                wanted - current,
                limits.maximum_jerk_rad_s3 * limits.period_s,
            )
            for current, wanted in zip(acceleration, desired, strict=True)
        )
        next_velocity = _joint(
            _clip(current + accel * limits.period_s, limits.maximum_velocity_rad_s)
            for current, accel in zip(velocity, next_acceleration, strict=True)
        )
        next_position = _joint(
            current + speed * limits.period_s
            for current, speed in zip(position, next_velocity, strict=True)
        )
        jerk = _joint(
            (new - old) / limits.period_s
            for new, old in zip(next_acceleration, acceleration, strict=True)
        )
        point = _StopPoint(next_position, next_velocity, next_acceleration, jerk)
        points.append(point)
        position, velocity, acceleration = next_position, next_velocity, next_acceleration
        if max(map(abs, velocity)) < 2e-5 and max(map(abs, acceleration)) < 2e-4:
            stable += 1
            if stable >= tail_steps:
                break
        else:
            stable = 0
    return points


def _ruckig_policy_points(
    state: ReleaseState,
    *,
    limits: StopLimits,
) -> list[_StopPoint]:
    otg = ruckig.Ruckig(6, limits.period_s)
    input_parameter = ruckig.InputParameter(6)
    output_parameter = ruckig.OutputParameter(6)
    input_parameter.current_position = list(state.position_rad)
    input_parameter.current_velocity = list(state.velocity_rad_s)
    input_parameter.current_acceleration = list(state.acceleration_rad_s2)
    input_parameter.target_velocity = [0.0] * 6
    input_parameter.target_acceleration = [0.0] * 6
    input_parameter.max_velocity = [limits.maximum_velocity_rad_s] * 6
    input_parameter.max_acceleration = [limits.maximum_acceleration_rad_s2] * 6
    input_parameter.max_jerk = [limits.maximum_jerk_rad_s3] * 6
    input_parameter.control_interface = ruckig.ControlInterface.Velocity
    input_parameter.synchronization = ruckig.Synchronization.Time
    input_parameter.duration_discretization = ruckig.DurationDiscretization.Discrete
    previous_acceleration = state.acceleration_rad_s2
    points: list[_StopPoint] = []
    maximum_steps = int(limits.maximum_duration_s / limits.period_s)
    finished = False
    tail_steps = int(round(0.25 / limits.period_s))
    for _ in range(maximum_steps):
        if finished:
            point = _StopPoint(
                points[-1].position,
                _joint((0.0,) * 6),
                _joint((0.0,) * 6),
                _joint((0.0,) * 6),
            )
        else:
            if not otg.validate_input(input_parameter, False, True):
                raise RuntimeError(f"Ruckig rejected stop state {state.state_id}")
            result = otg.update(input_parameter, output_parameter)
            if int(result) < 0:
                raise RuntimeError(f"Ruckig stop failed for {state.state_id}: {result}")
            acceleration = _joint(output_parameter.new_acceleration)
            point = _StopPoint(
                _joint(output_parameter.new_position),
                _joint(output_parameter.new_velocity),
                acceleration,
                _joint(
                    (new - previous) / limits.period_s
                    for new, previous in zip(acceleration, previous_acceleration, strict=True)
                ),
            )
            previous_acceleration = acceleration
            output_parameter.pass_to_input(input_parameter)
            finished = int(result) == int(ruckig.Result.Finished)
        points.append(point)
        if finished and len(points) >= tail_steps and all(
            candidate.velocity == (0.0,) * 6 and candidate.acceleration == (0.0,) * 6
            for candidate in points[-tail_steps:]
        ):
            break
    return points


def _cpp_policy_points(
    state: ReleaseState,
    *,
    limits: StopLimits,
    library_path: Path,
) -> list[_StopPoint]:
    start_ns = 1_000_000_000
    previous_acceleration = state.acceleration_rad_s2
    points: list[_StopPoint] = []
    with CppReferenceShaper(library_path) as shaper:
        shaper.initialize(
            position_rad=state.position_rad,
            velocity_rad_s=state.velocity_rad_s,
            acceleration_rad_s2=state.acceleration_rad_s2,
            minimum_position_rad=(-3.2,) * 6,
            maximum_position_rad=(3.2,) * 6,
            maximum_velocity_rad_s=(limits.maximum_velocity_rad_s,) * 6,
            maximum_acceleration_rad_s2=(limits.maximum_acceleration_rad_s2,) * 6,
            maximum_jerk_rad_s3=(limits.maximum_jerk_rad_s3,) * 6,
            now_ns=start_ns,
            safety_epoch=1,
        )
        shaper.replace_target(
            state.position_rad,
            sequence=1,
            source_monotonic_ns=start_ns,
            accepted_monotonic_ns=start_ns,
            valid_until_monotonic_ns=start_ns + int(limits.maximum_duration_s * 1e9),
        )
        shaper.request_controlled_stop(
            release_sequence=2,
            now_ns=start_ns,
            reason=StopReason.CLUTCH_RELEASE,
        )
        maximum_steps = int(limits.maximum_duration_s / limits.period_s)
        tail_steps = int(round(0.25 / limits.period_s))
        stopped_at: int | None = None
        for index in range(maximum_steps):
            output = shaper.tick(start_ns + index * int(limits.period_s * 1e9))
            acceleration = _joint(output.acceleration_rad_s2)
            point = _StopPoint(
                _joint(output.position_rad),
                _joint(output.velocity_rad_s),
                acceleration,
                _joint(
                    (new - previous) / limits.period_s
                    for new, previous in zip(
                        acceleration, previous_acceleration, strict=True
                    )
                ),
            )
            points.append(point)
            previous_acceleration = acceleration
            if output.output_mode == OutputMode.STOPPED:
                stopped_at = index
            if stopped_at is not None and index - stopped_at + 1 >= tail_steps:
                break
    return points


def _completion(
    points: Sequence[_StopPoint],
    final_position: Joint,
    *,
    velocity_threshold: float,
    acceleration_threshold: float,
    position_threshold: float,
    period_s: float,
) -> dict[str, object]:
    qualifies = [
        max(map(abs, point.velocity)) <= velocity_threshold
        and max(map(abs, point.acceleration)) <= acceleration_threshold
        and max(abs(q - final) for q, final in zip(point.position, final_position, strict=True)) <= position_threshold
        for point in points
    ]
    first: int | None = None
    suffix = True
    for index in range(len(points) - 1, -1, -1):
        suffix = suffix and qualifies[index]
        if suffix:
            first = index
    return {
        "completed": first is not None,
        "time_ms": None if first is None else (first + 1) * period_s * 1000.0,
        "thresholds": {
            "velocity_rad_s": velocity_threshold,
            "acceleration_rad_s2": acceleration_threshold,
            "position_rad": position_threshold,
            "sustained_to_end": True,
        },
    }


def _minimum_time_bound(speed: float, limits: StopLimits) -> float:
    transition_speed = limits.maximum_acceleration_rad_s2**2 / limits.maximum_jerk_rad_s3
    if speed <= transition_speed:
        return 2.0 * math.sqrt(speed / limits.maximum_jerk_rad_s3)
    return (
        2.0 * limits.maximum_acceleration_rad_s2 / limits.maximum_jerk_rad_s3
        + (speed - transition_speed) / limits.maximum_acceleration_rad_s2
    )


def _simulate(
    state: ReleaseState,
    *,
    policy: StopPolicy,
    limits: StopLimits,
    cpp_library_path: Path | None = None,
) -> tuple[dict[str, object], list[_StopPoint]]:
    if policy == "explicit_jerk_limited_zero_velocity":
        points = _ruckig_policy_points(state, limits=limits)
    elif policy == "cpp_explicit_jerk_limited_zero_velocity":
        if cpp_library_path is None:
            raise ValueError("C++ stop policy requires an explicit built library path")
        points = _cpp_policy_points(state, limits=limits, library_path=cpp_library_path)
    else:
        points = _tracking_policy_points(
            state, limits=limits, adaptive=policy == "adaptive_critically_damped"
        )
    if not points:
        raise RuntimeError("stop policy emitted no points")
    final_position = points[-1].position
    strict = _completion(
        points,
        final_position,
        velocity_threshold=1e-3,
        acceleration_threshold=1e-2,
        position_threshold=1e-4,
        period_s=limits.period_s,
    )
    practical = _completion(
        points,
        final_position,
        velocity_threshold=5e-3,
        acceleration_threshold=5e-2,
        position_threshold=5e-4,
        period_s=limits.period_s,
    )
    velocity_limit = limits.maximum_velocity_rad_s + 1e-8
    acceleration_limit = limits.maximum_acceleration_rad_s2 + 1e-8
    jerk_limit = limits.maximum_jerk_rad_s3 + 1e-7
    violations = {
        "velocity": sum(abs(value) > velocity_limit for point in points for value in point.velocity),
        "acceleration": sum(abs(value) > acceleration_limit for point in points for value in point.acceleration),
        "jerk": sum(abs(value) > jerk_limit for point in points for value in point.jerk),
    }
    rebound = [0] * 6
    reversals = [0] * 6
    previous_sign = [int(value > 1e-6) - int(value < -1e-6) for value in state.velocity_rad_s]
    initial_sign = previous_sign.copy()
    for point in points:
        for joint, velocity in enumerate(point.velocity):
            sign = int(velocity > 1e-6) - int(velocity < -1e-6)
            if sign and previous_sign[joint] and sign != previous_sign[joint]:
                reversals[joint] += 1
            if sign and initial_sign[joint] and sign != initial_sign[joint]:
                rebound[joint] += 1
            if sign:
                previous_sign[joint] = sign
    strict_index = (
        None if strict["time_ms"] is None else int(round(float(strict["time_ms"]) / (limits.period_s * 1000.0))) - 1
    )
    post_completion_drift = (
        0.0
        if strict_index is None
        else max(
            max(abs(q - points[strict_index].position[joint]) for joint, q in enumerate(point.position))
            for point in points[strict_index:]
        )
    )
    result = {
        "state_id": state.state_id,
        "state_class": state.state_class,
        "nominal_peak_velocity_rad_s": state.nominal_peak_velocity_rad_s,
        "dominant_joint": state.dominant_joint,
        "release_state": {
            "position_rad": list(state.position_rad),
            "velocity_rad_s": list(state.velocity_rad_s),
            "acceleration_rad_s2": list(state.acceleration_rad_s2),
            "previous_acceleration_rad_s2": list(state.previous_acceleration_rad_s2),
        },
        "policy": policy,
        "sample_count": len(points),
        "position_jump_rad": 0.0,
        "strict_completion": strict,
        "practical_completion": practical,
        "joint_stop_displacement_rad": [
            max(abs(point.position[joint] - state.position_rad[joint]) for point in points)
            for joint in range(6)
        ],
        "maximum_velocity_rad_s": max(abs(value) for point in points for value in point.velocity),
        "maximum_acceleration_rad_s2": max(abs(value) for point in points for value in point.acceleration),
        "maximum_jerk_rad_s3": max(abs(value) for point in points for value in point.jerk),
        "limit_violations": violations,
        "velocity_reversals_per_joint": reversals,
        "rebound_samples_per_joint": rebound,
        "direction_consistent": sum(rebound) == 0,
        "post_completion_drift_rad": post_completion_drift,
        "zero_acceleration_theoretical_minimum_time_ms": (
            _minimum_time_bound(state.nominal_peak_velocity_rad_s, limits) * 1000.0
            if max(map(abs, state.acceleration_rad_s2)) <= 1e-12
            else None
        ),
        "provenance": state.provenance,
    }
    return result, points


def simulate_controlled_stop(
    state: ReleaseState,
    *,
    policy: StopPolicy,
    limits: StopLimits = StopLimits(),
    cpp_library_path: Path | None = None,
) -> dict[str, object]:
    result, _ = _simulate(
        state, policy=policy, limits=limits, cpp_library_path=cpp_library_path
    )
    json.dumps(result, allow_nan=False)
    return result


def run_controlled_stop_sweep(
    *,
    model_path: Path,
    repository_commit: str,
    working_tree_dirty: bool,
    limits: StopLimits = StopLimits(),
    cpp_library_path: Path | None = None,
) -> dict[str, object]:
    model = PalmModel(model_path)
    results: list[dict[str, object]] = []
    policies: tuple[StopPolicy, ...] = (
        (*POLICIES, "cpp_explicit_jerk_limited_zero_velocity")
        if cpp_library_path is not None
        else POLICIES
    )
    for state in build_release_state_matrix():
        release_palm, _ = model.pose(state.position_rad)
        for policy in policies:
            result, points = _simulate(
                state,
                policy=policy,
                limits=limits,
                cpp_library_path=cpp_library_path,
            )
            result["palm_stop_displacement_m"] = max(
                float(np.linalg.norm(model.pose(point.position)[0] - release_palm))
                for point in points
            )
            results.append(result)
    summary: dict[str, object] = {}
    for policy in policies:
        rows = [row for row in results if row["policy"] == policy]
        strict_times = [
            float(row["strict_completion"]["time_ms"])  # type: ignore[index]
            for row in rows
            if row["strict_completion"]["time_ms"] is not None  # type: ignore[index]
        ]
        practical_times = [
            float(row["practical_completion"]["time_ms"])  # type: ignore[index]
            for row in rows
            if row["practical_completion"]["time_ms"] is not None  # type: ignore[index]
        ]
        summary[policy] = {
            "case_count": len(rows),
            "strict_completion_count": len(strict_times),
            "practical_completion_count": len(practical_times),
            "strict_stop_time_ms": _percentiles(strict_times),
            "practical_stop_time_ms": _percentiles(practical_times),
            "maximum_joint_stop_displacement_rad": max(
                max(row["joint_stop_displacement_rad"]) for row in rows  # type: ignore[arg-type]
            ),
            "maximum_palm_stop_displacement_m": max(float(row["palm_stop_displacement_m"]) for row in rows),
            "limit_violation_count": sum(
                sum(row["limit_violations"].values()) for row in rows  # type: ignore[union-attr]
            ),
            "direction_inconsistent_case_count": sum(not bool(row["direction_consistent"]) for row in rows),
            "maximum_post_completion_drift_rad": max(float(row["post_completion_drift_rad"]) for row in rows),
        }
    conformance: dict[str, object] | None = None
    if cpp_library_path is not None:
        python_rows = {
            str(row["state_id"]): row
            for row in results
            if row["policy"] == "explicit_jerk_limited_zero_velocity"
        }
        cpp_rows = {
            str(row["state_id"]): row
            for row in results
            if row["policy"] == "cpp_explicit_jerk_limited_zero_velocity"
        }
        comparisons: list[dict[str, object]] = []
        for state_id in sorted(python_rows):
            python_row = python_rows[state_id]
            cpp_row = cpp_rows[state_id]
            comparisons.append(
                {
                    "state_id": state_id,
                    "completion_equal": (
                        python_row["strict_completion"]["completed"]  # type: ignore[index]
                        == cpp_row["strict_completion"]["completed"]  # type: ignore[index]
                    ),
                    "strict_stop_time_delta_ms": abs(
                        float(python_row["strict_completion"]["time_ms"])  # type: ignore[index]
                        - float(cpp_row["strict_completion"]["time_ms"])  # type: ignore[index]
                    ),
                    "maximum_joint_displacement_delta_rad": max(
                        abs(float(left) - float(right))
                        for left, right in zip(
                            python_row["joint_stop_displacement_rad"],  # type: ignore[arg-type]
                            cpp_row["joint_stop_displacement_rad"],  # type: ignore[arg-type]
                            strict=True,
                        )
                    ),
                    "palm_displacement_delta_m": abs(
                        float(python_row["palm_stop_displacement_m"])
                        - float(cpp_row["palm_stop_displacement_m"])
                    ),
                    "direction_consistency_equal": (
                        python_row["direction_consistent"] == cpp_row["direction_consistent"]
                    ),
                    "velocity_peak_delta_rad_s": abs(
                        float(python_row["maximum_velocity_rad_s"])
                        - float(cpp_row["maximum_velocity_rad_s"])
                    ),
                    "acceleration_peak_delta_rad_s2": abs(
                        float(python_row["maximum_acceleration_rad_s2"])
                        - float(cpp_row["maximum_acceleration_rad_s2"])
                    ),
                    "jerk_peak_delta_rad_s3": abs(
                        float(python_row["maximum_jerk_rad_s3"])
                        - float(cpp_row["maximum_jerk_rad_s3"])
                    ),
                }
            )
        tolerances = {
            "strict_stop_time_delta_ms": 8.0,
            "maximum_joint_displacement_delta_rad": 1e-3,
            "palm_displacement_delta_m": 5e-4,
            "velocity_peak_delta_rad_s": 1e-3,
            "acceleration_peak_delta_rad_s2": 1.5,
            "jerk_peak_delta_rad_s3": 15.0,
        }
        conformance = {
            "classification": "offline envelope conformance; not tick-identical OTG equivalence",
            "python_backend": "Ruckig 0.19.4 velocity-interface time-synchronized discrete OTG",
            "cpp_backend": "independent analytic per-axis jerk-limited braking core",
            "tolerances": tolerances,
            "tolerance_basis": (
                "one 8 ms completion tick; <=1 mrad joint and <=0.5 mm palm "
                "envelope separation; derivative-peak tolerances distinguish "
                "the independent analytic profile from Ruckig rather than "
                "claiming tick-identical trajectories"
            ),
            "case_count": len(comparisons),
            "completion_mismatch_count": sum(
                not bool(row["completion_equal"]) for row in comparisons
            ),
            "direction_mismatch_count": sum(
                not bool(row["direction_consistency_equal"]) for row in comparisons
            ),
            "maxima": {
                field: max(float(row[field]) for row in comparisons)
                for field in tolerances
            },
            "within_tolerance": all(
                bool(row["completion_equal"])
                and bool(row["direction_consistency_equal"])
                and all(float(row[field]) <= limit + 1e-12 for field, limit in tolerances.items())
                for row in comparisons
            ),
            "cases": comparisons,
        }
    report: dict[str, object] = {
        "schema_version": "teleop_controlled_stop_sweep.v1",
        "repository_commit": repository_commit,
        "working_tree_dirty": working_tree_dirty,
        "period_s": limits.period_s,
        "limits": asdict(limits),
        "release_state_count": len(build_release_state_matrix()),
        "release_velocity_bands_rad_s": [0.02, 0.05, 0.10, 0.25, 0.50, 1.00],
        "completion_policies": {
            "strict": {"velocity_rad_s": 1e-3, "acceleration_rad_s2": 1e-2, "position_rad": 1e-4},
            "practical": {"velocity_rad_s": 5e-3, "acceleration_rad_s2": 5e-2, "position_rad": 5e-4},
        },
        "policy_definitions": {
            "stopping_point_tracking": "current Candidate C fixed braking-point target with 36/10 tracking law",
            "explicit_jerk_limited_zero_velocity": "Ruckig velocity-interface target v=0/a=0; position emerges from bounded braking trajectory",
            "cpp_explicit_jerk_limited_zero_velocity": "independent C++ analytic per-axis jerk-limited v=0/a=0 braking; only present when an explicit built library is supplied",
            "adaptive_critically_damped": "fixed braking point with speed-adaptive critical damping and the same discrete jerk/acceleration clamps",
        },
        "policy_summary": summary,
        "cases": results,
        "model": model.metadata(),
        "cpp_python_explicit_braking_conformance": conformance,
        "frame": "rh56_R_hand_base_link palm model",
        "limitations": [
            "synthetic deterministic joint release states; not Quest or physical robot data",
            "independent-axis command model; palm displacement is MuJoCo FK only",
            "no scheduler, IPC, SDK, controller, network, or plant dynamics",
        ],
        "physical_connections": False,
        "physical_commands": False,
        "jaka_sdk_loaded": False,
        "rh56_commands": 0,
    }
    json.dumps(report, allow_nan=False)
    return report
