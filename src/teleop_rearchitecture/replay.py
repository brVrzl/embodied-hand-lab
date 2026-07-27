"""Offline replay and common metrics for the rearchitecture prototypes."""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import time
from typing import Literal

import mujoco
import numpy as np

from .contracts import CommandState, JointCommand, LatestCommandMailbox
from .shapers import JerkBoundedPositionServo, ResolvedRateVelocityServo, ShaperLimits, ShaperPoint


Prototype = Literal["resolved_rate_velocity", "jerk_bounded_position"]


@dataclass(frozen=True, slots=True)
class ReplaySample:
    timestamp_ns: int
    joints: tuple[float, float, float, float, float, float]


def load_accepted_targets(path: Path) -> list[ReplaySample]:
    samples: list[ReplaySample] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if row.get("accepted") and row.get("output_applied"):
            joints = tuple(float(v) for v in row["accepted_joint_target_rad"])
            if len(joints) == 6:
                samples.append(ReplaySample(int(row["control_monotonic_ns"]), joints))
    if len(samples) < 2:
        raise ValueError(f"{path} does not contain at least two accepted targets")
    _validate_samples(samples)
    return samples


def _validate_samples(samples: list[ReplaySample]) -> None:
    if len(samples) < 2:
        raise ValueError("replay requires at least two target samples")
    if any(
        current.timestamp_ns <= previous.timestamp_ns
        for previous, current in zip(samples, samples[1:])
    ):
        raise ValueError("replay target timestamps must increase")


def causal_joint_target(
    samples: list[ReplaySample], timestamp_ns: int
) -> tuple[float, float, float, float, float, float]:
    """Return only the newest target that existed at ``timestamp_ns``."""

    _validate_samples(samples)
    index = bisect_right([sample.timestamp_ns for sample in samples], timestamp_ns) - 1
    if index < 0:
        raise ValueError("no causal target exists before the requested timestamp")
    return samples[index].joints


def interpolated_joint_target(
    samples: list[ReplaySample], timestamp_ns: int
) -> tuple[float, float, float, float, float, float]:
    """Return an explicitly non-causal timestamp-interpolated analysis target."""

    _validate_samples(samples)
    if timestamp_ns <= samples[0].timestamp_ns:
        return samples[0].joints
    if timestamp_ns >= samples[-1].timestamp_ns:
        return samples[-1].joints
    right = bisect_right([sample.timestamp_ns for sample in samples], timestamp_ns)
    left_sample, right_sample = samples[right - 1], samples[right]
    alpha = (timestamp_ns - left_sample.timestamp_ns) / (
        right_sample.timestamp_ns - left_sample.timestamp_ns
    )
    result = tuple(
        left + alpha * (right_value - left)
        for left, right_value in zip(
            left_sample.joints, right_sample.joints, strict=True
        )
    )
    return result  # type: ignore[return-value]


class _TcpForwardKinematics:
    def __init__(self, xml_path: Path) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)
        self.joint_qpos_ids = tuple(
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{index}")]
            for index in range(1, 7)
        )
        self.body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link"
        )
        if self.body_id < 0 or any(index < 0 for index in self.joint_qpos_ids):
            raise ValueError("expected Mini2 joints and RH56 palm body in model")

    def pose(self, joints: tuple[float, ...]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
        self.data.qpos[list(self.joint_qpos_ids)] = joints
        mujoco.mj_forward(self.model, self.data)
        position = tuple(float(value) for value in self.data.xpos[self.body_id])
        quat = np.zeros(4, dtype=float)
        mujoco.mju_mat2Quat(quat, self.data.xmat[self.body_id])
        return position, tuple(float(value) for value in quat)


def _orientation_error_rad(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    dot = abs(sum(a * b for a, b in zip(left, right, strict=True)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def _rms(values: list[float]) -> float:
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


@dataclass(frozen=True, slots=True)
class _TrackingError:
    timestamp_ns: int
    tcp_position_m: float
    tcp_orientation_rad: float
    joint_max_abs_rad: float
    joint_abs_rad: tuple[float, float, float, float, float, float]


def _tracking_error(
    *,
    timestamp_ns: int,
    commanded: tuple[float, ...],
    reference: tuple[float, ...],
    fk: _TcpForwardKinematics,
) -> _TrackingError:
    commanded_pose = fk.pose(commanded)
    reference_pose = fk.pose(reference)
    joint_abs = tuple(
        abs(command - target)
        for command, target in zip(commanded, reference, strict=True)
    )
    return _TrackingError(
        timestamp_ns=timestamp_ns,
        tcp_position_m=math.dist(commanded_pose[0], reference_pose[0]),
        tcp_orientation_rad=_orientation_error_rad(
            commanded_pose[1], reference_pose[1]
        ),
        joint_max_abs_rad=max(joint_abs),
        joint_abs_rad=joint_abs,  # type: ignore[arg-type]
    )


def _summarize_tracking(
    errors: list[_TrackingError], *, reference: str
) -> dict[str, object]:
    return {
        "reference": reference,
        "sample_count": len(errors),
        "tcp_position_error_m": {
            "rms": _rms([error.tcp_position_m for error in errors]),
            "peak": max(error.tcp_position_m for error in errors),
        },
        "tcp_orientation_error_rad": {
            "rms": _rms([error.tcp_orientation_rad for error in errors]),
            "peak": max(error.tcp_orientation_rad for error in errors),
        },
        "joint_position_error_rad": {
            "rms_max_abs": _rms([error.joint_max_abs_rad for error in errors]),
            "peak_max_abs": max(error.joint_max_abs_rad for error in errors),
            "per_joint_peak": [
                max(error.joint_abs_rad[joint] for error in errors)
                for joint in range(6)
            ],
        },
    }


def _summarize_dynamics(points: list[ShaperPoint]) -> dict[str, object]:
    def maximum(field: str) -> list[float]:
        return [
            max(abs(getattr(point, field)[joint]) for point in points)
            for joint in range(6)
        ]

    reversals = [0] * 6
    previous_direction = [0] * 6
    for point in points:
        for joint, velocity in enumerate(point.velocity_rad_s):
            direction = int(velocity > 1e-9) - int(velocity < -1e-9)
            if direction and previous_direction[joint] and direction != previous_direction[joint]:
                reversals[joint] += 1
            if direction:
                previous_direction[joint] = direction
    return {
        "sample_count": len(points),
        "joint_velocity_rad_s_peak": maximum("velocity_rad_s"),
        "joint_acceleration_rad_s2_peak": maximum("acceleration_rad_s2"),
        "joint_jerk_rad_s3_peak": maximum("jerk_rad_s3"),
        "direction_reversals": reversals,
    }


def _make_servo(
    prototype: Prototype,
    initial: tuple[float, ...],
    limits: ShaperLimits,
) -> ResolvedRateVelocityServo | JerkBoundedPositionServo:
    return (
        ResolvedRateVelocityServo(initial, limits)
        if prototype == "resolved_rate_velocity"
        else JerkBoundedPositionServo(initial, limits)
    )


def _clutch_release_stop(
    samples: list[ReplaySample],
    *,
    prototype: Prototype,
    limits: ShaperLimits,
    fk: _TcpForwardKinematics,
) -> dict[str, object]:
    servo = _make_servo(prototype, samples[0].joints, limits)
    mailbox = LatestCommandMailbox()
    release_source_index = len(samples) // 2
    release_source_timestamp_ns = samples[release_source_index].timestamp_ns
    next_input = 0
    now_ns = samples[0].timestamp_ns
    period_ns = int(limits.period_s * 1e9)
    while True:
        while next_input < len(samples) and samples[next_input].timestamp_ns <= now_ns:
            sample = samples[next_input]
            mailbox.publish(
                JointCommand(
                    next_input,
                    sample.timestamp_ns,
                    sample.joints,
                    CommandState.ACTIVE,
                    "clutch-release replay",
                )
            )
            next_input += 1
        command = mailbox.take_latest()
        if command is not None and command.joint_position_rad is not None:
            servo.set_target(
                command.joint_position_rad, timestamp_ns=command.generated_ns
            )
        point = servo.tick()
        if now_ns >= release_source_timestamp_ns:
            break
        now_ns += period_ns

    release_position = point.position_rad
    release_tcp = fk.pose(release_position)[0]
    release_velocity = point.velocity_rad_s
    servo.request_controlled_stop()
    stop_points: list[ShaperPoint] = []
    stable_cycles = 0
    for _ in range(1000):
        stop_point = servo.tick()
        stop_points.append(stop_point)
        if (
            max(abs(value) for value in stop_point.velocity_rad_s) < 1e-3
            and max(abs(value) for value in stop_point.acceleration_rad_s2) < 1e-2
        ):
            stable_cycles += 1
            if stable_cycles >= 5:
                break
        else:
            stable_cycles = 0
    tcp_stop_displacement = max(
        math.dist(fk.pose(stop_point.position_rad)[0], release_tcp)
        for stop_point in stop_points
    )
    return {
        "reference": "controlled stop triggered during motion at the middle source target",
        "release_source_index": release_source_index,
        "release_source_timestamp_ns": release_source_timestamp_ns,
        "release_servo_timestamp_ns": now_ns,
        "pre_release_joint_velocity_rad_s": list(release_velocity),
        "pre_release_peak_abs_velocity_rad_s": max(abs(value) for value in release_velocity),
        "stop_time_ms": len(stop_points) * limits.period_s * 1000.0,
        "stable_cycles_required": 5,
        "velocity_threshold_rad_s": 1e-3,
        "acceleration_threshold_rad_s2": 1e-2,
        "joint_stop_displacement_rad": [
            max(
                abs(stop_point.position_rad[joint] - release_position[joint])
                for stop_point in stop_points
            )
            for joint in range(6)
        ],
        "tcp_stop_displacement_m": tcp_stop_displacement,
        "settled": stable_cycles >= 5,
        "dynamics": _summarize_dynamics(stop_points),
    }


def run_replay(
    samples: list[ReplaySample],
    *,
    prototype: Prototype,
    xml_path: Path,
    limits: ShaperLimits = ShaperLimits(),
) -> dict[str, object]:
    """Replay accepted targets through an isolated latest-wins output model."""

    _validate_samples(samples)
    servo = _make_servo(prototype, samples[0].joints, limits)
    fk = _TcpForwardKinematics(xml_path)
    mailbox = LatestCommandMailbox()
    next_input = 0
    now_ns = samples[0].timestamp_ns
    # Allow a bounded post-input settling interval before measuring endpoint
    # error; this is still command-model evidence, not physical latency.
    final_ns = samples[-1].timestamp_ns + int(2.0e9)
    period_ns = int(limits.period_s * 1e9)
    active_points: list[ShaperPoint] = []
    settling_points: list[ShaperPoint] = []
    cpu_samples_ns: list[int] = []
    active_errors: list[_TrackingError] = []
    interpolated_errors: list[_TrackingError] = []
    settling_errors: list[_TrackingError] = []
    ages_ns: list[int] = []
    while now_ns <= final_ns:
        while next_input < len(samples) and samples[next_input].timestamp_ns <= now_ns:
            sample = samples[next_input]
            mailbox.publish(JointCommand(next_input, sample.timestamp_ns, sample.joints, CommandState.ACTIVE, "replay"))
            next_input += 1
        command = mailbox.take_latest()
        if command is not None and command.joint_position_rad is not None:
            servo.set_target(
                command.joint_position_rad, timestamp_ns=command.generated_ns
            )
        start_ns = time.perf_counter_ns()
        point = servo.tick()
        cpu_samples_ns.append(time.perf_counter_ns() - start_ns)
        if now_ns <= samples[-1].timestamp_ns:
            active_points.append(point)
            causal = causal_joint_target(samples, now_ns)
            interpolated = interpolated_joint_target(samples, now_ns)
            active_errors.append(
                _tracking_error(
                    timestamp_ns=now_ns,
                    commanded=point.position_rad,
                    reference=causal,
                    fk=fk,
                )
            )
            interpolated_errors.append(
                _tracking_error(
                    timestamp_ns=now_ns,
                    commanded=point.position_rad,
                    reference=interpolated,
                    fk=fk,
                )
            )
            ages_ns.append(
                max(0, now_ns - samples[max(0, next_input - 1)].timestamp_ns)
            )
        else:
            settling_points.append(point)
            settling_errors.append(
                _tracking_error(
                    timestamp_ns=now_ns,
                    commanded=point.position_rad,
                    reference=samples[-1].joints,
                    fk=fk,
                )
            )
        now_ns += period_ns

    settling_thresholds = {
        "tcp_position_m": 5e-4,
        "tcp_orientation_rad": 1e-3,
        "joint_max_abs_rad": 1e-3,
    }
    first_sustained_settle_ns: int | None = None
    suffix_within = True
    for error in reversed(settling_errors):
        within = (
            error.tcp_position_m <= settling_thresholds["tcp_position_m"]
            and error.tcp_orientation_rad <= settling_thresholds["tcp_orientation_rad"]
            and error.joint_max_abs_rad <= settling_thresholds["joint_max_abs_rad"]
        )
        suffix_within = suffix_within and within
        if suffix_within:
            first_sustained_settle_ns = error.timestamp_ns

    active_summary = _summarize_tracking(
        active_errors, reference="latest target with timestamp <= servo tick"
    )
    interpolated_summary = _summarize_tracking(
        interpolated_errors,
        reference="joint-linear interpolation at servo tick timestamp",
    )
    settling_summary = _summarize_tracking(
        settling_errors, reference="final target after final target timestamp"
    )
    settling_summary.update(
        {
            "window_duration_ms": (final_ns - samples[-1].timestamp_ns) / 1e6,
            "thresholds": settling_thresholds,
            "first_sustained_settle_ms": None
            if first_sustained_settle_ns is None
            else (first_sustained_settle_ns - samples[-1].timestamp_ns) / 1e6,
            "endpoint": {
                "tcp_position_error_m": settling_errors[-1].tcp_position_m,
                "tcp_orientation_error_rad": settling_errors[-1].tcp_orientation_rad,
                "joint_max_abs_error_rad": settling_errors[-1].joint_max_abs_rad,
            },
        }
    )
    target_periods_ms = [
        (current.timestamp_ns - previous.timestamp_ns) / 1e6
        for previous, current in zip(samples, samples[1:])
    ]
    return {
        "schema_version": "teleop_rearchitecture_replay.v2",
        "prototype": prototype,
        "input_target_count": len(samples),
        "source_target_period_ms": {
            "mean": statistics.fmean(target_periods_ms),
            "minimum": min(target_periods_ms),
            "maximum": max(target_periods_ms),
        },
        "target_timestamp_semantics": (
            "target replacement and candidate C feed-forward use each accepted "
            "target's control_monotonic_ns; servo ticks never select a future target"
        ),
        "output_ticks": len(active_points) + len(settling_points),
        "period_hz": 1.0 / limits.period_s,
        "mailbox_max_depth": 1,
        "mailbox_replaced_targets": mailbox.replaced,
        "native_ik_calls": 0,
        "rh56_commands": 0,
        "active_tracking": active_summary,
        "timestamp_interpolated_tracking": interpolated_summary,
        "settling": settling_summary,
        "tracking_model_error": "command-vs-accepted kinematic error only; no physical plant asserted",
        "active_output_dynamics": _summarize_dynamics(active_points),
        "settling_output_dynamics": _summarize_dynamics(settling_points),
        "command_age_ms": {"mean": statistics.fmean(ages_ns) / 1e6, "peak": max(ages_ns) / 1e6},
        "translation_latency_ms": "not identifiable without a physical/network timestamp; command-age reported instead",
        "rotation_latency_ms": "not identifiable without a physical/network timestamp; command-age reported instead",
        "clutch_release_stop": _clutch_release_stop(
            samples, prototype=prototype, limits=limits, fk=fk
        ),
        "cpu_tick_us": {"mean": statistics.fmean(cpu_samples_ns) / 1e3, "peak": max(cpu_samples_ns) / 1e3},
        "feasible_at_125_hz": max(cpu_samples_ns) < period_ns,
        "limits": asdict(limits),
    }
