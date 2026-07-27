"""Offline replay and common metrics for the rearchitecture prototypes."""

from __future__ import annotations

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
    return samples


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


def run_replay(
    samples: list[ReplaySample],
    *,
    prototype: Prototype,
    xml_path: Path,
    limits: ShaperLimits = ShaperLimits(),
) -> dict[str, object]:
    """Replay accepted targets through an isolated latest-wins output model."""

    servo = (
        ResolvedRateVelocityServo(samples[0].joints, limits)
        if prototype == "resolved_rate_velocity"
        else JerkBoundedPositionServo(samples[0].joints, limits)
    )
    fk = _TcpForwardKinematics(xml_path)
    mailbox = LatestCommandMailbox()
    next_input = 0
    now_ns = samples[0].timestamp_ns
    # Allow a bounded post-input settling interval before measuring endpoint
    # error; this is still command-model evidence, not physical latency.
    final_ns = samples[-1].timestamp_ns + int(2.0e9)
    period_ns = int(limits.period_s * 1e9)
    points: list[ShaperPoint] = []
    intended: list[tuple[float, ...]] = []
    cpu_samples_ns: list[int] = []
    position_errors: list[float] = []
    orientation_errors: list[float] = []
    ages_ns: list[int] = []
    reversals = [0] * 6
    previous_direction = [0] * 6
    while now_ns <= final_ns:
        while next_input < len(samples) and samples[next_input].timestamp_ns <= now_ns:
            sample = samples[next_input]
            mailbox.publish(JointCommand(next_input, sample.timestamp_ns, sample.joints, CommandState.ACTIVE, "replay"))
            next_input += 1
        command = mailbox.take_latest()
        if command is not None and command.joint_position_rad is not None:
            servo.set_target(command.joint_position_rad)
        start_ns = time.perf_counter_ns()
        point = servo.tick()
        cpu_samples_ns.append(time.perf_counter_ns() - start_ns)
        points.append(point)
        nearest = samples[min(next_input, len(samples) - 1)]
        intended.append(nearest.joints)
        command_pose = fk.pose(point.position_rad)
        intended_pose = fk.pose(nearest.joints)
        position_errors.append(math.dist(command_pose[0], intended_pose[0]))
        orientation_errors.append(_orientation_error_rad(command_pose[1], intended_pose[1]))
        ages_ns.append(max(0, now_ns - samples[max(0, next_input - 1)].timestamp_ns))
        for joint, velocity in enumerate(point.velocity_rad_s):
            direction = int(velocity > 1e-9) - int(velocity < -1e-9)
            if direction and previous_direction[joint] and direction != previous_direction[joint]:
                reversals[joint] += 1
            if direction:
                previous_direction[joint] = direction
        now_ns += period_ns
    max_by_joint = lambda field: [max(abs(getattr(p, field)[i]) for p in points) for i in range(6)]
    endpoint_settling = max(abs(value) for value in points[-1].target_error_rad)
    stop_start = len(points)
    servo.request_controlled_stop()
    for _ in range(1000):
        point = servo.tick()
        points.append(point)
        if max(abs(value) for value in point.velocity_rad_s) < 1e-3 and max(abs(value) for value in point.acceleration_rad_s2) < 1e-2:
            break
    return {
        "prototype": prototype,
        "input_target_count": len(samples),
        "output_ticks": len(points),
        "period_hz": 1.0 / limits.period_s,
        "mailbox_max_depth": 1,
        "mailbox_replaced_targets": mailbox.replaced,
        "native_ik_calls": 0,
        "rh56_commands": 0,
        "tcp_position_error_m": {"rms": _rms(position_errors), "peak": max(position_errors)},
        "tcp_orientation_error_rad": {"rms": _rms(orientation_errors), "peak": max(orientation_errors)},
        "tracking_model_error": "command-vs-accepted kinematic error only; no physical plant asserted",
        "joint_velocity_rad_s_peak": max_by_joint("velocity_rad_s"),
        "joint_acceleration_rad_s2_peak": max_by_joint("acceleration_rad_s2"),
        "joint_jerk_rad_s3_peak": max_by_joint("jerk_rad_s3"),
        "direction_reversals": reversals,
        "command_age_ms": {"mean": statistics.fmean(ages_ns) / 1e6, "peak": max(ages_ns) / 1e6},
        "translation_latency_ms": "not identifiable without a physical/network timestamp; command-age reported instead",
        "rotation_latency_ms": "not identifiable without a physical/network timestamp; command-age reported instead",
        "stop_time_ms": (len(points) - stop_start) * limits.period_s * 1000.0,
        "endpoint_settling_rad_before_release": endpoint_settling,
        "cpu_tick_us": {"mean": statistics.fmean(cpu_samples_ns) / 1e3, "peak": max(cpu_samples_ns) / 1e3},
        "feasible_at_125_hz": max(cpu_samples_ns) < period_ns,
        "limits": asdict(limits),
    }
