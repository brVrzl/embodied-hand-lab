#!/usr/bin/env python3
"""Offline Mini2 singularity scan and deterministic manual-run target replay."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from motion_input import Pose6D
from quest_jaka_sim.se3 import (
    bounded_pose_step,
    compose_pose,
    quaternion_angle_rad,
    quaternion_multiply_xyzw,
    relative_pose,
    rotvec_to_quaternion_xyzw,
)
from quest_jaka_sim.simulation import (
    FeasibilityReason,
    JakaMujocoSimulation,
    ReplayConfig,
    SharedJakaTargetGenerator,
)


def _pose(values: dict[str, Any]) -> Pose6D:
    return Pose6D(tuple(values["position_m"]), tuple(values["orientation_xyzw"]))


def _records(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def _replay(
    config: ReplayConfig,
    records: list[dict[str, Any]],
    initial_q: tuple[float, ...],
    *,
    simulation_generator: bool = False,
) -> dict[str, Any]:
    generator = (
        JakaMujocoSimulation(config)
        if simulation_generator
        else SharedJakaTargetGenerator(config)
    )
    generator.synchronize_authoritative_arm_joints(list(initial_q))
    active = [row for row in records if row.get("desired_tcp") is not None]
    recorded_reference = _pose(active[0]["arm_reference_pose"])
    replay_reference = generator.last_safe_target
    accepted = 0
    backtracks = 0
    rejections: Counter[str] = Counter()
    heartbeats = 0
    maximum_joint_step = 0.0
    maximum_joint_velocity = 0.0
    branch_switches = 0
    prior_ns: int | None = None
    longest_producer_gap_ns = 0
    prior_publication_ns: int | None = None
    trace: list[dict[str, Any]] = []
    limits = config.feasibility
    for row in active:
        now_ns = int(row["control_monotonic_ns"])
        dt_s = 1.0 / 60.0 if prior_ns is None else max(1e-6, (now_ns - prior_ns) / 1e9)
        prior_ns = now_ns
        desired = compose_pose(
            replay_reference,
            relative_pose(recorded_reference, _pose(row["desired_tcp"])),
        )
        max_translation = min(
            limits.maximum_target_jump_m,
            limits.maximum_tcp_velocity_m_s * dt_s,
        )
        max_rotation = min(
            limits.maximum_target_rotation_jump_rad,
            limits.maximum_tcp_angular_velocity_rad_s * dt_s,
        )
        trial, fraction = bounded_pose_step(
            generator.last_safe_target,
            desired,
            maximum_translation_m=float(np.nextafter(max_translation, 0.0)),
            maximum_rotation_rad=float(np.nextafter(max_rotation, 0.0)),
        )
        result = generator.evaluate(trial, dt_s=dt_s)
        attempts = [result.reason.value]
        tick_backtracks = 0
        while (
            not result.accepted
            and fraction > config.raw["shared_target_generation"]["minimum_continuation_fraction"]
            and tick_backtracks < config.raw["shared_target_generation"]["maximum_backtracks"]
        ):
            fraction *= 0.5
            trial, _ = bounded_pose_step(
                generator.last_safe_target,
                desired,
                maximum_translation_m=float(
                    np.linalg.norm(
                        np.asarray(desired.position_m)
                        - np.asarray(generator.last_safe_target.position_m)
                    )
                    * fraction
                ),
                maximum_rotation_rad=quaternion_angle_rad(
                    generator.last_safe_target.orientation_xyzw,
                    desired.orientation_xyzw,
                )
                * fraction,
            )
            tick_backtracks += 1
            result = generator.evaluate(trial, dt_s=dt_s)
            attempts.append(result.reason.value)
        backtracks += tick_backtracks
        if result.accepted:
            accepted += 1
            maximum_joint_step = max(
                maximum_joint_step,
                max(abs(value) for value in result.metrics.joint_delta_rad),
            )
            maximum_joint_velocity = max(
                maximum_joint_velocity, result.metrics.maximum_joint_velocity_rad_s
            )
        else:
            rejections[result.reason.value] += 1
            heartbeats += 1
        branch_switches += int(result.metrics.branch_switch)
        if prior_publication_ns is not None:
            longest_producer_gap_ns = max(
                longest_producer_gap_ns, now_ns - prior_publication_ns
            )
        prior_publication_ns = now_ns
        trace.append(
            {
                "control_monotonic_ns": now_ns,
                "requested_tcp": row["desired_tcp"],
                "continuation_tcp": {
                    "position_m": trial.position_m,
                    "orientation_xyzw": trial.orientation_xyzw,
                },
                "candidate_joint_rad": result.metrics.ik_candidate_rad,
                "j5_rad": result.metrics.wrist_bend_from_singularity_rad,
                "jacobian_condition": result.metrics.jacobian_condition,
                "minimum_singular_value": result.metrics.minimum_jacobian_singular_value,
                "singularity_direction": result.metrics.singularity_direction,
                "singularity_state": result.metrics.singularity_state,
                "damping": result.metrics.effective_ik_damping,
                "reason": result.reason.value,
                "accepted": result.accepted,
                "heartbeat": not result.accepted,
                "backtrack_fraction": fraction,
                "backtrack_count": tick_backtracks,
                "attempted_reasons": attempts,
            }
        )

    held_q = generator.last_safe_joint_target.copy()
    retreat_q = held_q.copy()
    retreat_q[4] += math.copysign(math.radians(1.0), float(retreat_q[4]))
    generator.ik.set_arm_joints_rad(retreat_q.tolist())
    retreat_pose = generator._kinematic_tcp_pose
    generator.ik.set_arm_joints_rad(held_q.tolist())
    retreat = generator.evaluate(retreat_pose, dt_s=1.0 / 60.0)
    recovery_jump = max(
        abs(value) for value in retreat.metrics.joint_delta_rad
    )
    return {
        "initial_joint_rad": initial_q,
        "active_ticks": len(active),
        "accepted_ticks": accepted,
        "rejected_ticks": dict(sorted(rejections.items())),
        "heartbeat_ticks": heartbeats,
        "continuation_backtracks": backtracks,
        "maximum_joint_step_rad": maximum_joint_step,
        "maximum_joint_velocity_rad_s": maximum_joint_velocity,
        "speed_boundary_rad_s": config.output_contract.maximum_velocity_rad_s,
        "speed_boundary_violations": int(
            maximum_joint_velocity > config.output_contract.maximum_velocity_rad_s
        ),
        "branch_switches": branch_switches,
        "longest_producer_publication_gap_ms": longest_producer_gap_ns / 1e6,
        "heartbeat_timeout_ms": config.raw["hardware_adapter"][
            "command_stream_timeout_ms"
        ],
        "synthetic_retreat": {
            "accepted": retreat.accepted,
            "reason": retreat.reason.value,
            "direction": retreat.metrics.singularity_direction,
            "branch_switch": retreat.metrics.branch_switch,
            "maximum_joint_delta_rad": recovery_jump,
            "maximum_joint_velocity_rad_s": recovery_jump * 60.0,
            "speed_boundary_violation": recovery_jump * 60.0
            > config.output_contract.maximum_velocity_rad_s,
            "continuous_from_held_target": recovery_jump
            <= limits.maximum_joint_target_jump_rad,
        },
        "final_safe_joint_rad": generator.last_safe_joint_target.tolist(),
        "trace": trace,
    }


def _scan(config: ReplayConfig, posture: tuple[float, ...]) -> list[dict[str, Any]]:
    generator = SharedJakaTargetGenerator(config)
    result = []
    for j5_deg in (-30, -20, -15, -14.968, -12, -10, -8, -7.5, -5, -3, -1, 0, 1, 3, 5, 7.5, 8, 10, 12, 15, 20, 30):
        q = np.asarray(posture, dtype=float)
        q[4] = math.radians(j5_deg)
        generator.ik.set_arm_joints_rad(q.tolist())
        jacp = np.zeros((3, generator.model.nv))
        jacr = np.zeros((3, generator.model.nv))
        mujoco.mj_jacBody(
            generator.model, generator.ik.data, jacp, jacr, generator.palm_body_id
        )
        linear = jacp[:, generator.arm_dof_ids]
        angular = jacr[:, generator.arm_dof_ids]
        spatial = np.vstack(
            (linear, generator.jacobian_rotation_characteristic_length_m * angular)
        )
        singular = np.linalg.svd(spatial, compute_uv=False)
        unscaled = np.vstack((linear, angular))
        directions = {
            "+X_1mm": (0.001, 0, 0, 0, 0, 0),
            "-X_1mm": (-0.001, 0, 0, 0, 0, 0),
            "+Y_1mm": (0, 0.001, 0, 0, 0, 0),
            "-Y_1mm": (0, -0.001, 0, 0, 0, 0),
            "+Z_1mm": (0, 0, 0.001, 0, 0, 0),
            "-Z_1mm": (0, 0, -0.001, 0, 0, 0),
            "+roll_1deg": (0, 0, 0, math.radians(1), 0, 0),
            "-roll_1deg": (0, 0, 0, -math.radians(1), 0, 0),
            "+pitch_1deg": (0, 0, 0, 0, math.radians(1), 0),
            "-pitch_1deg": (0, 0, 0, 0, -math.radians(1), 0),
            "+yaw_1deg": (0, 0, 0, 0, 0, math.radians(1)),
            "-yaw_1deg": (0, 0, 0, 0, 0, -math.radians(1)),
        }
        inverse = np.linalg.pinv(unscaled)
        required = {
            name: float(np.max(np.abs(inverse @ np.asarray(delta))))
            for name, delta in directions.items()
        }
        wrist_alignment = abs(
            float(
                np.dot(angular[:, 3], angular[:, 5])
                / (
                    np.linalg.norm(angular[:, 3])
                    * np.linalg.norm(angular[:, 5])
                )
            )
        )
        ik_checks: dict[str, Any] = {}
        for name, delta in directions.items():
            generator.synchronize_authoritative_arm_joints(q.tolist())
            current_pose = generator.current_tcp_pose
            translation = np.asarray(delta[:3], dtype=float)
            rotation = tuple(float(value) for value in delta[3:])
            target_pose = Pose6D(
                tuple(np.asarray(current_pose.position_m) + translation),
                quaternion_multiply_xyzw(
                    rotvec_to_quaternion_xyzw(rotation),
                    current_pose.orientation_xyzw,
                ),
            )
            ik_result = generator.evaluate(target_pose, dt_s=1.0 / 60.0)
            ik_checks[name] = {
                "accepted": ik_result.accepted,
                "reason": ik_result.reason.value,
                "position_error_m": ik_result.metrics.ik_error_m,
                "orientation_error_rad": ik_result.metrics.ik_orientation_error_rad,
                "branch_switch": ik_result.metrics.branch_switch,
                "maximum_joint_delta_rad": max(
                    abs(value) for value in ik_result.metrics.joint_delta_rad
                ),
            }
        result.append(
            {
                "j5_deg": j5_deg,
                "singular_values": singular.tolist(),
                "minimum_singular_value": float(singular[-1]),
                "condition_number": float(singular[0] / max(singular[-1], 1e-12)),
                "manipulability": float(np.prod(singular)),
                "wrist_axis_abs_cosine": wrist_alignment,
                "representative_max_abs_joint_delta_rad": required,
                "ik_direction_checks": ik_checks,
            }
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--manual-log", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    config = ReplayConfig.load(args.config)
    records = _records(args.manual_log)
    physical_q = tuple(
        next(
            row["measured_joint_position_rad"]
            for row in records
            if row.get("measured_joint_position_rad") is not None
        )
    )
    old_rejections = Counter(
        row["reason"]
        for row in records
        if row.get("desired_tcp") is not None and not row.get("accepted", False)
    )
    physical_shared = _replay(config, records, physical_q)
    physical_simulation = _replay(
        config, records, physical_q, simulation_generator=True
    )
    report = {
        "schema_version": "quest_jaka_singularity_audit.v1",
        "config": str(args.config),
        "manual_log": str(args.manual_log),
        "old_recorded_run": {
            "accepted_ticks": sum(bool(row.get("accepted")) for row in records),
            "rejections": dict(sorted(old_rejections.items())),
            "continuation_backtracks": sum(
                int(row.get("continuation_backtracks", 0)) for row in records
            ),
            "native_outcome": "command_stream_timeout",
        },
        "physical_start_replay": physical_shared,
        "same_physical_start_simulation_replay": physical_simulation,
        "pre_adapter_parity_same_start": {
            "trace_exact_equal": physical_shared["trace"]
            == physical_simulation["trace"],
            "final_safe_joint_exact_equal": physical_shared["final_safe_joint_rad"]
            == physical_simulation["final_safe_joint_rad"],
        },
        "simulation_start_replay": _replay(
            config, records, config.initial_arm_joints_rad
        ),
        "model_scan": _scan(config, physical_q),
    }
    encoded = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
