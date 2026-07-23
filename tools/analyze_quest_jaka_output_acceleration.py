#!/usr/bin/env python3
"""Replay logged filtered TCP targets through the shared output contract."""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path

import numpy as np

from motion_input import Pose6D
from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import bounded_pose_step, quaternion_angle_rad


def _pose(value: dict[str, object]) -> Pose6D:
    return Pose6D(tuple(value["position_m"]), tuple(value["orientation_xyzw"]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control-log", type=Path, required=True)
    parser.add_argument("--worker-metrics", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    worker = json.loads(args.worker_metrics.read_text(encoding="utf-8"))
    loaded = ReplayConfig.load(args.config)
    config = replace(
        loaded,
        output_contract=replace(
            loaded.output_contract,
            maximum_velocity_rad_s=float(worker["output_joint_velocity_boundary_rad_s"]),
            maximum_acceleration_rad_s2=float(worker["diagnostic_joint_acceleration_boundary_rad_s2"]),
        ),
    )
    rows = [
        json.loads(line)
        for line in args.control_log.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    old_accepted = [row for row in rows if row.get("accepted_target_sequence") is not None]
    generator = SharedJakaTargetGenerator(config)
    generator.synchronize_authoritative_arm_joints(worker["post_edg_authoritative_q_hold_rad"])
    shared = config.raw["shared_target_generation"]
    minimum_fraction = float(shared["minimum_continuation_fraction"])
    maximum_backtracks = int(shared["maximum_backtracks"])
    previous_ns: int | None = None
    accepted = 0
    holds = 0
    backtracked = 0
    first_changed_sequence: int | None = None
    maximum_acceleration = 0.0
    evidence: list[dict[str, object]] = []

    for row in old_accepted:
        now_ns = int(row["control_monotonic_ns"])
        dt_s = 1.0 / 60.0 if previous_ns is None else max((now_ns - previous_ns) / 1e9, 1e-6)
        previous_ns = now_ns
        desired = _pose(row["filtered_tcp_target"])
        maximum_translation = min(
            config.feasibility.maximum_target_jump_m,
            config.feasibility.maximum_tcp_velocity_m_s * dt_s,
        )
        maximum_rotation = min(
            config.feasibility.maximum_target_rotation_jump_rad,
            config.feasibility.maximum_tcp_angular_velocity_rad_s * dt_s,
        )
        candidate, fraction = bounded_pose_step(
            generator.last_safe_target,
            desired,
            maximum_translation_m=float(np.nextafter(maximum_translation, 0.0)),
            maximum_rotation_rad=float(np.nextafter(maximum_rotation, 0.0)),
        )
        attempts: list[dict[str, object]] = []
        result = generator.evaluate(candidate, dt_s=dt_s, generated_monotonic_ns=now_ns)
        while True:
            maximum_acceleration = max(
                maximum_acceleration,
                result.metrics.predicted_output_maximum_joint_acceleration_rad_s2,
            )
            attempts.append({
                "fraction": fraction,
                "reason": result.reason.value,
                "maximum_acceleration_rad_s2": result.metrics.predicted_output_maximum_joint_acceleration_rad_s2,
                "violating_joints_zero_based": list(result.metrics.output_acceleration_violating_joint_indices),
            })
            if result.accepted or fraction <= minimum_fraction or len(attempts) > maximum_backtracks:
                break
            fraction *= 0.5
            candidate, _ = bounded_pose_step(
                generator.last_safe_target,
                desired,
                maximum_translation_m=float(np.linalg.norm(
                    np.asarray(desired.position_m) - np.asarray(generator.last_safe_target.position_m)
                ) * fraction),
                maximum_rotation_rad=quaternion_angle_rad(
                    generator.last_safe_target.orientation_xyzw, desired.orientation_xyzw
                ) * fraction,
            )
            result = generator.evaluate(candidate, dt_s=dt_s, generated_monotonic_ns=now_ns)
        changed = (not result.accepted) or fraction < 1.0
        if changed and first_changed_sequence is None:
            first_changed_sequence = int(row["accepted_target_sequence"])
        if result.accepted:
            accepted += 1
            backtracked += int(fraction < 1.0)
        else:
            holds += 1
        if changed:
            evidence.append({
                "old_accepted_sequence": row["accepted_target_sequence"],
                "source_sequence": row.get("accepted_source_sequence"),
                "control_monotonic_ns": now_ns,
                "final_fraction": fraction,
                "final_reason": result.reason.value,
                "attempts": attempts,
            })

    recovery = []
    recovery_time = (previous_ns or 1_000_000_000)
    for attempt in range(1, 11):
        recovery_time += 16_666_667
        result = generator.evaluate(
            generator.last_safe_target,
            dt_s=1.0 / 60.0,
            generated_monotonic_ns=recovery_time,
        )
        recovery.append({"attempt": attempt, "reason": result.reason.value})
        if result.accepted:
            break

    report = {
        "schema_version": "quest_jaka_output_acceleration_replay.v1",
        "source_control_log": str(args.control_log),
        "source_worker_metrics": str(args.worker_metrics),
        "initial_joint_position_rad": worker["post_edg_authoritative_q_hold_rad"],
        "servo_period_ns": config.output_contract.servo_period_ns,
        "velocity_boundary_rad_s": config.output_contract.maximum_velocity_rad_s,
        "acceleration_boundary_rad_s2": config.output_contract.maximum_acceleration_rad_s2,
        "old_accepted_target_count": len(old_accepted),
        "corrected_accepted_count": accepted,
        "corrected_backtracked_count": backtracked,
        "corrected_hold_count": holds,
        "first_old_sequence_changed": first_changed_sequence,
        "maximum_previewed_acceleration_rad_s2": maximum_acceleration,
        "native_defensive_acceleration_fault_reachable": False,
        "recovery_without_restart": bool(recovery and recovery[-1]["reason"] == "ACCEPTED"),
        "recovery_attempts": recovery,
        "changed_sequence_evidence": evidence,
        "notes": "Filtered TCP targets and physical q_hold are replayed through the current shared IK, continuation, and 8 ms output contract; no JAKA or MuJoCo plant is used.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
