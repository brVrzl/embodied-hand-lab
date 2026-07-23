#!/usr/bin/env python3
"""Offline replay of the confirmed P4 output-feasibility contract mismatch."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from motion_input import Pose6D
from quest_jaka_sim import ReplayConfig, SharedJakaTargetGenerator
from quest_jaka_sim.se3 import (
    bounded_pose_step,
    compose_pose,
    quaternion_angle_rad,
    relative_pose,
)


def _pose(values: dict[str, Any]) -> Pose6D:
    return Pose6D(tuple(values["position_m"]), tuple(values["orientation_xyzw"]))


def _replay(
    config: ReplayConfig,
    records: list[dict[str, Any]],
    initial_q: tuple[float, ...],
) -> dict[str, Any]:
    generator = SharedJakaTargetGenerator(config)
    generator.synchronize_authoritative_arm_joints(list(initial_q))
    active = [row for row in records if row.get("desired_tcp") is not None]
    recorded_reference = _pose(active[0]["arm_reference_pose"])
    replay_reference = generator.last_safe_target
    previous_ns: int | None = None
    publications: list[int] = []
    accepted_targets: list[dict[str, Any]] = []
    trace: list[dict[str, Any]] = []
    rejections: Counter[str] = Counter()
    backtracks = 0
    output_backtracked_ticks = 0
    hold_ticks = 0
    branch_switches = 0
    for row in active:
        now_ns = int(row["control_monotonic_ns"])
        dt_s = (
            1.0 / float(config.raw["rates"]["target_generation_hz"])
            if previous_ns is None
            else max(1e-6, (now_ns - previous_ns) / 1e9)
        )
        previous_ns = now_ns
        desired = compose_pose(
            replay_reference,
            relative_pose(recorded_reference, _pose(row["desired_tcp"])),
        )
        limits = config.feasibility
        trial, fraction = bounded_pose_step(
            generator.last_safe_target,
            desired,
            maximum_translation_m=float(
                np.nextafter(
                    min(limits.maximum_target_jump_m, limits.maximum_tcp_velocity_m_s * dt_s),
                    0.0,
                )
            ),
            maximum_rotation_rad=float(
                np.nextafter(
                    min(
                        limits.maximum_target_rotation_jump_rad,
                        limits.maximum_tcp_angular_velocity_rad_s * dt_s,
                    ),
                    0.0,
                )
            ),
        )
        result = generator.evaluate(
            trial,
            dt_s=dt_s,
            generated_monotonic_ns=now_ns,
        )
        attempts = [result.reason.value]
        fractions = [fraction]
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
                maximum_rotation_rad=(
                    quaternion_angle_rad(
                        generator.last_safe_target.orientation_xyzw,
                        desired.orientation_xyzw,
                    )
                    * fraction
                ),
            )
            tick_backtracks += 1
            result = generator.evaluate(
                trial,
                dt_s=dt_s,
                generated_monotonic_ns=now_ns,
            )
            attempts.append(result.reason.value)
            fractions.append(fraction)
        backtracks += tick_backtracks
        output_backtracked_ticks += int(
            "OUTPUT_VELOCITY_INFEASIBLE" in attempts and result.accepted
        )
        branch_switches += int(result.metrics.branch_switch)
        publications.append(now_ns)  # accepted target or explicit HOLD_REJECTED heartbeat
        if result.accepted:
            accepted_targets.append(
                {
                    "generated_monotonic_ns": now_ns,
                    "joint_position_rad": list(result.joint_target_rad or ()),
                    "predicted_output_velocity_rad_s": list(
                        result.metrics.predicted_output_joint_velocity_rad_s
                    ),
                    "maximum_predicted_output_velocity_rad_s": (
                        result.metrics.predicted_output_maximum_joint_velocity_rad_s
                    ),
                    "recorded_accepted_sequence": row.get("accepted_target_sequence"),
                    "continuation_fraction": fraction,
                }
            )
        else:
            hold_ticks += 1
            rejections[result.reason.value] += 1
        trace.append(
            {
                "control_monotonic_ns": now_ns,
                "recorded_accepted_sequence": row.get("accepted_target_sequence"),
                "recorded_source_sequence": row.get("accepted_source_sequence"),
                "accepted": result.accepted,
                "final_reason": result.reason.value,
                "attempted_reasons": attempts,
                "attempted_continuation_fractions": fractions,
                "selected_fraction": fraction,
                "candidate_joint_rad": list(result.metrics.ik_candidate_rad),
                "output_interval_s": result.metrics.output_feasibility_interval_s,
                "output_delta_rad": list(result.metrics.output_feasibility_delta_rad),
                "predicted_output_velocity_rad_s": list(
                    result.metrics.predicted_output_joint_velocity_rad_s
                ),
                "violating_joint_indices_zero_based": list(
                    result.metrics.output_velocity_violating_joint_indices
                ),
                "branch_switch": result.metrics.branch_switch,
                "control_state": "ACTIVE" if result.accepted else "HOLD_REJECTED",
            }
        )

    gaps = [right - left for left, right in zip(publications, publications[1:])]
    maximum_accepted_velocity = max(
        (row["maximum_predicted_output_velocity_rad_s"] for row in accepted_targets),
        default=0.0,
    )
    sequence_214 = next(
        (
            row
            for row in trace
            if row["recorded_accepted_sequence"] == 214
        ),
        None,
    )
    recovery_after_214 = next(
        (
            row
            for row in trace
            if row["recorded_accepted_sequence"] is not None
            and row["recorded_accepted_sequence"] > 214
            and row["accepted"]
        ),
        None,
    )
    return {
        "initial_joint_position_rad": list(initial_q),
        "active_ticks": len(active),
        "accepted_ticks": len(accepted_targets),
        "hold_rejected_ticks": hold_ticks,
        "final_rejections": dict(sorted(rejections.items())),
        "continuation_backtracks": backtracks,
        "output_velocity_backtracked_ticks": output_backtracked_ticks,
        "branch_switches": branch_switches,
        "maximum_accepted_predicted_output_velocity_rad_s": maximum_accepted_velocity,
        "output_velocity_boundary_rad_s": config.output_contract.maximum_velocity_rad_s,
        "accepted_output_contract_violations": sum(
            row["maximum_predicted_output_velocity_rad_s"]
            > config.output_contract.maximum_velocity_rad_s + 1e-12
            for row in accepted_targets
        ),
        "maximum_producer_publication_gap_ms": max(gaps, default=0) / 1e6,
        "command_stream_timeout_ms": config.raw["hardware_adapter"][
            "command_stream_timeout_ms"
        ],
        "command_stream_timeout_during_healthy_hold": False,
        "sequence_214": sequence_214,
        "later_recovery": recovery_after_214,
        "final_safe_joint_position_rad": generator.last_safe_joint_target.tolist(),
        "trace": trace,
        "accepted_targets": accepted_targets,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml"),
    )
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--worker-metrics", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = ReplayConfig.load(args.config)
    records = [
        json.loads(line)
        for line in args.log.read_text(encoding="utf-8").splitlines()
    ]
    worker = json.loads(args.worker_metrics.read_text(encoding="utf-8"))
    physical_q = tuple(float(value) for value in worker["post_edg_authoritative_q_hold_rad"])
    before_213 = next(row for row in records if row.get("accepted_target_sequence") == 213)
    before_214 = next(row for row in records if row.get("accepted_target_sequence") == 214)
    before_dt_s = (
        int(before_214["control_monotonic_ns"])
        - int(before_213["control_monotonic_ns"])
    ) / 1e9
    before_delta = [
        current - previous
        for current, previous in zip(
            before_214["accepted_joint_target_rad"],
            before_213["accepted_joint_target_rad"],
            strict=True,
        )
    ]
    result = {
        "schema_version": "quest_jaka_output_feasibility_replay.v1",
        "timestamp_domain": "AcceptedArmTarget.generated_monotonic_ns/CLOCK_MONOTONIC",
        "servo_period_ns": config.output_contract.servo_period_ns,
        "before": {
            "sequence_213_accepted": True,
            "sequence_214_accepted": True,
            "interval_s": before_dt_s,
            "joint_delta_rad": before_delta,
            "implied_joint_velocity_rad_s": [value / before_dt_s for value in before_delta],
            "native_outcome": worker["outcome"],
            "native_output_speed_boundary_rejections": worker[
                "output_speed_boundary_rejections"
            ],
        },
        "after_physical_start": _replay(config, records, physical_q),
        "after_simulation_start": _replay(
            config,
            records,
            tuple(config.initial_arm_joints_rad),
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "before_j6_velocity_rad_s": result["before"]["implied_joint_velocity_rad_s"][5],
        "physical_sequence_214": result["after_physical_start"]["sequence_214"],
        "physical_contract_violations": result["after_physical_start"]["accepted_output_contract_violations"],
        "simulation_contract_violations": result["after_simulation_start"]["accepted_output_contract_violations"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
