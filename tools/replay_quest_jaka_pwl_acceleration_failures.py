#!/usr/bin/env python3
"""Replay the three recorded PWL acceleration failures without a plant."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path

from teleoperation.pwl_transition_replay import replay_transition_case


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixture",
        type=Path,
        default=Path(
            "tests/fixtures/"
            "quest_jaka_pwl_acceleration_failures_20260723_24.json"
        ),
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fixture = json.loads(args.fixture.read_text(encoding="utf-8"))
    results = []
    for case in fixture["cases"]:
        replay = replay_transition_case(
            previous_position_rad=case["previous_emitted_position_rad"],
            previous_velocity_rad_s=case["previous_emitted_velocity_rad_s"],
            previous_acceleration_rad_s2=case[
                "previous_emitted_acceleration_rad_s2"
            ],
            proposed_position_rad=case["proposed_emitted_position_rad"],
            destination_position_rad=case["destination_position_rad"],
            first_dt_s=case["command_interval_ns"] / 1e9,
            boundary_rad_s2=case["boundary_rad_s2"],
        )
        result = asdict(replay)
        result.pop("selected_samples")
        result.update(
            name=case["name"],
            source_log_prefix=case["source_log_prefix"],
            fault_joint_one_based=case["fault_joint_one_based"],
            recorded_fault_acceleration_rad_s2=case[
                "recorded_fault_acceleration_rad_s2"
            ],
            replayed_fault_acceleration_rad_s2=case[
                "proposed_emitted_acceleration_rad_s2"
            ][case["fault_joint_one_based"] - 1],
            final_hard_termination_disappears=(
                replay.current_would_terminate and replay.recovered
            ),
        )
        results.append(result)
    report = {
        "schema_version": "quest_jaka_pwl_acceleration_replay.v1",
        "fixture": str(args.fixture),
        "plant_instantiated": False,
        "network_used": False,
        "hardware_commands": 0,
        "cases": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if all(
        case["final_hard_termination_disappears"] for case in results
    ) else 2


if __name__ == "__main__":
    raise SystemExit(main())
