#!/usr/bin/env python3
"""Run the simulation-only RH56DFX H0 actuator self-test."""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import yaml

from rh56_sim import Rh56H0SelfTest


DEFAULT_MODEL = Path("assets/jaka_rh56_visual_coacd.xml")
DEFAULT_ARM_CONFIG = Path("configs/sim/quest_hts_jaka_mini2_live_demo.yaml")


def _default_log_path() -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S%z")
    return Path("logs/rh56_h0") / f"rh56_h0_{stamp}.jsonl"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", action="store_true", help="show the passive MuJoCo viewer")
    parser.add_argument("--cycle-seconds", type=float, default=2.0)
    parser.add_argument("--amplitude-scale", type=float, default=0.15)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--log-path", type=Path)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--arm-config", type=Path, default=DEFAULT_ARM_CONFIG)
    return parser


def _initial_arm_joints(path: Path) -> tuple[float, ...]:
    values = yaml.safe_load(path.read_text(encoding="utf-8"))
    joints = tuple(float(value) for value in values["simulation"]["initial_arm_joints_rad"])
    if len(joints) != 6:
        raise ValueError("arm config must contain six initial_arm_joints_rad values")
    return joints


def main() -> int:
    args = _parser().parse_args()
    log_path = args.log_path or _default_log_path()
    print("JAKA hardware control disabled")
    print("RH56 hardware control disabled")
    print("H0 hand self-test active")
    print(f"H0 log path: {log_path.resolve()}")

    runner = Rh56H0SelfTest(
        model_path=args.model,
        log_path=log_path,
        cycle_seconds=args.cycle_seconds,
        amplitude_scale=args.amplitude_scale,
        repeat=args.repeat,
        initial_arm_joints_rad=_initial_arm_joints(args.arm_config),
    )
    print(f"H0 mapping: {runner.mapping_rows()}")
    print(
        "H0 initial penetrating contacts: "
        f"{len(runner.initial_penetrating_contacts)} "
        f"{runner.initial_penetrating_contacts}"
    )
    result = runner.run(viewer=args.viewer)
    print(
        "H0 result: "
        f"completed={result.completed} interrupted={result.interrupted} "
        f"steps={result.step_count} invalid={result.invalid_count} "
        f"saturation={result.saturation_count} "
        f"arm_target_unchanged={result.arm_target_unchanged}"
    )
    return 0 if result.completed and result.invalid_count == 0 and result.arm_target_unchanged else 2


if __name__ == "__main__":
    raise SystemExit(main())
