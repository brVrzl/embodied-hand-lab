"""Unified, offline-safe command line for maintained repository workflows."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import sys
from typing import Iterator

from .doctor import (
    collect_doctor_report,
    render_summary,
    repository_root,
    write_report,
)


@contextmanager
def _working_directory(path: Path) -> Iterator[None]:
    previous = Path.cwd()
    os.chdir(path)
    try:
        yield
    finally:
        os.chdir(previous)


def _sim_smoke(config_path: Path, duration_s: float) -> dict[str, object]:
    if not math.isfinite(duration_s) or duration_s <= 0.0 or duration_s > 5.0:
        raise ValueError("simulation smoke duration must be in (0, 5] seconds")
    from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig

    root = repository_root()
    resolved = (
        config_path.resolve()
        if config_path.is_absolute()
        else (root / config_path).resolve()
    )
    with _working_directory(root):
        config = ReplayConfig.load(resolved)
        simulation = JakaMujocoSimulation(config)
        initial_contacts = int(simulation.data.ncon)
        initial_joints = simulation.arm_joints_rad
        simulation.capture_reference()
        simulation.step(duration_s)
        final_joints = simulation.arm_joints_rad
        if not all(math.isfinite(float(value)) for value in final_joints):
            raise RuntimeError("MuJoCo produced a non-finite arm joint state")
        return {
            "schema_version": "embodied_lab.sim_smoke.v1",
            "status": "passed",
            "validation_level": "offline_simulation",
            "config": str(resolved),
            "mjcf": str(config.mjcf_path),
            "duration_s": duration_s,
            "model": {
                "nq": simulation.model.nq,
                "nv": simulation.model.nv,
                "nu": simulation.model.nu,
                "neq": simulation.model.neq,
            },
            "initial_contact_count": initial_contacts,
            "final_contact_count": int(simulation.data.ncon),
            "maximum_joint_drift_rad": max(
                abs(float(after) - float(before))
                for before, after in zip(
                    initial_joints, final_joints, strict=True
                )
            ),
            "hardware_connections_attempted": False,
        }


def _top_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="embodied-lab",
        description=(
            "Maintained offline workflows. Physical robot gates remain separate "
            "operator-selected scripts with explicit safety prerequisites."
        ),
    )
    commands = parser.add_subparsers(dest="command", required=True)
    doctor = commands.add_parser(
        "doctor",
        help="read-only host, dependency, storage, and device-path inventory",
    )
    doctor.add_argument("--json", action="store_true", help="print full JSON")
    doctor.add_argument("--output", type=Path, help="atomically save full JSON")

    simulation = commands.add_parser("sim", help="MuJoCo offline workflows")
    sim_commands = simulation.add_subparsers(dest="sim_command", required=True)
    smoke = sim_commands.add_parser(
        "smoke", help="load, reset, and step the default model headlessly"
    )
    smoke.add_argument(
        "--config",
        type=Path,
        default=Path("configs/sim/quest_hts_jaka_mini2_offline.yaml"),
    )
    smoke.add_argument("--duration-sec", type=float, default=0.02)

    commands.add_parser(
        "dataset",
        add_help=False,
        help="validate, index, normalize, or export canonical episodes",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments and arguments[0] == "dataset":
        from episode_dataset.cli import main as dataset_main

        return dataset_main(arguments[1:], prog="embodied-lab dataset")
    args = _top_parser().parse_args(arguments)
    if args.command == "doctor":
        report = collect_doctor_report()
        if args.output is not None:
            write_report(args.output, report)
        print(
            json.dumps(report, indent=2, sort_keys=True)
            if args.json
            else render_summary(report)
        )
        return 0 if report["status"] == "ready_offline" else 1
    if args.command == "sim" and args.sim_command == "smoke":
        try:
            report = _sim_smoke(args.config, args.duration_sec)
        except Exception as exc:
            print(
                f"Offline simulation smoke failed: {type(exc).__name__}: {exc}",
                file=sys.stderr,
            )
            return 2
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    raise AssertionError(f"unhandled command: {args.command}")


if __name__ == "__main__":
    raise SystemExit(main())
