"""CLI for the bounded offline MuJoCo benchmark."""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import sys

from embodiment_core.doctor import repository_root

from . import (
    BenchmarkConfig,
    BenchmarkConfigurationError,
    run_mujoco_joint_reach_preshape,
    write_benchmark_result,
)


def build_parser(*, prog: str | None = None) -> argparse.ArgumentParser:
    root = repository_root()
    parser = argparse.ArgumentParser(
        prog=prog,
        description=(
            "Run an offline-only MuJoCo smoke benchmark for bounded JAKA joint "
            "tracking and RH56 actuator-joint pre-shape. It does not test grasp, "
            "lift, physical hardware, or sim-to-real transfer."
        )
    )
    parser.add_argument(
        "config",
        nargs="?",
        type=Path,
        default=root / "configs" / "benchmark" / "smoke.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "build" / "validation" / "benchmark.json",
        help=(
            "atomically written JSON (default: "
            "build/validation/benchmark.json)"
        ),
    )
    parser.add_argument("--seed", type=int)
    return parser


def main(argv: list[str] | None = None, *, prog: str | None = None) -> int:
    root = repository_root()
    args = build_parser(prog=prog).parse_args(argv)
    try:
        config = BenchmarkConfig.load(args.config, repository_root=root)
        if args.seed is not None:
            config = replace(config, seed=args.seed)
        if args.output.resolve() in {
            config.source_path,
            config.replay_config_path,
        }:
            raise BenchmarkConfigurationError(
                "output must not overwrite a benchmark or simulation config"
            )
        result = run_mujoco_joint_reach_preshape(
            config, repository_root=root
        )
        output = write_benchmark_result(args.output, result)
    except (
        BenchmarkConfigurationError,
        FileNotFoundError,
        KeyError,
        ValueError,
    ) as exc:
        print(f"Benchmark configuration or model error: {exc}", file=sys.stderr)
        return 2
    print(
        f"benchmark={result['benchmark_id']} status={result['status']} "
        f"failure_reason={result['failure_reason']} result={output}"
    )
    if result["status"] == "passed":
        return 0
    if result["status"] == "failed":
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
