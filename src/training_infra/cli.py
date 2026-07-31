"""CLI for the optional distributed communication smoke test."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from .distributed import DistributedContext
from .smoke import TorchUnavailableError, probe_torch, run_distributed_smoke


def build_parser(*, prog: str | None = None) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=prog,
        description=(
            "Validate PyTorch process-group initialization, rank/device mapping, "
            "all-reduce, a DistributedSampler partition, and clean shutdown. "
            "This is not a model training test."
        )
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="report PyTorch/distributed capabilities without starting collectives",
    )
    parser.add_argument(
        "--device", choices=("auto", "cpu", "cuda"), default="auto"
    )
    parser.add_argument(
        "--backend", choices=("auto", "gloo", "nccl"), default="auto"
    )
    parser.add_argument("--sampler-size", type=int, default=16)
    parser.add_argument("--timeout-seconds", type=int, default=60)
    parser.add_argument("--result-json", type=Path)
    return parser


def main(argv: list[str] | None = None, *, prog: str | None = None) -> int:
    args = build_parser(prog=prog).parse_args(argv)
    try:
        context = DistributedContext.from_environ()
    except ValueError as exc:
        print(f"Invalid distributed environment: {exc}", file=sys.stderr)
        return 2
    if args.check:
        report = probe_torch(context)
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if report["status"] == "ready" else 1
    try:
        result = run_distributed_smoke(
            context,
            device_choice=args.device,
            backend_choice=args.backend,
            sampler_size=args.sampler_size,
            timeout_seconds=args.timeout_seconds,
            result_json=args.result_json,
        )
    except (TorchUnavailableError, RuntimeError, ValueError) as exc:
        print(f"Distributed smoke test unavailable or failed: {exc}", file=sys.stderr)
        return 2
    if context.is_rank_zero:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
