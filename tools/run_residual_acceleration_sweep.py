#!/usr/bin/env python3
"""Generate the deterministic SDK-free residual-acceleration stop sweep."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_rearchitecture.cpp_shaping import default_cpp_library
from teleop_rearchitecture.residual_braking import run_residual_acceleration_sweep


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, default=default_cpp_library(ROOT))
    parser.add_argument("--model", type=Path, default=ROOT / "data/sim_assets/jaka_rh56.xml")
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "docs/research/teleop_rearchitecture/results/residual_acceleration_stop_sweep.json",
    )
    args = parser.parse_args()
    report = run_residual_acceleration_sweep(args.library, args.model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], sort_keys=True))
    return 0 if report["summary"]["unexpected_failure_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
