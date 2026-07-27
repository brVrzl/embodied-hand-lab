#!/usr/bin/env python3
"""Generate SDK-free recoverable engagement continuity evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_rearchitecture.cpp_shaping import default_cpp_library
from teleop_rearchitecture.recovery_evidence import build_recovery_evidence


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, default=default_cpp_library(ROOT))
    parser.add_argument("--model", type=Path, default=ROOT / "data/sim_assets/jaka_rh56.xml")
    parser.add_argument(
        "--output", type=Path,
        default=ROOT / "docs/research/teleop_rearchitecture/results/reengagement_continuity.json",
    )
    args = parser.parse_args()
    report = build_recovery_evidence(args.library, args.model)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
