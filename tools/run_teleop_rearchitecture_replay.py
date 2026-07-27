#!/usr/bin/env python3
"""Run offline-only rearchitecture replay; this tool imports no hardware SDK."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from teleop_rearchitecture.replay import load_accepted_targets, run_replay


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--accepted-targets", type=Path, required=True)
    parser.add_argument("--prototype", choices=("resolved_rate_velocity", "jerk_bounded_position"), required=True)
    parser.add_argument("--model", type=Path, default=Path("data/sim_assets/jaka_rh56.xml"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run_replay(
        load_accepted_targets(args.accepted_targets), prototype=args.prototype, xml_path=args.model
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
