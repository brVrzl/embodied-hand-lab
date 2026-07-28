#!/usr/bin/env python3
"""Generate the deterministic offline Candidate C controlled-stop sweep."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from teleop_rearchitecture.stop_sweep import run_controlled_stop_sweep


def _git_value(*arguments: str) -> str:
    return subprocess.run(
        ("git", *arguments), check=True, text=True, capture_output=True
    ).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=Path("data/sim_assets/jaka_rh56.xml"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--cpp-library",
        type=Path,
        help="explicit built offline C++ reference library; adds C++ braking conformance",
    )
    args = parser.parse_args()
    commit = _git_value("rev-parse", "HEAD")
    dirty = bool(_git_value("status", "--porcelain"))
    payload = run_controlled_stop_sweep(
        model_path=args.model,
        repository_commit=commit,
        working_tree_dirty=dirty,
        cpp_library_path=args.cpp_library,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
