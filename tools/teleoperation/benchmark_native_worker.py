#!/usr/bin/env python3
"""Repeat native no-robot timing runs and preserve each result separately."""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--worker", type=Path, default=Path("build/jaka_servo_worker/jaka_servo_worker"))
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--runs", type=int, default=3)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    results = []
    with tempfile.TemporaryDirectory(prefix="jaka-timing-") as directory:
        for run in range(args.runs):
            metrics = Path(directory) / f"run-{run}.json"
            socket = Path(directory) / f"run-{run}.sock"
            subprocess.run([str(args.worker), "--mode", "dry-run", "--duration-s", str(args.duration_s),
                            "--target-socket", str(socket), "--metrics-file", str(metrics)], check=True)
            results.append(json.loads(metrics.read_text()))
    payload = {"schema_version": "jaka_worker_benchmark.v1", "runs": results}
    rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered)
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
