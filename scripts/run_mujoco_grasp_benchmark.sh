#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/mujoco_rh56_grasp_benchmark.py --objects all --max-candidates 72 --duration 4.0 "$@"
