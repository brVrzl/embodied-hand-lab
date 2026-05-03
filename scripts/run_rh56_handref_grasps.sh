#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/rh56_handref_grasp_planner.py --objects all --max-candidates 80 "$@"
