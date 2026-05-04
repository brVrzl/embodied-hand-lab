#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/rh56_handref_grasp_planner.py \
  --objects foam_block_40mm \
  --max-candidates 24 \
  --duration 5.0 \
  --out-dir data/mujoco_handref_grasps_smoke \
  "$@"
