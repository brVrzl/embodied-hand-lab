#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
source scripts/source_ros2.sh
PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"
"$PYTHON_BIN" tools/run_real_jaka_teledex_arm_teleop.py "$@"
