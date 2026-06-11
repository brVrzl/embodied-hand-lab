#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source scripts/source_ros2_humble.sh

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"

echo "[run_xbox_ros2_teleop] Starting Xbox ROS2 intent publisher..."
"$PYTHON_BIN" tools/run_xbox_ros2_teleop.py "$@"
