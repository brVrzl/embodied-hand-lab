#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source scripts/source_ros2.sh

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi

export PYTHONPATH="src:${PYTHONPATH:-}"

echo "[run_real_arm_hand_ros2_bridge] Starting ROS2 JAKA+RH56 bridge..."
"$PYTHON_BIN" tools/run_real_arm_hand_ros2_bridge.py "$@"
