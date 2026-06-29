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

echo "[run_xbox_rviz_shadow] Starting RViz-only Xbox preview. No hardware commands are published."
"$PYTHON_BIN" tools/run_xbox_rviz_shadow.py "$@"
