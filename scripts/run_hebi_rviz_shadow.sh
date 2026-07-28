#!/usr/bin/env bash
set -euo pipefail

# Retired compatibility entry; not a current recommended or production path.
cd "$(dirname "$0")/.."
source scripts/source_ros2.sh
PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"
"$PYTHON_BIN" tools/run_hebi_rviz_shadow.py "$@"
