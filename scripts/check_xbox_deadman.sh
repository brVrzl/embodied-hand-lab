#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"
"$PYTHON_BIN" tools/check_xbox_deadman.py "$@"
