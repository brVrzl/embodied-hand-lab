#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi

echo "[record_real_arm_hand_state] Recording read-only JAKA+RH56 state JSONL..."
"$PYTHON_BIN" tools/record_real_arm_hand_state.py "$@" || {
  echo "[record_real_arm_hand_state] State recording failed." >&2
  exit 1
}
