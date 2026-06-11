#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi

echo "[check_jaka_servo_joint_stream] Running bounded JAKA joint servo stream check..."
"$PYTHON_BIN" tools/check_jaka_servo_joint_stream.py "$@" || {
  echo "[check_jaka_servo_joint_stream] Servo stream check failed." >&2
  exit 1
}
