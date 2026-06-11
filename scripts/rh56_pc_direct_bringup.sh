#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[rh56_pc_direct_bringup] Running RH56 PC-direct USB-RS485 bring-up..."
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
if [[ ! -x "${PYTHON_BIN}" ]]; then
  PYTHON_BIN="python3"
fi
"${PYTHON_BIN}" tools/rh56_pc_direct_bringup.py "$@" || {
  echo "[rh56_pc_direct_bringup] Bring-up failed." >&2
  exit 1
}
