#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[check_jaka_edg_servo_capability] Inspecting JAKA EDG/servo SDK capability..."
python3 tools/check_jaka_edg_servo_capability.py "$@" || {
  echo "[check_jaka_edg_servo_capability] Capability check failed." >&2
  exit 1
}
