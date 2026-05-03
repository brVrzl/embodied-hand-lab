#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_arm_hand_stack] Starting arm + hand stack..."
python3 -m robot_bringup.cli --arm-hand-only || {
  echo "[start_arm_hand_stack] Failed to start arm + hand stack." >&2
  exit 1
}

