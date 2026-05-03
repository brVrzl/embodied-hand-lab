#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_quadruped_stack] Starting quadruped stack..."
python3 -m robot_bringup.cli --quadruped-only || {
  echo "[start_quadruped_stack] Failed to start quadruped stack." >&2
  exit 1
}

