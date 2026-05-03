#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_mock_stack] Starting mock embodied lab stack..."
python3 -m robot_bringup.cli || {
  echo "[start_mock_stack] Failed to start mock stack." >&2
  exit 1
}

