#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[export_lerobot_dataset] Exporting LeRobot stub dataset..."
python3 -m lerobot_bridge.cli "$@" || {
  echo "[export_lerobot_dataset] Export failed." >&2
  exit 1
}

