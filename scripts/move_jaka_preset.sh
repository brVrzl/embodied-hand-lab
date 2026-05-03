#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[move_jaka_preset] Moving JAKA to preset..."
python3 tools/move_jaka_preset.py "$@" || {
  echo "[move_jaka_preset] Failed to move preset." >&2
  exit 1
}
