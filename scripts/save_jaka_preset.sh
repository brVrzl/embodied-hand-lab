#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[save_jaka_preset] Saving JAKA joint preset..."
python3 tools/save_jaka_preset.py "$@" || {
  echo "[save_jaka_preset] Failed to save preset." >&2
  exit 1
}
