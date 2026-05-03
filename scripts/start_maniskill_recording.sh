#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_maniskill_recording] Starting ManiSkill simulation recording..."
python3 -m sim_maniskill.cli "$@" || {
  echo "[start_maniskill_recording] Recording failed." >&2
  exit 1
}
