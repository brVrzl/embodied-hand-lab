#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_maniskill_teleop] Starting ManiSkill keyboard teleop..."
python3 -m sim_maniskill.teleop "$@" || {
  echo "[start_maniskill_teleop] Teleop failed." >&2
  exit 1
}
