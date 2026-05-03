#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[export_maniskill_scene_preview] Exporting ManiSkill scene preview..."
python3 -m sim_maniskill.scene_preview "$@" || {
  echo "[export_maniskill_scene_preview] Preview export failed." >&2
  exit 1
}
