#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[view_maniskill_scene] Opening ManiSkill scene viewer..."
python3 -m sim_maniskill.view_scene "$@" || {
  echo "[view_maniskill_scene] Viewer exited with an error." >&2
  exit 1
}
