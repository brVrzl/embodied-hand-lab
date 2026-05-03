#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[start_data_recording] Starting minimal episode recording..."
python3 -m data_recorder.cli "$@" || {
  echo "[start_data_recording] Recording failed." >&2
  exit 1
}

