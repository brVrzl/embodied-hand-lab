#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[check_rh56_connection] Running safe RH56 connectivity check..."
python3 tools/check_rh56_connection.py "$@" || {
  echo "[check_rh56_connection] Connectivity check failed." >&2
  exit 1
}

