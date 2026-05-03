#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[check_jaka_connection] Running safe JAKA connectivity check..."
python3 tools/check_jaka_connection.py "$@" || {
  echo "[check_jaka_connection] Connectivity check failed." >&2
  exit 1
}

