#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[check_jaka_zero_motion] Running JAKA zero-motion validation..."
python3 tools/check_jaka_zero_motion.py "$@" || {
  echo "[check_jaka_zero_motion] Validation failed." >&2
  exit 1
}

