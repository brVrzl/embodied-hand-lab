#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[check_jaka_small_tcp_motion] Running bounded JAKA small TCP motion check..."
python3 tools/check_jaka_small_tcp_motion.py "$@" || {
  echo "[check_jaka_small_tcp_motion] Check failed." >&2
  exit 1
}
