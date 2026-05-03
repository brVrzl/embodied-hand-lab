#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

cd "${REPO_ROOT}"
echo "[arm_hand_smoke_test] Running arm + hand smoke test..."
python3 tools/arm_hand_smoke_test.py "$@" || {
  echo "[arm_hand_smoke_test] Smoke test failed." >&2
  exit 1
}
