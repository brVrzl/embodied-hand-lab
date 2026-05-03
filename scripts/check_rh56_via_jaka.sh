#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[check_rh56_via_jaka] Running RH56 smoke test via JAKA tool RS485..."
cd "$ROOT_DIR"

if [[ ! -d ".venv" ]]; then
  echo "[check_rh56_via_jaka] Missing .venv. Run: python -m venv .venv && source .venv/bin/activate && pip install -e \".[dev]\"" >&2
  exit 1
fi

source .venv/bin/activate
PYTHONPATH="${ROOT_DIR}/src:${PYTHONPATH:-}" python tools/check_rh56_via_jaka.py "$@"
