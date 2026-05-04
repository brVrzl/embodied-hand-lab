#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH=src:tools exec .venv/bin/python tools/evaluate_jaka_rh56_lift_hold.py "$@"
