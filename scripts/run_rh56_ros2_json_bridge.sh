#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
# shellcheck disable=SC1091
source scripts/source_ros2.sh
export PYTHONPATH="src:${PYTHONPATH:-}"
exec .venv/bin/python tools/run_rh56_ros2_json_bridge.py "$@"
