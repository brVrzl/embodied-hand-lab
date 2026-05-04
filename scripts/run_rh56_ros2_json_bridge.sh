#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
PYTHONPATH=src exec .venv/bin/python tools/run_rh56_ros2_json_bridge.py "$@"
