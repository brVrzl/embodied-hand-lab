#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -z "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X1 ]; then
  export DISPLAY=:1
fi

if [ "$#" -eq 0 ]; then
  set -- --scenario cube_in_hand --viewer
fi

.venv/bin/python tools/debug_mujoco_jaka_rh56_viewer.py "$@"
