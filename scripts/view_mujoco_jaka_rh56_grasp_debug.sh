#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -z "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X1 ]; then
  export DISPLAY=:1
fi
if [ -z "${DISPLAY:-}" ]; then
  echo "[view_mujoco_jaka_rh56_grasp_debug] DISPLAY is empty." >&2
  echo "Reconnect with SSH X11 forwarding, for example: ssh -Y thor@THOR_IP" >&2
  exit 1
fi

if [ "$#" -eq 0 ]; then
  set -- --scenario cube_in_hand --viewer --duration 0
fi

.venv/bin/python tools/debug_mujoco_jaka_rh56_viewer.py "$@"
