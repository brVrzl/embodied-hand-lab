#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
if [ -z "${DISPLAY:-}" ]; then
  echo "[view_mujoco_rh56_pose_contact] DISPLAY is empty." >&2
  echo "Reconnect with SSH X11 forwarding, for example: ssh -Y thor@THOR_IP" >&2
  exit 1
fi
.venv/bin/python tools/view_mujoco_rh56_pose_contact.py "$@"
