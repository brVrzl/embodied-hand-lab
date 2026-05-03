#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/view_mujoco_ycb_object.py "$@"
