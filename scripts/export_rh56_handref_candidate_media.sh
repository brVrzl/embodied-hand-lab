#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -z "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X1 ]; then
  export DISPLAY=:1
fi

.venv/bin/python tools/export_handref_candidate_media.py "$@"
