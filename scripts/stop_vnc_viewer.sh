#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${1:-1}"
vncserver -kill ":${DISPLAY_NUM}"
