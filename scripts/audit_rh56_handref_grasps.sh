#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/audit_handref_grasp_dataset.py "$@"
