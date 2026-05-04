#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
.venv/bin/python tools/train_handref_candidate_ranker.py "$@"
