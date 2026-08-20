#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"
export PYTHONPATH="${ROOT_DIR}/src:${PYTHONPATH:-}"
PYTHON_BIN="${VLA_PYTHON:-/home/thor/projects/upstreams/lerobot-env/bin/python}"

if [[ ! -x "${PYTHON_BIN}" && -x "${ROOT_DIR}/.venv/Scripts/python.exe" ]]; then
  PYTHON_BIN="${ROOT_DIR}/.venv/Scripts/python.exe"
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "missing project Python: ${PYTHON_BIN}" >&2
  echo "set VLA_PYTHON to a valid Python executable" >&2
  exit 2
fi

if [[ $# -eq 0 ]]; then
  cat >&2 <<'EOF'
Usage:
  scripts/run_act_dagger.sh --runtime-config ... --checkpoint ... --output ... [--total-duration-sec SEC]
  scripts/run_act_dagger.sh --offline-demo --output ...
EOF
  exit 2
fi

exec "${PYTHON_BIN}" "${ROOT_DIR}/tools/act_dagger_rollout.py" "$@"
