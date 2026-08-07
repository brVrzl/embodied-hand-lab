#!/usr/bin/env bash
# Formal single-receiver JAKA Mini2 + PC-direct RH56DFX teleoperation gate.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
RUNTIME_CONFIG=""

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_rh56_teleop.sh \
    --runtime-config configs/data_collection/physical_collection.yaml

All device, control, collection, and output paths are read from the runtime
YAML. The command-line options below are operator acknowledgements only; they
are not configuration overrides.

Options:
  --runtime-config PATH       required host/device/collection YAML
  -h, --help
EOF
}

need_value() {
  [[ $# -ge 2 && -n "${2-}" ]] || {
    echo "Missing value: $1" >&2
    exit 2
  }
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-config)
      need_value "$@"
      RUNTIME_CONFIG="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${RUNTIME_CONFIG}" ]] || {
  echo "--runtime-config is required" >&2
  exit 2
}
[[ -f "${RUNTIME_CONFIG}" ]] || {
  echo "runtime config not found: ${RUNTIME_CONFIG}" >&2
  exit 2
}
[[ -x "${PYTHON_BIN}" ]] || {
  echo "Python is not executable: ${PYTHON_BIN}" >&2
  exit 2
}

cmd=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py combined-normal-teleop
  --runtime-config "${RUNTIME_CONFIG}"
)
exec "${cmd[@]}"
