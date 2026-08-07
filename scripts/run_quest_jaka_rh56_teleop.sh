#!/usr/bin/env bash
# Formal single-receiver JAKA Mini2 + PC-direct RH56DFX teleoperation gate.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
RUNTIME_CONFIG=""
ESTOP_ACCESSIBLE="false"
WORKSPACE_CLEAR="false"
NO_AUTO_RETRY="false"
HAND_PREREQUISITES_COMPLETE="false"
PLANT_FREE_NO_NETWORK_CHECK="false"

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_rh56_teleop.sh \
    --runtime-config configs/data_collection/physical_collection.yaml \
    --hand-prerequisites-complete --no-auto-retry \
    --estop-accessible --workspace-clear

All device, control, collection, and output paths are read from the runtime
YAML. The command-line options below are only operator acknowledgements or an
offline validation switch; they are not configuration overrides.

Options:
  --runtime-config PATH       required host/device/collection YAML
  --hand-prerequisites-complete
  --no-auto-retry
  --estop-accessible
  --workspace-clear
  --plant-free-no-network-check  validate before sockets/hardware
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
    --hand-prerequisites-complete)
      HAND_PREREQUISITES_COMPLETE="true"
      shift
      ;;
    --no-auto-retry)
      NO_AUTO_RETRY="true"
      shift
      ;;
    --estop-accessible)
      ESTOP_ACCESSIBLE="true"
      shift
      ;;
    --workspace-clear)
      WORKSPACE_CLEAR="true"
      shift
      ;;
    --plant-free-no-network-check)
      PLANT_FREE_NO_NETWORK_CHECK="true"
      shift
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
[[ "${ESTOP_ACCESSIBLE}" == true && "${WORKSPACE_CLEAR}" == true && \
   "${NO_AUTO_RETRY}" == true && "${HAND_PREREQUISITES_COMPLETE}" == true ]] || {
  echo "E-stop, workspace, no-retry, and completed hand prerequisites are required" >&2
  exit 2
}
[[ -x "${PYTHON_BIN}" ]] || {
  echo "Python is not executable: ${PYTHON_BIN}" >&2
  exit 2
}

cmd=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py combined-normal-teleop
  --runtime-config "${RUNTIME_CONFIG}"
  --no-auto-retry
  --estop-accessible
  --workspace-clear
)
if [[ "${PLANT_FREE_NO_NETWORK_CHECK}" == true ]]; then
  cmd+=(--plant-free-no-network-check)
fi
exec "${cmd[@]}"
