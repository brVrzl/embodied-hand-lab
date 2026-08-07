#!/usr/bin/env bash
# Operator-facing bounded normal-speed Quest/JAKA PWL teleoperation wrapper.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

RUNTIME_CONFIG=""
RH56_PATH_ABSENT="false"
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_bounded_teleop.sh \
    --runtime-config configs/data_collection/physical_collection.yaml \
    --rh56-command-path-absent

Runs one bounded normal-speed Quest/JAKA arm-only teleoperation attempt through
the production AcceptedArmTarget + configured PWL path. Releasing left index pauses
the arm; pressing it again captures a fresh reference and resumes. It never
commands RH56.
通过 production AcceptedArmTarget + config 配置的 PWL 路径执行一次有界正常速度机械臂遥操作，
不会命令 RH56。

Required / 必填:
  --runtime-config PATH
  --rh56-command-path-absent

Options / 可选:
  --python PATH           Python interpreter (default: .venv/bin/python)
  -h, --help              Show this help without connecting to hardware

Project-selected normal teleoperation run limits / 项目选定的正常遥操作参数:
  J1-J6: 1.5 rad/s
These are not official JAKA Mini2 maximum speeds.
这些参数不是 JAKA Mini2 官方最大速度。

Retained hard stops / 保留硬停止:
  controller alarm, SDK error, hard timing fault, tracking fault, shared/native
  velocity or final acceleration contract violation, sustained recoverable
  acceleration hold, stale input/heartbeat, and Ctrl+C.

Arm clutch / 机械臂离合:
  release left index -> bounded native pause; press again -> fresh reference
  capture and resume. A stale or invalid clutch signal remains a hard stop.

Recoverable transition / 可恢复过渡:
  an isolated PWL acceleration transition is held back before SDK dispatch;
  the worker emits a bounded continuation from its last safe output and keeps
  processing the latest fresh target without restarting EDG or IK.
EOF
}

require_value() {
  if [[ $# -lt 2 || -z "${2-}" ]]; then
    echo "Missing value / 缺少参数值: ${1}" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --runtime-config) require_value "$@"; RUNTIME_CONFIG="$2"; shift 2 ;;
    --python) require_value "$@"; PYTHON_BIN="$2"; shift 2 ;;
    --rh56-command-path-absent) RH56_PATH_ABSENT="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option / 未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "${REPO_ROOT}"

if [[ -z "${RUNTIME_CONFIG}" ]]; then
  echo "--runtime-config is required / 必须提供 --runtime-config" >&2
  exit 2
fi
if [[ "${RH56_PATH_ABSENT}" != "true" ]]; then
  echo "--rh56-command-path-absent is required / 必须提供 --rh56-command-path-absent" >&2
  exit 2
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python is not executable / Python 不可执行: ${PYTHON_BIN}" >&2
  exit 2
fi

echo "PHYSICAL_GATE=bounded-normal-teleop"
echo "RUNTIME_CONFIG=${RUNTIME_CONFIG}"
echo "ARM_CLUTCH=release pauses; press again captures a fresh reference and resumes"
echo "STOP=Ctrl+C, stale input, controller/native hard fault, or duration elapsed"
echo "No RH56 command or controller configuration write is performed."
echo "不发送 RH56 命令，也不写入任何控制器配置。"

CMD=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py bounded-normal-teleop
  --runtime-config "${RUNTIME_CONFIG}"
  --rh56-command-path-absent
)

# One exec, no retry loop.
exec env PYTHONPATH=src "${CMD[@]}"
