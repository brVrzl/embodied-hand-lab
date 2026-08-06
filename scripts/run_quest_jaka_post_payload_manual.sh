#!/usr/bin/env bash
# Bounded operator-facing wrapper for the current post-payload physical gate.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROBOT_IP=""
EDG_STATE_IP="192.168.71.19"
BIND_HOST="0.0.0.0"
UDP_PORT="9000"
ALLOWED_SENDER=""
DURATION_SEC="30"
CONFIG="configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
WORKER="build/jaka_servo_worker/jaka_servo_worker"
LOG_PREFIX=""
ESTOP_ACCESSIBLE="false"
WORKSPACE_CLEAR="false"
RH56_PATH_ABSENT="false"
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_post_payload_manual.sh \
    --robot-ip ROBOT_IPV4 \
    --estop-accessible \
    --workspace-clear \
    --rh56-command-path-absent \
    [options]

Runs only the current bounded post-payload Quest/JAKA physical diagnostic.
仅运行当前受限的 post-payload Quest/JAKA 真机诊断，不会进入其他 gate。

Required / 必填:
  --robot-ip IPV4
  --estop-accessible
  --workspace-clear
  --rh56-command-path-absent

Options / 可选:
  --edg-state-ip IPV4     EDG state host (default: 192.168.71.19)
  --bind HOST             Quest UDP bind host (default: 0.0.0.0)
  --port PORT             Quest/CTRL UDP port (default: 9000)
  --allowed-sender IPV4   Accept Quest packets only from this sender
  --duration-sec SEC      Positive duration, at most 60 (default: 30)
  --config PATH           Shared live configuration
  --worker PATH           Native JAKA worker
  --log-prefix PATH       Output prefix without extension
  --python PATH           Python interpreter (default: .venv/bin/python)
  -h, --help              Show this help without connecting to hardware

Fixed safety arguments / 固定安全参数:
  stage: post-payload-diagnostic
  shared output joint velocity limit: 1.0 rad/s
  recover before SDK dispatch from isolated PWL acceleration transitions;
  sustained holds and final hard-boundary violations still stop

Stop / 停止:
  Release the left-index clutch or press Ctrl+C. Any controller/SDK fault is a
  hard stop and must not be automatically cleared or retried.
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
    --robot-ip) require_value "$@"; ROBOT_IP="$2"; shift 2 ;;
    --edg-state-ip) require_value "$@"; EDG_STATE_IP="$2"; shift 2 ;;
    --bind) require_value "$@"; BIND_HOST="$2"; shift 2 ;;
    --port) require_value "$@"; UDP_PORT="$2"; shift 2 ;;
    --allowed-sender) require_value "$@"; ALLOWED_SENDER="$2"; shift 2 ;;
    --duration-sec) require_value "$@"; DURATION_SEC="$2"; shift 2 ;;
    --config) require_value "$@"; CONFIG="$2"; shift 2 ;;
    --worker) require_value "$@"; WORKER="$2"; shift 2 ;;
    --log-prefix) require_value "$@"; LOG_PREFIX="$2"; shift 2 ;;
    --python) require_value "$@"; PYTHON_BIN="$2"; shift 2 ;;
    --estop-accessible) ESTOP_ACCESSIBLE="true"; shift ;;
    --workspace-clear) WORKSPACE_CLEAR="true"; shift ;;
    --rh56-command-path-absent) RH56_PATH_ABSENT="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option / 未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "${REPO_ROOT}"

if [[ -z "${ROBOT_IP}" ]]; then
  echo "--robot-ip is required / 必须提供 --robot-ip" >&2
  exit 2
fi
if [[ "${ESTOP_ACCESSIBLE}" != "true" || "${WORKSPACE_CLEAR}" != "true" || "${RH56_PATH_ABSENT}" != "true" ]]; then
  echo "All three safety confirmations are required / 必须提供三项安全确认" >&2
  exit 2
fi
if ! [[ "${DURATION_SEC}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "Invalid duration / duration 格式无效: ${DURATION_SEC}" >&2
  exit 2
fi
if ! awk -v value="${DURATION_SEC}" 'BEGIN { exit !(value > 0 && value <= 60) }'; then
  echo "Duration must be >0 and <=60 seconds / 时长必须大于 0 且不超过 60 秒" >&2
  exit 2
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python is not executable / Python 不可执行: ${PYTHON_BIN}" >&2
  exit 2
fi
if [[ ! -x "${WORKER}" ]]; then
  echo "Native worker is not executable / 原生 worker 不可执行: ${WORKER}" >&2
  echo "Build it first with cmake; no hardware connection was attempted." >&2
  exit 2
fi
if [[ ! -f "${CONFIG}" ]]; then
  echo "Configuration not found / 配置不存在: ${CONFIG}" >&2
  exit 2
fi

if [[ -z "${LOG_PREFIX}" ]]; then
  timestamp="$(date +%Y%m%d_%H%M%S)"
  LOG_PREFIX="logs/quest_jaka_post_payload_manual_${timestamp}"
fi

echo "PHYSICAL_GATE=post-payload-diagnostic"
echo "CONFIG=${CONFIG}"
echo "DURATION_SEC=${DURATION_SEC}"
echo "STOP=release left-index clutch or Ctrl+C"
echo "No RH56 command, payload write, TCP write, installation write, or safety-limit write is performed."
echo "不发送 RH56 命令，不写入 payload、TCP、安装方向或控制器安全限制。"

CMD=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py post-payload-diagnostic
  --config "${CONFIG}"
  --worker "${WORKER}"
  --robot-ip "${ROBOT_IP}"
  --edg-state-ip "${EDG_STATE_IP}"
  --bind "${BIND_HOST}"
  --port "${UDP_PORT}"
  --duration-sec "${DURATION_SEC}"
  --estop-accessible
  --workspace-clear
  --rh56-command-path-absent
  --run-output-joint-velocity-limit-rad-s 1.0
  --recover-output-acceleration-transition
  --log "${LOG_PREFIX}.jsonl"
  --summary "${LOG_PREFIX}_summary.json"
  --metrics "${LOG_PREFIX}_worker.json"
  --native-telemetry "${LOG_PREFIX}_native.jsonl"
  --event-extract "${LOG_PREFIX}_events.jsonl"
)
[[ -n "${ALLOWED_SENDER}" ]] && CMD+=(--allowed-sender "${ALLOWED_SENDER}")

exec env PYTHONPATH=src "${CMD[@]}"
