#!/usr/bin/env bash
# Operator-facing bounded normal-speed Quest/JAKA PWL teleoperation wrapper.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROBOT_IP=""
EDG_STATE_IP="192.168.71.19"
BIND_HOST="0.0.0.0"
UDP_PORT="9000"
ALLOWED_SENDER=""
DURATION_SEC="30"
OUTPUT_GENERATOR=""
JOINT_VELOCITY_LIMITS=("1.5" "1.5" "1.5" "1.2" "1.2" "1.2")
CONFIG="configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
WORKER="build/jaka_servo_worker/jaka_servo_worker"
LOG_DIR="logs"
ESTOP_ACCESSIBLE="false"
WORKSPACE_CLEAR="false"
RH56_PATH_ABSENT="false"
NO_AUTO_RETRY="false"
PLANT_FREE_NO_NETWORK_CHECK="false"
OUTPUT_JERK_LIMIT_RAD_S3=""
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"

EXPECTED_OUTPUT_GENERATOR="pwl-8ms"
PROJECT_SHARED_HARD_VELOCITY_RAD_S="3.141592653589793"

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_bounded_teleop.sh \
    --robot-ip ROBOT_IPV4 \
    --output-generator pwl-8ms \
    --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2 \
    --no-auto-retry \
    --estop-accessible \
    --workspace-clear \
    --rh56-command-path-absent \
    [options]

Runs one bounded normal-speed Quest/JAKA arm-only teleoperation attempt through
the production AcceptedArmTarget + 8 ms PWL path. Releasing left index pauses
the arm; pressing it again captures a fresh reference and resumes. It never
commands RH56.
通过 production AcceptedArmTarget + 8 ms PWL 路径执行一次有界正常速度机械臂遥操作，
不会命令 RH56。

Required / 必填:
  --robot-ip IPV4
  --output-generator pwl-8ms
  --no-auto-retry
  --estop-accessible
  --workspace-clear
  --rh56-command-path-absent

Options / 可选:
  --edg-state-ip IPV4     EDG state host (default: 192.168.71.19)
  --bind HOST             Quest UDP bind host (default: 0.0.0.0)
  --port PORT             Quest/CTRL UDP port (default: 9000)
  --allowed-sender IPV4   Accept Quest packets only from this sender
  --duration-sec SEC      Positive duration, at most 60 (default: 30)
  --joint-velocity-limits-rad-s J1 J2 J3 J4 J5 J6
                          J1-J3 default 1.5; J4-J6 default 1.2 rad/s
  --config PATH           Shared production live configuration
  --worker PATH           Native JAKA worker
  --output-joint-jerk-limit-rad-s3 VALUE
                          Override the config project-selected jerk shaper
  --log-dir PATH          Timestamped output directory parent (default: logs)
  --python PATH           Python interpreter (default: .venv/bin/python)
  --plant-free-no-network-check
                          Validate the complete command without sockets/hardware
  -h, --help              Show this help without connecting to hardware

Project-selected normal teleoperation run limits / 项目选定的正常遥操作参数:
  J1-J3: 1.5 rad/s
  J4-J6: 1.2 rad/s
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
    --robot-ip) require_value "$@"; ROBOT_IP="$2"; shift 2 ;;
    --edg-state-ip) require_value "$@"; EDG_STATE_IP="$2"; shift 2 ;;
    --bind) require_value "$@"; BIND_HOST="$2"; shift 2 ;;
    --port) require_value "$@"; UDP_PORT="$2"; shift 2 ;;
    --allowed-sender) require_value "$@"; ALLOWED_SENDER="$2"; shift 2 ;;
    --duration-sec) require_value "$@"; DURATION_SEC="$2"; shift 2 ;;
    --output-generator) require_value "$@"; OUTPUT_GENERATOR="$2"; shift 2 ;;
    --joint-velocity-limits-rad-s)
      if [[ $# -lt 7 ]]; then
        echo "Six J1-J6 velocity values are required / 必须提供 J1-J6 六个速度值" >&2
        exit 2
      fi
      JOINT_VELOCITY_LIMITS=("$2" "$3" "$4" "$5" "$6" "$7")
      shift 7
      ;;
    --config) require_value "$@"; CONFIG="$2"; shift 2 ;;
    --worker) require_value "$@"; WORKER="$2"; shift 2 ;;
    --output-joint-jerk-limit-rad-s3) require_value "$@"; OUTPUT_JERK_LIMIT_RAD_S3="$2"; shift 2 ;;
    --log-dir) require_value "$@"; LOG_DIR="$2"; shift 2 ;;
    --python) require_value "$@"; PYTHON_BIN="$2"; shift 2 ;;
    --no-auto-retry) NO_AUTO_RETRY="true"; shift ;;
    --estop-accessible) ESTOP_ACCESSIBLE="true"; shift ;;
    --workspace-clear) WORKSPACE_CLEAR="true"; shift ;;
    --rh56-command-path-absent) RH56_PATH_ABSENT="true"; shift ;;
    --plant-free-no-network-check) PLANT_FREE_NO_NETWORK_CHECK="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option / 未知参数: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "${REPO_ROOT}"

if [[ -z "${ROBOT_IP}" ]]; then
  echo "--robot-ip is required / 必须提供 --robot-ip" >&2
  exit 2
fi
if [[ "${OUTPUT_GENERATOR}" != "${EXPECTED_OUTPUT_GENERATOR}" ]]; then
  echo "Required output generator / 必须使用输出生成器: ${EXPECTED_OUTPUT_GENERATOR}" >&2
  exit 2
fi
if [[ "${NO_AUTO_RETRY}" != "true" ]]; then
  echo "--no-auto-retry is required / 必须提供 --no-auto-retry" >&2
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
for value in "${JOINT_VELOCITY_LIMITS[@]}"; do
  if ! awk -v value="${value}" -v hard="${PROJECT_SHARED_HARD_VELOCITY_RAD_S}" \
      'BEGIN { exit !(value > 0 && value <= hard) }'; then
    echo "Each joint velocity must be >0 and <=pi rad/s / 每个关节速度必须大于 0 且不超过 pi rad/s" >&2
    exit 2
  fi
done
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

timestamp="$(date +%Y%m%d_%H%M%S)"
LOG_PREFIX="${LOG_DIR%/}/quest_jaka_bounded_teleop_${timestamp}_$$"
if [[ "${PLANT_FREE_NO_NETWORK_CHECK}" != "true" ]]; then
  mkdir -p "${LOG_DIR}"
fi

echo "PHYSICAL_GATE=bounded-normal-teleop"
echo "OUTPUT_GENERATOR=${OUTPUT_GENERATOR}"
echo "JOINT_VELOCITY_LIMITS_RAD_S=${JOINT_VELOCITY_LIMITS[*]}"
echo "DURATION_SEC=${DURATION_SEC}"
echo "LOG_PREFIX=${LOG_PREFIX}"
echo "NO_AUTO_RETRY=true"
echo "ARM_CLUTCH=release pauses; press again captures a fresh reference and resumes"
echo "STOP=Ctrl+C, stale input, controller/native hard fault, or duration elapsed"
echo "No RH56 command or controller configuration write is performed."
echo "不发送 RH56 命令，也不写入任何控制器配置。"

CMD=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py bounded-normal-teleop
  --config "${CONFIG}"
  --worker "${WORKER}"
  --robot-ip "${ROBOT_IP}"
  --edg-state-ip "${EDG_STATE_IP}"
  --bind "${BIND_HOST}"
  --port "${UDP_PORT}"
  --duration-sec "${DURATION_SEC}"
  --output-generator "${OUTPUT_GENERATOR}"
  --run-output-joint-velocity-limits-rad-s "${JOINT_VELOCITY_LIMITS[@]}"
  --no-auto-retry
  --estop-accessible
  --workspace-clear
  --rh56-command-path-absent
  --recover-output-acceleration-transition
  --log "${LOG_PREFIX}.jsonl"
  --summary "${LOG_PREFIX}_summary.json"
  --metrics "${LOG_PREFIX}_worker.json"
  --capture "${LOG_PREFIX}_capture.jsonl"
  --native-telemetry "${LOG_PREFIX}_native.jsonl"
  --event-extract "${LOG_PREFIX}_events.jsonl"
)
[[ -n "${ALLOWED_SENDER}" ]] && CMD+=(--allowed-sender "${ALLOWED_SENDER}")
[[ -n "${OUTPUT_JERK_LIMIT_RAD_S3}" ]] && CMD+=(--output-joint-jerk-limit-rad-s3 "${OUTPUT_JERK_LIMIT_RAD_S3}")
if [[ "${PLANT_FREE_NO_NETWORK_CHECK}" == "true" ]]; then
  CMD+=(--plant-free-no-network-check)
fi

# One exec, no retry loop.
exec env PYTHONPATH=src "${CMD[@]}"
