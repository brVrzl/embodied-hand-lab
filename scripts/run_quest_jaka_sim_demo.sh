#!/usr/bin/env bash
# Thin, simulation-only wrapper around tools/quest_jaka_mujoco_sim.py live-6dof.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONFIG="configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
BIND_HOST="0.0.0.0"
UDP_PORT="9000"
PROJECT_IP=""
ALLOWED_SENDER=""
DURATION_SEC="600"
REPORT=""
OUTPUT=""
EVENTS=""
ARM_EMITTED_EVENTS=""
ARM_OUTPUT_MODE="shaped-500hz"
TELEMETRY_HZ="2"
VIEWER_FLAG="--viewer"
IK_DEBUG_FLAG=""
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
DESKTOP_DISPLAY="${DISPLAY-}"
DESKTOP_XAUTHORITY="${XAUTHORITY-}"

usage() {
  cat <<'EOF'
用法：
  ./scripts/run_quest_jaka_sim_demo.sh [选项]

simulation-only Quest 3 -> JAKA Mini2 MuJoCo 6D 相对遥操作演示。
本脚本只调用 tools/quest_jaka_mujoco_sim.py live-6dof，不包含真机后端。

选项：
  --config PATH           演示 YAML（默认 configs/sim/quest_hts_jaka_mini2_live_demo.yaml）
  --bind HOST             UDP bind host（默认 0.0.0.0）
  --port PORT             Quest/CTRL 共用 UDP 端口（默认 9000）
  --project-ip IPV4       显示给 Quest 操作者填写的主机 IPv4（默认由正式入口探测）
  --allowed-sender IPV4   只接受指定 Quest IPv4（默认不限制）
  --duration-sec SEC      最长运行时间（演示默认 600；正式 CLI 默认 180）
  --report PATH           最终 JSON 报告路径（默认时间戳路径）
  --output PATH           原始 UDP JSONL 记录路径（默认时间戳路径）
  --events PATH           每控制 tick 的 JSONL 事件路径（默认由 report 派生）
  --arm-emitted-events PATH  125 Hz arm emitted JSONL（仅 JAKA-equivalent 模式）
  --arm-output-mode MODE  shaped-500hz（默认）或 jaka-equivalent-125hz
  --telemetry-hz HZ       终端状态输出频率（默认 2；0 关闭）
  --ik-debug              显示可选 joint/IK/奇异性/continuation 诊断
  --viewer                打开 MuJoCo viewer（演示默认）
  --no-viewer             仅用于无图形环境的启动/接收检查
  --display DISPLAY       viewer 使用的 X11 display（SSH 下可指定，例如 :1）
  --xauthority PATH       viewer 使用的 Xauthority 文件
  --python PATH           Python 解释器（默认 .venv/bin/python）
  -h, --help              显示本帮助

MuJoCo model、HTS/controller stale timeout、滤波、IK 与 clutch 阈值均由
--config 指向的当前正式 YAML 管理；当前入口没有 log-level 或键盘 clutch 模式。
EOF
}

require_value() {
  if [[ $# -lt 2 || -z "${2-}" ]]; then
    echo "缺少参数值：${1}" >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) require_value "$@"; CONFIG="$2"; shift 2 ;;
    --bind) require_value "$@"; BIND_HOST="$2"; shift 2 ;;
    --port) require_value "$@"; UDP_PORT="$2"; shift 2 ;;
    --project-ip) require_value "$@"; PROJECT_IP="$2"; shift 2 ;;
    --allowed-sender) require_value "$@"; ALLOWED_SENDER="$2"; shift 2 ;;
    --duration-sec) require_value "$@"; DURATION_SEC="$2"; shift 2 ;;
    --report) require_value "$@"; REPORT="$2"; shift 2 ;;
    --output) require_value "$@"; OUTPUT="$2"; shift 2 ;;
    --events) require_value "$@"; EVENTS="$2"; shift 2 ;;
    --arm-emitted-events) require_value "$@"; ARM_EMITTED_EVENTS="$2"; shift 2 ;;
    --arm-output-mode) require_value "$@"; ARM_OUTPUT_MODE="$2"; shift 2 ;;
    --telemetry-hz) require_value "$@"; TELEMETRY_HZ="$2"; shift 2 ;;
    --ik-debug) IK_DEBUG_FLAG="--ik-debug"; shift ;;
    --viewer) VIEWER_FLAG="--viewer"; shift ;;
    --no-viewer) VIEWER_FLAG="--no-viewer"; shift ;;
    --display) require_value "$@"; DESKTOP_DISPLAY="$2"; shift 2 ;;
    --xauthority) require_value "$@"; DESKTOP_XAUTHORITY="$2"; shift 2 ;;
    --python) require_value "$@"; PYTHON_BIN="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "${REPO_ROOT}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "Python 不可执行：${PYTHON_BIN}" >&2
  echo "请先按 README 创建 .venv，或使用 --python 指定解释器。" >&2
  exit 2
fi
if [[ ! -f "${CONFIG}" ]]; then
  echo "配置文件不存在：${CONFIG}" >&2
  exit 2
fi

# SSH shells do not inherit the physical GNOME desktop's DISPLAY/XAUTHORITY.
# If the same user owns a local GNOME session, reuse only those two display
# variables; no desktop process is started or modified.
if [[ "${VIEWER_FLAG}" == "--viewer" && -z "${DESKTOP_DISPLAY}" ]]; then
  while IFS= read -r desktop_pid; do
    [[ "${desktop_pid}" =~ ^[0-9]+$ ]] || continue
    desktop_environment="/proc/${desktop_pid}/environ"
    [[ -r "${desktop_environment}" ]] || continue
    while IFS='=' read -r name value; do
      case "${name}" in
        DISPLAY) DESKTOP_DISPLAY="${value}" ;;
        XAUTHORITY) DESKTOP_XAUTHORITY="${value}" ;;
      esac
    done < <(tr '\0' '\n' < "${desktop_environment}")
    [[ -n "${DESKTOP_DISPLAY}" ]] && break
  done < <(pgrep -u "$(id -u)" -x gnome-shell || true)
fi

if [[ "${VIEWER_FLAG}" == "--viewer" && -z "${DESKTOP_DISPLAY}" ]]; then
  echo "未找到可用图形桌面 DISPLAY；SSH 启动请使用 --display 和 --xauthority。" >&2
  exit 2
fi
if [[ -n "${DESKTOP_XAUTHORITY}" && ! -r "${DESKTOP_XAUTHORITY}" ]]; then
  echo "Xauthority 不可读：${DESKTOP_XAUTHORITY}" >&2
  exit 2
fi

echo "SAFETY=SIMULATION_ONLY；不会导入、初始化或连接 JAKA / Inspire RH56DFX 真机 SDK"
echo "ENTRY=tools/quest_jaka_mujoco_sim.py live-6dof"
echo "CONFIG=${CONFIG}"
echo "ARM_OUTPUT=${ARM_OUTPUT_MODE}"
echo "QUEST_UDP=${PROJECT_IP:-<自动探测>}:${UDP_PORT}（unicast；host bind=${BIND_HOST}）"
[[ "${VIEWER_FLAG}" == "--viewer" ]] && echo "VIEWER_X11=DISPLAY=${DESKTOP_DISPLAY} XAUTHORITY=${DESKTOP_XAUTHORITY:-<未设置>}"
echo "请先在 Quest 端打开带 CTRL sidecar 的 Hand Tracking Streamer，开启右手、Head Pose、Debug Info，确认 CTRL sender 后 Start Streaming。"
echo "左控制器：INDEX=机械臂 hold-to-run/reference capture；GRIP=仿真 RH56 手；不是空格键。"
echo "连续性：MuJoCo 入口沿完整 6D SE(3) 目标分段推进；奇异/限位拒绝时保持 index 并把手退回即可恢复，硬 gate 不变。"

CMD=(
  "${PYTHON_BIN}" tools/quest_jaka_mujoco_sim.py live-6dof
  --config "${CONFIG}"
  --bind "${BIND_HOST}"
  --port "${UDP_PORT}"
  --duration-sec "${DURATION_SEC}"
  --telemetry-hz "${TELEMETRY_HZ}"
  --arm-output-mode "${ARM_OUTPUT_MODE}"
  "${VIEWER_FLAG}"
)
[[ -n "${PROJECT_IP}" ]] && CMD+=(--project-ip "${PROJECT_IP}")
[[ -n "${ALLOWED_SENDER}" ]] && CMD+=(--allowed-sender "${ALLOWED_SENDER}")
[[ -n "${REPORT}" ]] && CMD+=(--report "${REPORT}")
[[ -n "${OUTPUT}" ]] && CMD+=(--output "${OUTPUT}")
[[ -n "${EVENTS}" ]] && CMD+=(--events "${EVENTS}")
[[ -n "${ARM_EMITTED_EVENTS}" ]] && CMD+=(--arm-emitted-events "${ARM_EMITTED_EVENTS}")
[[ -n "${IK_DEBUG_FLAG}" ]] && CMD+=("${IK_DEBUG_FLAG}")

# exec keeps one foreground process, so Ctrl-C/window close reaches the Python
# finally blocks that stop the receiver thread and close the viewer/socket.
ENVIRONMENT=(PYTHONPATH=src)
[[ -n "${DESKTOP_DISPLAY}" ]] && ENVIRONMENT+=(DISPLAY="${DESKTOP_DISPLAY}")
[[ -n "${DESKTOP_XAUTHORITY}" ]] && ENVIRONMENT+=(XAUTHORITY="${DESKTOP_XAUTHORITY}")
exec env "${ENVIRONMENT[@]}" "${CMD[@]}"
