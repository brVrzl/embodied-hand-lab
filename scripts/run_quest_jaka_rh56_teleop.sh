#!/usr/bin/env bash
# Formal single-receiver JAKA Mini2 + PC-direct RH56DFX teleoperation gate.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"
export PYTHONPATH="${REPO_ROOT}/src:${PYTHONPATH:-}"

ROBOT_IP=""
EDG_STATE_IP="192.168.71.19"
RH56_DEVICE=""
BIND_HOST="0.0.0.0"
UDP_PORT="9000"
ALLOWED_SENDER=""
DURATION_SEC="300"
CONFIG="configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
RH56_CONFIG="configs/hand/rh56_pc_direct_teleop.yaml"
RH56_SCHEDULER_PROFILE="fast40"
WORKER="build/jaka_servo_worker/jaka_servo_worker"
LOG_DIR="logs"
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
ESTOP_ACCESSIBLE="false"
WORKSPACE_CLEAR="false"
NO_AUTO_RETRY="false"
HAND_PREREQUISITES_COMPLETE="false"
PLANT_FREE_NO_NETWORK_CHECK="false"
ALLOW_DIRECT_CH341_DEVICE="false"
NATIVE_CONTROL_CPU=""
NATIVE_CONTROL_REALTIME_PRIORITY="10"
EPISODE_DATA_CONFIG=""
EPISODE_ROOT="data/episodes"
TASK_NAME="fixed_bottle_pick_lift_10cm_hold_3s_replace"
OPERATOR="unknown"
EPISODE_PREVIEW="false"

usage() {
  cat <<'EOF'
Usage / 用法:
  ./scripts/run_quest_jaka_rh56_teleop.sh \
    --robot-ip ROBOT_IPV4 \
    --rh56-device /dev/serial/by-id/... \
    --hand-prerequisites-complete --no-auto-retry \
    --estop-accessible --workspace-clear [options]

Executing this full real-device entry with the explicitly selected robot and
RH56 device is an operator-initiated bounded operation. It reuses one Quest
receiver, one JAKA SDK/native session, the
shared 20 ms target generator, and the production PC-direct RH56 controller.
Left index controls only the arm; grip controls only the hand. Releasing either
clutch holds that subsystem and does not end the combined process.

It never clears RH56 errors, writes speed/force, or opens the hand. Production
arm and hand safety limits remain enabled. Stage 1 read-only and Stage 2 bounded
hand checks must be completed before using this entry.

Options:
  --edg-state-ip IPV4       default 192.168.71.19
  --bind HOST               Quest bind, default 0.0.0.0
  --port PORT               Quest/CTRL port, default 9000
  --allowed-sender IPV4
  --duration-sec SEC        >0 and <=300, default 300
  --config PATH
  --rh56-config PATH
  --rh56-scheduler-profile baseline|fast30|fast40|fast50
  --native-control-cpu CPU  required; reserve one verified CPU for native control
                            control thread uses fixed SCHED_FIFO priority 10;
                            inherited RLIMIT_RTPRIO >=10 is required before I/O
  --allow-direct-ch341-device
                            allow only an identity-checked /dev/ttyCH341USB<N>
                            when the host's custom driver creates no by-id link
  --worker PATH
  --log-dir PATH
  --python PATH
  --episode-data-config PATH
                            enable canonical dual-camera physical episode capture
  --episode-root PATH       default data/episodes
  --task-name NAME          dataset task identifier
  --operator ID             dataset operator identifier
  --episode-preview         show the existing dual-camera preview
  --plant-free-no-network-check  validate both gates without sockets/hardware
EOF
}

need_value() { [[ $# -ge 2 && -n "${2-}" ]] || { echo "Missing value: $1" >&2; exit 2; }; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip) need_value "$@"; ROBOT_IP="$2"; shift 2 ;;
    --edg-state-ip) need_value "$@"; EDG_STATE_IP="$2"; shift 2 ;;
    --rh56-device) need_value "$@"; RH56_DEVICE="$2"; shift 2 ;;
    --bind) need_value "$@"; BIND_HOST="$2"; shift 2 ;;
    --port) need_value "$@"; UDP_PORT="$2"; shift 2 ;;
    --allowed-sender) need_value "$@"; ALLOWED_SENDER="$2"; shift 2 ;;
    --duration-sec) need_value "$@"; DURATION_SEC="$2"; shift 2 ;;
    --config) need_value "$@"; CONFIG="$2"; shift 2 ;;
    --rh56-config) need_value "$@"; RH56_CONFIG="$2"; shift 2 ;;
    --rh56-scheduler-profile) need_value "$@"; RH56_SCHEDULER_PROFILE="$2"; shift 2 ;;
    --native-control-cpu) need_value "$@"; NATIVE_CONTROL_CPU="$2"; shift 2 ;;
    --allow-direct-ch341-device) ALLOW_DIRECT_CH341_DEVICE="true"; shift ;;
    --worker) need_value "$@"; WORKER="$2"; shift 2 ;;
    --log-dir) need_value "$@"; LOG_DIR="$2"; shift 2 ;;
    --python) need_value "$@"; PYTHON_BIN="$2"; shift 2 ;;
    --episode-data-config) need_value "$@"; EPISODE_DATA_CONFIG="$2"; shift 2 ;;
    --episode-root) need_value "$@"; EPISODE_ROOT="$2"; shift 2 ;;
    --task-name) need_value "$@"; TASK_NAME="$2"; shift 2 ;;
    --operator) need_value "$@"; OPERATOR="$2"; shift 2 ;;
    --episode-preview) EPISODE_PREVIEW="true"; shift ;;
    --estop-accessible) ESTOP_ACCESSIBLE="true"; shift ;;
    --workspace-clear) WORKSPACE_CLEAR="true"; shift ;;
    --no-auto-retry) NO_AUTO_RETRY="true"; shift ;;
    --hand-prerequisites-complete) HAND_PREREQUISITES_COMPLETE="true"; shift ;;
    --plant-free-no-network-check) PLANT_FREE_NO_NETWORK_CHECK="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "${ROBOT_IP}" ]] || { echo "--robot-ip is required" >&2; exit 2; }
if [[ "${RH56_DEVICE}" == /dev/serial/by-id/* ]]; then
  :
elif [[ "${ALLOW_DIRECT_CH341_DEVICE}" == true && "${RH56_DEVICE}" =~ ^/dev/ttyCH341USB[0-9]+$ ]]; then
  :
else
  echo "--rh56-device must be /dev/serial/by-id/...; direct tty requires --allow-direct-ch341-device and /dev/ttyCH341USB<N>" >&2
  exit 2
fi
[[ "${ESTOP_ACCESSIBLE}" == true && "${WORKSPACE_CLEAR}" == true && "${NO_AUTO_RETRY}" == true && "${HAND_PREREQUISITES_COMPLETE}" == true ]] || {
  echo "E-stop, workspace, no-retry, and completed hand prerequisites are required" >&2; exit 2;
}
[[ "${NATIVE_CONTROL_CPU}" =~ ^[0-9]+$ ]] || {
  echo "--native-control-cpu is required and must be a nonnegative integer" >&2; exit 2;
}
awk -v value="${DURATION_SEC}" 'BEGIN { exit !(value > 0 && value <= 300) }' || { echo "duration must be >0 and <=300" >&2; exit 2; }
[[ -x "${PYTHON_BIN}" ]] || { echo "Python is not executable: ${PYTHON_BIN}" >&2; exit 2; }
[[ -x "${WORKER}" ]] || { echo "Native worker is not executable: ${WORKER}" >&2; exit 2; }
if [[ "${EPISODE_PREVIEW}" == true && -z "${EPISODE_DATA_CONFIG}" ]]; then
  echo "--episode-preview requires --episode-data-config" >&2
  exit 2
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
prefix="${LOG_DIR%/}/quest_jaka_rh56_combined_${timestamp}_$$"
if [[ "${PLANT_FREE_NO_NETWORK_CHECK}" != true ]]; then mkdir -p "${LOG_DIR}"; fi

echo "PHYSICAL_GATE=combined-normal-teleop"
echo "QUEST_RECEIVERS=1 JAKA_SDK_SESSIONS=1 RH56_TRANSPORT=pc-direct-usb-rs485"
echo "ARM_CLUTCH=left-index; release pauses and re-engage resumes"
echo "HAND_CLUTCH=grip; release holds and re-engage resumes"
echo "RH56_AUTOMATIC_CONFIG_WRITES=none"
echo "DURATION_SEC=${DURATION_SEC} LOG_PREFIX=${prefix}"

cmd=("${PYTHON_BIN}" tools/quest_jaka_hardware.py combined-normal-teleop
  --config "${CONFIG}" --worker "${WORKER}"
  --robot-ip "${ROBOT_IP}" --edg-state-ip "${EDG_STATE_IP}"
  --bind "${BIND_HOST}" --port "${UDP_PORT}" --duration-sec "${DURATION_SEC}"
  --rh56-device "${RH56_DEVICE}"
  --rh56-config "${RH56_CONFIG}"
  --rh56-scheduler-profile "${RH56_SCHEDULER_PROFILE}"
  --output-generator pwl-8ms
  --run-output-joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2
  --recover-output-acceleration-transition --no-auto-retry
  --estop-accessible --workspace-clear
  --log "${prefix}.events.jsonl" --summary "${prefix}.summary.json"
  --metrics "${prefix}.native_metrics.json" --capture "${prefix}.hts.jsonl"
  --native-telemetry "${prefix}.native_cycles.jsonl"
  --event-extract "${prefix}.event_extract.jsonl"
  --rh56-log "${prefix}.rh56.jsonl")
if [[ -n "${ALLOWED_SENDER}" ]]; then cmd+=(--allowed-sender "${ALLOWED_SENDER}"); fi
cmd+=(--native-control-cpu "${NATIVE_CONTROL_CPU}")
cmd+=(--native-control-realtime-priority "${NATIVE_CONTROL_REALTIME_PRIORITY}")
if [[ "${ALLOW_DIRECT_CH341_DEVICE}" == true ]]; then cmd+=(--allow-direct-ch341-device); fi
if [[ -n "${EPISODE_DATA_CONFIG}" ]]; then
  cmd+=(--episode-data-config "${EPISODE_DATA_CONFIG}" --episode-root "${EPISODE_ROOT}")
  cmd+=(--task-name "${TASK_NAME}" --operator "${OPERATOR}")
fi
if [[ "${EPISODE_PREVIEW}" == true ]]; then cmd+=(--episode-preview); fi
if [[ "${PLANT_FREE_NO_NETWORK_CHECK}" == true ]]; then cmd+=(--plant-free-no-network-check); fi
exec "${cmd[@]}"
