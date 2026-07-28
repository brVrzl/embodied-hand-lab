#!/usr/bin/env bash
# One explicitly bounded research thin-adapter JAKA/Quest gate.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
EXPECTED_APPROVAL="I_AUTHORIZE_ONE_BOUNDED_RESEARCH_THIN_ADAPTER_JAKA_GATE"
ROBOT_IP=""
EDG_STATE_IP="192.168.71.19"
APPROVAL=""
DURATION_SEC="30"
BIND_HOST="0.0.0.0"
UDP_PORT="9000"
ALLOWED_SENDER=""
LOG_PREFIX=""
PYTHON_BIN="${REPO_ROOT}/.venv/bin/python"
WORKER="build/research_thin_jaka_worker/research_thin_jaka_worker"
CONFIG="configs/sim/quest_hts_jaka_mini2_live_demo.yaml"
ESTOP_ACCESSIBLE="false"
MANUAL_STOP_ACCESSIBLE="false"
WORKSPACE_CLEAR="false"
KNOWN_HEALTHY_POSTURE="false"
RH56_PATH_ABSENT="false"

usage() {
  cat <<'EOF'
Usage:
  scripts/run_quest_jaka_research_thin_gate.sh \
    --robot-ip IPV4 \
    --approval I_AUTHORIZE_ONE_BOUNDED_RESEARCH_THIN_ADAPTER_JAKA_GATE \
    --operator-present --known-healthy-posture --estop-accessible \
    --manual-stop-accessible --workspace-clear --rh56-command-path-absent

This is the single research thin-adapter physical gate. It is capped at 30 s,
uses C++ shaping at 125 Hz, never imports or commands RH56, and performs no
payload/TCP/installation/safety-setting writes.
EOF
}

OPERATOR_PRESENT="false"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-ip) ROBOT_IP="${2:?missing robot IP}"; shift 2 ;;
    --edg-state-ip) EDG_STATE_IP="${2:?missing EDG state IP}"; shift 2 ;;
    --approval) APPROVAL="${2:?missing approval}"; shift 2 ;;
    --duration-sec) DURATION_SEC="${2:?missing duration}"; shift 2 ;;
    --bind) BIND_HOST="${2:?missing bind host}"; shift 2 ;;
    --port) UDP_PORT="${2:?missing port}"; shift 2 ;;
    --allowed-sender) ALLOWED_SENDER="${2:?missing sender}"; shift 2 ;;
    --log-prefix) LOG_PREFIX="${2:?missing log prefix}"; shift 2 ;;
    --operator-present) OPERATOR_PRESENT="true"; shift ;;
    --known-healthy-posture) KNOWN_HEALTHY_POSTURE="true"; shift ;;
    --estop-accessible) ESTOP_ACCESSIBLE="true"; shift ;;
    --manual-stop-accessible) MANUAL_STOP_ACCESSIBLE="true"; shift ;;
    --workspace-clear) WORKSPACE_CLEAR="true"; shift ;;
    --rh56-command-path-absent) RH56_PATH_ABSENT="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

cd "${REPO_ROOT}"
[[ -n "${ROBOT_IP}" ]] || { echo "--robot-ip is required" >&2; exit 2; }
[[ "${APPROVAL}" == "${EXPECTED_APPROVAL}" ]] || {
  echo "exact approval required: ${EXPECTED_APPROVAL}" >&2; exit 2;
}
for gate in OPERATOR_PRESENT KNOWN_HEALTHY_POSTURE ESTOP_ACCESSIBLE \
            MANUAL_STOP_ACCESSIBLE WORKSPACE_CLEAR RH56_PATH_ABSENT; do
  [[ "${!gate}" == "true" ]] || { echo "missing safety gate: ${gate}" >&2; exit 2; }
done
awk -v value="${DURATION_SEC}" 'BEGIN { exit !(value > 0 && value <= 30) }' || {
  echo "duration must be in (0,30] seconds" >&2; exit 2;
}
[[ -x "${WORKER}" ]] || { echo "worker is not built: ${WORKER}" >&2; exit 2; }
[[ -x "${PYTHON_BIN}" ]] || { echo "Python is not executable: ${PYTHON_BIN}" >&2; exit 2; }

if [[ -z "${LOG_PREFIX}" ]]; then
  LOG_PREFIX="logs/quest_jaka_research_thin_$(date +%Y%m%d_%H%M%S)"
fi

echo "PHYSICAL_GATE=research-thin-bounded"
echo "DURATION_SEC=${DURATION_SEC}"
echo "PAUSE_POLICY=repeat_stopped_position_required"
echo "STOP=release clutch for controlled pause; Ctrl+C/manual stop/E-stop for terminal stop"
echo "RH56 commands are absent. Controller settings are read only."

CMD=(
  "${PYTHON_BIN}" tools/quest_jaka_hardware.py research-thin-bounded
  --config "${CONFIG}"
  --worker "${WORKER}"
  --robot-ip "${ROBOT_IP}"
  --edg-state-ip "${EDG_STATE_IP}"
  --bind "${BIND_HOST}"
  --port "${UDP_PORT}"
  --duration-sec "${DURATION_SEC}"
  --approval "${APPROVAL}"
  --estop-accessible
  --workspace-clear
  --rh56-command-path-absent
  --run-output-joint-velocity-limits-rad-s 0.35 0.35 0.35 0.50 0.50 0.50
  --output-generator cpp-reference-v1
  --no-auto-retry
  --log "${LOG_PREFIX}.jsonl"
  --summary "${LOG_PREFIX}_summary.json"
  --metrics "${LOG_PREFIX}_worker.json"
  --capture "${LOG_PREFIX}_capture.jsonl"
  --native-telemetry "${LOG_PREFIX}_native.jsonl"
)
[[ -n "${ALLOWED_SENDER}" ]] && CMD+=(--allowed-sender "${ALLOWED_SENDER}")
exec env PYTHONPATH=src "${CMD[@]}"
