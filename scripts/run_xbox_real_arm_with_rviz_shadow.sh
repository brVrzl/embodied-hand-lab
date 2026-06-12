#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source scripts/source_ros2_humble.sh

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"

ARM_TELEOP_JSONL="${ARM_TELEOP_JSONL:-outputs/xbox_real_arm_teleop_$(date +%Y%m%d_%H%M%S).jsonl}"
ARM_TELEOP_MAX_PALM_VELOCITY_M_S="${ARM_TELEOP_MAX_PALM_VELOCITY_M_S:-0.15}"
ARM_TELEOP_MAX_WRIST_ROLL_VELOCITY_RAD_S="${ARM_TELEOP_MAX_WRIST_ROLL_VELOCITY_RAD_S:-0.08}"
ARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S="${ARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S:-0.45}"
ARM_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2="${ARM_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2:-1.50}"
ARM_TELEOP_MAX_SESSION_EXCURSION_RAD="${ARM_TELEOP_MAX_SESSION_EXCURSION_RAD:-0.20}"
ARM_TELEOP_MAX_SESSION_PALM_EXCURSION_M="${ARM_TELEOP_MAX_SESSION_PALM_EXCURSION_M:-0.0}"
ARM_TELEOP_TCP_VELOCITY_HORIZON_SEC="${ARM_TELEOP_TCP_VELOCITY_HORIZON_SEC:-0.12}"
ARM_TELEOP_MAX_TCP_TARGET_OFFSET_M="${ARM_TELEOP_MAX_TCP_TARGET_OFFSET_M:-0.010}"
ARM_TELEOP_MAX_RAW_IK_ERROR_RAD="${ARM_TELEOP_MAX_RAW_IK_ERROR_RAD:-0.08}"
ARM_TELEOP_TARGET_DEADBAND_M="${ARM_TELEOP_TARGET_DEADBAND_M:-0.0003}"
ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_RAD="${ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_RAD:-0.030}"
ARM_TELEOP_JOINT_TRACKING_RELEASE_RAD="${ARM_TELEOP_JOINT_TRACKING_RELEASE_RAD:-0.020}"
ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_FAULT_RAD="${ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_FAULT_RAD:-0.055}"
ARM_TELEOP_JOINT_TRACKING_HOLD_MIN_SEC="${ARM_TELEOP_JOINT_TRACKING_HOLD_MIN_SEC:-0.04}"
ARM_TELEOP_SATURATION_HOLD_SEC="${ARM_TELEOP_SATURATION_HOLD_SEC:-0.05}"
ARM_TELEOP_JOINT_LIMIT_MARGIN_DEG="${ARM_TELEOP_JOINT_LIMIT_MARGIN_DEG:-10.0}"
ARM_TELEOP_PRIME_AFTER_ENABLE_TICKS="${ARM_TELEOP_PRIME_AFTER_ENABLE_TICKS:-5}"
ARM_TELEOP_STEP_NUM="${ARM_TELEOP_STEP_NUM:-3}"
ARM_TELEOP_SDK_SERVO_FILTER="${ARM_TELEOP_SDK_SERVO_FILTER:-auto}"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "[run_xbox_real_arm_with_rviz_shadow] Starting real bridge with arm teleop..."
"$PYTHON_BIN" tools/run_real_arm_hand_ros2_bridge.py \
  --enable-arm-teleop \
  --arm-teleop-jsonl "$ARM_TELEOP_JSONL" \
  --arm-teleop-max-palm-velocity-m-s "$ARM_TELEOP_MAX_PALM_VELOCITY_M_S" \
  --arm-teleop-max-wrist-roll-velocity-rad-s "$ARM_TELEOP_MAX_WRIST_ROLL_VELOCITY_RAD_S" \
  --arm-teleop-max-joint-velocity-rad-s "$ARM_TELEOP_MAX_JOINT_VELOCITY_RAD_S" \
  --arm-teleop-max-joint-acceleration-rad-s2 "$ARM_TELEOP_MAX_JOINT_ACCELERATION_RAD_S2" \
  --arm-teleop-max-session-excursion-rad "$ARM_TELEOP_MAX_SESSION_EXCURSION_RAD" \
  --arm-teleop-max-session-palm-excursion-m "$ARM_TELEOP_MAX_SESSION_PALM_EXCURSION_M" \
  --arm-teleop-tcp-velocity-horizon-sec "$ARM_TELEOP_TCP_VELOCITY_HORIZON_SEC" \
  --arm-teleop-max-tcp-target-offset-m "$ARM_TELEOP_MAX_TCP_TARGET_OFFSET_M" \
  --arm-teleop-max-raw-ik-error-rad "$ARM_TELEOP_MAX_RAW_IK_ERROR_RAD" \
  --arm-teleop-target-deadband-m "$ARM_TELEOP_TARGET_DEADBAND_M" \
  --arm-teleop-max-joint-tracking-error-rad "$ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_RAD" \
  --arm-teleop-joint-tracking-release-rad "$ARM_TELEOP_JOINT_TRACKING_RELEASE_RAD" \
  --arm-teleop-max-joint-tracking-error-fault-rad "$ARM_TELEOP_MAX_JOINT_TRACKING_ERROR_FAULT_RAD" \
  --arm-teleop-joint-tracking-hold-min-sec "$ARM_TELEOP_JOINT_TRACKING_HOLD_MIN_SEC" \
  --arm-teleop-saturation-hold-sec "$ARM_TELEOP_SATURATION_HOLD_SEC" \
  --arm-teleop-joint-limit-margin-deg "$ARM_TELEOP_JOINT_LIMIT_MARGIN_DEG" \
  --arm-teleop-prime-after-enable-ticks "$ARM_TELEOP_PRIME_AFTER_ENABLE_TICKS" \
  --arm-teleop-step-num "$ARM_TELEOP_STEP_NUM" \
  --arm-teleop-sdk-servo-filter "$ARM_TELEOP_SDK_SERVO_FILTER" \
  "$@" &
pids+=("$!")

sleep 2

echo "[run_xbox_real_arm_with_rviz_shadow] Starting RViz stack..."
./scripts/run_jaka_rh56_rviz.sh &
pids+=("$!")

sleep 2

echo "[run_xbox_real_arm_with_rviz_shadow] Starting shadow mirror from /jaka/teleop_palm_target_jog..."
"$PYTHON_BIN" tools/run_xbox_rviz_shadow.py \
  --action-topic /jaka/teleop_palm_target_jog \
  --real-arm-joint-topic /jaka/joint_states &
pids+=("$!")

echo "[run_xbox_real_arm_with_rviz_shadow] Starting Xbox intent publisher..."
"$PYTHON_BIN" tools/run_xbox_ros2_teleop.py
