#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

if [ -z "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X1 ]; then
  export DISPLAY=:1
fi

# shellcheck disable=SC1091
source scripts/source_ros2_humble.sh

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi
export PYTHONPATH="src:${PYTHONPATH:-}"

if ! "$PYTHON_BIN" - <<'PY' 2>/dev/null
import ctypes
for library in ("liborocos-kdl.so.1.5", "libtinyxml.so.2.6.2", "libassimp.so.5", "libignition-math6.so.6"):
    ctypes.CDLL(library)
PY
then
  echo "[run_jaka_rh56_rviz] Missing robot_state_publisher or RViz runtime libraries." >&2
  echo "Install the Ubuntu Jammy packages: sudo apt install liborocos-kdl1.5 libtinyxml2.6.2v5 libassimp5 libignition-math6-6" >&2
  exit 1
fi

URDF_PATH="${URDF_PATH:-outputs/rviz/jaka_rh56_preview.urdf}"
"$PYTHON_BIN" tools/export_jaka_rh56_rviz_urdf.py --output "$URDF_PATH"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "[run_jaka_rh56_rviz] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH" --ros-args \
  -p publish_frequency:=50.0 &
pids+=("$!")

echo "[run_jaka_rh56_rviz] Starting JAKA+RH56 joint-state fusion..."
"$PYTHON_BIN" tools/run_jaka_rh56_rviz_joint_state_bridge.py &
pids+=("$!")

echo "[run_jaka_rh56_rviz] Starting RViz..."
ros2 run rviz2 rviz2 -d configs/rviz/jaka_rh56.rviz
