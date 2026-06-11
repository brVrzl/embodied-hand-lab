#!/usr/bin/env bash
set -euo pipefail

# shellcheck disable=SC1091
source "$(dirname "$0")/source_ros2_humble.sh"

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"
if [ ! -x "$PYTHON_BIN" ]; then
  PYTHON_BIN=python3
fi

echo "ROS_DISTRO=${ROS_DISTRO:-}"
echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
command -v ros2 || {
  echo "ros2 command not found" >&2
  exit 1
}

"$PYTHON_BIN" - <<'PY'
imports = [
    "rclpy",
    "sensor_msgs.msg",
    "geometry_msgs.msg",
    "std_msgs.msg",
]
for name in imports:
    module = __import__(name, fromlist=["*"])
    print(f"{name}: {getattr(module, '__file__', 'ok')}")
PY

ros2 --help >/dev/null
ros2 interface show sensor_msgs/msg/JointState >/dev/null
ros2 bag record --help >/dev/null
