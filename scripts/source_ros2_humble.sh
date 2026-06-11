#!/usr/bin/env bash
# Source this file from bash:
#   source scripts/source_ros2_humble.sh

if [ -f "$HOME/ros2_humble/ros2-linux/setup.bash" ]; then
  # ROS setup scripts may read unset variables such as COLCON_TRACE.
  case "$-" in
    *u*) _PALMHAND_ROS2_HAD_NOUNSET=1 ;;
    *) _PALMHAND_ROS2_HAD_NOUNSET=0 ;;
  esac
  set +u
  # shellcheck disable=SC1091
  source "$HOME/ros2_humble/ros2-linux/setup.bash"
  if [ "$_PALMHAND_ROS2_HAD_NOUNSET" = 1 ]; then
    set -u
  else
    set +u
  fi
  unset _PALMHAND_ROS2_HAD_NOUNSET
elif [ -f /opt/ros/humble/setup.bash ]; then
  case "$-" in
    *u*) _PALMHAND_ROS2_HAD_NOUNSET=1 ;;
    *) _PALMHAND_ROS2_HAD_NOUNSET=0 ;;
  esac
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [ "$_PALMHAND_ROS2_HAD_NOUNSET" = 1 ]; then
    set -u
  else
    set +u
  fi
  unset _PALMHAND_ROS2_HAD_NOUNSET
else
  echo "ROS2 Humble setup.bash not found under ~/ros2_humble or /opt/ros/humble." >&2
  return 1 2>/dev/null || exit 1
fi

if [ -d "$HOME/ros2_humble/sysroot/usr/lib/x86_64-linux-gnu" ]; then
  export LD_LIBRARY_PATH="$HOME/ros2_humble/sysroot/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi
