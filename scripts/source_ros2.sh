#!/usr/bin/env bash
# Source this file from bash:
#   source scripts/source_ros2.sh
#
# Defaults to ROS 2 Jazzy on Ubuntu 24.04 / Jetson Thor. Set ROS_DISTRO=humble
# before sourcing if you are on the older Ubuntu 22.04 workstation.

_PALMHAND_ROS_DISTRO="${ROS_DISTRO:-jazzy}"
_PALMHAND_ROS_SETUP=""

if [ -f "/opt/ros/${_PALMHAND_ROS_DISTRO}/setup.bash" ]; then
  _PALMHAND_ROS_SETUP="/opt/ros/${_PALMHAND_ROS_DISTRO}/setup.bash"
elif [ "$_PALMHAND_ROS_DISTRO" = "humble" ] && [ -f "$HOME/ros2_humble/ros2-linux/setup.bash" ]; then
  _PALMHAND_ROS_SETUP="$HOME/ros2_humble/ros2-linux/setup.bash"
fi

if [ -z "$_PALMHAND_ROS_SETUP" ]; then
  echo "ROS2 setup.bash not found for ROS_DISTRO=${_PALMHAND_ROS_DISTRO}." >&2
  echo "Expected /opt/ros/${_PALMHAND_ROS_DISTRO}/setup.bash." >&2
  return 1 2>/dev/null || exit 1
fi

# ROS setup scripts may read unset variables such as COLCON_TRACE.
case "$-" in
  *u*) _PALMHAND_ROS2_HAD_NOUNSET=1 ;;
  *) _PALMHAND_ROS2_HAD_NOUNSET=0 ;;
esac
set +u
# shellcheck disable=SC1090
source "$_PALMHAND_ROS_SETUP"
if [ "$_PALMHAND_ROS2_HAD_NOUNSET" = 1 ]; then
  set -u
else
  set +u
fi

if [ "$_PALMHAND_ROS_DISTRO" = "humble" ] && [ -d "$HOME/ros2_humble/sysroot/usr/lib/x86_64-linux-gnu" ]; then
  export LD_LIBRARY_PATH="$HOME/ros2_humble/sysroot/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
fi

unset _PALMHAND_ROS_SETUP
unset _PALMHAND_ROS_DISTRO
unset _PALMHAND_ROS2_HAD_NOUNSET
