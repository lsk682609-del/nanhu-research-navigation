#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

if ! ros2 pkg prefix livox_ros_driver2 &>/dev/null; then
  echo "ERROR: package 'livox_ros_driver2' not on AMENT_PREFIX_PATH." >&2
  echo "  Build: cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi

ros2 launch livox_ros_driver2 msg_MID360_launch.py "$@"
