#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

if ! ros2 pkg prefix nav2_controller &>/dev/null; then
  echo "ERROR: package 'nav2_controller' not on AMENT_PREFIX_PATH." >&2
  echo "  Build: cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi
if ! ros2 pkg prefix nav2_a_star_rpp &>/dev/null; then
  echo "ERROR: package 'nav2_a_star_rpp' not found — waypoint_nav2.yaml uses RegulatedPurePursuitController from this package." >&2
  echo "  You built only nav2_controller; also build:  colcon build --symlink-install --packages-select nav2_a_star_rpp" >&2
  echo "  Or full workspace:  cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi

ros2 launch robot_launch nx_waypoint_nav.launch.py