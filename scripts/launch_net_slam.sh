#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

if ! ros2 pkg prefix laser_slam_3d &>/dev/null; then
  echo "ERROR: package 'laser_slam_3d' not on AMENT_PREFIX_PATH." >&2
  echo "  Build: cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi
if ! ros2 pkg prefix nav2_controller &>/dev/null; then
  echo "ERROR: package 'nav2_controller' not on AMENT_PREFIX_PATH." >&2
  echo "  Build: cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi
if ! ros2 pkg prefix nav2_a_star_rpp &>/dev/null; then
  echo "ERROR: package 'nav2_a_star_rpp' not found." >&2
  echo "  Build: cd \"${SCRIPT_DIR}\" && bash scripts/build.sh" >&2
  exit 1
fi

ros2 launch robot_launch nx_net_nav_slam.launch.py \
    map_data_path:=/home/nhy/ww/map/sll1f \
    use_rviz:=false \
    "$@"
    # map_data_path:=/home/nhy/ww/map/hx_glim_2026_06_06 \

