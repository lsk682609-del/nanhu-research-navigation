#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

ros2 topic pub /cancel_navigation std_msgs/msg/Empty "{}" --once
