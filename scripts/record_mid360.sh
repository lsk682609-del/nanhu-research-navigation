#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

BAG_DIR="$(pwd)"

echo "[$(date '+%H:%M:%S')] Starting MID360 driver..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
MID360_PID=$!

echo "[$(date '+%H:%M:%S')] Waiting 10s for LiDAR to initialize..."
sleep 10

echo "[$(date '+%H:%M:%S')] Starting rosbag record (/livox/lidar, /livox/imu)..."
ros2 bag record -o "${BAG_DIR}/mid360_$(date +%Y%m%d_%H%M%S)" /livox/lidar /livox/imu

kill $MID360_PID 2>/dev/null || true
