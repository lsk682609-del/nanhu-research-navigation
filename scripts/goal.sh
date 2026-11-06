#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS="${NS:-car1}"
X="${1:-12.0}"
Y="${2:-8.0}"
TIMEOUT="${TIMEOUT:-120}"

source /opt/ros/humble/setup.bash
source "${ROOT}/install/setup.bash"
set -u

if ! ros2 action list 2>/dev/null | grep -q "/${NS}/navigate_to_pose"; then
  echo "[goal] 未找到 /${NS}/navigate_to_pose，请先启动导航栈" >&2
  exit 2
fi

echo "[goal] 目标 (${X}, ${Y}) @ ${NS}，超时 ${TIMEOUT}s"
timeout "${TIMEOUT}" ros2 action send_goal "/${NS}/navigate_to_pose" \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
