#!/bin/bash
set -e

# 启动导航 + 底盘驱动（不带 SLAM、雷达、RViz）
# 用法: ./3588_launch.sh [namespace]  默认 car6

CAR=${1:-car6}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../install/setup.bash"
source "${SCRIPT_DIR}/../../co-search/install/setup.bash"

ros2 launch robot_launch 3588_net_nav.launch.py use_rviz:=false namespace:=${CAR}
