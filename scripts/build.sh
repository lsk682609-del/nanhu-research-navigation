#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# livox_ros_driver2 依赖 CMake 变量 DISTRO_ROS 判断 ROS 版本：
# 缺失时会走错分支，把接口头文件目录解析成 NOTFOUND 而构建失败。
DISTRO_ROS="${ROS_DISTRO:-humble}"
echo "[build] distro=${DISTRO_ROS}"

colcon build --symlink-install \
  --cmake-args -DBUILD_TESTING=OFF -DDISTRO_ROS="${DISTRO_ROS}" "$@"
