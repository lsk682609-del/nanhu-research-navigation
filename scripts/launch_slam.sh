#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/install/setup.bash

ros2 launch laser_slam_3d localization.launch.py
