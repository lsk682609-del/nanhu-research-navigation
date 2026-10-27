#!/bin/bash

# 启动 Scout Mini 驱动
source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/install/setup.bash
ros2 launch scout_base scout_mini_base.launch.py port_name:=can_usb
