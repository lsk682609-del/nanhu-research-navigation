source /opt/ros/humble/setup.bash

ros2 launch mavros px4.launch fcu_url:="/dev/ttyACM0:230400"
