# Gazebo 车辆与传感器仿真

ROS 2 Humble + Gazebo Classic 11 仿真工作空间，包含四轮差速车辆、Livox Mid-360 非重复扫描雷达，以及基于 `realsense_gazebo_plugin` 的 RealSense D435i 相机。

车辆模型的 Livox 插件已配置为仅发布车体前方 180°（以 `livox_frame` 的
X 正方向为中心，左右各 90°）。`/livox/lidar/pointcloud` 和
`/livox/lidar` CustomMsg 均只包含这个范围的射线；导航转发出的
`cur_lid` 因而也是前 180°。插件默认仍支持全周发布，可通过模型里的
`front_half_only` 参数控制。

可使用 `ros2 launch vehicle_gazebo_sim hui_path.launch.py` 启动 10 m × 10 m、通道净宽
2 m 的“回”字形双墙环境；车辆默认生成在下侧通道中心。

30 m × 30 m 桥梁避障场景可用 `ros2 launch vehicle_gazebo_sim square_bridges_30m.launch.py`
启动（无界面添加 `gui:=false`）。围墙内表面为 x/y=±15 m；车辆默认从
(-10, -10) 朝东出发。沿距墙 5 m 的正方形路径，南、东、北三边依次布置
5°、10°、15° 桥，每座桥由 3 m 上坡、5 m 平台、3 m 下坡组成；路径上另有
圆柱和箱体障碍。对应路网和导航入口分别为
`robot_launch/maps/topo_map_square_bridges_30m.yaml` 与
`ros2 launch robot_launch gazebo_square_bridges_nav.launch.py`。

## 主要输出

| 话题 | 类型 | 目标频率 |
|---|---|---:|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10 Hz |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | 10 Hz |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | 30 Hz |
| `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | 30 Hz |
| `/livox/imu`、`/camera/imu` | `sensor_msgs/msg/Imu` | 200 Hz |

RGB 与深度图像均为 1280×720。深度传感器使用 RGB 相机的位姿、视场角和
`camera_color_optical_frame`，因此两个图像的内参与像素坐标一一对应。RGB 图像编码为
RGB8。Livox 标准点云使用 `livox_frame` 坐标系。

## 编译

```bash
cd /home/lxh/nanhu-research-navigation/src/gazebo_sim
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

colcon build \
  --packages-select livox_ros_driver2 ros2_livox_simulation realsense_gazebo_plugin vehicle_gazebo_sim \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## 启动

```bash
cd /home/lxh/nanhu-research-navigation/src/gazebo_sim
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch vehicle_gazebo_sim simulation.launch.py
```

无界面运行：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py gui:=false
```

启用 RViz：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py rviz:=true
```

## 验证频率

```bash
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /camera/color/image_raw
```

如果相机或 Livox 话题没有出现，请确认启动前执行了：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
```

完整的依赖安装、启动参数、ROS 2 接口、车辆控制、传感器原理、性能配置及故障排查，请阅读 [vehicle_gazebo_sim 文档](src/vehicle_gazebo_sim/README.md)。
