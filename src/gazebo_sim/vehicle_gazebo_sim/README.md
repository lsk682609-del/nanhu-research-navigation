# vehicle_gazebo_sim

基于 ROS 2 Humble 和 Gazebo Classic 11 的四轮差速车辆仿真环境。项目提供车辆控制、Livox Mid-360 非重复扫描雷达、基于 PAL `realsense_gazebo_plugin` 的 RealSense D435i、IMU、里程计及 TF，可作为定位、建图和导航算法的仿真数据源。

本项目只启动仿真环境和传感器，不会自动启动 FAST-LIO、SLAM Toolbox、Nav2、点云转激光或其他定位导航节点。

## 功能

- Gazebo Classic 室内测试场景
- 10 m × 10 m、净宽 2 m 的“回”字形双墙通道场景
- 四轮差速车辆和 `/cmd_vel` 控制接口
- 基于 `mid360.csv` 实际扫描序列的 Livox Mid-360 仿真
- Livox `CustomMsg` 与标准 `PointCloud2` 同时输出
- D435i RGB、深度、双红外、RGB-D 点云、相机内参和 6 轴 IMU
- 雷达 IMU、相机 IMU、里程计和机器人 TF
- 可选 Gazebo GUI 和 RViz2

当前配置针对实时运行进行了优化：Livox 点云目标频率为 10 Hz，RGB 图像目标频率为 30 Hz。在测试主机上同时订阅两者时，实测约为 9.9 Hz 和 29.6 Hz。

## 工作空间结构

```text
gazebo_sim/
├── src/
│   ├── vehicle_gazebo_sim/          # 车辆、世界、启动文件和 RViz 配置
│   ├── livox_laser_simulation_RO2/  # Livox Gazebo 传感器插件
│   ├── livox_ros_driver2/            # Livox ROS 2 消息定义
│   └── realsense_gazebo_plugin/      # PAL RealSense ROS 2 Gazebo 插件
├── build/
├── install/
└── log/
```

主要文件：

```text
vehicle_gazebo_sim/launch/simulation.launch.py
vehicle_gazebo_sim/launch/hui_path.launch.py
vehicle_gazebo_sim/urdf/simple_car.urdf.xacro
vehicle_gazebo_sim/worlds/test_world.world
vehicle_gazebo_sim/worlds/hui_path.world
vehicle_gazebo_sim/rviz/sensors.rviz
livox_laser_simulation_RO2/scan_mode/mid360.csv
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- `gazebo_ros_pkgs`
- `xacro`
- `robot_state_publisher`
- RViz2（仅在需要可视化时使用）

可安装常用依赖：

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-robot-state-publisher \
  ros-humble-rqt-image-view \
  ros-humble-rviz2 \
  ros-humble-xacro
```

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

Livox 插件包含大量射线碰撞计算，应使用 Release 构建。插件本身也会在未指定构建类型时默认选择 Release。

## 启动

每个新终端都需要先加载环境：

```bash
cd /home/lxh/nanhu-research-navigation/src/gazebo_sim
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
```

启动 Gazebo 和车辆：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py
```

启动“回”字形通道（道路中心线每边长 10 m，通道净宽 3 m，车辆默认位于下侧通道中心）：

```bash
ros2 launch vehicle_gazebo_sim hui_path.launch.py
```

该场景的墙厚为 0.2 m、高度为 1.5 m。外墙外轮廓为 13.4 m × 13.4 m，内墙外轮廓为
7 m × 7 m，外墙内表面与内墙外表面之间的净距离为 3 m。
道路中心线四个拐点为 (±5, ±5) m，每段长 10 m，一圈长 40 m；车辆默认出生位置为 (0, -5) m。

无 Gazebo 图形界面运行：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py gui:=false
```

同时启动 RViz：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py rviz:=true
```

指定车辆出生位置和航向角：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py \
  x:=1.0 y:=2.0 z:=0.15 yaw:=1.57
```

可用启动参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `use_sim_time` | `true` | ROS 节点使用 Gazebo 仿真时钟 |
| `world` | `test_world.world` | `simulation.launch.py` 加载的世界文件绝对路径 |
| `gui` | `true` | 是否启动 Gazebo 客户端界面 |
| `rviz` | `false` | 是否启动 RViz2 |
| `x` | `0.0` | 车辆出生位置 X，单位 m |
| `y` | `0.0` | 车辆出生位置 Y，单位 m |
| `z` | `0.15` | 车辆出生高度，单位 m |
| `yaw` | `0.0` | 车辆出生航向角，单位 rad |

## ROS 2 接口

| 方向 | 话题 | 消息类型 | 频率/说明 |
|---|---|---|---|
| 输入 | `/cmd_vel` | `geometry_msgs/msg/Twist` | 四轮差速速度控制 |
| 输出 | `/odom` | `nav_msgs/msg/Odometry` | Gazebo 里程计 |
| 输出 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Livox 格式，10 Hz |
| 输出 | `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | 标准点云，10 Hz，`livox_frame` |
| 输出 | `/livox/imu` | `sensor_msgs/msg/Imu` | 雷达 IMU，200 Hz |
| 输出 | `/camera/color/image_raw` | `sensor_msgs/msg/Image` | 1280×720、RGB8、30 Hz |
| 输出 | `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB 相机内参 |
| 输出 | `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 1280×720、与 RGB 像素对齐、30 Hz |
| 输出 | `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 与 RGB 相同的内参和 `camera_color_optical_frame` |
| 输出 | `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | RGB-D 点云 |
| 输出 | `/camera/infra1/image_rect_raw` | `sensor_msgs/msg/Image` | 左红外图像，424×240，30 Hz |
| 输出 | `/camera/infra1/camera_info` | `sensor_msgs/msg/CameraInfo` | 左红外相机内参 |
| 输出 | `/camera/infra2/image_rect_raw` | `sensor_msgs/msg/Image` | 右红外图像，424×240，30 Hz |
| 输出 | `/camera/infra2/camera_info` | `sensor_msgs/msg/CameraInfo` | 右红外相机内参 |
| 输出 | `/camera/imu` | `sensor_msgs/msg/Imu` | D435i 组合 IMU，200 Hz，`camera_imu_optical_frame` |
| 输出 | `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo 仿真时钟 |
| 输出 | `/tf`、`/tf_static` | `tf2_msgs/msg/TFMessage` | 机器人坐标变换 |

差速插件默认不广播 `odom` 到 `base_footprint` 的 TF，避免和后续 SLAM 或定位系统产生重复 TF。

## 控制车辆

持续发送前进指令：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

原地左转：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

停止车辆：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

## 检查传感器

查看相关话题：

```bash
ros2 topic list | grep -E 'livox|camera'
```

检查点云和图像频率：

```bash
ros2 topic hz /livox/lidar/pointcloud
ros2 topic hz /camera/color/image_raw
```

检查一帧消息：

```bash
ros2 topic echo --once /livox/lidar/pointcloud
ros2 topic echo --once /camera/color/image_raw
```

可使用 RViz 的 `PointCloud2` 显示 `/livox/lidar/pointcloud`，Fixed Frame 设置为 `base_footprint` 或 `livox_frame`。查看 RGB 图像可以使用 RViz 的 `Image` 显示，或运行：

```bash
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw
```

## Livox 仿真实现

`ros2_livox_simulation` 插件读取约 80 万条 Mid-360 方位角和天顶角时序。每帧从扫描序列的下一段继续推进，因此输出保留 Livox 的非重复扫描特征，而不是规则二维激光栅格。

原始扫描段包含 30,000 个方向。当前配置通过 `downsample=20` 均匀抽取 1,500 条 ODE 射线，以便 Gazebo Classic 在目标主机上保持实时运行。这个参数只减少单帧点密度，不改变 10 Hz 扫描周期和扫描轨迹推进速度。

需要更高点密度时，可以在 `urdf/simple_car.urdf.xacro` 中减小 `downsample`，但 ODE 射线碰撞开销会明显增加，可能同时降低雷达和相机的墙钟发布频率。

## RealSense D435i 仿真实现

相机模型位于 `urdf/realsense_d435i.urdf.xacro`，参考 PAL Robotics 的 ROS 2 插件和 ROS 1 D435i 描述实现。模型包含独立的 depth、color、左右红外和 IMU 坐标系；所有光学坐标系遵循 REP-103/OpenCV 约定：+X 向右、+Y 向下、+Z 向前。

`realsense_gazebo_plugin` 负责发布 RGB、16UC1 深度图、双红外图像、相机内参和带 RGB 字段的有组织点云。D435i IMU 由 `libgazebo_ros_imu_sensor.so` 提供，并发布组合角速度和线加速度到 `/camera/imu`。

当前世界使用 1 ms 物理步长和 1000 Hz 物理更新率，使 200 Hz IMU 的 5 ms 周期能够被 Gazebo 准确调度。综合负载测试中 RGB、深度和红外约为 29.5 Hz，IMU 约为 196 Hz，Livox 约为 9.8 Hz。

## 常见问题

### 没有相机或 Livox 话题

如果 Gazebo 日志出现以下错误：

```text
Failed to load plugin libgazebo_ros_camera.so: libCameraPlugin.so: cannot open shared object file
Failed to load plugin libros2_livox.so: libRayPlugin.so: cannot open shared object file
```

说明启动终端没有加载 Gazebo 插件路径。停止仿真，在同一个终端按顺序执行：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
ros2 launch vehicle_gazebo_sim simulation.launch.py
```

### 话题存在但没有数据

先检查发布者：

```bash
ros2 topic info -v /livox/lidar/pointcloud
ros2 topic info -v /camera/color/image_raw
```

图像和 RGB-D 点云插件可能按订阅者按需工作。启动 `ros2 topic hz`、RViz 或图像查看器后再观察数据。

### 频率低于目标值

检查仿真是否以接近实时速度运行：

```bash
ros2 topic hz /clock
ps -C gzserver -o pid,pcpu,pmem,etime,args
```

建议：

- 使用 `-DCMAKE_BUILD_TYPE=Release` 重新编译 Livox 插件。
- 不需要 Gazebo 界面时使用 `gui:=false`。
- 确认没有启动多个 `gzserver` 实例。
- 避免同时打开多个高分辨率图像和点云显示窗口。
- 若修改了 Livox `downsample`，点数越多，CPU 开销越大。

### 找不到 `livox_ros_driver2/msg/CustomMsg`

确认已编译消息包，并在当前终端加载工作空间：

```bash
cd /home/lxh/nanhu-research-navigation/src/gazebo_sim
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

## 清理构建产物

需要完全重新构建时，可以删除工作空间中的 `build`、`install` 和 `log` 目录，然后重新执行编译命令。删除前应确认终端位于本工作空间，避免误删其他路径。

## 许可证

`vehicle_gazebo_sim` 使用 Apache-2.0 许可证。Livox 仿真插件和驱动相关代码遵循各自目录中的许可证文件。
