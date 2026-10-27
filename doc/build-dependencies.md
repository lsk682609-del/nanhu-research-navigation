# 编译依赖记录

本文档记录编译 `nav` 工作区全部包所需的额外依赖。

## 环境信息

- **系统**：Ubuntu 22.04 (L4T R36.5, Jetson Orin NX)
- **ROS2**：Humble (已安装于 `/opt/ros/humble`)

---

## 一、完整安装命令

```bash
# 构建工具
sudo apt install -y python3-colcon-common-extensions

# 系统库
sudo apt install -y \
  libasio-dev \
  libfmt-dev \
  libyaml-cpp-dev \
  libgoogle-glog-dev \
  libceres-dev \
  libsuitesparse-dev

# OpenCV 4.8.0 (NVIDIA Jetson 源，替换 Ubuntu 的 4.5.4)
# 如果已安装 4.5.4 相关包，先卸载：
sudo apt remove -y libopencv-core4.5d libopencv-calib3d4.5d libopencv-contrib4.5d \
  libopencv-dnn4.5d libopencv-features2d4.5d libopencv-flann4.5d libopencv-highgui4.5d \
  libopencv-imgcodecs4.5d libopencv-imgproc4.5d libopencv-ml4.5d libopencv-objdetect4.5d \
  libopencv-photo4.5d libopencv-shape4.5d libopencv-stitching4.5d libopencv-video4.5d \
  libopencv-videoio4.5d libopencv-viz4.5d python3-opencv
sudo apt install -y libopencv
sudo apt install -y ros-humble-cv-bridge

# ROS2 依赖包
sudo apt install -y \
  ros-humble-libg2o \
  ros-humble-geographic-msgs \
  ros-humble-geodesy \
  ros-humble-bond \
  ros-humble-bondcpp \
  ros-humble-diagnostic-updater \
  ros-humble-angles \
  ros-humble-laser-geometry \
  ros-humble-pcl-conversions \
  ros-humble-rclcpp-components \
  ros-humble-rclcpp-lifecycle \
  ros-humble-rosbag2-cpp \
  ros-humble-message-filters \
  ros-humble-tf2-sensor-msgs \
  ros-humble-visualization-msgs \
  ros-humble-action-msgs \
  ros-humble-lifecycle-msgs \
  ros-humble-std-srvs
```

## 二、依赖分类

### 2.1 构建工具

| 包名 | 用途 |
|------|------|
| `python3-colcon-common-extensions` | colcon 构建系统 |

### 2.2 系统 C++ 库

| 包名 | 用途 | 被依赖的包 |
|------|------|-----------|
| `libasio-dev` | 异步网络库 (CAN/串口通信) | `ugv_sdk` |
| `libfmt-dev` | 现代 C++ 格式化库 | `nav2_input_bridge` |
| `libyaml-cpp-dev` | YAML 解析库 | `nav2_util` |
| `libgoogle-glog-dev` | 日志库 | `laser_slam_3d` |
| `libceres-dev` | 非线性优化库 | `laser_slam_3d` (通过 g2o 间接依赖) |
| `libsuitesparse-dev` | 稀疏矩阵库 (含 CSparse) | `laser_slam_3d` |

### 2.3 OpenCV

| 包名 | 说明 |
|------|------|
| `libopencv` (4.8.0) | NVIDIA Jetson 单包 OpenCV，替换 Ubuntu 的 4.5.x 拆分包 |
| `ros-humble-cv-bridge` | ROS2 与 OpenCV 的桥接（需重新安装以适配新版 so） |

### 2.4 ROS2 包

| 包名 | 被依赖的包 |
|------|-----------|
| `ros-humble-libg2o` | `laser_slam_3d` (图优化后端) |
| `ros-humble-geographic-msgs` | `nav2_input_bridge` |
| `ros-humble-geodesy` | `laser_slam_3d` |
| `ros-humble-bond` / `ros-humble-bondcpp` | `nav2_util` |
| `ros-humble-diagnostic-updater` | `nav2_lifecycle_manager` |
| `ros-humble-angles` | `nav2_controller` |
| `ros-humble-laser-geometry` | `laser_slam_3d` |
| `ros-humble-pcl-conversions` | `laser_slam_3d` |
| `ros-humble-rclcpp-components` | `nav2_*` |
| `ros-humble-rclcpp-lifecycle` | `nav2_lifecycle_manager` |
| `ros-humble-rosbag2-cpp` | `nav2_log_record` |
| `ros-humble-message-filters` | `laser_slam_3d` |
| `ros-humble-tf2-sensor-msgs` | `laser_slam_3d` |
| `ros-humble-visualization-msgs` | `laser_slam_3d` |
| `ros-humble-action-msgs` | `nav2_*` |
| `ros-humble-lifecycle-msgs` | `nav2_lifecycle_manager` |
| `ros-humble-std-srvs` | `laser_slam_3d` |

## 三、构建

```bash
cd /home/lxh/nanhu-research-navigation
source /opt/ros/humble/setup.bash
bash scripts/build.sh
```

---

## 四、常见问题

### OpenCV 版本不匹配

**现象**：
```
The imported target "opencv_core" references the file
   "/usr/lib/libopencv_core.so.4.8.0"
but this file does not exist.
```

**原因**：NVIDIA Jetson 的 `libopencv-dev` 为 4.8.0，但 Ubuntu 的 runtime 是 4.5.4。解决见上文 OpenCV 安装步骤。

### livox_ros_driver2 并行编译找不到

**原因**：`nav2_input_bridge` 的 `package.xml` 缺少 `<depend>livox_ros_driver2</depend>` 声明，colcon 不知道要等 livox 先编完。

**修复**：已在 `src/navigation2/bridge/nav2_input_bridge/package.xml` 中添加了该依赖。

### laser_slam_3d 编译失败（缺少 CSparse/G2O/Ceres）

**现象**：`CSPARSE_INCLUDE_DIR-NOTFOUND` / `G2O_INCLUDE_DIR-NOTFOUND` / `ceres/internal/fixed_array.h: No such file`

**解决**：安装 `libsuitesparse-dev`、`ros-humble-libg2o`、`libceres-dev`。
