# plan_env：深度与语义融合环境感知

`plan_env` 是一个独立的 ROS 2 感知功能包，用于将对齐彩色图、深度图和 TwinLiteNetPlus 可通行区域掩码转换为局部环境地图。当前只采用无里程计的单帧方式，每组新输入都会完全重建当前局部地图。

当前阶段只实现感知和可视化输出，不修改现有导航框架，也不向 Nav2 Costmap 注入数据。`/grid_map/fused_costmap` 仅作为后续导航接入时的预留输出。

## 1. 功能概览

该功能包完成以下工作：

- 根据深度图恢复相机点云；
- 在 `base_link` 下逐帧重建地图，不维护历史帧占用概率；
- 通过一个配置项选择纯模型边界、纯深度几何或两者融合；
- 在功能包内部运行精简后的 TwinLiteNetPlus 道路分割网络，不依赖工程根目录下的原始模型仓库；
- 通过射线清空和占用更新区分自由空间与障碍物；
- 从 TwinLiteNetPlus 掩码中提取与机器人相连的左边界、右边界和前边界；
- 使用对齐深度恢复稀疏边界位置，并直接栅格化到二维地图；
- 输出纯深度地图、边界地图、融合地图和可选的稀疏边界调试点云；
- 提供相机外参标定工具，并将结果写入 YAML 文件。

## 2. 系统原理

### 2.1 数据流程

```text
/camera/color/image_raw
          │
          ▼
 TwinLiteNetPlus
          │ mono8 可通行掩码
          ▼
/twinlitenet/drivable_mask ───────┐
                                  ├─► 连通域 ─► 左/右/前边界 ─► 深度恢复 ─► 二维边界栅格
/camera/depth/image_raw ──────────┤                                      │
/camera/color/camera_info ────────┘                                      ├─► 二维按位融合
                                                                         │
深度图 ─► 稀疏三维点 ─► 射线更新 ─► 单帧二维几何栅格 ─────────────────────┘
```

语义相关模式对深度图和语义掩码执行近似时间同步，默认允许的最大时间差为 80 ms；纯深度模式只订阅深度图。三种模式均不创建里程计订阅。由于深度已经对齐到彩色图像，深度图、掩码和 `CameraInfo` 必须对应同一图像尺寸和像素坐标。

### 2.2 深度图反投影

根据彩色相机内参，将深度像素 `(u, v, d)` 恢复为相机光学坐标系下的三维点：

```text
z = d
x = (u - cx) × z / fx
y = (v - cy) × z / fy
```

其中 `fx`、`fy`、`cx`、`cy` 来自 `/camera/color/camera_info`。`16UC1` 深度图默认以毫米表示，使用 `depth_scale: 1000.0` 转换为米；`32FC1` 深度图直接按米处理。

点通过 YAML 中的 `T_base_camera_optical` 从相机光学坐标系转换到机器人坐标系，不再进行里程计位姿变换：

```text
p_base = T_base_camera_optical × p_camera
```

### 2.3 几何占用地图

几何层采用三维栅格和射线更新：

- 从相机原点到深度测量点执行射线遍历，沿途栅格作为自由空间更新；
- 射线末端作为占用空间更新；
- 每组有效输入到达后清空旧地图，不执行 hit/miss 概率投票：射线途经体素直接设为自由、有效深度终点直接设为占用，并保证占用终点不会被其他射线的自由更新覆盖；
- gridmap 内不执行障碍膨胀，膨胀由后续 Nav2 costmap 统一处理。

这种方法不仅保留障碍点，还显式表示传感器已经观测到的自由空间。

### 2.4 可通行域边界提取

推理节点以 `output.mask_threshold` 将模型概率转换为 `mono8` 掩码。地图节点不再订阅和投影整幅 `32FC1` 概率图，而是执行以下轻量流程：

1. 在掩码中寻找与图像底部中心区域相连的可通行连通域，过滤远处孤立误检；
2. 按行取得该连通域的最左和最右像素，形成左右边界；
3. 按列取得最上方像素，形成前边界；
4. 在边界内侧的 `5×5` 深度窗口中取中值，降低语义边缘与深度跳变造成的抖动；
5. 深度无效时可使用已标定相机姿态与 `semantic.ground_height` 做地面射线交点回退；
6. 将边界点转换到 `base_link`/`map` 后直接用 Bresenham 算法画入二维栅格。

边界不会被构造成密集三维墙，也不会再次执行三维投影。`boundary.max_forward_distance` 默认将机器人前向距离限制为 6 m；道路延伸出图像或前边界深度不可用时，会在 6 m 处用左右边界闭合一条横向虚拟边界。相机近处且位于正前方小区域内的边界由 `near_ignore_distance` 和 `center_ignore_half_width` 忽略。

边界在本功能包中始终保持单栅格宽。车辆轮廓和安全距离由后续 Nav2 inflation layer 统一处理，避免重复膨胀。

### 2.5 融合规则

`/grid_map/depth_costmap` 在每个新深度帧的射线投影过程中直接生成：高度位于
`projection.obstacle_min_height` 至 `projection.obstacle_max_height` 范围内的射线途经格记为自由，
有效深度终点记为占用，并采用终点占用优先。取值为 `100/0/-1`，分别表示占用、已观测自由和未知。
该实现不再维护概率累积缓冲区。选择 `perception.mode: depth_only` 后，地图节点不会订阅或等待掩码，且 launch 不会启动模型推理进程，此时该话题就是完全独立的纯深度二维占据地图。

每次完成新地图后，程序只计算当前模式需要的 OccupancyGrid 并保存消息缓存；50 ms 可视化定时器只发布缓存，不再重复扫描三维体素。
因此话题发布频率可能高于实际地图更新频率，应通过消息时间戳判断数据是否来自新帧。

`/grid_map/fused_costmap` 使用 ROS `nav_msgs/OccupancyGrid` 的标准数值：

| 条件 | 输出值 | 含义 |
| --- | ---: | --- |
| 深度终点占用 | `100` | 几何障碍 |
| 左、右或前语义边界 | `100` | 单栅格宽的虚拟墙 |
| 深度射线已观测自由 | `0` | 自由空间 |
| 深度未观测且无边界 | `-1` | 未知 |

当前输出只用于感知结果检查。后续接入导航时，可将其转换为轻量 Nav2 Costmap Layer；现阶段没有更改导航规划逻辑。

## 3. 坐标系与外参

默认使用以下坐标约定：

- `base_link`：`x` 向前、`y` 向左、`z` 向上；
- 相机光学坐标系：`x` 向右、`y` 向下、`z` 向前；
- 输出坐标系由 `mapping.frame_id` 指定，默认使用 `base_link`。

默认配置假设相机高度约为 0.60 m，并使用标准的光学坐标系到机器人坐标系旋转。相机的 `x`、`y`、`yaw` 安装偏差默认均为 0，应在实机测量或标定后写入 YAML。

> 注意：将完整的 4×4 外参直接设为单位矩阵，会把光学坐标轴当成 `base_link` 坐标轴，通常会导致点云方向错误。配置中的“零偏差”指安装平移偏差和偏航修正为零，不代表省略光学坐标系到机器人坐标系的标准轴转换。

## 4. 输入与输出

### 4.1 输入话题

| 默认话题 | 消息类型 | 说明 |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | TwinLiteNetPlus 彩色输入 |
| `/camera/color/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | Gemini 335L 可选 JPEG 彩色输入，由解码节点恢复为 `bgr8` |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 已对齐到彩色图的深度，支持 `16UC1` 和 `32FC1` |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/msg/CompressedImage` | Gemini 335L 可选 `16UC1; compressedDepth` 输入 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 对齐投影所使用的彩色相机内参 |
| `/twinlitenet/drivable_mask` | `sensor_msgs/msg/Image` | TwinLiteNetPlus 阈值化后的 `mono8` 可通行掩码 |
| `/camera/gyro_accel/sample` | `sensor_msgs/msg/Imu` | 标定工具使用的 IMU 数据 |

如果相机驱动分别发布 `/camera/accel/sample` 和 `/camera/gyro/sample`，需要先生成标准 `sensor_msgs/msg/Imu` 消息，或者在后续实机驱动中使用合并后的 `/camera/gyro_accel/sample`。运行地图节点本身不依赖 IMU，IMU 只用于可选标定。

### 4.2 输出话题

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/grid_map/depth_cloud` | `sensor_msgs/msg/PointCloud2` | 当前深度帧转换后的点云 |
| `/grid_map/occupancy` | `sensor_msgs/msg/PointCloud2` | 几何占用栅格点云 |
| `/grid_map/depth_costmap` | `nav_msgs/msg/OccupancyGrid` | 深度几何体素投影得到的二维占据地图 |
| `/grid_map/drivable_boundary_cloud` | `sensor_msgs/msg/PointCloud2` | 稀疏边界调试点；由 `boundary.publish_debug_cloud` 控制 |
| `/grid_map/drivable_boundary_costmap` | `nav_msgs/msg/OccupancyGrid` | 仅包含单栅格宽语义边界的二维地图 |
| `/grid_map/fused_costmap` | `nav_msgs/msg/OccupancyGrid` | 深度地图与边界地图按占用优先融合的结果 |

地图节点只创建当前模式需要的发布器：`semantic_only` 只提供边界地图与边界调试点，`depth_only` 只提供深度地图与几何点云，`fused` 提供全部三种 OccupancyGrid。

`plan_env` 只输出原始几何占据和语义融合结果，不执行水平或垂直障碍膨胀；机器人外形和安全距离膨胀由后续 Nav2 costmap 统一处理。

TwinLiteNetPlus 推理节点还会输出：

| 话题 | 消息类型 | 内容 |
| --- | --- | --- |
| `/twinlitenet/drivable_mask` | `sensor_msgs/msg/Image` | 地图使用的阈值化道路掩码，编码为 `mono8` |
| `/twinlitenet/drivable_probability` | `sensor_msgs/msg/Image` | 可选 `32FC1` 概率调试输出；默认关闭 |
| `/twinlitenet/lane_mask` | `sensor_msgs/msg/Image` | 可选车道线二值掩码，编码为 `mono8`；由 `lane.enabled` 和 `lane.publish_mask` 控制 |
| `/twinlitenet/lane_probability` | `sensor_msgs/msg/Image` | 可选车道线 `32FC1` 概率图；默认关闭 |
| `/twinlitenet/segmentation` | `sensor_msgs/msg/Image` | 彩色分割叠加图，绿色为可通行区域、红色为车道线；由 `debug.enabled` 控制 |
| `/twinlitenet/inference_time_ms` | `std_msgs/msg/Float32` | 神经网络前向推理耗时 |
| `/twinlitenet/total_processing_time_ms` | `std_msgs/msg/Float32` | 包含预处理和发布准备的总耗时 |

标定节点在启用地面平面优化时还会发布原始点云、机器人坐标系点云和地面内点点云，便于在 RViz2 中检查标定结果。

## 5. 编译

### 5.1 依赖

- Ubuntu 与 ROS 2 Humble；
- OpenCV；
- PCL；
- PyTorch、NumPy、OpenCV Python 和 `rclpy` 推理运行时；
- Conda 环境 `twinLiteNetPlus` 为可选项，没有 Conda 时可使用系统 Python 在 CPU 上推理。

功能包已经内置精简模型代码和默认 `medium.pth` 权重，不再需要工程根目录中的 `TwinLiteNetPlus` 文件夹。Python 依赖清单位于：

```text
src/plan_env/requirements-conda.txt
```

PyTorch 应根据本机 CUDA 或后续 RK3588 推理后端单独安装；不要直接通过该清单覆盖已经可用的 PyTorch。

启动文件会自动检查运行环境：优先使用可用的 `twinLiteNetPlus` Conda 环境；找不到 Conda、环境不存在或环境缺少推理依赖时，检查系统 Python，并在依赖完整时强制使用 CPU。如果系统 Python 同样缺少依赖，推理节点不会启动，但会输出明确的警告且不会产生 `FileNotFoundError: conda`。此时需先为系统 Python 安装对应平台的 PyTorch、NumPy 和 OpenCV。

### 5.2 编译功能包

在工程根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select plan_env
source install/setup.bash
```

编译安装后，`plan_env` 会包含以下推理运行文件：

```text
install/plan_env/lib/plan_env/plan_env_drivable_inference
install/plan_env/lib/plan_env/plan_env_perception/
install/plan_env/share/plan_env/models/medium.pth
```

重构版保留共享编码器、可通行区域解码器和可选车道线解码器。`lane.enabled: false` 时不会构造、加载或执行车道线分支，从而保持原有的低开销运行方式；启用后两个任务共享一次编码器前向计算。训练、数据集、评估和视频演示代码均不进入 `plan_env`。

## 6. 相机外参标定

标定配置位于：

```text
src/plan_env/config/camera_calibration.yaml
```

关键开关含义如下：

| 参数 | 含义 |
| --- | --- |
| `calibration.enabled` | 是否运行标定流程；关闭后不处理传感器数据，也不会输出标定 YAML |
| `calibration.use_imu` | 是否利用静止状态下的重力方向估计相机滚转角和俯仰角 |
| `calibration.use_ground_plane_refinement` | 是否再利用深度点云拟合地面，优化滚转角、俯仰角和安装高度 |

IMU 重力只能约束滚转角和俯仰角，不能可靠估计 `x`、`y` 和 `yaw`。这三项应通过实测填写到配置中，默认值均为 0。

### 6.1 推荐标定方式

1. 将机器人停放在水平、平整且前方地面无遮挡的位置；
2. 保持机器人和相机静止；
3. 先启用 IMU 标定，检查转换后的点云是否水平；
4. 如果地面深度质量稳定，再启用地面平面优化；
5. 将生成的 YAML 作为地图节点的外参配置加载。

执行标定：

```bash
ros2 launch plan_env camera_calibration.launch.py \
  config:=src/plan_env/config/camera_calibration.yaml \
  imu_topic:=/camera/gyro_accel/sample \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  output_yaml:=/tmp/gemini330l_extrinsic.yaml
```

标定完成后会生成 `/tmp/gemini330l_extrinsic.yaml`。其中 `grid_map_node.ros__parameters.grid_map.depth_extrinsic` 是按行排列的 4×4 齐次变换矩阵，可直接作为地图启动文件的参数覆盖层。

地面平面优化不是地图正常运行的必需条件。如果现场地面不平、深度空洞较多或相机前方存在大量障碍，应保持该选项关闭，以免使用错误平面修正外参。

## 7. 运行方式

### 7.1 选择地图模式

每个设备 YAML 顶部只有一个模式配置项：

```yaml
/**:
  ros__parameters:
    perception.mode: fused
```

`perception.mode` 支持三个值：

| 配置值 | 模型推理 | 深度射线地图 | 发布的地图 |
| --- | --- | --- | --- |
| `semantic_only` | 启用 | 不构建几何障碍层 | `/grid_map/drivable_boundary_costmap` |
| `depth_only` | 不启动 | 启用 | `/grid_map/depth_costmap` |
| `fused` | 启用 | 启用 | 上述两层及 `/grid_map/fused_costmap` |

三种模式均为单帧并输出到 `mapping.frame_id`。`semantic_only` 仍使用对齐深度估计模型边界的空间位置，但不会把普通深度障碍写入 costmap；这是现有边界实现方式所必需的。切换模式后需要重新启动 launch，因为订阅、推理进程和发布器会按模式重新创建。

### 7.2 一体化启动

一体化启动会同时运行 TwinLiteNetPlus 推理节点和 `plan_env` 地图节点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch plan_env perception.launch.py \
  conda_env:=twinLiteNetPlus \
  extrinsic_config:=/tmp/gemini330l_extrinsic.yaml
```

当 YAML 设置为 `depth_only` 时，一体化 launch 会跳过 Conda/系统 Python 推理进程，只启动深度地图节点。

如果尚未生成标定文件，可省略 `extrinsic_config`，程序会使用 `depth_grid_map.yaml` 中的默认外参。

默认权重由 `plan_env` 自动定位，不需要填写 `weight`。一体化启动可使用以下推理参数：

| Launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `inference_runtime` | `auto` | `auto` 自动选择 Conda/系统 Python；也可设为 `conda` 或 `system` |
| `conda_env` | `twinLiteNetPlus` | PyTorch 推理环境 |
| `system_python` | 启动 ROS 2 的 Python | 无 Conda 时使用的 Python 可执行文件 |
| `model_config` | `medium` | 网络宽度配置，必须与权重匹配 |
| `weight` | 包内 `models/medium.pth` | 权重路径；使用其他模型时可覆盖 |
| `image_size` | `640` | 推理输入尺寸，必须为 32 的正整数倍 |
| `device` | `auto` | 自动选择 `cuda:0` 或 `cpu`，也可显式指定 |

推理和 gridmap 的话题名统一由 `src/plan_env/config/depth_grid_map.yaml` 控制，不再通过 Launch 参数分别配置：

```yaml
/**:
  ros__parameters:
    perception.mode: fused

plan_env_drivable_inference:
  ros__parameters:
    topics.color_image: /camera/color/image_raw
    topics.drivable_probability: /twinlitenet/drivable_probability
    topics.drivable_mask: /twinlitenet/drivable_mask
    topics.lane_probability: /twinlitenet/lane_probability
    topics.lane_mask: /twinlitenet/lane_mask
    output.publish_probability: false
    output.publish_mask: true
    output.mask_threshold: 0.65
    lane.enabled: false
    lane.publish_probability: false
    lane.publish_mask: true
    lane.mask_threshold: 0.50
    input_qos.reliability: best_effort
    input_qos.depth: 1
    watchdog.enabled: true
    watchdog.input_timeout_sec: 3.0
    watchdog.processing_timeout_sec: 10.0
    watchdog.check_period_sec: 1.0
    debug.enabled: true
    debug.image_topic: /twinlitenet/segmentation

grid_map_node:
  ros__parameters:
    topics.depth_image: /camera/depth/image_raw
    topics.camera_info: /camera/color/camera_info
    topics.drivable_mask: /twinlitenet/drivable_mask
    mapping.frame_id: base_link
    topics.depth_costmap: /grid_map/depth_costmap
    topics.boundary_costmap: /grid_map/drivable_boundary_costmap
    topics.boundary_cloud: /grid_map/drivable_boundary_cloud
    topics.fused_costmap: /grid_map/fused_costmap
```

- `topics.*`：配置推理节点和 gridmap 节点使用的输入、输出话题；两处的 `topics.drivable_mask` 应保持一致；
- `output.publish_probability: false`：不构造和发布大尺寸 `32FC1` ROS 消息，降低内存带宽；仅在分析模型原始概率时打开；
- `output.publish_mask: true`：发布边界提取所需的 `mono8` 掩码；
- `output.mask_threshold`：概率转掩码和绿色调试叠加图共同使用的阈值；
- `lane.enabled`：是否加载并执行 TwinLiteNetPlus 车道线解码器；默认关闭，车道线不参与 gridmap；
- `lane.publish_probability`：是否发布车道线 `32FC1` 概率图，通常保持关闭以降低带宽；
- `lane.publish_mask`：是否创建 `/twinlitenet/lane_mask` 发布器；仅在 `lane.enabled: true` 时生效；
- `lane.mask_threshold`：车道线概率转为二值掩码及红色调试叠加的阈值；
- `debug.enabled: true`：允许发布叠加图；只有存在订阅者时才执行图像叠加和消息构造；
- `debug.enabled: false`：不创建叠加图发布器，也不执行相应处理，可降低 CPU 和内存带宽占用；
- `debug.image_topic`：调试叠加图话题名称；
- `input_qos.reliability`：彩色图订阅可靠性，可设为 `reliable` 或 `best_effort`，应与相机发布端匹配；
- `input_qos.depth`：彩色图订阅的 `KEEP_LAST` 队列深度，实时推理建议使用 `1`；
- `watchdog.input_timeout_sec`：超过该时间没有进入彩色图回调时，自动销毁并重建订阅；
- `watchdog.processing_timeout_sec`：输入仍正常但单次推理超过该时间时输出错误日志，便于区分 DDS 断流和 CUDA 停滞。

在 RViz2 中添加 `Image`，选择 `/twinlitenet/segmentation` 即可查看结果：绿色表示可通行区域，启用车道线检测后红色表示车道线。也可单独查看 `/twinlitenet/lane_mask`。修改 YAML 后需要重新启动推理节点。

也可以只启动包内推理节点：

```bash
# Conda 运行
conda run --no-capture-output -n twinLiteNetPlus \
  python -u install/plan_env/lib/plan_env/plan_env_drivable_inference \
  --ros-args --params-file src/plan_env/config/depth_grid_map.yaml

# 无 Conda，使用系统 Python 和 CPU
python3 -u install/plan_env/lib/plan_env/plan_env_drivable_inference \
  --device cpu \
  --ros-args --params-file src/plan_env/config/depth_grid_map.yaml
```

### 7.3 Jetson Orin GPU 启动

Jetson Orin（Ubuntu 22.04、JetPack 6.2、CUDA 12.6、Python 3.10、ROS 2 Humble）可直接使用通过 `--system-site-packages` 创建的 venv。默认环境路径为 `~/venvs/twinlite`，启动文件会使用该环境的 Python、`cuda:0` 和 FP16 模型推理。该启动方式默认加载 `depth_grid_map_jetson_orin_realsense_d435.yaml`，使用 D435 对齐深度并以单帧模式运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch plan_env perception_jetson_orin.launch.py
```

可先验证环境：

```bash
~/venvs/twinlite/bin/python -c \
  "import cv2, numpy, rclpy, torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
```

如果 venv 或外参文件位于其他路径：

```bash
ros2 launch plan_env perception_jetson_orin.launch.py \
  venv_python:=/absolute/path/to/venvs/twinlite/bin/python \
  extrinsic_config:=/tmp/gemini330l_extrinsic.yaml
```

可覆盖的主要参数为 `venv_python`、`model_config`、`weight`、`image_size`、`device`、`grid_config` 和 `extrinsic_config`。正常启动后推理日志中的 `device` 应为 `cuda:0`，`half` 应为 `True`。

D435 当前实测配置为彩色与对齐深度 640×480@30 Hz、对齐深度 `16UC1`、`align_depth.enable=true`、`enable_sync=true`。专用 YAML 使用实时 `CameraInfo`，并记录了实测内参作为回退值。其彩色发布端实测为 `RELIABLE + KEEP_LAST(1)`，因此专用 YAML 默认设置 `input_qos.reliability: reliable` 和 `input_qos.depth: 1`，并启用 3 秒输入看门狗。外参暂按相机水平正装、`camera_link` 位于 `base_link` 上方 0.60 m 且 X/Y 偏移为零设置；实机测得安装位置后应更新 `grid_map.depth_extrinsic`。

### 7.4 Gemini 335L 压缩图像启动

Gemini 335L 专用启动文件会增加一个双路解码节点：将 JPEG 彩色图恢复为 `bgr8`，将 ROS `16UC1; compressedDepth` 去除传输头并解码 PNG，随后把标准 `sensor_msgs/msg/Image` 交给原有推理和地图节点：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch plan_env perception_gemini335l_compressed.launch.py
```

默认配置文件为 `config/depth_grid_map_gemini335l_compressed.yaml`，主要开关如下：

```yaml
compressed_input.enabled: true
compressed_input.color_enabled: true
compressed_input.depth_enabled: true
compressed_input.require_aligned_depth: true
compressed_input.qos.reliability: best_effort
compressed_input.qos.depth: 1
```

解码后的内部话题为：

- `/plan_env/gemini335l/color/image_raw`：`1280×720 bgr8`（尺寸以实机输入为准）；
- `/plan_env/gemini335l/depth/image_raw`：`16UC1` 对齐深度；
- 图像时间戳和 `frame_id` 原样保留，不重新打时间戳。

`compressed_input.require_aligned_depth: true` 会比较彩色和深度的宽高及 `frame_id`。若不一致，节点输出错误日志并停止发布解码深度，防止把未对齐深度通过简单缩放错误融合。只有在关闭语义、使用 `/camera/depth/camera_info` 做纯深度诊断时，才可以临时设为 `false`。

所检查的 `rosbag2_2026_08_26-16_25_06` 中，彩色为 `1280×720 JPEG / camera_color_optical_frame`，深度为 `848×480 16UC1 compressedDepth / camera_depth_optical_frame`。两者时间戳相差约 `0.17 ms`，但没有对齐，bag 中也没有相机间 TF/外参，因此该 bag 可验证解码和彩色推理，不能验证语义深度融合。实机驱动需要先启用 depth-to-color registration，并确保解码后的深度尺寸和 `frame_id` 与彩色一致。

如果系统 Python 缺少 OpenCV，但某个 venv/Conda Python 同时可以导入 `cv2`、`numpy` 和 `rclpy`，可指定：

```bash
ros2 launch plan_env perception_gemini335l_compressed.launch.py \
  decoder_python:=/absolute/path/to/python
```

### 7.5 只运行地图节点

当 TwinLiteNetPlus 推理节点已经单独运行，并持续发布 `/twinlitenet/drivable_mask` 时，可以只启动融合地图：

```bash
ros2 launch plan_env depth_grid_map.launch.py \
  config:=src/plan_env/config/depth_grid_map.yaml \
  extrinsic_config:=/tmp/gemini330l_extrinsic.yaml
```

要单独检查深度几何地图，将对应 YAML 中的配置改为：

```yaml
perception.mode: depth_only
```

然后只启动地图节点。此时节点直接订阅深度图，不订阅
`/twinlitenet/drivable_mask`，主要检查 `/grid_map/depth_cloud`、
`/grid_map/occupancy` 和 `/grid_map/depth_costmap`。完成纯深度检查后，将模式改为
`fused` 并使用 `perception_jetson_orin.launch.py`，再对比
`/grid_map/depth_costmap` 与 `/grid_map/fused_costmap`。参数切换后需要重启地图节点。

### 7.6 检查运行状态

```bash
ros2 topic hz /twinlitenet/drivable_mask
ros2 topic hz /grid_map/depth_cloud
ros2 topic echo /grid_map/depth_costmap --once
ros2 topic echo /grid_map/drivable_boundary_costmap --once
ros2 topic hz /grid_map/drivable_boundary_cloud
ros2 topic echo /grid_map/fused_costmap --once
```

建议在 RViz2 中添加以下显示项：

- `PointCloud2`：`/grid_map/depth_cloud`，检查点云方向和高度；
- `Map`：`/grid_map/depth_costmap`，检查不含语义分类的二维几何栅格；
- `PointCloud2`：`/grid_map/drivable_boundary_cloud`，检查左右及前边界的空间位置；
- `Map`：`/grid_map/drivable_boundary_costmap`，检查单栅格宽二维边界；
- `Map`：`/grid_map/fused_costmap`，检查二维栅格状态。

## 8. 主要参数

地图配置位于：

```text
src/plan_env/config/depth_grid_map.yaml
```

### 8.1 工作模式

| 参数 | 说明 |
| --- | --- |
| `perception.mode` | `semantic_only`、`depth_only` 或 `fused`；三种模式均为单帧 |
| `mapping.frame_id` | 单帧地图输出坐标系，默认 `base_link` |

### 8.2 地图与深度

| 参数 | 说明 |
| --- | --- |
| `grid_map.resolution` | 栅格分辨率，单位为米 |
| `grid_map.size_x/y/z` | 固定单帧三维局部地图范围 |
| `grid_map.depth_filter_mindist` / `grid_map.depth_filter_maxdist` | 有效深度范围 |
| `grid_map.k_depth_scaling_factor` | 整数深度到米的换算比例 |
| `grid_map.skip_pixel` | 深度图采样步长，当前默认 `4`；640×480 图像约采样 1.9 万条射线 |

### 8.3 几何占用

| 参数 | 说明 |
| --- | --- |
| `grid_map.max_ray_length` | 射线更新的最大长度 |

### 8.4 语义边界

| 参数 | 说明 |
| --- | --- |
| `semantic.ground_height` | 地面在地图坐标系中的参考高度 |
| `semantic.ground_tolerance` | 边界深度点允许偏离地面的高度 |
| `boundary.max_forward_distance` | 前向边界最大距离，默认 6 m |
| `boundary.near_ignore_distance` | 可忽略的近处边界前向范围 |
| `boundary.center_ignore_half_width` | 近处忽略区域的左右半宽 |
| `boundary.pixel_step` | 图像边界采样步长，越大 CPU 越低 |
| `boundary.min_component_area` | 可通行连通域最小像素面积 |
| `boundary.depth_window_size` | 边界内侧深度中值窗口，必须是奇数 |
| `boundary.max_neighbor_jump` | 相邻边界点允许连接的最大二维距离 |
| `boundary.seed_roi_width_ratio/height_ratio` | 图像底部中心种子区域相对尺寸 |
| `boundary.use_ground_plane_fallback` | 边界深度无效时是否与标定地面求交 |
| `boundary.publish_debug_cloud` | 是否缓存并发布稀疏边界调试点云 |
| `projection.obstacle_min_height` / `projection.obstacle_max_height` | 几何障碍投影到二维地图时的高度范围 |

`output.mask_threshold` 位于推理节点参数中，控制概率转掩码阈值。首次实机测试时，优先确认外参、深度单位、时间戳和图像对齐，再调整掩码阈值、连通域面积和边界连续性参数。

## 9. 实机验证顺序

建议按以下顺序逐项确认：

1. 彩色图与对齐深度图的分辨率、时间戳和视场一致；
2. TwinLiteNetPlus 掩码与彩色图尺寸一致，且道路区域为白色；
3. 查看启动日志中的 `perception mode`，确认只创建了该模式需要的推理和地图输出；
4. `/grid_map/depth_cloud` 位于车辆前方，地面基本水平且高度正确；
5. 几何障碍物与真实位置一致，移动时不会出现明显重影；
6. `/grid_map/drivable_boundary_cloud` 的左右边界贴合道路两侧，前边界不超过 6 m；
7. `/grid_map/drivable_boundary_costmap` 只有单栅格宽，不存在感知层膨胀；
8. 最后对比 `/grid_map/depth_costmap` 和 `/grid_map/fused_costmap`。

## 10. 常见问题

### 没有语义边界输出

依次检查：

- `/twinlitenet/drivable_mask` 是否持续发布，编码是否为 `mono8`；
- `output.publish_mask` 是否为 `true`；
- 深度图、掩码和 `CameraInfo` 的宽高是否完全一致；
- 检查深度与掩码时间差是否小于同步窗口；
- 可通行区域是否接触图像底部中心种子区，面积是否超过 `boundary.min_component_area`；
- 深度是否有效、外参和 `semantic.ground_height` 是否正确。

### 点云不在车辆前方或地面不水平

通常是 `T_base_camera_optical` 不正确。先确认相机光学坐标轴约定，再执行 IMU 标定；`yaw`、`x` 和 `y` 仍需在 YAML 中测量填写。

### 标定工具无法生成 YAML

检查 IMU 消息是否到达、机器人是否静止、采样数量是否足够，以及角速度和重力模长是否满足配置中的稳定性条件。启用地面优化时，还需确保前方存在足够多的有效地面深度点。

### 车辆运动时地图结果抖动

当前地图只表达最新一帧，不使用里程计补偿或历史累积。运动时的抖动主要来自深度噪声、模型边界变化、时间同步和相机外参；规划侧可根据需要进行时间滤波，但本感知层保持确定性的单帧输出。

## 11. 当前边界

- 本功能包仅完成感知层，不改动现有导航逻辑；
- 没有订阅二维激光点云或激光雷达话题；
- `/grid_map/fused_costmap` 尚未直接配置为 Nav2 Costmap 数据源；
- 当前包内只携带默认的 `medium.pth`；选择 `nano`、`small` 或 `large` 时必须提供对应权重；
- 后续迁移到 RK3588 时，需要替换或适配 PyTorch 推理后端，并重新验证图像吞吐、时间同步和内存占用，但 ROS 2 话题和地图接口可保持不变。
