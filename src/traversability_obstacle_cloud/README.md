# traversability_obstacle_cloud

面向奥比中光（Orbbec）深度相机的 ROS 2 单帧可通行性分析包。每一帧独立完成：

1. IMU 重力方向低通滤波；
2. 点云光学轴转换及 Roll/Pitch 重力对齐；
3. 前视 ROI 裁剪与可选轻量降采样；
4. 构建包含 min/max/mean/count/variance 的局部 2.5D 高程栅格；
5. 根据高度连续性、坡度、粗糙度和台阶生成候选地面；
6. UNKNOWN 短距离桥接；
7. 从车辆前中央种子进行 8 邻域连通扩展；
8. 发布 Ground、Obstacle 和 Unknown 点云。

不进行多帧点云累积，也不包含自由空间体素或离线关键帧模块。

## 接口

当前在线相机输入：

- `/camera/depth/image_raw`：原始深度图，由包内投影节点转换（支持 `16UC1`、`mono16` 和 `32FC1`）
- `/camera/gyro_accel/sample`：同时包含线加速度与角速度的 `sensor_msgs/msg/Imu`
- `/depth/points`：包内生成的光学坐标 `PointCloud2`

默认输出：

- `/ground_points`
- `/traversability/ground_grid_points`：绿色 Ground 栅格中心组成的规则连续地面点云
- `/traversability/ground_boundary_points`：有序 U 形边界点云，依次为左、前、右边界，
  不包含靠近相机的后边界；intensity 1/2/3 分别表示左/右/前
- `/obstacle_points`
- `/unknown_points`
- `/traversability/grid`：栅格中心调试点云，intensity 1 为 Ground、2 为 Obstacle
- `/camera/color/image_raw`：解码后的 `bgr8` RGB 图像
- `/traversability/rgb_overlay`：Ground/Obstacle/Unknown 投影叠加图
- `/traversability/pothole_detections`：RKNN 模型输出的原图坐标二维检测框
- `/traversability/pothole_boundary_points`：检测框与负高度差共同确认后的闭合凹坑边界；
  intensity 为同一帧内的轮廓编号

用于 Nav2 的新增适配输出：

- `/traversability/costmap/depth_obstacles`：从 `/obstacle_points` 中截取指定前向、横向和
  高度范围，再经局部相对地面高度、体素和半径邻域过滤后的深度障碍；
- `/traversability/costmap/depth_ground_rejected`：低于局部拟合地面相对高度门限的点，
  仅用于 RViz 和离线调参，不接入 Nav2；
- `/traversability/costmap/depth_noise`：被半径邻域过滤器剔除的孤立点，仅用于 RViz
  调参与核查，不接入 Nav2；
- `/traversability/costmap/boundary_constraints`：只保留 intensity 1/2 的左右边界，按
  local costmap 分辨率补密并抬到障碍高度窗口内；默认还会执行横向跳点剔除、空间曲率
  平滑和自适应帧间滤波，输出仍以 intensity 1/2 区分左/右。
- `/traversability/costmap/pothole_constraints`：闭合、补密并抬高后的凹坑虚拟边界，作为
  独立 observation source 接入边界障碍层。

适配节点是独立可执行文件，不会改变分类节点原有话题的内容、频率或开关。

除核心输出 `/ground_points` 外，其余输出可以在参数文件中独立关闭。关闭后节点会跳过
对应数据构造与序列化；关闭 RGB 叠加图还会停止订阅彩色图像：

```yaml
publish:
  ground_grid: true
  boundary: true
  obstacle: false
  unknown: false
  grid: false
  rgb_overlay: false
```

`/ground_points` 始终保持真实深度测量点，不进行补点；
`/traversability/ground_grid_points` 每个绿色 Ground 栅格输出一个中心点，同时包含
`max_unknown_gap_cells` 允许的短距离桥接单元，适合需要规则、连续地面表示的下游模块。

## RKNN 凹坑检测与边界融合

`camera_nav2_perception.launch.py` 默认在 RK3588 上启动 `pothole_detector_node`。模型节点
订阅 `/camera/color/image_raw`，保持原图时间戳发布二维检测框。可通过
`use_pothole_detection:=false` 关闭模型节点。捆绑的 RKNN Runtime 只支持 AArch64，
因此 x86 开发机仍可构建地面、融合和适配代码，但不会生成 `pothole_detector_node`。

二维框不会直接作为导航边界。可通行性节点缓存对应时间的重力对齐高程栅格，在 Pothole
框内寻找相对附近 Ground 栅格明显降低的连通区域，再提取闭合轮廓。默认要求时间差不超过
0.05 秒、相对下降至少 0.05 米，并过滤少于 3 个栅格的区域。相关参数位于
`traversability_obstacle_cloud_node.ros__parameters.pothole`。

局部参考面采用平面拟合，拒绝退化、坡度超过25度或RMSE超过0.03米的拟合。
当前实现仅使用有真实深度观测的负高差区域；无深度凹坑不自动生成禁行框。
边界精度受0.15米高程栅格限制，补密不会提高测量精度。
适配器检查整条轮廓，越出ROI或有大缺口时拒绝该轮廓，避免裁剪后错误闭合。
输入超过0.5秒未更新时持续发布空点云。尚需用标定后的实机数据验证三维定位和Nav2绕行。

检测图片保存由同一参数文件控制：

```yaml
pothole_detector_node:
  ros__parameters:
    save:
      enabled: true
      directory: "/home/cat/nanhu-research-navigation/pics"
      write_metadata: true
```

模型仍保留原始两类输出顺序，但节点会过滤 SpeedBump，仅发布、绘制和保存 Pothole。
只有至少检测到一个 Pothole 时才写文件。每帧保存一张 `detection_*.jpg`，图中包含类别、
置信度和检测框；同名 `detection_*.txt` 保存原图尺寸以及每个框的类别、置信度和
`left top right bottom` 原图坐标。`pics/.gitignore` 防止运行数据被提交到仓库。

当最终 `/traversability/costmap/pothole_constraints` 非空时，适配节点还会保存
`pothole_constraints_<秒>_<纳秒>_<写入标识>.pcd`（二进制XYZI）及同名TXT元数据。
这是补密、抬高后的Nav2约束点云，不是坑底原始深度点云。空检测和超时清空不写PCD。
开关为 `camera_costmap_cloud_node.ros__parameters.pothole.save.enabled`，默认true；
`pothole.save.directory` 默认指向上述pics目录，独立于模型图片保存开关。
PCD时间戳来自匹配的深度帧，图片来自RGB帧，两者允许最多0.05秒差异，因此文件名
不一定同时间戳；TXT保留点云frame、精确时间戳和点数供关联。每个PCD包含该帧所有有效轮廓。

凹坑三维定位依赖准确的彩深关系。当前参数中的 `color_extrinsic` 仍是零值初始值，现有
bag 的彩色图和深度图分辨率、frame 均不同，且不包含相机间外参，因此只能验证二维推理和
文件保存。向 Nav2 使用前必须启用相机的 depth-to-color registration，或者填入标定得到的
depth 到 color 旋转和平移。

若改用对齐深度，必须同时更新深度投影节点的尺寸、内参、光学frame、相机到车体外参及
边界FOV参数；不能只打开驱动registration而继续使用原生深度标定。

构建并运行（显式指定源配置，修改保存开关后重启即可生效）：

```bash
colcon build --packages-select traversability_obstacle_cloud robot_launch
source install/setup.bash
ros2 launch traversability_obstacle_cloud camera_nav2_perception.launch.py \
  depth_input_mode:=raw \
  params_file:=/home/cat/nanhu-research-navigation/src/traversability_obstacle_cloud/config/camera_ground_extraction.yaml
```

## 在线运行

`camera_nav2_perception.launch.py` 默认同时启动 Orbbec Gemini 330 驱动，等价于：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
  enable_sync_output_accel_gyro:=true
```

因此在线运行只需启动组合 launch；同步 IMU 输出默认开启：

```bash
ros2 launch traversability_obstacle_cloud camera_nav2_perception.launch.py \
  depth_input_mode:=raw
```

如果相机驱动已由其他进程启动，或者正在播放 rosbag，应传入
`use_orbbec_camera:=false`，避免重复占用相机设备。

launch 默认启动 `depth_cloud_projector_node`，将持续发布的
`/camera/depth/image_raw` 转为光学坐标 `/depth/points`。`16UC1`/`mono16` 深度值按
`camera.depth_scale`（默认 0.001 米）换算，`32FC1` 深度值直接按米使用。地面节点从
`/camera/gyro_accel/sample` 的同一条 IMU 消息读取线加速度和角速度：加速度估计重力
方向，角速度用于运动状态门控，旋转过快时暂停更新重力滤波。

深度投影节点有两种互斥的输入模式：

- `raw`：在线运行时订阅 `/camera/depth/image_raw`；
- `compressed_depth`（当前参数文件默认）：订阅 `/camera/depth/image_raw/compressedDepth`，解码
  `16UC1; compressedDepth` PNG 后再投影为 `/depth/points`。

模式默认读取参数文件中的 `depth_cloud_projector_node.ros__parameters.input_mode`。
launch 参数 `depth_input_mode` 默认为 `config`，不会覆盖 YAML；只有显式传入 `raw` 或
`compressed_depth` 时才覆盖参数文件。

```bash
colcon build --packages-select traversability_obstacle_cloud
source install/setup.bash
ros2 launch traversability_obstacle_cloud camera_single_frame_ground.launch.py
```

播放数据集 `rosbag2_2026_08_26-16_25_06` 时：

```bash
ros2 launch traversability_obstacle_cloud camera_single_frame_ground.launch.py \
  use_sim_time:=true depth_input_mode:=compressed_depth

ros2 bag play \
  /home/huibo/tdr/navigation/nanhu-research-navigation/rosbag/rosbag2_2026_08_26-16_25_06 \
  --clock
```

相机驱动已经直接发布 `/depth/points` 时，可关闭深度转换节点：

```bash
ros2 launch traversability_obstacle_cloud camera_single_frame_ground.launch.py \
  use_depth_projector:=false
```

虽然 launch 文件名保留了 `single_frame`，其含义是“每次判断仅采用当前一帧”；节点会
连续处理所有输入帧。

## 接入 Nav2 local costmap

### 疑似内部障碍事件记录

先构建并source工作空间，在三个终端（使用相同ROS_DOMAIN_ID）依次运行：

```bash
ros2 run traversability_obstacle_cloud record_obstacle_events.py \
  --params src/traversability_obstacle_cloud/config/camera_ground_extraction.yaml \
  --output /tmp/camera_obstacle_events

ros2 launch traversability_obstacle_cloud camera_nav2_perception.launch.py \
  use_orbbec_camera:=false use_sim_time:=true depth_input_mode:=compressed_depth

ros2 bag play rosbag/rosbag2_2026_08_26-16_25_06 --clock --rate 0.5
```

脚本默认关联该bag，监听实时话题，不负责启动感知或播放bag。严格匹配障碍、平滑边界、
视场分类边界的同一header时间戳；只有左右有效边界之间且离边界至少0.10m的点触发。
不会跨越大于0.10m的边界采样缺口，也会排除视场负标签区段。
默认至少5点连续3帧才触发，保留触发前2秒和最后触发后3秒，连续事件自动合并。
环形缓存上限512MiB，达到上限时前窗口可能缩短；检测摘要记录实际缓存起始时间。
使用Ctrl-C结束录制会关闭当前bag；回放暂停不会按墙钟强行结束事件。

每个事件目录包含bag和detections.jsonl，记录点数、包围盒、精确时间戳及边界内/无边界
走廊两类计数。运行目录保存源bag元数据及其SHA256、参数副本和代码版本/差异。
该校验值不是整个db3的校验值。加 `--fallback` 可记录没有完整边界时固定前向走廊内
的障碍（默认x=0.3～4m，半宽0.3m）。通过 `--help` 查看阈值和窗口选项。

事件仅表示疑似问题，不证明误报。当前版本记录现有话题，不包含尚未发布的逐栅格分类原因、
滤波后IMU旋转或点簇分解；包围盒属于所有触发点。原数据集没有里程计，因此不做运动补偿。
精确重现滤波历史请从原bag开始回放。调试输出被上游关闭时，相应话题不会出现在事件中。

视场过滤默认开启。分类节点将边界栅格四角用该帧IMU矫正的逆变换投影回深度图，
接近左右图像边缘且外侧缺少可靠高度突变/粗糙度观测的边界，不注入costmap。
原始 `ground_boundary_points` 保持不变；新增 `/traversability/classified_boundary_points`
使用 intensity 1/2/3 表示有效左/右/前边界，-1/-2 表示视场截断的左/右边界。
适配节点默认订阅新话题；负标签区域禁止输出补点，并立即失效对应历史状态。
空间平滑按视场缺口分段，避免跨段拟合。

参数位于分类节点 `boundary.fov_filter`：`enabled` 控制开关，
`horizontal_margin_px` 默认20像素，增大后剔除范围增加。
`width/fx/cx` 必须与深度投影节点的图像宽度及内参一致（不是彩色相机内参）；
适配节点 `boundary.fov_cell_size` 必须与分类节点 `grid.resolution` 一致，默认0.15米。
关闭过滤请设置 `boundary.fov_filter.enabled: false` 并重启感知链。
此规则仅识别左右视场截断，不将所有深度缺测区域判定为可通行。

本仓库当前的 `src/robot_launch/config/nx_net_nav.yaml` 保留了雷达的现有配置，并把
相机信息按用途拆分为：

- `obstacle_layer`：雷达 `pointcloud` 与相机 `camera_depth`，共同经过原有的
  `inflation_layer`（`inflation_radius: 1.0`、`cost_scaling_factor: 5.0`）；
- `camera_boundary_obstacle_layer`：只消费左右虚拟边界，`marking: true`、
  `clearing: false`；
- `camera_boundary_inflation_layer`：只在边界写入之后运行，采用较小的
  `inflation_radius: 0.45` 和更快的 `cost_scaling_factor: 8.0`。`0.45 m` 略大于
  当前 footprint 计算出的约 `0.405 m` 包络半径，避免 Nav2 的安全检查报错。

插件顺序是隔离两种膨胀的关键：普通障碍层、普通膨胀层、边界障碍层、边界膨胀层。
第二个膨胀层也会扫描已有致命栅格，但其半径更小、代价衰减更快，按最大代价合并后不会
扩大普通障碍已经形成的 1.0 m 膨胀；边界则不会进入前面的普通膨胀层。整个方案只复用
Nav2 现有 `ObstacleLayer` 和 `InflationLayer`，没有修改 `nav2_costmap_2d` 源码。

构建并启动相机链：

```bash
colcon build --packages-select traversability_obstacle_cloud robot_launch
source install/setup.bash

# 在线原始深度图
ros2 launch traversability_obstacle_cloud camera_nav2_perception.launch.py \
  depth_input_mode:=raw

# rosbag compressedDepth
ros2 launch traversability_obstacle_cloud camera_nav2_perception.launch.py \
  use_orbbec_camera:=false use_sim_time:=true depth_input_mode:=compressed_depth
```

相机链与导航 launch 可分别启动；`controller_server` 内部 local costmap 会自动订阅上述
两个话题。只启动原 `camera_single_frame_ground.launch.py` 时不会启动 costmap 适配节点，
因此原有使用方式保持不变。

适配参数位于 `config/camera_ground_extraction.yaml` 的
`camera_costmap_cloud_node` 段：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `depth.min_height` / `depth.max_height` | 0.10 / 1.20 m | 注入的深度障碍高度范围 |
| `depth.x_min` / `depth.x_max` | 0.30 / 4.0 m | 前向范围 |
| `depth.abs_y_max` | 3.0 m | 左右最大范围 |
| `depth.voxel_size` | 0.05 m | 深度障碍降采样体素 |
| `depth.relative_ground.enabled` | true | 启用局部相对地面高度过滤 |
| `depth.relative_ground.min_height` | 0.10 m | 点高出局部拟合地面至少该高度才作为障碍 |
| `depth.relative_ground.search_radius` | 0.45 m | 搜索局部地面栅格的水平半径 |
| `depth.relative_ground.min_neighbors` | 4 | 拟合局部地面平面所需的最少地面栅格数 |
| `depth.relative_ground.max_plane_rmse` | 0.08 m | 接受局部地面拟合的最大均方根误差 |
| `depth.relative_ground.max_slope_deg` | 25.0° | 接受局部参考平面的最大坡度 |
| `depth.relative_ground.max_age` | 0.15 s | 障碍帧与最近地面栅格帧允许的最大时间差 |
| `depth.relative_ground.keep_without_ground` | true | 无可靠局部地面时保留原障碍点，避免漏障 |
| `depth.relative_ground.publish_rejected_points` | true | 发布被相对高度过滤的点供调试 |
| `depth.relative_ground.rejected_topic` | `/traversability/costmap/depth_ground_rejected` | 相对地面过滤调试话题 |
| `depth.denoise.enabled` | true | 启用单帧半径离群点过滤 |
| `depth.denoise.search_radius` | 0.12 m | 搜索邻居的三维欧氏距离半径 |
| `depth.denoise.min_neighbors` | 3 | 半径内至少需要的其他点数，不包含自身 |
| `depth.denoise.publish_rejected_points` | true | 发布被剔除点供调试；不影响过滤结果 |
| `depth.denoise.rejected_topic` | `/traversability/costmap/depth_noise` | 被剔除点的话题 |
| `boundary.include_left/right` | true / true | 注入左右边界 |
| `boundary.include_front` | false | 默认不把视野最远端当成实体墙 |
| `boundary.x_min` / `boundary.x_max` | 1.2 / 5.0 m | 裁掉相机近端过窄边界，并限制最远边界 |
| `boundary.marking_height` | 0.30 m | 虚拟边界写入的高度 |
| `boundary.point_spacing` | 0.05 m | 边界补点间距 |
| `boundary.max_segment_length` | 0.45 m | 超过该长度的感知缺口不连接 |
| `boundary.smoothing.enabled` | true | 只平滑Nav2边界输出；原始边界话题不变 |
| `boundary.smoothing.min_valid_points` | 4 | 每侧启动曲线平滑所需的最少候选点 |
| `boundary.smoothing.median_window` | 5 | 沿前向列计算横向中值的奇数窗口 |
| `boundary.smoothing.max_lateral_jump` | 0.30 m | 偏离局部中值超过该距离的点被剔除 |
| `boundary.smoothing.spatial_smoothing` | 0.35 | 二阶差分空间平滑强度，范围0～1 |
| `boundary.smoothing.spatial_iterations` | 3 | 空间平滑迭代次数 |
| `boundary.smoothing.resample_spacing` | 0.05 m | 平滑曲线固定X重采样间距 |
| `boundary.smoothing.temporal_alpha` | 0.20 | 小变化时的时间滤波系数；越小越稳定但延迟越大 |
| `boundary.smoothing.adaptive_alpha_max` | 0.45 | 边界明显变化时允许增大的最大滤波系数 |
| `boundary.smoothing.max_temporal_jump` | 0.40 m | 单帧参与时间融合的最大横向变化 |
| `boundary.smoothing.max_hold_frames` | 3 | 局部缺测时保持历史边界的最大帧数 |
| `boundary.smoothing.reset_timeout` | 0.50 s | 输入中断超过该时间后清空历史状态 |
| `boundary.smoothing.min_corridor_width` | 0.70 m | 左右边界过窄或交叉时不发布对应约束 |

分类节点输出的是已重力对齐的车体局部坐标。当前导航将 `base_link` 作为二维车体坐标，
所以适配配置把输出 frame 设置为 `base_link`，这里只做坐标语义重标记，不执行通用 TF
变换。如果部署中的 `base_link` 不是水平二维坐标，应将 `output_frame` 改为实际的水平
车体 frame，并保证该 frame 到 `map` 的 TF 可用，不能仅靠修改 frame 名称补偿外参。

可用以下命令确认数据已进入链路：

```bash
ros2 topic hz /traversability/costmap/depth_obstacles
ros2 topic hz /traversability/costmap/depth_noise
ros2 topic hz /traversability/costmap/depth_ground_rejected
ros2 topic hz /traversability/costmap/boundary_constraints
ros2 topic echo /local_costmap/costmap --once
```

在 RViz 中同时显示 `depth_obstacles`、`depth_ground_rejected` 和 `depth_noise`。
`depth_ground_rejected` 表示低于局部地面相对高度门限的点；若低矮实体障碍大量进入该
话题，应减小 `relative_ground.min_height`。若上坡或破损路面的点仍进入
`depth_obstacles`，可适当增大 `relative_ground.search_radius`，但不建议关闭
`keep_without_ground`，否则无地面参考的区域会形成感知盲区。若实体障碍也大量落入
`depth_noise`，应优先增大 `search_radius` 或减小 `min_neighbors`；若孤立飞点仍进入
`depth_obstacles`，则反向调整。完成调参后可关闭 `publish_rejected_points` 以减少调试话题
带宽。

边界平滑只作用于 `/traversability/costmap/boundary_constraints`，不会修改
`/traversability/ground_boundary_points`，因此可在 RViz 中叠加两个话题比较。若输出仍然
抖动，优先减小 `temporal_alpha` 或增大 `median_window`；若弯道跟随明显滞后，则增大
`temporal_alpha` 和 `adaptive_alpha_max`。设置 `boundary.smoothing.enabled: false` 可完全
恢复原来的边界补点行为。

## RGB 可视化

launch 默认启动 `rgb_image_decoder_node`，将 Orbbec 的
`/camera/color/image_raw/compressed` JPEG 图像解码成标准
`/camera/color/image_raw`（`sensor_msgs/msg/Image`，`bgr8`）。查看方式：

```bash
rqt_image_view /camera/color/image_raw
```

也可以在 RViz2 中添加 `Image` 显示项并选择 `/camera/color/image_raw`。不需要
RGB 时可通过 `use_rgb_decoder:=false` 关闭解码节点。

`/traversability/rgb_overlay` 将 2.5D 栅格中心投影到对应 RGB 图像。原始彩色输入
支持 `bgr8`、`rgb8`、`bgra8`、`rgba8` 和 `mono8`，节点会统一转换为 `bgr8`。
在线配置默认使用收到的最近彩色帧，不会因彩色与深度帧率或时间戳稍有差异而停止发布。
当前配置只显示绿色 Ground；Obstacle 和 Unknown 可通过 `color_camera.show_obstacle`、
`color_camera.show_unknown` 单独开启。当前 bag 没有包含深度到彩色相机的
精确外参，因此默认使用光轴重合近似；投影若有固定偏移，应校准参数文件中的
`color_extrinsic`。

叠加图左上角显示由 IMU 重力方向计算的相机姿态偏差。IMU 使用光学轴
（X右、Y下、Z前）时，节点先转换为
X前、Y左、Z上，再按下式计算：

```text
Roll  = atan2(up_y, up_z)
Pitch = atan2(-up_x, sqrt(up_y² + up_z²))
```

只显示 Roll/Pitch，不从加速度推算 Yaw。可用 `color_camera.show_attitude` 关闭文字。

## 从 rosbag 导出彩色图像

构建并加载工作空间后，可将 bag 中每一帧彩色图像保存为 JPG。脚本同时支持
`sensor_msgs/msg/Image` 和 `sensor_msgs/msg/CompressedImage`：

```bash
ros2 run traversability_obstacle_cloud extract_bag_color_images.py \
  /path/to/rosbag /path/to/output_images
```

默认自动选择 `/camera/color/image_raw` 或
`/camera/color/image_raw/compressed`。也可以明确指定话题和 JPEG 质量：

```bash
ros2 run traversability_obstacle_cloud extract_bag_color_images.py \
  /path/to/rosbag ./images \
  --topic /camera/color/image_raw --quality 95
```
