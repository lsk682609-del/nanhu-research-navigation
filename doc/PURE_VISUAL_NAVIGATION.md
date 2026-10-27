# 纯视觉导航：架构、原理与操作方法

本文描述 `camera_boundary_navigation` 的 Gazebo 纯视觉局部导航，运行证据采集于 2026-10-13。运行状态是当时快照，重启、移动车辆或改变场景后应重新采样。

## 当前行为更新：无边界前行

### Gazebo目标速度

仿真覆盖设定 `control.nominal_linear_velocity=0.5`、`maximum_linear_velocity=0.5`（m/s）。
1 m/s试运行中出现进入死胡同的情况，已回调至0.5 m/s。基础实机配置保持原值。
0.5 m/s是直线目标和上限，转弯、障碍净距及安全前缀仍会降低速度。
按现有减速度0.5 m/s²、反应时间0.35 s和余量0.25 m，0.5 m/s需要约
`0.5×0.35+0.5²/(2×0.5)+0.25=0.675 m`安全长度；最小候选弧长仍为1 m。
修改后需构建安装配置并重启导航，运行中参数不会自动更新。

### 当前规划更新：主动远离与双边界居中

双侧边界有效且连续共同观测跨度达到拟合最小跨度时，模式为 `CORRIDOR_CENTER`，
以两侧拟合曲线的中线为参考。沿共同区间扣除车辆安全余量（与障碍膨胀半径一致，
倾斜边界还补偿斜率），候选不得越过安全区间；过窄时 `corridor_too_narrow` 停车。
每侧选择最长连续观测段，超过0.20 m的纵向缺口不桥接。仅一侧有效时跟随该侧，
优先使用配置侧；两侧不足时使用无边界前行。
直线轨迹膨胀后净距低于 `avoidance.preferred_clearance=0.20 m` 时，不再因
几何安全和边界死区保留直行，而由 VFH 主动搜索净距更大的路径。
候选增加舒适净距惩罚和走廊中心偏差代价，继续保留曲率平滑和实际命令安全检查。
诊断 `path_clearance` 为膨胀障碍外净距，`corridor_valid/usable` 表示共同走廊及其可用性。
舒适净距不足但所有改进候选不可行时，仍可选择原安全低速轨迹，不强行转入未知地面。

### 当前输入更新：处理后边界与障碍

导航现订阅 `/traversability/costmap/boundary_constraints` 和
`/traversability/costmap/depth_obstacles`，Ground 仍来自 `/traversability/grid`。
`camera_only_navigation.launch.py` 和 `gazebo_camera_navigation.launch.py` 均启动适配器。
当前链路：深度图 → 投影 → 可通行分类 → `camera_costmap_cloud_node` → 处理后边界/障碍
→ 无边界前行或边界跟随 → VFH → 安全门 → 速度输出。
分类节点的原始边界和分类边界是适配器上游输入及调试数据，不再直接供导航使用。
纯视觉专用 `processed_perception.yaml` 保持适配器输出坐标为 `gravity_aligned_base`，
深度障碍 x 下限为0 m，边界 x 下限为0.6 m；Nav2启动的原有base_link配置不受影响。
障碍已体素降采样及去噪，导航每个安全栅格1点即可触发障碍判定。
下文运行采样和链路表保留为改造前诊断记录，以本节为当前输入配置。

已增加 `boundaryless.enabled=true`（默认开启）。下文停车诊断及调整建议记录的是改造前行为。
现在边界拟合失败时以机体正前方为目标，状态为 `navigation_mode=BOUNDARYLESS_FORWARD`，
真实的 `boundary_valid` 仍为 false。直线安全则前进，遇障由 VFH 绕行，无安全路径则停车。
Ground 覆盖、最小安全弧长、制动检查和输入同步新鲜度继续生效；合法空边界可进入此模式，
边界话题断流仍停车。边界恢复后自动切回 `BOUNDARY_FOLLOW`，由稳定规划及控制滤波调整。
没有航向或 odom 反馈，所以前行指当前机体前方，绕障后不保证恢复原始全局航向。
此模式要求 `avoidance.enabled=true`；关闭避障时须同时关闭 `boundaryless.enabled`。
无需扩大拟合窗口即可支持空旷区域，已有进程需重新构建并重启加载新行为。

## 1. 当前车辆不动的原因

实际状态为 `BOUNDARY_INVALID`，`avoidance_mode=BLOCKED`、`avoidance_reason=boundary_invalid`，线速度和角速度均为零。当前 `enabled=true`、`paused=false`；导航在 `/cmd_vel` 发布，Gazebo 的 `gazebo_ros_skid_steer_drive` 正确订阅该话题。使能及底盘话题连接不是本次阻塞原因。

导航订阅 `/traversability/classified_boundary_points` 并跟随右边界。运行参数要求 `boundary.fit_x_min=0.6`、`fit_x_max=2.0`、`minimum_points=8`、`minimum_x_span=1.0`。单帧点云统计如下：

| 点云与标签 | 总点数 | 全部纵向范围 x（m） | 拟合窗口内点数 | 窗口内跨度（m） |
| --- | ---: | --- | ---: | ---: |
| 分类右边界，2 | 22 | 1.625–4.775 | 3 | 0.30 |
| 分类右视场截断点，-2 | 6 | 0.725–1.475 | 6 | 0.75 |
| 分类左边界，1 | 22 | 1.625–4.775 | 3 | 0.30 |
| 分类左视场截断点，-1 | 6 | 0.725–1.475 | 6 | 0.75 |
| 原始右边界，2 | 28 | 0.725–4.775 | 9 | 1.20 |

点云坐标系为 `gravity_aligned_base`，上述分类边界、原始边界、栅格的采样时间戳相同（955.956 s 仿真时间）。栅格有 376 个 Ground 单元和 53 个 Obstacle 单元，说明感知在输出数据。

原始右边界在窗口内看似满足拟合条件，是因为包含了相机视场截断形成的伪边界。过滤后仅剩 3 个有效点，首先在 `BoundaryModel::fit()` 的最少点数检查处失败；跨度也不足。这次切换分类话题生效了，但原有短距离拟合窗口没有覆盖足够的有效边界。

可验证的下一步是扩大仿真专用的拟合上限，例如试验 `boundary.fit_x_max=3.5`，保留最少点数及跨度检查。当前正标签在更远区域存在，因此扩大窗口有机会恢复拟合，但需要检查曲线残差、偏移路径及 Ground 覆盖，不能据此保证车辆一定运动。另一种办法是调整起始位置，让真实道路边界进入近处有效视野。当前左右侧近距离统计相同，单纯切换跟随侧不能解决这次问题。

原诊断阶段只形成文档；随后已实现无边界前行，无需修改拟合阈值。现有节点尚未重启。不要通过关闭 FOV 分类或回到原始边界来掩盖真实边界不足。

## 2. 当前架构与数据链路

```text
Gazebo 深度相机
  /camera/depth/image_raw
       │ sensor_msgs/Image，raw 深度
       ▼
depth_cloud_projector_node
  /depth/points（camera_color_optical_frame）
       ▼
traversability_obstacle_cloud_node
  固定相机外参 → 局部栅格 → Ground/Obstacle 分类 → 边界提取
       ├─ /traversability/classified_boundary_points ─┐
       ├─ /traversability/grid ──────────────────────┤
       ├─ /obstacle_points ──────────────────────────┤
       │                                            ▼
       │                       camera_boundary_navigation_node
       │                       边界拟合 → 稳定规划 → VFH → 安全门
       │                                            │
       │                                          /cmd_vel
       │                                            ▼
       │                              gazebo_ros_skid_steer_drive
       │                                            ▼
       │                                          车辆运动
       └─ 原始边界、Ground 点云、叠加图 → RViz 调试

/clock → 各处理节点 use_sim_time=true
```

| 节点 | 主要输入 | 主要输出/作用 |
| --- | --- | --- |
| `depth_cloud_projector_node` | `/camera/depth/image_raw` | `/depth/points`，深度投影 |
| `traversability_obstacle_cloud_node` | `/depth/points` | 分类边界、栅格和障碍点云 |
| `camera_boundary_navigation_node` | 分类边界、`/traversability/grid`、`/obstacle_points` | `/cmd_vel`、路径、目标点、状态 |
| `gazebo_ros_skid_steer_drive` | `/cmd_vel` | 底盘运动 |
| `rviz` | 调试话题及 TF | 可视化 |

ROS 图里还存在 LiDAR、相机 IMU 和机器人 TF 节点；存在这些节点不代表导航使用它们。本配置 `require_imu=false`，导航不订阅 LiDAR、odom 或定位地图，也不依赖视觉里程计。RGB 可用于感知叠加图，不是本路径规划的主要输入。

## 3. 算法原理

### 深度投影与可通行分类

针孔模型将像素和深度转换为光学点云：`X=(u-cx)*Z/fx`、`Y=(v-cy)*Z/fy`。Gazebo 配置为 1280×720，`fx=fy=924.2773806188661`、`cx=640.5`、`cy=360.5`。raw `32FC1` 深度以米为单位；`depth_scale` 用于整型深度。

点云经固定外参转换到车辆局部坐标：前移 0.25 m、高 0.55 m、俯角 25°。输出 `gravity_aligned_base` 的 x 向前、y 向左。当前仿真关闭 IMU 依赖，采用固定安装姿态；车辆明显俯仰时应重新核对地面分类效果。

感知在 0.15 m 栅格内统计高度、方差及粗糙度，由前方中央 Ground 种子进行连通扩展，同时约束坡度和高度跳变。Ground 的每列左右极值组成左右边界，最远列形成前边界。

### 视场边缘与边界拟合

原始 `/traversability/ground_boundary_points` 保留全部边界。分类话题将触碰左右图像边缘且缺少物理障碍证据的点标为负标签：1/2/3 为左/右/前边界，-1/-2 为左右视场截断点。分类话题仍包含负标签点，所以在 RViz 中切换到该话题并不意味着点会消失；负标签点只是被标记为不应作为道路边界。

导航只选取配置侧的正标签点，在拟合窗口内用 RANSAC 和迭代离群点剔除拟合 `y=a0+a1*x+a2*x²`，检查点数、跨度和残差，并进行时间滤波。向道路内部偏移 0.65 m 形成参考线，以 Bézier 连接车辆原点与偏移边界。

### 稳定规划、自适应 VFH 与控制

稳定规划首先验证直行路径；直线安全且边界距离位于死区内时保持直行。边界偏差或直线受阻连续满足确认帧数后才切换曲率，抑制左右抖动。

正常路径受阻时，VFH 用障碍直方图给方向评分，生成前进圆弧，按目标方向、平滑性、障碍、参考曲率及换侧代价选优。每个曲率候选从最长可见弧长（最大 2.0 m）开始，以 0.1 m 步长缩短至最少 1.0 m，并受搜索半视角约束。只有完整前缀通过机体 Ground 覆盖和障碍膨胀检查才允许低速执行。缩短的弧长降低速度上限；最大候选长度不保证一定存在安全路径。

Pure Pursuit 根据前视点生成曲率和角速度，结合速度、角速度、加速度及滤波限制。速度还受停车距离与障碍净距限制。安全门检查使能、暂停、输入新鲜度、Ground 和障碍，并再次验证实际命令的停车轨迹。默认边界无效时以前方为目标进行 VFH 规划；关闭无边界模式后仍要求有效边界。

### 同步与停车条件

分类边界、栅格和障碍点云必须具有相同源时间戳和正确坐标系；输入新鲜度按源时间检查。当前输入超时 0.25 s，Gazebo 未来时间容差 0.10 s，恢复需要 5 个新有效同步帧。

本系统是单边跟随的局部导航，不提供全局定位、目的地到达判断、地图记忆或倒车脱困；边界不可见、路径无安全 Ground 或近场障碍出现时停车。

## 4. 配置来源

感知基础参数：`src/traversability_obstacle_cloud/config/camera_ground_extraction.yaml`。
导航基础参数：`src/camera_boundary_navigation/config/boundary_follower.yaml`。
Gazebo 覆盖：`src/camera_boundary_navigation/config/gazebo_camera_navigation.yaml`。
组合启动：`src/camera_boundary_navigation/launch/gazebo_camera_navigation.launch.py`。

加载顺序为基础参数、Gazebo 覆盖、launch 显式参数。仿真覆盖把边界话题改为分类边界；基础导航配置仍默认原始边界。操作时以 `ros2 param get/dump` 的运行值为准。主要参数在启动时加载，修改 YAML 后需要重新安装配置并重启相关 launch。

## 5. 操作方法

### 构建与环境

```bash
cd /home/lxh/nanhu-research-navigation
source /opt/ros/humble/setup.bash
colcon build --packages-select traversability_obstacle_cloud camera_boundary_navigation
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source install/setup.bash
```

Gazebo 已运行时无需重复启动仿真。先停止旧的纯视觉导航 launch，避免重复发布点云或速度。

### 隔离观察

```bash
ros2 launch camera_boundary_navigation gazebo_camera_navigation.launch.py
```

默认启动未使能，速度输出在 `/camera_navigation/cmd_vel`。该话题默认不接 Gazebo 底盘。观察 `/camera_navigation/status`，确认输入、边界和规划有效。

### 闭环运行

```bash
ros2 launch camera_boundary_navigation gazebo_camera_navigation.launch.py \
  cmd_vel_topic:=/cmd_vel enabled_on_startup:=false
```

在另一个已加载 ROS 环境的终端使能：

```bash
ros2 topic pub --once /camera_navigation/enable std_msgs/msg/Bool '{data: true}'
```

即使使能，安全门仍可能停车；本次 `BOUNDARY_INVALID` 就属于这种情况。

### 停止、暂停与恢复

```bash
# 禁止运动
ros2 topic pub --once /camera_navigation/enable std_msgs/msg/Bool '{data: false}'
# 暂停
ros2 topic pub --once /pause std_msgs/msg/Bool '{data: true}'
# 解除暂停；仍须使能并满足安全恢复条件
ros2 topic pub --once /pause std_msgs/msg/Bool '{data: false}'
```

### 检查与可视化

```bash
ros2 topic echo /camera_navigation/status --once
ros2 topic echo /cmd_vel --once
ros2 topic info -v /cmd_vel
ros2 param get /camera_boundary_navigation_node boundary_topic
ros2 param dump /camera_boundary_navigation_node
ros2 topic hz /depth/points
ros2 topic hz /traversability/classified_boundary_points
```

RViz 固定坐标系设为 `gravity_aligned_base`；显示分类边界、栅格、障碍点云，以及 `/camera_navigation/nominal_path`、`reference_path`、`target_point`、`avoidance_markers`。用 intensity 着色区分正负边界标签。停止时目标点可能回到原点，应结合状态解释，不能单靠目标点判断控制故障。

| 状态/现象 | 首先检查 |
| --- | --- |
| `DISABLED` | `/camera_navigation/enable` 与 enabled 状态 |
| `PAUSED` | `/pause` |
| `WAITING_FOR_INPUT` | 三路点云时间戳、坐标系、断流和 `/clock` |
| `BOUNDARY_INVALID` | 指定侧正标签数量、拟合窗口、跨度和残差 |
| `FRONT_TOO_CLOSE` | 可执行路径的连续安全长度 |
| `PATH_NOT_ON_GROUND` | 机体圆形覆盖是否全部位于 Ground |
| `PATH_BLOCKED` / `BLOCKED` | 近场急停、障碍膨胀、候选轨迹及具体 reason |
| 有非零速度但车辆不动 | 实际速度话题、底盘订阅、Gazebo 是否暂停 |

## 6. 关闭无边界模式时的原诊断调整方案

1. 保留分类边界输入和 FOV 分类，确认右边界正标签分布。
2. 在仿真专用覆盖中试验扩大 `boundary.fit_x_max`，或调整起始位置，使窗口内达到至少 8 点及 1.0 m 跨度。
3. 重启后确认状态不再是 `BOUNDARY_INVALID`，检查曲线残差与实际参考路径。
4. 若转为 `PATH_BLOCKED`，继续检查机体 Ground 覆盖及障碍净距；恢复边界拟合并不代表避障必然通过。
5. 使能后检查 `/cmd_vel` 和实际车辆运动，记录闭环结果。

## 7. 源码定位

- `src/traversability_obstacle_cloud/src/depth_cloud_projector_node.cpp`：深度投影。
- `src/traversability_obstacle_cloud/src/traversability_obstacle_cloud_node.cpp`：Ground/Obstacle 分类、边界提取和 FOV 负标签。
- `src/camera_boundary_navigation/src/boundary_model.cpp`：拟合窗口、RANSAC 与有效性检查。
- `src/camera_boundary_navigation/src/stable_local_planner.cpp`：稳定直行和转向确认。
- `src/camera_boundary_navigation/src/vfh_local_planner.cpp`：候选弧长、评分和制动轨迹检查。
- `src/camera_boundary_navigation/src/boundary_controller.cpp`：参考路径、Pure Pursuit 与速度滤波。
- `src/camera_boundary_navigation/src/safety_gate.cpp`：输入、恢复和几何安全门。
- `src/camera_boundary_navigation/src/camera_boundary_navigation_node.cpp`：订阅、同步帧提交、状态和命令发布。
