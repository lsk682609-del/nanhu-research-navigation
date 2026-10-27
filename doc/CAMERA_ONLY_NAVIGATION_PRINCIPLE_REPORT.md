# 纯相机导航功能模块原理报告

> 文档状态：对应当前工作区实现  
> 编写日期：2026-09-02  
> 适用场景：LiDAR、定位和里程计均失效，不使用视觉/深度里程计，车辆只需沿单一道路低速持续向前

## 1. 结论摘要

本模块可以在不依赖 LiDAR、地图、定位、odom 和深度里程计的条件下，利用深度相机单帧数据与 IMU 重力方向完成短距离局部导航。它不是完整意义上的自主导航系统，而是一个“无定位应急前进控制器”：每一帧都在车体局部坐标系中重新识别可通行地面，提取道路左右及最前方边界，以指定的一侧边界作为横向参考，生成短距离安全路径并输出低速控制指令。

当前实现的核心原则是“直行优先，边界只作约束”：

- 边界距离处于允许死区且直线路径安全时，强制保持直线，不跟随边界分割噪声左右摆动。
- 只有连续多帧表明车辆偏离参考距离，或直行路径已经不安全时，才进入转弯。
- 转弯曲率同时受到历史曲率惩罚和单帧变化上限约束，降低路径跳变。
- 最终路径必须同时通过地面覆盖、前方距离和障碍物检查，否则发布零速度。
- 控制输出默认位于隔离话题 `/camera_navigation/cmd_vel`，启动后默认不允许运动。

该方案复用了现有 `traversability_obstacle_cloud` 的可通行区域检测，但将无定位控制独立放入 `camera_boundary_navigation` 功能包，从而避免修改现有基于 LiDAR 和定位的主导航链路。

## 2. 设计目标与前提

### 2.1 目标

1. LiDAR 和定位链路失效后，车辆仍能沿前方唯一道路低速行驶。
2. 不累计历史位姿，不构建全局地图，不要求 odom 或 TF 位姿变换。
3. 参考一侧道路边界保持安全距离，边界小幅抖动时优先直行。
4. 遇到道路弯曲、地面不可通行、障碍物或输入中断时作出保守响应。

### 2.2 成立前提

- 前方只有一条可通行道路，不需要处理岔路选择、掉头、倒车和到达目标点。
- 相机能看到足够长的路面，深度图具有可用的尺度和相机内参。
- IMU 可提供可靠的重力方向；系统只使用其滚转、俯仰意义上的重力对齐，不由此获得全局航向或位置。
- 相机外参、安装高度、深度尺度和车体宽度相关安全参数已经标定。
- 车辆以低速运行，单帧局部反应足以覆盖制动距离和控制延迟。

若这些前提不成立，本模块应停车，而不是试图代替完整的定位和全局规划系统。

## 3. 系统组成与数据流

```text
/camera/depth/image_raw ───────────────┐
                                      ├─ 深度投影 ─→ /depth/points ─┐
/camera/depth/image_raw/compressedDepth┘                            │
                                                                   ├─ 可通行性检测
/camera/gyro_accel/sample → 重力方向估计与点云姿态校正 ─────────────┘
                         │
                         ├─→ /traversability/ground_boundary_points
                         ├─→ /traversability/grid
                         └─→ /obstacle_points
                                      │
                                      ▼
                         单侧边界拟合与稳定局部规划
                                      │
                         地面/障碍/新鲜度安全门
                                      │
                              Pure Pursuit 控制
                                      │
                         /camera_navigation/cmd_vel
```

模块分为两个 ROS 2 功能包：

| 功能包 | 职责 | 是否依赖 odom/定位 |
|---|---|---|
| `traversability_obstacle_cloud` | 深度输入、点云生成、重力对齐、地面/障碍分类和 U 形边界提取 | 否 |
| `camera_boundary_navigation` | 单侧边界拟合、稳定路径选择、安全检查和速度控制 | 否 |

所有路径规划均在 `gravity_aligned_base` 中完成，规定 `x` 向前、`y` 向左。每个相机帧产生一条从车辆当前原点出发的局部路径；车辆移动后，下一帧再次从新原点规划。因此发布的 `Path` 不能当作世界坐标中的历史轨迹。

## 4. 深度输入与点云生成

系统支持两种深度输入方式：

| 使用环境 | 输入话题 | 模式 |
|---|---|---|
| 在线相机 | `/camera/depth/image_raw` | `raw` |
| rosbag 数据集 | `/camera/depth/image_raw/compressedDepth` | `compressed_depth` |

二者承载的本质都是逐像素深度。区别在于消息传输编码：原始深度图可直接读取，`compressedDepth` 必须先解码并恢复深度数值。解码完成后，两种输入共用同一套投影、地面识别和规划逻辑。

针孔相机模型将像素 `(u,v)` 和深度 `Z` 投影为相机光学坐标中的三维点：

```text
X = (u - cx) Z / fx
Y = (v - cy) Z / fy
Z = depth_raw × depth_scale
```

随后根据配置的光学轴方向和相机外参转换到车体坐标。IMU 加速度经过有效幅值和角速度门限后用于估计重力方向，并对点云做重力对齐。这里没有计算车辆在世界中的位置，也没有积分得到航向轨迹。

当前感知配置的关键量包括：`848×480`、`fx=fy=412.523773`、`cx=421.35`、`cy=241.9875`、`depth_scale=0.001`，相机高度 `0.60 m`。这些值必须与实际相机或数据集一致，否则地面高度、坡度和横向边界都会产生系统误差。

## 5. 单帧可通行区域识别

### 5.1 二维栅格统计

重力对齐后的点云在车辆水平面上划分为分辨率 `0.15 m` 的二维栅格。每个栅格统计点数、高度分布、方差、粗糙度、坡度和相邻高度变化，再判定为：

- `Ground`：满足地面几何约束且能从车辆前方种子区域连通到达。
- `Obstacle`：有观测但不满足地面条件，或不可作为连通地面。
- `Unknown`：点数不足或没有可靠观测。

当前地面候选主要受下列参数限制：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `grid.resolution` | `0.15 m` | 地面分类与边界提取的空间尺度 |
| `grid.min_points` | `2` | 单元格形成有效统计所需点数 |
| `grid.max_variance` | `0.0025 m²` | 限制高度离散程度 |
| `grid.max_roughness` | `0.05 m` | 限制局部粗糙度 |
| `grid.max_slope_deg` | `18°` | 限制地面坡度 |
| `grid.max_step` | `0.05 m` | 限制相邻地面台阶 |
| `grid.ground_point_tolerance` | `0.05 m` | 限制点云高于预计地面的允许高度 |
| `seed.x_min/x_max` | `0.5/1.5 m` | 车辆前方地面连通生长的种子范围 |
| `seed.half_width` | `0.5 m` | 种子区域横向半宽 |
| `max_unknown_gap_cells` | `2` | 允许桥接的短 Unknown 间隙 |

地面边缘障碍被误识别为 Ground，通常与栅格过大、坡度/台阶/粗糙度阈值过宽、Unknown 桥接过强或种子区域覆盖障碍有关。调参时应优先使用录包回放观察分类结果，避免只为消除误检而将阈值收紧到正常地面也无法连通。

### 5.2 连通地面与 U 形边界

算法从车辆前方种子区域开始，对候选地面做连通生长，避免把远处但与车辆不可达的平面直接认作当前道路。允许跨越不超过 `max_unknown_gap_cells` 的短缺测间隙。

对每个有效前向栅格列，提取连通地面的最左和最右单元；再取最远有效列作为前边界，形成不包含车辆后方边界的 U 形包围线。边界点云 `intensity` 约定为：

- `1`：左边界；
- `2`：右边界；
- `3`：最前方边界。

前边界用来限制可见规划距离和判断前方是否过近；左右边界用于选择一侧进行道路跟随。

## 6. 单侧边界模型

配置 `reference_side: left/right` 决定使用左边界或右边界。当前配置为 `right`，因此只提取 `intensity=2` 的点参与拟合。

### 6.1 鲁棒二次曲线拟合

在 `x=0.6～2.0 m` 范围内，将边界拟合为：

```text
y_b(x) = a0 + a1 x + a2 x²
```

处理过程为：

1. 过滤非有限点和拟合距离范围外的点，并按 `x` 排序。
2. 使用确定性的三点 RANSAC 选择初始模型，离群阈值为 `0.18 m`。
3. 对内点迭代最小二乘重拟合 3 次。
4. 要求至少 8 个内点、前向跨度至少 `1.0 m`、均方根残差不超过 `0.12 m`。
5. 对 `a0、a1、a2` 作时间低通滤波，当前新观测权重 `α=0.30`。

置信度由内点数量、前向跨度和拟合残差共同计算。任何硬条件不满足时，边界无效，安全门停车。

### 6.2 边界偏移路径与 Bézier 过渡

传统边界跟随路径沿曲线法向向道路内部偏移 `d=0.65 m`。设边界导数为 `y'_b(x)`，单位法向由参考侧决定：

```text
左边界向内法向：n = ( y', -1) / sqrt(1 + y'²)
右边界向内法向：n = (-y',  1) / sqrt(1 + y'²)
p_ref(x) = (x, y_b(x)) + d n
```

偏移路径的第一个点通常不在车辆原点。为避免原点直接连到该点形成折线，代码使用三次 Bézier 曲线连接：

```text
B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
```

其中 `P0` 为车辆原点，`P1` 沿车辆 `x` 轴，`P3` 为偏移边界起点，`P2` 沿偏移边界在连接点的反切向设置。这样起点朝向和连接点切向连续。

在稳定规划开启时，这条完整偏移路径主要用于估计边界所建议的转弯曲率；最终发布的主路径通常是直线或定曲率圆弧。关闭 `stability.enabled` 后，系统才直接追踪上述边界偏移路径。

## 7. 直行优先的稳定局部规划

### 7.1 候选路径

每个同步相机帧先生成零曲率直线路径。规划弧长为：

```text
horizon = min(stability.path_horizon, boundary.x_max, front_range)
```

当前 `path_horizon=2.5 m`，但边界拟合上限为 `2.0 m`，因此实际路径通常不会超过 `2.0 m`。路径按 `0.10 m` 采样，所以长度为 `2.0 m` 时共有约 21 个点（包含原点和终点）。

转弯时生成定曲率圆弧。以沿弧长 `s` 的曲率 `κ` 表示：

```text
x(s) = sin(κs) / κ
y(s) = [1 - cos(κs)] / κ
```

当 `κ=0` 时退化为 `x=s, y=0` 的直线。

### 7.2 边界距离死区

在 `x=0.8 m` 处评价车辆与参考边界的横向距离。目标距离为 `0.65 m`，死区半宽为 `0.20 m`，因此距离在 `0.45～0.85 m` 内且直线路径安全时，忽略边界曲线的小幅弯曲并强制直行。

以右边界为例：

- 距右边界小于 `0.45 m`：车辆过于靠右，需要向左转，`κ>0`。
- 距右边界大于 `0.85 m`：车辆离右边界过远，需要向右回归，`κ<0`。
- 距离位于死区内：保持直行，除非直线路径已经不安全。

### 7.3 连续多帧状态切换

单帧边界越界不会立即触发转弯。相同转向方向必须连续出现 5 帧才进入转弯状态。恢复直行条件后，也必须连续保持 8 帧才退出转弯。

在等待转弯确认期间，如果直线路径本身不安全，安全门会停车，而不是继续沿不安全直线行驶。这会牺牲短时连续性，但能避免在证据不足时盲目转向。

### 7.4 曲率变化惩罚

进入转弯后，边界路径提供目标曲率 `κ_des`。选择曲率先进行加权平滑：

```text
κ_sel = (w_track κ_des + w_change κ_prev) / (w_track + w_change)
```

当前 `w_track=4`、`w_change=12`，历史曲率权重更高；随后再将单帧变化限制为：

```text
|κ_sel - κ_prev| ≤ 0.05 1/m
```

转弯曲率绝对值不会低于 `0.05 1/m`。该设计使路径对边界的局部锯齿和分割误差不敏感，同时保留持续道路弯曲时逐步入弯的能力。

## 8. 前视控制原理

规划器发布整条局部参考路径，控制器实际追踪的是路径上的前视点。它从路径第二个点开始，选择第一个满足下式的点：

```text
sqrt(x_t² + y_t²) ≥ lookahead_distance
```

当前前视距离为 `1.5 m`；如果路径不足 `1.5 m`，则使用最后一个点。Pure Pursuit 根据车辆原点到前视点的几何关系计算曲率：

```text
κ = 2 y_t / (x_t² + y_t²)
ω = v κ
```

因此，前视点不是路径终点。前视点之后保留路径，是因为：

- 安全门需要检查整条当前规划路径，而不是只检查控制目标之前的部分；
- 下一帧目标选择和路径显示需要完整的可见局部走廊；
- 较完整的几何形状能避免把前视距离误当成规划距离。

线速度随曲率增大而降低：

```text
v = clamp(v_nominal / (1 + gain |κ|), v_min, v_max)
```

当前直行期望速度为 `0.25 m/s`，范围 `0.10～0.30 m/s`，角速度上限 `0.35 rad/s`。角速度还受 `0.50 rad/s²` 的变化率限制，最终速度命令以 `α=0.30` 低通滤波。

## 9. 安全门与停车条件

非零速度只有在所有检查同时通过时才允许发布：

| 状态 | 含义 | 控制输出 |
|---|---|---|
| `DISABLED` | 未显式使能 | 零速度 |
| `PAUSED` | 收到暂停指令 | 零速度 |
| `WAITING_FOR_INPUT` | 输入超时或尚未满足恢复帧数 | 零速度 |
| `BOUNDARY_INVALID` | 边界拟合或路径无效 | 零速度 |
| `FRONT_TOO_CLOSE` | 前方有效边界不足 `1.0 m` | 零速度 |
| `PATH_NOT_ON_GROUND` | 路径未被 Ground 栅格覆盖 | 零速度 |
| `PATH_BLOCKED` | 路径邻近障碍或近场存在障碍 | 零速度 |
| `TRACKING` | 全部检查通过 | 发布控制指令 |

边界、Ground 栅格和障碍点云必须具有完全相同的消息时间戳，节点才提交一帧，防止混用不同相机帧的几何结果。提交后任一必需输入超过 `0.25 s` 未更新，立即停车。

Ground 覆盖从 `x=0.60 m` 开始检查，以避开深度相机近场地面盲区；这一豁免不适用于障碍物。车辆前方 `0.70 m`、中心线左右 `0.40 m` 内的有效障碍会直接触发停车。障碍栅格至少需要 3 个点，以抑制孤立深度噪声；整条路径还要求与障碍保持 `0.40 m` 净空。

任何停车或无效状态解除后，必须连续 5 个新相机帧通过检查才能恢复 `TRACKING`，避免状态在临界条件附近快速启停。

## 10. ROS 2 接口

### 10.1 主要输入

| 话题 | 类型 | 用途 |
|---|---|---|
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 在线相机原始深度输入 |
| `/camera/depth/image_raw/compressedDepth` | `sensor_msgs/CompressedImage` | 数据集压缩深度输入 |
| `/camera/gyro_accel/sample` | `sensor_msgs/Imu` | 重力方向估计 |
| `/traversability/ground_boundary_points` | `sensor_msgs/PointCloud2` | 左、右、前地面边界 |
| `/traversability/grid` | `sensor_msgs/PointCloud2` | Ground/Obstacle/Unknown 栅格 |
| `/obstacle_points` | `sensor_msgs/PointCloud2` | 障碍物安全检查 |
| `/camera_navigation/enable` | `std_msgs/Bool` | 运动使能 |
| `/pause` | `std_msgs/Bool` | 暂停控制 |

### 10.2 主要输出

| 话题 | 类型 | 用途 |
|---|---|---|
| `/depth/points` | `sensor_msgs/PointCloud2` | 深度图投影后的中间点云 |
| `/camera_navigation/reference_path` | `nav_msgs/Path` | 当前局部参考路径，仅用于局部控制和调试 |
| `/camera_navigation/target_point` | `geometry_msgs/PointStamped` | 当前 Pure Pursuit 前视点 |
| `/camera_navigation/status` | `diagnostic_msgs/DiagnosticArray` | 状态、边界质量、规划模式和控制量 |
| `/camera_navigation/cmd_vel` | `geometry_msgs/Twist` | 默认与底盘隔离的控制输出 |

不建议在完成实车安全验证前直接将控制输出改为底盘 `/cmd_vel`。

## 11. 启动与配置原则

组合启动文件使用两个独立的参数文件参数：

- `perception_params_file`：感知参数；
- `navigation_params_file`：边界导航参数。

两个子启动文件内部都使用名为 `params_file` 的参数，因此组合启动时通过独立 `GroupAction` 作用域传递，避免感知参数覆盖导航参数并导致 `reference_side` 等配置回退到代码默认值。

在线相机示例：

```bash
ros2 launch camera_boundary_navigation camera_only_navigation.launch.py \
  depth_input_mode:=raw \
  enabled_on_startup:=false
```

数据集示例：

```bash
ros2 launch camera_boundary_navigation camera_only_navigation.launch.py \
  use_sim_time:=true \
  depth_input_mode:=compressed_depth \
  enabled_on_startup:=false
```

修改 `src` 下配置后，需要重新构建并重新加载安装空间，或在启动时显式传入源码参数文件。运行时应使用以下命令确认节点实际参数，而不能只检查文件内容：

```bash
ros2 param get /camera_boundary_navigation_node reference_side
ros2 param get /camera_boundary_navigation_node stability.enabled
```

## 12. 当前验证证据

### 12.1 单元测试

当前包含 7 个核心测试场景：

1. 二次曲线拟合并剔除离群点；
2. 左、右边界偏移路径的转向方向；
3. Bézier 连接段与偏移边界的切向连续性；
4. 边界曲率存在但距离处于死区时保持直线；
5. 连续多帧转弯确认与曲率变化惩罚；
6. 安全恢复帧数及障碍物拒绝；
7. Ground 近场盲区豁免不影响近场障碍检查。

### 12.2 指定 rosbag 冒烟验证

使用数据集：

```text
/home/huibo/tdr/navigation/nanhu-research-navigation/rosbag/
rosbag2_2026_08_26-16_25_06
```

以 `compressed_depth` 模式回放时，已观察到：

- 节点加载 `side=right`；
- 有效直线路径发布 21 个点，对应约 `2.0 m`、`0.10 m` 采样；
- 直行状态输出约 `linear=0.25 m/s`、`angular=0.0 rad/s`；
- 检测到路径障碍时进入 `PATH_BLOCKED` 并输出零速度；
- 障碍解除且连续有效帧满足要求后恢复跟踪。

这些结果证明数据链路、稳定直行选择、安全停车和恢复机制可以工作，但不等价于完成真车全场景验证。

## 13. 局限性与主要风险

1. **没有定位能力。** 系统不知道车辆在地图中的位置和全局航向，不能返回历史路径、到达全局目标或判断累计漂移。
2. **只能做短视距反应。** 规划范围受深度有效距离、相机视场、边界拟合上限和前边界共同限制。
3. **依赖深度质量。** 玻璃、积水、强光、黑色吸光表面、运动模糊和压缩异常可能产生空洞或假障碍。
4. **依赖重力估计。** 车辆剧烈加减速或 IMU 振动会污染加速度方向，进而影响地面高度和坡度判断。
5. **单侧参考不是完整道路理解。** 当前只用选定边界控制横向距离，另一侧边界主要隐含在 Ground 和障碍安全检查中；极窄道路或两侧同时收缩时可能保守停车。
6. **二次曲线表达能力有限。** 很急的复合弯、断续边界和局部错误分割可能无法用单条二次曲线可靠表达。
7. **没有车辆状态反馈。** 控制器假设速度指令能够被准确执行，未利用轮速、横摆角速度或转角反馈补偿打滑和执行器误差。
8. **参数与车辆强相关。** `offset`、`path_clearance`、近场停止范围和速度上限必须按车辆外形、制动距离及相机安装重新验证。

因此，该模块适合作为低速、单路、受控环境中的 LiDAR 失效降级模式，不应宣称为无定位条件下的通用自动驾驶方案。

## 14. 实车启用检查清单

- 校验相机内参、深度尺度、相机高度、轴向和 IMU 方向。
- 在静止状态确认地面点、左右/前边界和障碍点云与真实道路一致。
- 确认 `reference_side` 的运行时参数值，而非只看源码或安装目录。
- 测量车辆半宽和最大制动距离，复核 `offset`、`path_clearance`、`obstacle_stop_distance`。
- 保持输出话题隔离，先架空轮或使用急停监护进行低速测试。
- 分别验证输入断流、边界丢失、近场障碍、道路终点、急弯和深度空洞时必定停车。
- 确认底盘仲裁器能够让人工急停和上层暂停覆盖相机导航输出。
- 完成验证后再显式重映射控制话题，并保留速度上限和看门狗。

## 15. 后续演进建议

在保持无 odom 约束的前提下，可按以下顺序增强：

1. 同时利用左右边界构建完整走廊，但仍允许只跟随指定一侧，提升道路变窄时的安全性。
2. 生成多条直线/定曲率候选轨迹，在 Ground 覆盖、障碍净空、边界距离和曲率变化之间评分，而不是只有单一转弯候选。
3. 接入语义分割模型输出的可通行掩膜，与几何 Ground 结果做置信度融合；模型不能绕过深度障碍安全门。
4. 使用 IMU 横摆角速度只作短时转弯阻尼和控制反馈，不积分为里程计，从而保持“无定位”设计边界。
5. 根据边界拟合置信度、有效视距和深度缺失率自适应降低速度。

## 16. 代码与配置依据

- 感知说明：[`src/traversability_obstacle_cloud/README.md`](../src/traversability_obstacle_cloud/README.md)
- 感知配置：[`camera_ground_extraction.yaml`](../src/traversability_obstacle_cloud/config/camera_ground_extraction.yaml)
- 地面分类与边界提取：[`traversability_obstacle_cloud_node.cpp`](../src/traversability_obstacle_cloud/src/traversability_obstacle_cloud_node.cpp)
- 导航模块说明：[`src/camera_boundary_navigation/README.md`](../src/camera_boundary_navigation/README.md)
- 导航配置：[`boundary_follower.yaml`](../src/camera_boundary_navigation/config/boundary_follower.yaml)
- 边界拟合：[`boundary_model.cpp`](../src/camera_boundary_navigation/src/boundary_model.cpp)
- 路径和控制：[`boundary_controller.cpp`](../src/camera_boundary_navigation/src/boundary_controller.cpp)
- 稳定局部规划：[`stable_local_planner.cpp`](../src/camera_boundary_navigation/src/stable_local_planner.cpp)
- 安全门：[`safety_gate.cpp`](../src/camera_boundary_navigation/src/safety_gate.cpp)
- 节点集成与状态：[`camera_boundary_navigation_node.cpp`](../src/camera_boundary_navigation/src/camera_boundary_navigation_node.cpp)
- 组合启动：[`camera_only_navigation.launch.py`](../src/camera_boundary_navigation/launch/camera_only_navigation.launch.py)
- 核心测试：[`test_boundary_navigation.cpp`](../src/camera_boundary_navigation/test/test_boundary_navigation.cpp)
