# 本工作区相机导航学习路线

本工作区的相机导航是一套局部、低速、无全局定位的导航系统。它不使用 LiDAR、轮速里程计、SLAM 或全局地图来决定当前这一帧怎么走；它只根据深度相机和可通行区域判断，在车辆当前的重力对齐局部坐标系中生成一条安全前进曲线，并输出速度。

## 1. 先记住整条数据链

```text
深度相机图像
  ├─ 深度图 ─► depth_cloud_projector_node ─► /depth/points
  └─ RGB图 ─► TwinLiteNet/分割 ─► /twinlitenet/drivable_mask

/depth/points + /twinlitenet/drivable_mask
                     │
                     ▼
traversability_obstacle_cloud_node
  ├─ Ground 栅格       ─► /traversability/grid
  ├─ 道路边界           ─► /traversability/ground_boundary_points
  └─ 障碍/可视边界      ─► camera_costmap_cloud_node

camera_costmap_cloud_node
  ├─ 过滤后边界         ─► /traversability/costmap/boundary_constraints
  └─ 过滤后障碍         ─► /traversability/costmap/depth_obstacles

camera_boundary_navigation_node
  ├─ /traversability/costmap/boundary_constraints
  ├─ /traversability/grid
  └─ /traversability/costmap/depth_obstacles
             │
             ▼
  边界拟合 → 稳定局部规划 → VFH避障 → 安全门 → 速度滤波
             │
             └─► /camera_navigation/cmd_vel
```

Gazebo 中最后一个话题默认是隔离的 `/camera_navigation/cmd_vel`。确认安全后，才通过 launch 参数改成底盘使用的 `/cmd_vel`。

## 2. 每个节点只负责什么

### `depth_cloud_projector_node`

把深度图按针孔模型投影为 `PointCloud2`。Gazebo 配置使用 1280×720、`fx=fy=924.277...`、`cx=640.5`、`cy=360.5`，深度坐标系是 `camera_color_optical_frame`。

它只解决“像素深度在哪里”；它不判断地面、道路或障碍。

### `traversability_obstacle_cloud_node`

把点云转换到局部车体坐标，结合深度几何和可通行分割得到 Ground、Obstacle 和道路边界。Gazebo 专用配置固定相机相对 `base_footprint` 的外参：前方 0.25 m、高 0.55 m、俯视 25°，并关闭 IMU 依赖。

它解决“哪些点属于地面、哪些点属于障碍、道路边界在哪里”。它不是路径跟踪器，也不发布速度。

### `camera_costmap_cloud_node`

这是相机感知和相机跟随器之间的适配层。它对边界进行视场截断过滤、平滑和补点，对障碍进行地面高度过滤、体素降采样和离群点处理，然后输出跟随器真正订阅的两个话题。纯视觉配置把输出统一到 `gravity_aligned_base`，约定 x 向前、y 向左。

### `camera_boundary_navigation_node`

这是控制核心。它要求边界、Ground 栅格和障碍点云使用相同时间戳，并在每一帧合并后才规划。输入任意一个断流、过期、未来时间戳超限或字段不合法，安全门都会停车。

它的内部顺序是：

1. 从 intensity=1/2 的边界点中选择左右边界；
2. 用鲁棒二次曲线拟合边界；
3. 两侧都有效时优先取安全走廊中线，单侧有效时向道路内部偏移；
4. 边界不足且 `boundaryless.enabled=true` 时，生成机体正前方的无边界路径；
5. 稳定规划器先判断直行是否安全，避免分割抖动导致左右摇摆；
6. 正常路径被障碍阻挡时，VFH 采样前进圆弧，检查 Ground 覆盖、障碍膨胀、曲率和可停车距离；
7. 最终通过安全门和速度滤波后发布 `Twist`。

## 3. 三个最重要的坐标约定

- 感知输出和跟随器输入的 `header.frame_id` 必须是 `gravity_aligned_base`；
- x 轴是车辆前方，y 轴是车辆左侧，z 轴向上；
- 该系统没有 odom/航向反馈，因此“前进”始终表示当前车体前方，绕障后不能保证回到原始全局航向。

这也是它与 SLAM/Nav2 路网导航的根本区别：相机导航不解决“我在地图哪里”，只解决“我现在这一帧往哪里安全走”。

## 4. 两个启动入口

### Gazebo 纯视觉仿真

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch camera_boundary_navigation gazebo_camera_navigation.launch.py \
  enabled_on_startup:=false \
  cmd_vel_topic:=/camera_navigation/cmd_vel
```

这个 launch 启动深度投影、可通行/障碍处理、适配器和相机跟随器。它默认 `use_sim_time=true`，并加载 Gazebo 专用相机内参和固定外参。

### 真实相机链

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch camera_boundary_navigation camera_only_navigation.launch.py \
  depth_input_mode:=compressed_depth \
  enabled_on_startup:=false \
  cmd_vel_topic:=/camera_navigation/cmd_vel
```

该入口还会包含相机驱动/深度处理 launch。真实设备的深度话题、相机内参、外参和 `use_sim_time` 必须按实际硬件配置。

## 5. 启动后先看什么

先不要打开运动，先确认感知链：

```bash
ros2 node list
ros2 topic list | grep -E 'camera|depth|traversability|camera_navigation'
ros2 topic echo /traversability/grid --once
ros2 topic echo /traversability/costmap/boundary_constraints --once
ros2 topic echo /traversability/costmap/depth_obstacles --once
ros2 topic echo /camera_navigation/status --once
```

重点看四件事：

1. 三个输入点云是否存在；
2. 三个输入的 `header.stamp` 是否一致；
3. `header.frame_id` 是否都是 `gravity_aligned_base`；
4. status 是否说明 `BOUNDARY_FOLLOW` 或 `BOUNDARYLESS_FORWARD`，而不是 `BLOCKED`。

然后再验证使能逻辑：

```bash
ros2 topic echo /camera_navigation/cmd_vel
ros2 topic pub --once /camera_navigation/enable std_msgs/msg/Bool "{data: true}"
```

默认需要连续通过 `safety.recovery_valid_frames=5` 帧，才会从停车状态恢复。任何输入超出 `safety.input_timeout=0.25` s，都会重新停车。

## 6. 如何读状态诊断

`/camera_navigation/status` 中最有价值的字段是：

- `navigation_mode`：`BOUNDARY_FOLLOW` 或 `BOUNDARYLESS_FORWARD`；
- `boundary_valid`：边界拟合是否成功；
- `avoidance_mode`：正常跟随、避障、回归或阻塞；
- `avoidance_reason`：停车的具体原因；
- `path_clearance`：路径到膨胀障碍外缘的距离；
- `corridor_valid/corridor_usable`：双边界走廊是否存在且足够宽；
- `free_length`：当前路径前方连续安全长度；
- `speed_limit`：安全检查后允许的速度。

判断故障时按层次看：

| 现象 | 优先怀疑 |
|---|---|
| 没有 `/depth/points` | 深度输入话题、编码或投影参数 |
| 有点云但没有 Ground | 相机外参、深度单位、地面高度或点云坐标系 |
| Ground 有、边界没有 | 分割 mask、边界提取阈值或视场过滤 |
| 三个输入存在但 status 为 blocked | 时间戳不同步、Ground 覆盖不足、障碍过近或安全弧长不足 |
| status 有效但 cmd_vel 为零 | 未 enable、pause、生效帧数未满足或安全门仍停车 |
| cmd_vel 有值但车不动 | cmd_vel 话题没有接到底盘/Gazebo 控制器 |

不要先调速度参数来解决上游没有数据的问题。

## 7. 当前参数的学习重点

先理解会改变“是否允许运动”的参数：

- `motion.enabled_on_startup`：默认 false；
- `boundaryless.enabled`：无有效边界时是否允许正前方前行；
- `avoidance.enabled`：是否启用 VFH，开启无边界模式时必须开启；
- `safety.input_timeout`：输入断流多久停车；
- `safety.minimum_front_range`：安全路径至少要有多长；
- `safety.path_clearance`：车辆包络和障碍之间的最小安全距离；
- `safety.recovery_valid_frames`：从停车恢复前需要连续多少帧有效输入。

再理解影响跟踪形状的参数：

- `boundary.fit_x_min/fit_x_max`：拟合使用的纵向窗口；
- `boundary.minimum_points/minimum_x_span`：边界是否足够支撑拟合；
- `boundary.offset`：相对边界向道路内部的偏移；
- `control.lookahead_distance`：纯追踪前视距离；
- `control.maximum_abs_curvature`：异常曲率的硬保护；
- `avoidance.horizon/minimum_path_length`：避障候选的最大/最小前进长度。

## 8. 当前学习阶段

建议按四个阶段推进：

1. **只看感知**：不 enable，确认深度点云、Ground、边界和障碍都在同一坐标系。
2. **合成输入测试**：运行 `test_vfh_node_integration.py`，学习正常跟随、障碍阻塞、恢复和断流停车。
3. **Gazebo 隔离速度闭环**：保持 `/camera_navigation/cmd_vel`，确认命令生成和停车逻辑，不接真实底盘。
4. **监督接底盘**：确认急停和速度上限后，才将 `cmd_vel_topic` 改成 `/cmd_vel`，低速验证车辆运动。

当前最有信息价值的下一步是完成第 1 阶段：先启动 Gazebo 纯视觉 launch，采集 `/camera_navigation/status` 和三个输入点云的 header/frame，再根据第 6 节定位是否卡在感知、同步还是安全门。
