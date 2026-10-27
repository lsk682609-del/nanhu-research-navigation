# SLAM、路网导航与 map_edit 接口移植分析

本文记录当前源码能够直接证明的连接关系，供更换机器人底盘、雷达、IMU 或计算板时逐项对照。

分析对象：

- 当前工作区的 `map_edit/`、`src/navigation2/`、`src/robot_launch/`。
- SLAM 参考实现 `/home/lxh/nanhu-huibo-slam-0916/src/fast_livo_with_bievr/`。

文中把“源码已经连接”与“运行时已经验证”分开。本文使用的是静态源码证据；当前工作区没有正在运行的 ROS 节点，因此不能据此宣称整套系统已经完成运行闭环。

## 1. 结论先行

这三个部分不是一个进程里的紧耦合模块，而是通过两类接口连接：

1. `map_edit` 与 `nav2_net_planner` 通过同一种路网 YAML 文件交换地图数据。
2. `map_edit` 与导航系统通过 ROS 2 服务、话题和自定义 `nav2_msgs` 消息交互；SLAM 与导航通过里程计、点云和 TF 交互。

运行链可以画成：

```text
LiDAR + IMU
    │
    ▼
FAST-LIVO/BIEVR-LIO（定位、里程计、点云、TF）
    │  /car1/odom、/car1/cloud_registered、TF
    ▼
nav2_input_bridge
    │  odom_local、cur_lid、map→base_link
    ▼
local_costmap + RegulatedPurePursuitController
    │
    │  FollowPath action
    ▼
nav2_net_system（net_navigator）
    │  ComputePathToPose action
    ▼
nav2_net_planner（读取拓扑 YAML）
    │
    ▼
controller_server → /cmd_vel → 底盘驱动
```

`map_edit` 位于这条链的“地图编辑和任务下发”一侧：

```text
map_edit 保存 topo YAML ─────► nav2_net_planner 加载拓扑
map_edit 点击目标点 ────────► /navigation_request
                              │
                              ▼
                         net_navigator
                              │
                              ├─► /compute_path_to_pose
                              └─► /follow_path
map_edit ◄──── /navigation_status、/navigation_global_path、/odom_local
```

因此，`map_edit` 不负责 SLAM，也不直接控制底盘；它负责编辑节点/边、保存地图、显示车辆和导航路径，并通过 ROS 服务提交目标。

## 2. 各目录的职责边界

### 2.1 SLAM：`nanhu-huibo-slam-0916/fast_livo_with_bievr`

SLAM 节点由 `LIVMapper` 统一接收传感器输入，再把输入路由到 BIEVR-LIO Pipeline、RTK/固定滞后前端和 MapManager。源码注释明确说明算法模块本身不直接订阅传感器，ROS 输入先进入 `LIVMapper`。

输入参数来自 `config/car5_rslidar_rtk.yaml`：

| 输入 | 默认示例 | 消息类型 | 用途 |
|---|---|---|---|
| `common.lid_topic` | `/car1/rslidar_points` | `sensor_msgs/msg/PointCloud2` | LiDAR 点云 |
| `common.imu_topic` | `/car1/rslidar_imu_data` | `sensor_msgs/msg/Imu` | IMU 预积分、去畸变和状态估计 |
| `rtk.topic` | `/rtk/gga` | `rtk_serial_reader/msg/Gga` | RTK 位置约束 |
| `rtk.heading_topic` | `/rtk/hpr` | `rtk_serial_reader/msg/Hpr` | 双天线航向约束 |

`initializeSubscribersAndPublishers()` 创建 LiDAR generic subscription、IMU subscription，以及可选的 GGA/HPR subscription。迁移到底盘时，首先要替换的不是导航参数，而是这些真实驱动话题、消息类型和 QoS。

SLAM 的主要输出如下：

| 输出 | 类型 | 语义 |
|---|---|---|
| `/<car_name>/odom` | `nav_msgs/msg/Odometry` | 标准 ENU 位姿和速度，供导航桥接或控制使用 |
| `/<car_name>/lio_odom` | `nav_msgs/msg/Odometry` | LIO 原始局部里程计 |
| `/<car_name>/path` | `nav_msgs/msg/Path` | 统一 ENU 轨迹 |
| `/<car_name>/lio_path` | `nav_msgs/msg/Path` | 启动校正后的 LIO 轨迹 |
| `/<car_name>/rtk_odom` | `nav_msgs/msg/Odometry` | RTK/融合定位结果 |
| `/<car_name>/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 已配准、转换到 ENU 的点云 |
| `/<car_name>/cloud_effected` | `sensor_msgs/msg/PointCloud2` | 有效点云 |
| `/<car_name>/mavros/vision_pose/pose` | `geometry_msgs/msg/PoseStamped` | 给飞控或外部视觉定位接口的位姿 |

标准输出的坐标约定由源码直接固定：

- `Odometry.header.frame_id = enu_frame`，配置默认是 `enu`。
- `Odometry.child_frame_id = <car_name>/base_link`。
- TF 广播的是 `enu -> <car_name>/base_link`。
- 点云输出的 `header.frame_id` 也是 `enu`。

这意味着把真实 SLAM 接到当前导航时，必须处理 `enu` 与导航使用的 `map` 之间的关系，并处理带车辆前缀的 child frame。不能只把 `/car1/odom` 改名成 `/odom` 就认为 TF 已经正确。

### 2.2 导航输入桥：`nav2_input_bridge`

`InputBridgeNode` 的接口定义在 `src/navigation2/bridge/nav2_input_bridge/src/input_bridge_node.cpp`：

| 参数 | 输入/输出 | 默认意义 |
|---|---|---|
| `local_odom_topic` | 输入 `Odometry` | 外部定位/SLAM 里程计 |
| `odom_output_topic` | 输出 `Odometry` | 默认 `odom_local` |
| `lidar_pcl_topic` | 输入 `PointCloud2` | 原始或 SLAM 点云 |
| `lidar_output_topic` | 输出 `PointCloud2` | 默认 `cur_lid` |
| `global_frame` | TF 父坐标系 | 默认 `map` |
| `robot_base_frame` | TF 子坐标系 | 默认 `base_link` |
| `goal_input_topic` | 输入 `NavSatFix` | GPS 目标 |
| `goal_output_topic` | 输出 `PoseStamped` | 默认 `goal_pose` |

桥接器对里程计做两件事：发布配置的 `odom_output_topic`，并广播 `global_frame -> robot_base_frame` 的 TF。点云则按配置裁剪/转换后发布为 `cur_lid`。当前 `nx_net_nav.yaml` 中控制器使用 `odom_local`，局部代价地图使用 `cur_lid`，这两项必须与 bridge 的输出名一致。

桥接器还负责把 GPS 目标转成平面 `PoseStamped`。如果使用纯 ENU 地图目标，`map_edit` 可以直接调用 `/navigation_request`，绕过 `/gp_goal`；如果使用经纬度目标，`map_edit` 发布 `/gp_goal`，由 bridge 根据参考原点转换。

### 2.3 路网规划：`nav2_net_planner`

规划器是 LifecycleNode，配置时加载 `topo_map_path`，并提供标准 Nav2 `ComputePathToPose` action：

- 起点：若 action 未提供 start，规划器通过 TF 查询 `robot_base_frame -> global_frame`。
- 终点：`ComputePathToPose` goal 中的 `PoseStamped`。
- 规划：把当前位姿和目标吸附到拓扑节点，使用图搜索得到节点序列。
- 输出：稀疏路径 `navigation_global_path`，可选的显示路径 `topology_plan`。
- 路径坐标系：配置的 `global_frame`，当前默认 `map`。

规划器不订阅 LiDAR，也不运行 SLAM；它只需要一棵可用的 TF 树和一个与当前全局坐标系一致的拓扑 YAML。

### 2.4 导航状态机：`nav2_net_system`

`net_navigator` 提供三种入口：

- `goal_pose` 话题；
- `navigate_to_pose` action；
- `navigation_request` 服务。

收到目标后，状态机先调用 `compute_path_to_pose`，再调用 `follow_path`。它还发布：

- `global_path`：当前执行的路径；
- `navigation_status`：自定义 `nav2_msgs/msg/NavigationStatus`；
- `nav_status`：兼容旧接口的 `std_msgs/msg/Int32`；
- `navigation_request` 服务的成功/失败响应。

因此 `map_edit` 的 ROS 导航按钮实际调用的是 `navigation_request` 服务，而不是直接调用 controller。服务请求的目标固定写成 `frame_id = "map"`，坐标值直接使用地图的东、北、上坐标。

### 2.5 `map_edit`

`map_edit` 有两个相互独立的能力：

1. 离线编辑/保存：加载 `map:`、`lla_mode:`、`origin:`、`node:`、`edge:` 段，编辑节点、边和区域，再序列化为拓扑文本。
2. 在线 ROS 接口：创建 `map_edit_goal_publisher` 节点，调用导航服务，订阅导航状态、导航全局路径和里程计。

在线接口由源码直接给出：

- `/navigation_request`（或带命名空间的版本）：下发 `nav2_msgs/srv/NavigationRequest`；
- `/navigation_status`：接收导航状态；
- `/navigation_global_path`：接收规划器发布的稀疏路径；
- `/odom_local`：更新编辑器中的车辆标记；
- `/gp_goal`：经纬度模式下发布 `sensor_msgs/msg/NavSatFix`。

调用服务时，`map_edit` 将节点的 ENU 坐标写入 `target_pose`，并将 `header.frame_id` 固定为 `map`。所以编辑器显示的地图坐标必须和导航的 `map` 坐标严格同源。

## 3. 拓扑地图文件是否真正打通

### 3.1 文件格式是兼容的

当前 `map_edit/include/topo_map.hpp` 与 `src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp` 都解析以下段：

```text
map:
<map_id> <map_name>
lla_mode:
0 或 1
origin:
<latitude> <longitude> <ellipsoid_height>
node:
<id> <type> <map_id> <x> <y> <z> [lat lon alt]
edge:
<id> <type> <map_id> <from> <to> <obt>
```

两边都使用 `node.id` 和 `edge.nodeId1/nodeId2` 建图，规划代价由节点的 ENU 欧氏距离计算。规划器不读取 PGM 栅格，也不读取 `regions` 区域段。

### 3.2 `map_edit` 的区域数据不是路网规划输入

编辑器工程可以附带区域段，用来保存禁行区、可行驶区或栅格编辑结果；`nav2_net_planner` 的 `TopoMap` 只读 `map/node/edge`。因此：

- 节点和边会影响路网规划；
- 区域多边形不会自动进入 planner；
- 区域数据要影响避障，必须另有 costmap layer 或点云/边界话题接入。

当前工作区中，局部代价地图通过 `cur_lid`、相机深度障碍和边界点云处理局部安全性，这与拓扑 YAML 是两条不同的数据链。

### 3.3 经纬度模式存在两个实现细节

`map_edit` 使用局部 ENU 近似公式；导航 planner 版本使用 WGS84 ECEF→ENU 的完整计算。两者文件字段兼容，但如果节点同时保存 LLA 和 xyz，加载时两边都可能重新由 LLA 生成 xyz。迁移时应做到：

- 所有节点都保留有效的 `lat lon alt`；
- `origin` 与 SLAM/bridge 使用的 ENU 原点完全一致；
- 不要手工混用不同原点生成的 xyz；
- 以 planner 重新加载后的节点坐标为最终检查值。

对于几百米范围，球面近似和 WGS84 的差异通常较小，但换板子时最危险的不是公式差异，而是原点、轴向和坐标系名字不一致。

## 4. 三种运行模式的真实关系

### 4.1 Gazebo 路网导航模式

`src/robot_launch/launch/gazebo_net_nav.launch.py` 启动：

- Gazebo/世界提供机器人、`/odom`、传感器和 TF；
- `nav2_input_bridge` 把 `/odom` 与模拟点云转换为 `odom_local`、`cur_lid`；
- `nav2_net_planner` 读取 `topo_map_gazebo.yaml`；
- `net_navigator`、controller 和 lifecycle manager 完成规划/跟踪。

这一模式没有启动真实 SLAM。Gazebo 的 `/odom` 是定位输入，拓扑地图的坐标必须与 Gazebo 的 `map`/`odom` 初始对齐。

### 4.2 实机 LIO + 路网导航模式

`nx_net_nav_slam.launch.py` 同时启动 RoboSense 驱动、FAST-LIVO、bridge、planner、navigator、controller 和底盘驱动。静态源码能证明组件被一起启动，但是否真正互相连上，要检查 `nx_net_nav.yaml` 中的实际 topic：

- LIO 输出的 odom 必须等于 bridge 的 `local_odom_topic`；
- LIO/驱动输出的点云必须等于 bridge 的 `lidar_pcl_topic`；
- bridge 输出必须等于 controller 的 `odom_topic` 和 costmap 的点云输入；
- LIO 广播的 frame 与 planner/controller 的 `map`、`base_link` 必须通过 TF 连通。

启动文件里的静态 TF 是 `base_link -> lidar`，而参考 BIEVR-LIO 默认输出 child frame 带车辆前缀（如 `car1/base_link`），这在多车或命名空间迁移时尤其需要核对。不能同时让两个节点发布同一对 TF。

### 4.3 纯 waypoint 模式

`waypoint_nav2.launch.py` 使用固定 `map -> odom` TF 和 waypoint navigator，不依赖路网 planner。它适合验证 controller、底盘和 TF 的基础闭环，但不能证明 SLAM、拓扑地图或 `net_navigator` 正常。

## 5. 换板子时必须替换的接口

### 第一层：传感器和驱动

- LiDAR 话题名和消息类型；
- IMU 话题名、坐标轴方向、单位和时间戳；
- LiDAR→IMU 外参；
- `base_link`→LiDAR 静态 TF；
- 驱动 QoS；
- RTK GGA/HPR 话题和时间基准（如果启用 RTK）。

### 第二层：SLAM 输出

- `/carX/odom` 的 `header.frame_id`、`child_frame_id`；
- `/carX/cloud_registered` 的 frame 和点字段；
- TF 发布者只能有一个；
- `enu`、`map`、`odom` 三者的方向和原点；
- 里程计消息中的 pose 与 twist 是否使用同一坐标约定。

### 第三层：导航桥参数

- `local_odom_topic` 指向真正的 SLAM/底盘 odom；
- `lidar_pcl_topic` 指向真正的点云；
- `odom_output_topic` 与 controller `odom_topic` 一致；
- `lidar_output_topic` 与 costmap 的 `pointcloud_topic` 一致；
- `global_frame`、`robot_base_frame` 与 TF 树一致；
- GPS 目标的参考原点与拓扑地图 `origin` 一致。

### 第四层：拓扑地图

- 地图节点的 `x/y/z` 是否是导航 `map` 坐标；
- `node.id` 是否唯一；
- 每条边两端节点是否存在；
- 单向边的 `type` 和 `obt` 是否符合实际通行方向；
- 起点到目标的图是否连通；
- `max_snap_distance` 是否适合新地图的节点间距；
- planner 使用的地图文件是否就是 map_edit 最后保存的文件，而不是安装目录中的旧副本。

## 6. 建议的验证顺序

换板子后不要先调 controller 参数，按下面顺序验证：

1. **输入层**：确认 LiDAR、IMU、RTK 话题存在，频率和时间戳正常。
2. **SLAM 层**：确认 `/carX/odom`、`/carX/cloud_registered` 有数据，检查 frame 和 child frame。
3. **TF 层**：确认从 `map` 到 `base_link`、从 `base_link` 到 LiDAR 的 TF 唯一且无环。
4. **bridge 层**：确认 `odom_local`、`cur_lid` 有数据，且时间戳没有被改成未来时间或零时间。
5. **planner 层**：确认 topology YAML 成功加载，单独发送 `ComputePathToPose`，检查 `navigation_global_path`。
6. **controller 层**：确认 `FollowPath` action 可用，controller 发布 `/cmd_vel`。
7. **底盘层**：确认底盘驱动订阅的命令话题与 controller 输出一致，并在低速安全条件下观察实际运动。
8. **map_edit 层**：确认编辑器订阅的是同一命名空间的 `odom_local`、`navigation_status`、`navigation_global_path`，服务名也是同一命名空间。

每一层都应保留实际命令和输出；只有第 1～8 层全部通过，才能说“换板子移植完成”。

## 7. 当前源码暴露出的主要风险

1. **`map` 与 `enu` 命名没有天然等价关系。** SLAM 参考实现默认发布 `enu -> <car>/base_link`，导航默认要求 `map -> base_link`。必须由 bridge 或专门的 TF/定位节点明确完成坐标变换。
2. **车辆前缀可能不一致。** SLAM 输出 child frame 可能是 `car1/base_link`，导航配置通常写 `base_link`。若没有 namespace/TF remap，planner 的 TF 查询会失败。
3. **同一条 TF 不能多方发布。** 实机启动文件、SLAM、bridge 和静态 TF 都可能涉及 base/odom 链；迁移时要选定唯一发布者。
4. **`map_edit` 的在线接口依赖本工作区的自定义 `nav2_msgs`。** 不能只安装系统 Nav2 后运行编辑器；服务和消息定义必须来自当前工作区。
5. **拓扑 YAML 与栅格/区域数据是分开的。** 保存了区域或 PGM 并不代表 planner 会使用它；局部避障仍依赖 costmap 的点云/边界输入。
6. **安装目录可能保存旧地图。** launch 文件经常使用 package share 下的 `maps/`，而 map_edit 保存的是源码目录中的文件。发布前必须确认安装副本已更新。
7. **当前构建记录不能证明完整构建通过。** 现有 `log/build_2026-10-15_08-39-08/logger_all.log` 显示 `traversability_obstacle_cloud` 的 CMake 配置返回码为 1；这属于构建证据，不能被“部分 install 文件存在”替代。

## 8. 最小迁移配置模板

假设新板子的 SLAM 输出为 `/robot1/lio/odom` 和 `/robot1/lio/cloud`，导航仍使用 `map/base_link`，最小改动应集中在 bridge 配置：

```yaml
/**/nav2_input_bridge:
  ros__parameters:
    local_odom_topic: /robot1/lio/odom
    lidar_pcl_topic: /robot1/lio/cloud
    odom_output_topic: odom_local
    lidar_output_topic: cur_lid
    global_frame: map
    robot_base_frame: base_link
```

但这段参数只有在下面条件成立时才有意义：

- `/robot1/lio/odom` 的 pose 坐标已经能转换到 `map`；
- `/robot1/lio/cloud` 的 frame 能通过静态 TF 连接到 `base_link`；
- planner 的拓扑节点坐标与 `map` 同一原点和轴向；
- controller 订阅 `odom_local`，costmap 订阅 `cur_lid`；
- 底盘驱动订阅 controller 实际输出的 `/cmd_vel`。

## 9. 本轮状态

已完成静态接口追踪：

- ✓ 明确 SLAM 输入、输出、frame 和 TF 责任；
- ✓ 明确 bridge 的 odom/点云转换；
- ✓ 明确 planner、navigator、controller 的 action 链；
- ✓ 明确 map_edit 的文件接口和在线 ROS 接口；
- ✓ 明确拓扑 YAML 与区域/栅格数据的边界；
- ✓ 整理换板子时的接口替换清单和验证顺序。

尚未由本文件证明：

- ? 当前机器上完整 ROS 工作区是否能一次构建通过；
- ? 真实 SLAM 与当前导航 launch 是否已经运行并输出预期话题；
- ? `map -> base_link` TF 是否在运行时唯一且连续；
- ? controller 输出是否已经让 Gazebo 或真实底盘运动。

下一步应按第 6 节顺序做运行验证，并首先解决构建日志中最早的真实错误。
