# 南湖科研导航项目 · 快速梳理文档

> 面向人类开发者与 AI 智能体的项目总览。阅读本文件后应能理解：系统架构、节点职责、
> 话题数据流、配置入口、启动方式与常见改动位置。
>
> 更新日期：2026-08-25。仓库：`nanhu-research-navigation`（分支 `master`，最新提交 `cd54ff3 feat: 最新版本的导航代码`）

---

## 0. 项目一句话

基于 **ROS2 Humble** 的轮式机器人**路网拓扑导航**系统：以"路网图（拓扑地图）+ 图搜索"替代传统栅格全局规划，
配合 Nav2 生态的 `controller_server`（内部代价地图 + 局部避障），由 `net_navigator` 状态机串联，
`input_bridge` 负责里程计修正 / GPS 目标转换 / 点云裁剪转发。

**本仓库只包含导航代码**（`src/navigation2/` 16 个包 + `src/robot_launch/` 启动包）。
定位（FAST-LIVO2）、雷达驱动（RoboSense/Livox）、底盘驱动（CAN）、mavros 均为**外部依赖包**，
不在本仓库中（见 §11.1）。

## 1. 目录结构

```
nanhu-research-navigation/
├── src/
│   ├── navigation2/                 # 导航功能包（Nav2 官方 1.1.18 基线 + 自研修改）
│   │   ├── bridge/nav2_input_bridge/      # 自研：里程计修正、点云裁剪、GPS 目标
│   │   ├── planner/nav2_net_planner/      # 自研：路网拓扑规划（SNN 波前/A*）
│   │   ├── system/nav2_net_system/        # 自研：导航状态机 net_navigator
│   │   ├── system/nav2_log_record/        # 自研：日志落盘节点
│   │   ├── system/nav2_lifecycle_manager/ # 官方原版：生命周期管理
│   │   ├── controller/nav2_controller/    # 官方 + 小改：controller_server
│   │   ├── controller/nav2_a_star_rpp/    # 官方 RPP 大改：+Theta* 局部绕障、路径冻结
│   │   ├── map/nav2_costmap_2d/           # 官方 + Denoise 去噪层插件
│   │   └── common/  nav2_common、nav2_core、nav_2d_msgs、nav_2d_utils、
│   │                nav2_msgs、nav2_util、nav2_voxel_grid
│   │                # common 层除 nav2_msgs/nav2_core/nav2_common 有自定义扩展外，
│   │                # 其余为官方原版
│   ├── robot_launch/                  # launch/config/maps/rviz（纯 Python 包）
│   ├── traversability_obstacle_cloud/ # 深度相机单帧地面/障碍/边界提取
│   └── camera_boundary_navigation/    # 无odom的相机单边道路跟随与安全门
├── scripts/                          # 启动/构建/调试脚本（见 §8.2）
├── tools/                            # monitor_nav.py 资源监控 + 历史日志样本
├── doc/                              # 项目文档（本文件 + 旧文档，见 §12）
├── 3party/fmt/8.1.1/                 # 本地 fmt v8 库（解决系统 fmt v9 不兼容，见 §9.2）
├── build/ install/ log/              # colcon 产物（gitignore）
└── .gitignore                        # 忽略 logs/ *.log install/ build/ log/
```

### 每个导航包一句话

| 包 | 说明 |
|---|---|
| `nav2_input_bridge` | 数据桥：SLAM odom → 修正位姿/速度 + TF 广播；点云 XYZIRT→XYZI 裁剪转发；GPS(LLA)→ENU 目标/路径；gp_origin 与 mavros 握手 |
| `nav2_net_planner` | 路网拓扑全局规划：加载 `topo_map.yaml`，起点/终点吸附（snap），SNN 波前规划（A* 代码保留未用），路径密化 |
| `nav2_net_system` | 导航状态机 `net_navigator`：goal → 规划 → 切片 → FollowPath 循环 → 到达判定 |
| `nav2_controller` | Nav2 `controller_server`（官方 + `/pause` 暂停改动）；内部自建 `local_costmap` 节点 |
| `nav2_a_star_rpp` | RegulatedPurePursuit（官方基线 + **Theta\* 局部绕障重规划** + polyline 跟踪 + 路径切片冻结 + 碰撞检测可视化） |
| `nav2_costmap_2d` | 官方 1.1.18 + 自定义 **DenoiseLayer**（孤立噪点/小障碍团滤除）；obstacle_layer 含 livox 调试日志 |
| `nav2_log_record` | 订阅 `log/event`、`log/record`，写入 `./logs/navigation.log` 与 `./logs/recorder.csv` |
| `nav2_lifecycle_manager` | 官方原版：按 `node_names` 顺序配置/激活节点，bond 心跳，断连自动 respawn |
| `traversability_obstacle_cloud` | 深度图投影、IMU重力对齐、2.5D地面/障碍分类及U形地面边界输出 |
| `camera_boundary_navigation` | 无LiDAR/无odom场景：鲁棒拟合单侧边界，生成局部偏移路径并低速视觉伺服控制；默认禁止运动且输出话题与底盘隔离 |
| `nav2_common` | +自定义：`ExecStatus` 枚举（导航执行状态码，见 §6.3）、`sp_timer.h` 计时器 |
| `nav2_msgs` | +自定义：`Log.msg`、`Route/RouteNode/RouteEdge/EdgeCost.msg`、`SetRouteGraph.srv`、`ComputeRoute/ComputeAndTrackRoute/NavigateTrackPath.action`（路由消息族目前未被 net 链路使用） |
| `nav2_core` | +自定义：`route_exceptions.hpp` 路由异常族 |
| `nav2_util` / `nav_2d_msgs` / `nav_2d_utils` / `nav2_voxel_grid` | 官方原版 |

## 2. 系统架构与数据流

主链路（`nx_net_nav_slam.launch.py`，RoboSense + FAST-LIVO2 场景）：

```
                        ┌──────────────┐
                        │ rslidar_sdk  │──/rslidar_points(XYZIRT, frame=rslidar)──┐
                        └──────────────┘                                          │
                        ┌──────────────┐                                          ▼
                        │  fast_livo   │──/car1/odom(IMU位姿,无速度)──►┌────────────────────┐
                        └──────────────┘                           │   input_bridge      │
                                                                   │  · 帧间差分补速度      │
                        ┌──────────────┐                           │  · 减 body_offset    │
                        │   mavros     │◄──gp_origin 握手───────────│  · z 归零 / frame 重映射│
                        └──────────────┘                           │  · 点云裁剪转 XYZI    │
                        GPS目标(gp_goal/                            │  · GPS→ENU 目标转换   │
                        uav/target_gps/                             └───────┬──────┬───────┘
                        navigate_track_gp_path)                            │      │
                                  │                          /odom_local + /cur_lid   │
                                  │   (goal_pose/navigate_track_path)                  │
                                  ▼                                    │              │
                        ┌─────────────────┐◄──/goal_pose──            │              │
                        │  net_navigator  │                          │              │
                        │  (状态机)        │                          ▼              │
                        └───────┬─────────┘             ┌─────────────────────┐      │
                                │ /compute_path_to_pose │ controller_server   │◄─────┘
                                ▼                       │  (内部建 local_      │
                        ┌─────────────────┐             │   costmap: 点云→2D   │
                        │ net_planner_    │             │   代价图, inflation) │
                        │ action_server   │             │  · RPP 纯追踪         │
                        │ (路网图搜索)     │──/topology_plan │  · Theta* 局部绕障    │
                        └─────────────────┘             │  · 碰撞检测           │
                                                        └──────────┬──────────┘
                                                     /follow_path  │ /cmd_vel
                                ┌──────────────────────────────────┤
                                │ /global_path(剩余路径)           ▼
                        ┌───────┴─────────┐              ┌─────────────────┐
                        │  可视化/RViz     │              │ ros2can car_driver│
                        └─────────────────┘              │ (CAN→底盘电机)    │
                                                         └─────────────────┘
```

**五条链路速记**：

1. **定位链**：雷达点云 → FAST-LIVO2 → `/car1/odom`（IMU 位姿）→ `input_bridge` 减 body_offset、补速度、z 归零 → `/odom_local` + **TF 广播 map→base_link** → 控制器/代价地图消费。
2. **感知链**：`/rslidar_points`(XYZIRT) → `input_bridge` 裁剪（可选）转 XYZI → `/cur_lid`(frame=lidar) → controller_server 内部 local_costmap 的 obstacle_layer（TF 转 map 系、高度 0.2~0.65m 截取、3m 范围内标记/清除）→ inflation → 2D 代价图。
3. **规划链**：`/goal_pose`（map 系，由 bridge 从 GPS 转换或直接发布）→ `net_navigator` 请求 `/compute_path_to_pose` → `net_planner_action_server` 吸附路网 + 图搜索 + 0.1m 密化 → `/topology_plan`。
4. **执行链**：`net_navigator` 每 1Hz 把剩余路径按 30m 视野切片 → `/follow_path` action → `controller_server`（RPP+Theta\*）→ `/cmd_vel` → `car_driver` CAN 下发。
5. **日志链**：各节点 → `/log/event`、`/log/record` → `nav2_log_record` → `./logs/navigation.log`、`./logs/recorder.csv`。

## 3. TF 树

```
map ──→ base_link ──→ lidar (或 livox_frame / rslidar)
```

| TF | 谁发布 | 说明 |
|---|---|---|
| `map → base_link` | **input_bridge**（TF 广播，修正 body_offset 后） | 导航主链路的定位。SLAM 场景下取代 SLAM 未修正的 TF |
| `map → odom` | launch 中 static_transform_publisher（**恒等**，仅 `nx_net_nav.launch.py` / `net_nav2.launch.py` 等非 SLAM 场景） | 此时 odom→base_link 需外部节点（定位）提供 |
| `base_link → lidar` | launch 中 static_transform_publisher | 不同场景外参不同，见下表 |

base_link→雷达静态外参（三套 launch 各不相同）：

| launch | 子 frame | 平移 x/y/z (m) | 旋转 (roll,pitch,yaw) |
|---|---|---|---|
| `nx_net_nav_slam.launch.py`（RoboSense） | `lidar` | 0.20 / 0 / 0.30 | 0, 0.349, 0 |
| `nx_net_nav.launch.py`（MID360） | `livox_frame` | 0.16 / 0 / 0.25 | 0, 0.519, 0.007 |
| `3588_net_nav.launch.py`（无雷达） | `lidar` | 0.24 / 0 / 0.30 | 0, 0, 0 |

坐标系约定：`map` 全局固定（原点=SLAM 初始化点或 GPS 参考点 ENU 原点）；`base_link` 机器人地面投影中心（z 强制 0，2D 导航）；`lidar` 雷达系。

## 4. 节点详解

### 4.1 `net_planner_action_server`（包 nav2_net_planner）

- 类型：LifecycleNode，executable `net_planner_action_server`。`src/navigation2/planner/nav2_net_planner/src/net_planner_action_server.cpp`
- 职责：独立的 `ComputePathToPose` action server（不使用 planner_server + plugin 的官方架构，直接独立成节点）。
- 订阅/服务：TF（map→base_link 取机器人位姿）
- Action server：`/compute_path_to_pose`（`nav2_msgs/action/ComputePathToPose`）。`use_start=false` 时以 TF 位姿为起点。
- 发布：`/topology_plan`（`nav_msgs/Path`，transient_local）；`/log/event`
- 规划流程（`execute()`）：`topo_.planNodeIds(起点, 终点, max_snap_distance, edge_snap_enabled, ...)` → `buildPath`（起点实际位姿 + 途经节点 + 终点实际位姿，节点朝向=边方向）→ `densifyPolyline`（步长 `dense_path_step`=0.1m）→ succeed。
- 关键参数：`topo_map_path`（launch 覆盖为 `maps/topo_map.yaml`）、`max_snap_distance` 15.0、`edge_snap_enabled` false（false=只吸附节点；true=可投影到边上生成虚拟节点）、`dense_path_step` 0.1、`planner_path_topic` topology_plan。

### 4.2 路网拓扑与搜索算法（`topo_map.hpp` + `snn_topo_planner.cpp`）

- `include/nav2_net_planner/topo_map.hpp`（约 750 行，header-only）：拓扑图数据结构与解析
  - `Node{id, type, mapId, x,y,z, latDeg,lonDeg,altM}`；`Edge{id, type(Bidirectional/Unidirectional), mapId, nodeId1, nodeId2, obt, cost}`；`SubMap`
  - 支持 `lla_mode`：节点存经纬高，通过 `origin:` 参考点换算 ENU（当前 `topo_map.yaml` 中 `lla_mode: 0`，直接用平面 xy）
  - `snapPointForPlanning()`：起点/终点吸附到最近节点（或边上投影，若 `edgeSnap=true`），生成虚拟节点与附加邻接
  - `planNodeIds()`：吸附后调用 **`SnnTopoPlanner::plan()`（SNN 波前规划器）**；A*（`astarExtended`/`astar`）实现保留在文件中但**当前未被调用**
- `snn_topo_planner.{hpp,cpp}`：脉冲神经网络（SNN）波前规划——每个路网节点=一个神经元，边 cost=突触延迟；目标节点 t=0 注入电流放电，波前沿反向邻接传播；**首次放电时间即最短路径**（first-spike 最短路性质），一次性放电门控保证最短路。默认动力学：`u_rest=0, beta=-10, theta=1, I_spike=1, du_dt=1`。
- 另注：`net_planner.{hpp,cpp}` 中还有一个标准 Nav2 `GlobalPlanner` 插件实现（`nav2_net_planner::NetPlanner`，`global_planner_plugin.xml` 注册），支持动态参数热更新 topo_map——当前 launch **不加载它**（走独立 action server），属备用实现。

### 4.3 `net_navigator`（包 nav2_net_system）

- 类型：LifecycleNode，executable `net_navigator_node`。`src/navigation2/system/nav2_net_system/src/net_navigator.cpp`
- 职责：导航状态机（替代官方 bt_navigator 行为树）。

```
IDLE ──(新goal)──► PLANNING ──(规划成功)──► EXECUTING ──(剩余<0.3m)──► SUCCESS
 ▲                    │                        │
 │                    │ 规划失败/超时/取消       │ 取消
 └────────────────────┴────────────────────────┘
```

- 订阅：`/goal_pose`（`PoseStamped`，非 active 态忽略）、`/cancel_navigation`（`Empty`）
- 发布：`/global_path`（剩余路径，含机器人当前点，transient_local）、`/nav_status`（`std_msgs/Int32`，状态枚举）、`/log/event`、`/log/record`
- Action server：`/navigate_to_pose`（`nav2_msgs/action/NavigateToPose`，含 feedback：current_pose/navigation_time/distance_remaining）
- Action client：`/compute_path_to_pose`、`/follow_path`
- 主循环：100Hz tick（激活时），各状态分发：
  - **PLANNING**：防重复请求（`compute_request_sent_` + `planning_timeout_s`=10s 超时）；发 ComputePathToPose（`use_start=false`）；结果回调中校验目标未变（`sameGoalXY`，容差 `goal_match_tolerance`=0.05m）→ 存入 `mission_route_`/`remaining_path_` → EXECUTING。
  - **EXECUTING**（节流 `executing_freq_hz`=1Hz）：
    1. TF 取机器人位姿（失败→`NET_NO_ROBOT_POS`）
    2. `projectRobotOnPath` 投影 → 裁剪 `remaining_path_`（去掉已走过段）→ 发布 `/global_path`
    3. 剩余距离 `< goal_xy_tolerance`(0.3m) → SUCCESS（反馈 action 结果）
    4. 否则 `sliceRouteAhead`（从投影点向前截 `follow_slice_horizon_m`=30m，0.15m 采样）→ 切片 <2 点则重规划 → 发 `/follow_path`（controller_id=FollowPath）
    5. 每周期写 `net,exec` CSV 记录（机器人位姿、剩余距离、子目标点）
- 状态枚举：`IDLE/PLANNING/EXECUTING/SUCCESS/PLAN_FAILED/EXEC_FAILED`（`include/net_navigator.hpp:42`）

### 4.4 `controller_server`（包 nav2_controller）

- 类型：LifecycleNode，executable `controller_server`，同时注册为 rclcpp component。`src/navigation2/controller/nav2_controller/src/controller_server.cpp`
- 与官方差异（**仅一处实质改动**）：
  - 新增参数 `cmd_vel_pause_topic`（默认 `/pause`，config 设 `pause`）：订阅 `std_msgs/Bool`，true 时 `publishVelocity` 强制发布零速（控制器内部照常计算，解除即恢复）——用于调试/急停。
- 订阅：`odom_topic`（config=`odom_local`）、`speed_limit`、`pause`
- 发布：`/cmd_vel`（LifecyclePublisher）
- Action server：`/follow_path`
- **内部自建 `local_costmap` 节点**（`controller_server.cpp:69`）：costmap_2d 在 controller_server 进程内以独立线程运行，插件为 `obstacle_layer` + `inflation_layer`（参数见 §7.1），订阅 `/cur_lid` 点云。启动 launch 中**没有**单独的 costmap 节点。
- 控制器插件（plugins.xml）：`simple_progress_checker`/`pose_progress_checker`（ProgressChecker）、`simple_goal_checker`/`stopped_goal_checker`/`position_goal_checker`（GoalChecker）；实际运行插件 `nav2_a_star_rpp::RegulatedPurePursuitController`。

### 4.5 RPP 控制器（包 nav2_a_star_rpp，纯插件无独立可执行）

- 文件：`src/navigation2/controller/nav2_a_star_rpp/src/regulated_pure_pursuit_controller.cpp`（1424 行）+ `theta_star.cpp`
- 在官方 RegulatedPurePursuit 基础上新增（详见 `include/nav2_a_star_rpp/regulated_pure_pursuit_controller.hpp`）：

| 新增参数（config 现值） | 含义 |
|---|---|
| `lethal_cost` = 200 | Theta\* 判障阈值（≥此值不可通行） |
| `traversal_cost_weight` = 3.0 | Theta\* 代价图遍历代价权重（越大越绕障碍） |
| `prefer_planner_polyline_when_clear` = **true** | 折线畅通时密化跟踪、不跑 Theta\*；遇障才切换（config 已开启，代码默认 false） |
| `polyline_collision_check_resolution` = 0.05 | 折线判障采样步长 (m) |
| `polyline_track_max_length` = 15.0 | 折线密化/判障的最大弧长 (m) |
| `publish_global_path_slice_freeze` = true | 发布切片冻结信号 |
| `publish_merge_connection_point` = true | 发布 Theta\* 段与全局尾巴的衔接点 |

- **每拍流程**（`getLocalPlan`，cpp:1099-1284）：
  1. 绕障结束判断：机器人到达 `join_idx_`（Theta\* 段终点）→ 清 `avoid_flag_`
  2. 沿 `global_plan_` 扫最后一个代价可通行的点 → `local_goal`
  3. 折线判障（仅 `prefer_planner_polyline_when_clear=true` 时）：畅通→直接跟踪密化折线；遇障→
  4. **Theta\* 重规划**（8 邻域、任意角度 LOS、隔代父节点重连）：`setStartAndGoal(机器人位姿, local_goal)` → 生成路径 + 按 costmap 分辨率线性插值 → 与全局尾巴拼接成 `track_path_`
  5. 发布 `global_path_slice_freeze`（true=冻结全局切片）、`merge_path_connection_pose`（衔接点）
- 官方已有功能照常工作：`use_collision_detection`（对当前 cmd_vel 前向投影做 footprint 碰撞检测，碰撞抛异常停止；投影弧发 `lookahead_collision_arc`）、弯道曲率减速、终点减速、`use_rotate_to_heading` 原地转向（`RTP`=对准路径 / `RTG`=对准目标）、`use_interpolation` 预瞄插值等。
- 发布可视化：`received_global_plan`（变换剪枝后实际跟踪路径）、`lookahead_point`、`lookahead_collision_arc`、`global_path_slice_freeze`、`merge_path_connection_pose`（均挂在 controller_server 节点命名空间下）。
- Theta\* 代价细节（`theta_star.hpp:194-224`）：`getCost = 26 + 0.9*costmap_cost`；单格代价 `w_traversal_cost * cost²/lethal_cost²`；未知格按 253 计；g 含欧氏距离权重 2.0、启发权重 1.0。

### 4.6 `input_bridge`（包 nav2_input_bridge）

- 类型：LifecycleNode，executable `input_bridge_node`。`src/navigation2/bridge/nav2_input_bridge/src/input_bridge_node.cpp`
- **里程计链**（`onOdom`，订阅 `local_odom_topic`=`/odom`，best_effort）：
  1. 帧重映射 `world→map`、`car1/base_link→base_link`
  2. 减 body_offset（**IMU 位姿 → 机器人中心**：`offset_world = q_odom * offset_body`，config `body_offset_x=-0.40, z=0.50`）
  3. z 强制 0（2D 导航）
  4. 帧间差分补速度（位置差分→线速度；四元数差分→角速度）
  5. 发布 `/odom_local` + **TF 广播 map→base_link**（覆盖 SLAM 未修正 TF）
- **点云链**（`onLidarMsg`，订阅 `lidar_pcl_topic`=`/rslidar_points`）：两遍遍历，可选 crop 裁剪（`crop_enabled`=false），输出 XYZI 四字段 → `/cur_lid`（frame=`lidar`）
- **GPS 目标链**：订阅 `/gp_goal`、`/uav/target_gps`（NavSatFix）→ `llaToEnu`（参考点 `reference_latitude/longitude/altitude`）→ `/goal_pose`（map 系）。`require_nav_sat_fix=true` 时拒绝无固定解目标。**必须在 gp_origin 验证通过后才处理目标**。
- **GPS 路径链**：订阅 `/navigate_track_gp_path`（`nav_msgs/Path`，x/y/z=lat/lon/alt）→ 逐点 ENU 转换 → `/navigate_track_path`。
- **mavros gp_origin 握手**：每 500ms 发布 `/mavros/global_position/set_gp_origin`（配置参考点），订阅 `/mavros/global_position/gp_origin` 回读，一致后置 `gp_origin_verified_=true`（随后停止握手）。
- 看门狗：1Hz `checkDataStatus`，odom/雷达超时 2s 或点云为空 → error 日志。

### 4.7 `nav2_log_record`（包 nav2_log_record）

- 类型：LifecycleNode，executable `log_server_node`。`src/navigation2/system/nav2_log_record/src/log_server_node.cpp`
- 订阅：`/log/event`（`nav2_msgs/msg/Log{level, node_name, message}`）、`/log/record`（`std_msgs/String`，内容为已拼好的 CSV 行）
- 输出（**只写文件，不发布 ROS 话题**，目录 `./logs/`）：
  - `navigation.log`：事件日志，spdlog rotating 10MB×3，格式 `[时间] [nav2] [级别] 消息`
  - `recorder.csv`：结构化数据，rotating 100MB×20，每行 `<时间戳>,<data>`
- 参数：`log_output`（`both`=文件+屏幕 / `console`=仅屏幕 / 其他=仅文件）、`log_level`（info）

### 4.8 `lifecycle_manager`（包 nav2_lifecycle_manager，官方原版）

- executable `lifecycle_manager`，节点名 `lifecycle_manager_navigation`
- 参数：`node_names: ['nav2_log_record', 'nav2_input_bridge', 'net_planner_action_server', 'net_navigator', 'controller_server']`、`autostart: true`、`bond_timeout: 10.0`
- 行为：按顺序 CONFIGURE→ACTIVATE 各节点；ACTIVATE 后建 bond 心跳（0.1s 周期）；bond 断裂→整体 reset；节点恢复后 10s 窗口内自动重启。
- 服务：`/lifecycle_manager/manage_nodes`、`/lifecycle_manager/is_active`。
- 已知现象：`ros2 node list` 只显示 4 个节点（生命周期节点 DDS 发现层不可见），实际 7 个全部运行正常（见 `doc/project-setup-record.md` §4）。

### 4.9 外部节点（launch 引用、本仓库外）

| 节点 | 包 | 说明 |
|---|---|---|
| `rslidar_sdk_node` | rslidar_sdk | RoboSense 雷达驱动，发 `/rslidar_points` |
| `car1` (fastlivo_mapping) | fast_livo | FAST-LIVO2 定位，发 `/car1/odom` |
| MID360 驱动 | livox_ros_driver2 | 发 `/livox/lidar`、`/livox/imu` |
| `car_driver` | ros2can | 订阅 `/cmd_vel`，CAN 下发底盘（无 ROS 参数） |
| mavros `px4.launch` | mavros | FCU 连接 `/dev/ttyACM0:230400`，gp_origin 服务 |

### 4.10 无odom相机单边跟随（独立降级链）

- `traversability_obstacle_cloud` 发布 `/traversability/ground_boundary_points`，intensity
  1/2/3分别为左/右/前边界，并同步发布 Ground/Obstacle/Unknown 栅格结果。
- `camera_boundary_navigation_node` 不读取odom或TF；按时间戳组合边界、栅格和障碍数据，
  鲁棒拟合固定一侧边界。默认启用稳定局部规划：先检查车辆正前方的直线路径，直线安全且
  边界距离位于死区内时强制直行；同方向偏差持续多帧后才生成定曲率转弯路径，并惩罚相邻
  帧曲率变化。关闭稳定规划后可退回边界向内偏移的局部纯追踪路径。
- 默认 `motion.enabled_on_startup=false`，且速度发往隔离话题
  `/camera_navigation/cmd_vel`。输入超时、边界无效、路径离开Ground、路径附近存在障碍、
  前方范围不足或 `/pause=true` 时持续发布零速。
- 组合启动文件：`camera_boundary_navigation/launch/camera_only_navigation.launch.py`。
  该链路只适用于单一路面低速前进，不提供全局位置、目标到达和岔路选择能力。

## 5. 核心 Topic / Action 清单

| 名称 | 类型 | 发布者 → 订阅者 | 说明 |
|---|---|---|---|
| `/rslidar_points` | PointCloud2 (XYZIRT) | rslidar_sdk → input_bridge | 原始点云 |
| `/cur_lid` | PointCloud2 (XYZI, frame=lidar) | input_bridge → controller_server(内部 local_costmap) | 感知输入 |
| `/odom`（SLAM 原始位姿，frame world/car1_base） | Odometry | fast_livo → input_bridge | 无速度、IMU 系 |
| `/odom_local` | Odometry | input_bridge → controller_server | 修正后机器人中心位姿+速度 |
| `/goal_pose` | PoseStamped (map) | input_bridge（或外部直接发）→ net_navigator | 导航目标 |
| `/cancel_navigation` | Empty | 外部 → net_navigator | 取消导航 |
| `/gp_goal`、`/uav/target_gps` | NavSatFix | 外部 → input_bridge | GPS 目标（LLA） |
| `/navigate_track_gp_path` → `/navigate_track_path` | Path | 外部 → input_bridge → 外部 | GPS 路径转 ENU |
| `/topology_plan` | Path | net_planner_action_server →（可视化/RViz） | 全局规划结果 |
| `/global_path` | Path | net_navigator →（可视化） | 剩余待跟踪路径（含当前点） |
| `/nav_status` | Int32 | net_navigator → 外部 | 状态机状态码 |
| `/cmd_vel` | Twist | controller_server → car_driver | 速度指令 |
| `/traversability/ground_boundary_points` | PointCloud2 | traversability_obstacle_cloud → camera_boundary_navigation | 左/右/前U形地面边界 |
| `/camera_navigation/reference_path` | Path | camera_boundary_navigation → RViz | 每帧车辆局部坐标系下的边界偏移路径，仅用于调试 |
| `/camera_navigation/cmd_vel` | Twist | camera_boundary_navigation → 可选底盘重映射 | 默认隔离的无odom低速控制输出 |
| `/camera_navigation/status` | DiagnosticArray | camera_boundary_navigation → 监控 | TRACKING/障碍/超时/禁用等安全状态 |
| `/pause` | Bool | 外部 → controller_server | true=强制零速 |
| `/local_costmap/costmap` 等 | OccupancyGrid 等 | controller_server 内部 local_costmap | 代价地图（transient_local） |
| `received_global_plan` / `lookahead_point` / `lookahead_collision_arc` / `global_path_slice_freeze` / `merge_path_connection_pose` | Path / PointStamped / Path / Bool / PoseStamped | controller_server（RPP 插件） | 调试可视化 |
| `/log/event` | nav2_msgs/Log | 各节点 → log_server_node | 事件日志 |
| `/log/record` | std_msgs/String | 各节点 → log_server_node | CSV 数据行 |

Action：

| Action | 服务器 | 客户端 | 说明 |
|---|---|---|---|
| `/compute_path_to_pose` | net_planner_action_server | net_navigator（或命令行测试） | 路网规划 |
| `/follow_path` | controller_server | net_navigator | 路径跟踪（含切片） |
| `/navigate_to_pose` | net_navigator | 外部 | 高层导航入口（带 feedback） |

## 6. 日志系统

### 6.1 事件日志 navigation.log

格式：`[%Y-%m-%d %H:%M:%S.%e] [nav2] [级别] [节点名] 消息`。事件类型：input_bridge（gp_origin 设置/验证、GPS 目标/路径接收、数据超时）、net_navigator（状态切换、目标到达/取消、规划超时/失败、切片重规划）、net_planner（规划耗时 ms）、controller_server（pause 翻转）。

### 6.2 数据日志 recorder.csv（逗号分隔，行首时间戳）

| 行 | 发布者 | 字段 |
|---|---|---|
| `net,goal` | net_navigator | 时间, net, goal, exec_status, gx,gy,gz, gr,gpi,gyaw |
| `net,exec` | net_navigator | 时间, net, exec, exec_status, 机器人位姿 rxyz+rpy, 剩余距离, 子目标位姿 rxyz+rpy |
| `ctrl,cvc` | controller_server(RPP) | 时间, ctrl, cvc, exec_status, 位姿, carrot xyz(base_link 系), 当前 v/w, 指令 v/w, 方法(APC/RTG/RTP) |
| `gp_goal` / `gp_path` | input_bridge | 时间, gp_goal/path, lat, lon, alt |

`cvc_method`：`APC`=正常跟踪（曲率约束）/ `RTG`=原地转向目标朝向 / `RTP`=原地转向路径方向。详细示例见 `doc/日志记录.md`。

### 6.3 执行状态码（`nav2_common::ExecStatus`，`common/nav2_common/include/nav2_common/status.hpp`）

- 通用：`OK=0`（实际被 net_navigator 写入 `net,exec` 记录的仅有 0/101/103/105）
- net_navigator（100-149）：`NET_NO_ROBOT_POS=101`、`NET_FOLLOW_NOT_READY=103`、`NET_SLICE_TOO_SHORT=105`、`NET_PLANNING_TIMEOUT=106`、`NET_COMPUTE_FAIL=107`
- input_bridge（150-199）：`BRIDGE_NO_GPS_FIX=150`、`BRIDGE_ORIGIN_UNVERIFIED=151`（枚举已定义，**当前 input_bridge 代码未发布任何 exec 状态**）
- （waypoint_navigator 50-99 段供已移出的 waypoint 系统使用）

## 7. 配置文件

### 7.1 `src/robot_launch/config/nx_net_nav.yaml`（当前主配置，Nav2 风格多节点 YAML）

`/**/` 前缀为通配（任意命名空间）。要点：

- **controller_server**：`odom_topic: odom_local`、`cmd_vel_pause_topic: pause`、`controller_frequency: 10.0`
- **FollowPath（RPP）**：`lookahead_dist: 1.3`、`desired_linear_vel: 1.0`、`regulated_linear_scaling_min_radius: 3.0 / min_speed: 0.25`、`lethal_cost: 200`、`traversal_cost_weight: 3.0`、`min_approach_linear_velocity: 0.05`、`approach_velocity_scaling_dist: 1.0`、`use_collision_detection: true`、`max_allowed_time_to_collision_up_to_carrot: 1.0`、`use_rotate_to_heading: true`、`rotate_to_heading_angular_vel: 0.5`、`rotate_to_heading_min_angle: 0.5`、`max_angular_accel: 1.57`、`prefer_planner_polyline_when_clear: true`、`publish_global_path_slice_freeze: true`（§4.5 全表）
- **local_costmap**：rolling 10×10m、分辨率 0.05、footprint `[[0.3,0.25],[0.3,-0.25],[-0.3,-0.25],[-0.3,0.25]]`、插件 `obstacle_layer`+`inflation_layer`；obstacle：`topic: cur_lid`、`obstacle_max_range: 3.0`、高度 0.2~0.65m、`inf_is_valid: true`、`observation_persistence: 0.0`；inflation：`cost_scaling_factor: 5.0`、`inflation_radius: 1.0`。**DenoiseLayer 已编译但当前配置未启用**。
- **net_planner_action_server**：`max_snap_distance: 15.0`、`edge_snap_enabled: false`、`dense_path_step: 0.1`
- **net_navigator**：`goal_xy_tolerance: 0.3`、`follow_slice_horizon_m: 30.0`、`executing_freq_hz: 1.0`（注意：YAML 中 `enable_controller_slice_freeze`/`freeze_slice_topic` 为**遗留参数**，当前代码已不读取）
- **input_bridge**：`body_offset_x: -0.40 / body_offset_z: 0.50`、`reference_latitude: 30.8135718 / longitude: 120.8338169 / altitude: 12.318`（南湖测试场原点）、`crop_enabled: false`、`require_nav_sat_fix: true`（YAML 中 `target_detected_topic` 为遗留参数）
- **lifecycle_manager_navigation**：`node_names`、`autostart: true`

其他配置变体：`nx_net_nav_up.yaml`（无命名空间通配、odom=/odom、20Hz 的旧变体）、`net_nav2.yaml`（模拟场景，use_sim_time=true、含已废弃 freeze 参数）、`waypoint_nav2.yaml`/`nx_waypoint_nav.yaml`（waypoint 系统，配套包 `nav2_waypoint_system` 不在本仓库）、`slam.yaml`（旧 laser_slam_3d 参数）。

### 7.2 拓扑地图格式（`maps/topo_map.yaml`）

```yaml
map:          # 子地图段: id name
0 default
lla_mode:     # 0=平面坐标; 1=经纬高(需 origin: 段)
0
node:         # id mapId x y z lat lon alt(nan=无效)
0 0 0 12.5 0 nan nan nan
edge:         # id mapId nodeId1 nodeId2 边类型(0=双向,1=单向) cost
0 0 0 1 0 0
```

当前主用 `topo_map.yaml`（21 节点 / 26 边）；仓库另存多张场地地图：`topo_map1.yaml`、`topo_map_hx.yaml`、`topo_map_hx2.yaml`、`topo_map_hx3.yaml`（223 行，最大）、`topo_map_sll_2f.yaml`。支持 `#` 注释与 `origin:` 段。

## 8. 启动方式

### 8.1 launch 文件矩阵（`src/robot_launch/launch/`）

| launch 文件 | 场景 | 参数 | 说明 |
|---|---|---|---|
| `nx_net_nav_slam.launch.py` | **完整系统**：RoboSense + FAST-LIVO2 + 导航 + CAN 底盘 | `use_rslidar`, `use_rviz`, `livo_params_file`... | 主启动文件；SLAM 与雷达为外部包 |
| `nx_net_nav.launch.py` | MID360 + mavros + CAN 底盘 + 导航（无 SLAM） | `use_mid360`, `fcu_url`(默认 `/dev/ttyACM0:230400`), `use_rviz` | 定位由外部提供 `/odom`；static TF map→odom 恒等 |
| `3588_net_nav.launch.py` | **纯导航**（无雷达/SLAM/底盘），带 namespace 多机 | `namespace`(默认 `car1`)、`use_rviz` | 用于 3588 板；topic 命名空间化（`/car1/cur_lid`、`/car1/tf`），TF 全名 frame 共享 |
| `net_nav2.launch.py` | 仿真/桌面验证（static map→odom，无 bridge） | `use_sim_time`(默认 true) | 旧版精简 |
| `waypoint_nav2.launch.py` / `nx_waypoint_nav.launch.py` | waypoint 导航 | — | **引用本仓库不存在的包 `nav2_waypoint_system`，需配套仓库** |

### 8.2 scripts 脚本

| 脚本 | 作用 |
|---|---|
| `build.sh` | 全工作区编译：`colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"` |
| `launch_net.sh` | 校验依赖包后启动 `nx_net_nav.launch.py` |
| `launch_net_slam.sh` | 校验后启动 `nx_net_nav_slam.launch.py`（传 `map_data_path` 参数——现行 launch 文件已不声明，属遗留） |
| `launch_wn.sh` / `launch_car.sh` / `launch_loc.sh` / `launch_slam.sh` | waypoint / 纯底盘 / mavros / laser_slam_3d（旧版定位） |
| `3588_launch.sh` | 3588 板：启动 `3588_net_nav.launch.py`（namespace 参数，默认 car6，并 source 外部 `co-search` 工作区） |
| `start_mid360.sh` / `record_mid360.sh` | 启动 Livox MID360 / 录包（`/livox/lidar` `/livox/imu`） |
| `start_scout_mini.sh` | 启动 Scout Mini 底盘（scout_base 外部包） |
| `odom.sh` | 循环发布假定位 `/gps1/odom`（测试用） |
| `pub_gp_path.sh` | 发布 GPS 航点路径 `/navigate_track_gp_path`（南湖示例坐标） |
| `cancel.sh` | 发布 `/cancel_navigation` 取消导航 |

### 8.3 典型启动命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 完整系统（雷达+SLAM+导航+底盘，需外部包存在）
ros2 launch robot_launch nx_net_nav_slam.launch.py use_rviz:=false

# 纯导航离线验证（3588 板）
ros2 launch robot_launch 3588_net_nav.launch.py use_rviz:=false namespace:=car1

# 便捷脚本
bash scripts/launch_net.sh
bash scripts/3588_launch.sh car1
```

### 8.4 离线测试（无雷达无 SLAM）

```bash
# 1. 提供假定位：static TF map→base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

# 2. 发高层导航目标（navigate_to_pose action）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 3. 或直接测规划器
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, use_start: true, \
    start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 4. 观察
ros2 topic echo /topology_plan --once
ros2 topic echo /global_path
```

### 8.5 调试工具

- `python3 tools/monitor_nav.py [namespace]`：按模块（controller/net_planner/net_navigator/input_bridge/log/lifecycle）统计 CPU/内存，实时写文件并输出均值/峰值。
- 日志文件：运行时在**工作目录**生成 `./logs/navigation.log` 与 `./logs/recorder.csv`（仓库内 `scripts/logs/` 有历史样本）。

## 9. 构建

### 9.1 基本构建

```bash
cd <repo root>
source /opt/ros/humble/setup.bash
bash scripts/build.sh            # = colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

共 16 个 CMake 包 + 1 个纯 Python 包（robot_launch）。依赖清单（apt 与 ros-humble 包）见 `doc/build-dependencies.md`。

### 9.2 关键坑：fmt 版本（aarch64/Jetson 平台）

系统 `libfmt-dev` 为 v9，但 `libspdlog.so`（Nav2 日志链路）依赖 `libfmt.so.8`。解决：仓库内置 `3party/fmt/8.1.1/`（v8 头文件 + aarch64 so），`nav2_log_record` 与 `nav2_a_star_rpp` 的 CMakeLists 提供 `USE_SYSTEM_FMT` 开关（默认 OFF，走本地 v8 并设置 rpath）。详见 `doc/project-setup-record.md` §1。

### 9.3 已知构建/运行问题（摘录，详见 project-setup-record.md §4）

- `ros2 node list` 只显示 4 个节点：生命周期节点 DDS 发现不可见，实际运行正常。
- 无显示器环境 RViz 会 SIGSEGV → `use_rviz:=false`。
- 编译时跳过外部驱动包：`--packages-skip livox_ros_driver2 ros2can ugv_sdk scout_base scout_description scout_msgs rslidar_sdk rslidar_msg`（本仓库已不含这些包，仅在外围工作区场景需要）。

## 10. 常见修改导航（AI 代理速查）

| 想改什么 | 去哪里 |
|---|---|
| 线速度/预瞄/转弯/避障激进程度 | `src/robot_launch/config/nx_net_nav.yaml` → `FollowPath` 段 |
| 代价地图（footprint/障碍高度/膨胀） | 同上 → `local_costmap` 段 |
| 路网吸附距离/边投影开关/密化步长 | 同上 → `net_planner_action_server` 段（`topo_map_path` 由 launch 传入） |
| 拓扑地图（增删节点/边） | `src/robot_launch/maps/topo_map*.yaml`（node/edge 文本格式） |
| 图搜索算法 | `src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp` → `planNodeIds()`（当前 SNN；`astarExtended()` 可切换回 A*）；SNN 动力学在 `snn_topo_planner.hpp` `SnnDynamicsParameters` |
| 导航状态机行为（切片视野、节流频率、到达容差） | `src/navigation2/system/nav2_net_system/src/net_navigator.cpp` + config `net_navigator` 段 |
| Theta\* 局部避障逻辑 | `src/navigation2/controller/nav2_a_star_rpp/src/regulated_pure_pursuit_controller.cpp` `getLocalPlan()`（1099-1284 行）+ `theta_star.cpp` |
| odom 修正/body_offset/速度估算 | `src/navigation2/bridge/nav2_input_bridge/src/input_bridge_node.cpp` `onOdom()` + config |
| GPS 参考原点/目标处理 | config `nav2_input_bridge` 段 + `processGoalFix()`/`onGpPathFix()` |
| 日志格式/存储位置 | `src/navigation2/system/nav2_log_record/src/log_server_node.cpp` |
| 新消息/动作 | `src/navigation2/common/nav2_msgs/`（已有自定义 Route 消息族可复用） |
| 启动节点组合 | `src/robot_launch/launch/*.launch.py` |
| 记录数据字段（CSV） | 各节点 `publishRecord()` 调用点（net_navigator.cpp:286/382/646、RPP cpp:452-473、input_bridge.cpp:375/426） |

## 11. 重要注意与遗留问题

1. **仓库边界**：`src/driver/`、`src/localization/`（雷达驱动与 SLAM）已从仓库移除（见 git 状态 `D src/driver/...`），仅保留导航代码。`nx_net_nav_slam.launch.py` 引用的 `rslidar_sdk`、`fast_livo`、`ros2can`、`mavros` 及脚本引用的 `laser_slam_3d`、`livox_ros_driver2`、`scout_base` 等均为外部包，须在其他工作区安装并同时 source。
2. **旧文档差异**：`doc/nav_system.md` 描述的是上一代架构（`src/driver`、`src/localization` 尚在时），部分内容已过时——如 body_offset 已从 (0.20, 0, 0.30) 改为 (-0.40, 0, 0.50)、规划算法已从"A*"切换为 SNN 波前规划、`desired_linear_vel` 已从 0.5 改为 1.0。以本文档与代码为准。
3. **遗留参数**：`nx_net_nav.yaml` 中 `net_navigator.enable_controller_slice_freeze / freeze_slice_topic`、`nav2_input_bridge.target_detected_topic` 当前代码不读取；`launch_net_slam.sh` 传的 `map_data_path` 参数现行 launch 未声明。
4. **launch 引用不存在的包**：`waypoint_nav2.launch.py` / `nx_waypoint_nav.launch.py` 依赖 `nav2_waypoint_system`，本仓库无此包。
5. **MID360 场景话题不匹配**：`nx_net_nav.launch.py` 启动 MID360 驱动（发 `/livox/lidar`），但主配置 `nx_net_nav.yaml` 中 input_bridge 的 `lidar_pcl_topic: rslidar_points` —— 该场景下点云链不会连通（RoboSense 场景正常）。
6. **DenoiseLayer** 已编译但未在 `nx_net_nav.yaml` 启用（plugins 只有 obstacle+inflation）；启用需在 plugins 列表加 `denoise_layer`。
7. **fmt 库**：`nav2_log_record`/`nav2_a_star_rpp` 的 CMake 默认链接本地 `3party/fmt/8.1.1`（aarch64 专属路径），x86_64 机器构建可能需 `-DUSE_SYSTEM_FMT=ON`。
8. `prefer_planner_polyline_when_clear` 代码默认 false，但当前 config 已置 true（折线畅通时不跑 Theta\*）。
9. `nav2_msgs` 中自定义的 Route 消息族 / `SetRouteGraph` / ComputeRoute 等 action 目前**没有节点使用**（疑似为下一阶段功能预留）。

## 12. 相关文档索引

| 文档 | 内容 | 状态 |
|---|---|---|
| `doc/PROJECT_GUIDE.md`（本文件） | 项目总览、架构、节点、配置、启动、速查 | **最新，以此为准** |
| `doc/CAMERA_ONLY_NAVIGATION_PRINCIPLE_REPORT.md` | 无LiDAR、无定位、无odom条件下的纯相机导航原理、算法、安全机制与局限 | 有效（对应当前实现） |
| `doc/日志记录.md` | navigation.log / recorder.csv 字段级格式与示例 | 有效（对应现行代码） |
| `doc/nav_system.md` | 上一代系统说明（含驱动/SLAM） | **部分过时**（见 §11.2） |
| `doc/project-setup-record.md` | 编译记录、3588 部署、已知问题 | 有效（历史记录） |
| `doc/build-dependencies.md` | 依赖安装清单 | 有效（含已移出包的依赖，按需裁剪） |
