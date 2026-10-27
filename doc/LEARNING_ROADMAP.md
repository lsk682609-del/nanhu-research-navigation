# 南湖路网导航 · 完整学习路线图

> 目标：不只是"会发目标点"，而是从零**独立复刻**出这个工作区的能力——能自己搭 Gazebo 世界、建路网、配导航栈、调通并解释每一步。
> 本文所有命令都在 `/home/lxh/nanhu-research-navigation` 下验证过。
> 配套阅读：[PROJECT_GUIDE.md](PROJECT_GUIDE.md)（总览）、[GAZEBO_NET_NAV.md](GAZEBO_NET_NAV.md)（仿真入口）、[NAVIGATION_EXTERNAL_INTERFACE.md](NAVIGATION_EXTERNAL_INTERFACE.md)（对外接口）。

---

## 0. 全局地图：先把"数据流"刻在脑子里

```
Gazebo 世界(test_world.world)
        │  车身/传感器模型、物理、坡道桥梁
        ▼
┌───────────── Gazebo 插件 ─────────────┐
│ 差速底盘插件 → /odom(轮速)             │
│ Livox mid360 插件 → /livox/lidar/pointcloud │
│ RealSense 插件 → 深度/彩色图           │
└───────────────────────────────────────┘
        │
        ▼
nav2_input_bridge   ── 里程计 + 世界原点 ──→ TF: map→base_link，/odom
        │
        ▼
nav2_costmap_2d（local costmap）
   ├─ slope_traversability_layer  点云→坡度/台阶/残差→致命格点
   └─ inflation_layer             致命格点→膨胀(内切半径内=253)
        │
        ▼
nav2_net_planner（全局规划器：在拓扑图上求解）
   读 topo_map_gazebo.yaml（节点+边）
   目标吸附到最近节点 → 求解 → 节点序列 → 米制路径
   └─ 两套全局算法同在一处：SNN 脉冲波前（当前启用） / A*（保留的对照实现）
        │
        ▼
nav2_a_star_rpp（控制器 = 纯追踪 RPP + 本地重规划）
   全局路径判障 → 通过则直跟；不通过则重规划＋汇入点/切片/尾段拼接
   └─ 局部算法可选：theta_star（默认） / scan（A* + B样条，见 nav2_scan_planner）
        │
        ▼
/car1/cmd_vel  →  差速底盘插件  →  Gazebo 里的车动起来
        │
        ▼
nav2_net_system（net_navigator 状态机）
   上报 nav_status，收 /goal_pose、/cancel_navigation、/pause
```

**一句话记忆**：世界 → 传感器 → 桥接/定位 → 代价地图 → 路网规划 → 控制器 → 底盘。
每一层都能单独研究，也能单独替换。

---

## 阶段 0 · 环境、构建与"调试基本功"

### 0.1 两个工作区（最容易踩的坑）

| 工作区 | 路径 | 负责 |
|---|---|---|
| 主工作区 `nav` | 仓库根目录 `/home/lxh/nanhu-research-navigation`（含顶层 `src/`、`install/`） | 导航栈（`src/navigation2/*`）、`robot_launch`、相机导航等 |
| 独立工作区 `gazebo_sim` | `src/gazebo_sim`（含 `src/`、自己的 `install/`） | 车辆模型、传感器、世界、Gazebo 插件 |

两者**各自独立构建、各自 source**。这就是为什么启动仿真和启动导航栈要 source 两组 setup。

### 0.2 构建

```bash
# 主工作区（仓库根目录）
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # 或直接用仓库脚本
./scripts/build.sh

# 仿真工作区
cd src/gazebo_sim
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`--symlink-install` 很关键：改了 yaml/launch/地图文件，**不用重新 build**，`install/` 里是指向 `src/` 的软链，直接生效。（本轮修 `topo_map_gazebo.yaml` 就是靠这个即时生效。）

依赖问题见 [build-dependencies.md](build-dependencies.md) 与 [project-setup-record.md](project-setup-record.md)（含 `3party/fmt` 版本不匹配的处理）。

### 0.3 环境变量规则

```bash
export ROS_DOMAIN_ID=42      # 两个终端必须一致，否则互相看不见
```
每个新终端都要按顺序 source：
```bash
source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source /home/lxh/nanhu-research-navigation/install/setup.bash
```
Gazebo 找不到模型/资源时，补一句 `source /usr/share/gazebo/setup.sh`。

### 0.4 调试基本功（这一节最重要，务必练熟）

```bash
# 系统级
ros2 node list                       # 有哪些节点在跑
ros2 node info /car1/net_planner_action_server
ros2 topic list                      # 有哪些话题
ros2 topic info /goal_pose -v        # 谁发谁收（排查"发了没人理"）
ros2 topic hz /livox/lidar/pointcloud
ros2 topic echo /car1/nav_status --once
ros2 param list /car1.net_planner_action_server
ros2 param get /car1.local_costmap/local_costmap plugins
ros2 service list
ros2 run tf2_ros tf2_echo base_link livox_frame     # 看外参
ros2 run tf2_tools view_frames                      # 导出 TF 树 PDF
rqt_graph                                           # 图形化节点/话题连接
```

**日志三个地方**（本轮排查靠的就是它）：
1. 启动终端的屏幕输出；
2. `~/.ros/log/` 下每个节点一个文件：`ls -t ~/.ros/log/*.log | head`——例如 `controller_server_*.log`；
3. 项目自己的记录：`src/logs/navigation.log`（规划/状态机）与 `snn_runs/`（每次运行快照）。

**练习 0**：起仿真后，用 `ros2 node list` + `rqt_graph` 画出完整节点拓扑；用 `ros2 topic info -v` 找出 `/goal_pose` 的发布者与订阅者各是谁。

---

## 阶段 1 · Gazebo：会用，并且会造世界

### 1.1 三个场景

```bash
# 主场景（默认 test_world.world）
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
ros2 launch vehicle_gazebo_sim simulation.launch.py

# 回字形双墙 10×10m
ros2 launch vehicle_gazebo_sim hui_path.launch.py

# 30×30m 桥梁避障场景（配套路网与导航入口不同）
ros2 launch vehicle_gazebo_sim square_bridges_30m.launch.py
ros2 launch robot_launch gazebo_square_bridges_nav.launch.py
```
无界面（跑批量/远程）：`gui:=false rviz:=false`。

### 1.2 Gazebo 命令行（不靠 GUI 也能查）

```bash
gz topic -l                          # 世界里的 gazebo topic
gz model -l                          # 列出世界里的模型
gz model -m <name> -i                # 模型位姿
gz world -p                          # 暂停
gz physics                           # 物理参数
```
值得记的一条：Gazebo 世界原点 = 导航 `map` 原点，**yaw=0 朝东、yaw=π/2 朝北**，所以"东=+x、北=+y"。

### 1.3 要学会读/改的东西

- 世界文件：[test_world.world](../src/gazebo_sim/vehicle_gazebo_sim/worlds/test_world.world)——墙（`Wall_*`）、圆柱、木箱、**15° 上下坡**、以及各模型的 `<pose>`。
- 车模型与插件：[vehicle_gazebo_sim](../src/gazebo_sim/vehicle_gazebo_sim)——URDF/SDF、差速底盘插件、`tricycle`/`scout_mini` 底盘差异。
- 传感器仿真：[livox_laser_simulation_RO2](../src/gazebo_sim/livox_laser_simulation_RO2)（Livox Mid-360 扫描模式由 CSV 定义）、[realsense_gazebo_plugin](../src/gazebo_sim/realsense_gazebo_plugin)。
- 注意差速插件 **`publish_odom_tf=false`**：`map→base_link` 只由 `input_bridge` 发布，绝不能让两个节点抢同一条动态 TF。

**练习 1**：把车的出生点 `<pose>` 改到 (10,0)，重启仿真确认；把某个 `Wall_*` 换个位置，观察代价地图变化。
**练习 2**：给世界加一个 0.3 m 高的箱子，重启后看它是怎么进 costmap 的。

---

## 阶段 2 · RViz：可视化与交互

### 2.1 打开方式

- 仿真 launch 带 `rviz:=true`；或导航入口默认会起一个（[gazebo_net_nav.launch.py](../src/robot_launch/launch/gazebo_net_nav.launch.py)）。
- 配置在 [robot_launch/rviz](../src/robot_launch/rviz)，参数文件里也有 `nav2.rviz`。

### 2.2 RViz 必设项

- **Fixed Frame = `map`**（否则一切错位）。
- 必加 Display：
  - `TF`：看整棵坐标树；
  - `LaserScan`/`PointCloud2` → `/car1/cur_lid`（喂给规划器的裁剪点云）；
  - `Map` → `/car1/local_costmap/costmap`（**代价地图颜色**：RViz 收到的是已翻译的占据值——0 自由、1..98 膨胀梯度、99 内切(原253)、100 致命(原254)、-1 未知。`Color Scheme: costmap` 下 **青=99(内切)、品红=100(致命)**，低代价蓝→红渐变；`Color Scheme: map` 下 99/100 都接近黑色，分不出来）；
  - `Path` → 规划输出（拓扑路径与全局路径，名字用 `ros2 topic list | grep -i path` 现查）；
  - `PoseStamped` → `/goal_pose` 显示目标。
- 交互工具 **2D Goal Pose**：在 `map` 平面上点击 → 发布 `/goal_pose`（`PoseStamped`，frame=map）。**这就是"发布目标点"的本质**，不是 RViz 特殊功能。

**练习 3**：只留 `TF + costmap + 规划路径 + goal` 四个显示，把车开到障碍附近，观察代价地图上的红色区域和规划路径的关系。
**练习 4**：用 RViz 的 `Publish Point`（点击取坐标）去核对路网节点坐标，确认"目标必须落在节点 4 m 内"这件事。

---

## 阶段 3 · 定位与 TF（input_bridge）

阅读：[nav2_input_bridge](../src/navigation2/bridge/nav2_input_bridge)，配置 [gazebo_net_nav.yaml](../src/robot_launch/config/gazebo_net_nav.yaml)。

核心概念：
- 里程计（轮速）在仿真里是"真值"，但真机有漂移——理解为什么要有一层 bridge 把里程计和全局原点接起来。
- 经纬度 ↔ 米：拓扑地图的 `lla_mode` / `origin` 决定是直接读 x/y 还是由 lat/lon 换算。
- `map→base_link` 应**只有一条**发布链。

**练习 5**：`tf2_echo base_link livox_frame` 核对雷达外参（文档期望平移约 `(0.24, 0, 0.30)`）；`view_frames` 导出整棵树并画出。

---

## 阶段 4 · 拓扑路网 + 全局规划（A\* / SNN 双算法，本项目最有特色的部分）

阅读：[nav2_net_planner](../src/navigation2/planner/nav2_net_planner)、地图 [topo_map_gazebo.yaml](../src/robot_launch/maps/topo_map_gazebo.yaml)、生成脚本 [generate_gazebo_topo_map.py](../scripts/generate_gazebo_topo_map.py)。

### 4.1 地图格式（纯文本）

```
map:
lla_mode: 1            # 1=用经纬高换算；x/y 才是最终米制坐标
origin: <lat> <lon> <alt>
node: id type mapId  x(东) y(北) z(上)  lat lon alt
edge: id type mapId  from to obt
```
查看节点的命令（也见 [GAZEBO_NET_NAV.md](GAZEBO_NET_NAV.md)）：
```bash
python3 - <<'EOF'
sec=None
for line in open('/home/lxh/nanhu-research-navigation/src/robot_launch/maps/topo_map_gazebo.yaml'):
    line=line.split('#')[0].strip()
    if not line: continue
    if line.rstrip(':') in ('map','lla_mode','origin','node','edge'):
        sec=line.rstrip(':'); continue
    p=line.split()
    if sec=='node': print(f'节点 {p[0]:>4}  ENU=({p[3]}, {p[4]}, {p[5]})')
    elif sec=='edge': print(f'边   {p[0]:>4}  {p[3]} -> {p[4]}')
EOF
```
运行时确认实际加载了哪个文件：`ros2 param get /car1/net_planner_action_server topo_map_path`，并看该节点启动日志 `Loaded topology ... (N nodes)`。

### 4.2 吸附与规划

- 目标必须吸附到路网节点：`max_snap_distance`（当前 4.0 m）、`edge_snap_enabled`（当前 false）。超距 → 规划失败 → `nav_status=4`。
- 规划输出是"起点节点→…→终点节点"的序列，再展开成米制路径；首尾再补上机器人实际位置/用户目标点。
- 日志解读（[navigation.log](../src/logs/navigation.log)）：
```
[SNN] plan request: start=(...) goal=(...) max_snap_dist=4.00 edge_snap=false
[SNN] plan result: status=Success planned_nodes=11 total_delay=50.8
      start_spike_time=50.8 fired_neurons=18 synaptic_events=35 elapsed=0.02ms
```
`planned_nodes` = 经过的节点数；`total_delay`/`start_spike_time`/`fired_neurons`/`synaptic_events` 是脉冲网络求解的痕迹，去 `topo_map.hpp` / 规划器源码看它们怎么算。

- 地图生成：脚本从 world 自动提取可通行骨架 → 输出 node/edge，并做**净距自检**（往返一致性 3.81e-08 m、最小净距 0.936 m）。带 roll/pitch 的几何被当作可通行坡道跳过（本轮刚修好这个校验）。

**练习 6**：把 `max_snap_distance` 改到 15 试一次，体会"能点任意位置"的代价（末端直线段变长、易穿墙）。
**练习 7**：手工在 yaml 里加一个节点和两条边，重启 `net_planner`，观察规划路径是否走了新边。
**练习 8**：跑一次生成脚本（带正确的 `--world`），对比输出的节点表与你手改后的差异。

### 4.3 两套全局规划算法：A\* 与 SNN（重点）

两套算法**都在 `nav2_net_planner` 里**，共用同一张拓扑图、同一套吸附逻辑，只是求解内核不同：

| | A\* | SNN |
|---|---|---|
| 入口 | `TopoMap::astar()` / `astarExtended()`（[topo_map.hpp#L759](../src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp#L759-L762)） | `SnnTopoPlanner`（[snn_topo_planner.hpp](../src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/snn_topo_planner.hpp)） |
| 原理 | 图上经典 A\*：启发式 + 边的 cost | 脉冲神经网络（LIF 神经元）波前传播求解 |
| 当前状态 | 代码保留，**已无任何调用**（可作对照/基准） | **当前生产路径** |
| 日志痕迹 | 无专项统计 | `planned_nodes / fired_neurons / synaptic_events / total_delay / start_spike_time` |

关键证据在 `planNodeIds()`（[topo_map.hpp#L811-L813](../src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp#L811-L820)）：

```cpp
// SNN 波前规划器替换 A*
SnnTopoPlanner snn;
auto result = snn.plan(*this, start_id, goal_id);
```

即 **A\* 是原实现，SNN 是后来替换上去的**。A\* 代码仍在（`astar()` / `astarExtended()`），但没有任何地方再调用它，也**没有运行时可切换的参数**。

一个重要细节：`astarExtended(startId, goalId, extra_nodes, extra_adj)` 的参数与 SNN 路径**完全对齐**（同样吃吸附产生的虚拟起终点节点），所以换回 A\* 不会丢掉吸附能力——这是两者可以互为对照的前提。

**三种"切换"方式**（由易到难）：
1. **对照阅读**（建议先做）：同一对起终点分别用 A\* 与 SNN 求路径，人工比对节点序列与总长；
2. **退回 A\***：把 `planNodeIds` 里 `snn.plan(...)` 换成 `astar(start_id, goal_id)`，重新 build；
3. **做成运行时切换**（正解）：加参数 `planner_algorithm: "astar" | "snn"`，在 `planNodeIds` 里按参数分发；注意 `SnnTopoPlanner` 的 `setTopology`/`clearTopology`/`hasTopology` 生命周期。

**练习 8A**：读 `astarExtended` 与 [snn_topo_planner.cpp](../src/navigation2/planner/nav2_net_planner/src/snn_topo_planner.cpp)，用文字说清 A\* 的启发式取什么、SNN 的"延迟/波前"对应 A\* 的哪个量。
**练习 8B**：临时改成走 A\*，重新 build 跑一次，比对 `planned_nodes` 与路径长度是否与 SNN 一致。
**练习 8C**：实现 `planner_algorithm` 参数，做成运行时切换（先想清楚切换时 SNN 拓扑缓存怎么处理）。
**练习 8D**：写脚本对同一对起终点各跑 N 次，自动比对两种算法的节点序列与总长度。

---

## 阶段 5 · 代价地图与"可通行性"判断

阅读：[nav2_costmap_2d](../src/navigation2/map/nav2_costmap_2d)，插件 [slope_traversability_layer.cpp](../src/navigation2/map/nav2_costmap_2d/plugins/slope_traversability_layer.cpp)，配置 [gazebo_net_nav.yaml](../src/robot_launch/config/gazebo_net_nav.yaml) 的 `local_costmap` 段。

要点：
- 本仿真局部代价地图**只有两层**：`slope_traversability_layer` + `inflation_layer`（没有 obstacle_layer）。
- 该插件把点云按 0.05 m 格点统计 `low/high`，判据大致是：坡度过大 / 邻域平面拟合残差过大 + 高度跳变 / 稀疏邻域跳变 → 致命。参数：`max_slope_deg`、`max_step_height`、`max_ground_residual`、`obstacle_height`、`max_range`。
- 膨胀层：致命格点周围 `inscribed_radius` 内为 253，控制器判障阈值正是 `cost >= INSCRIBED_INFLATED_OBSTACLE(253)`。

**真实案例（务必亲手复现）**：设计上可通行的 15° 坡道，其**薄板边沿（约 0.019 m 翘起）被误判为致命**，在坡道两侧形成一条 253 带。车压在中线 `y≈13` 能过；被绕障逻辑带到南缘 `y≈11.97` 就"位姿碰撞"冻住。复现步骤：把车开到 `(13.5, 11.95)`，RViz 看 `/car1/local_costmap/costmap` 是否一片 253。

**练习 9**：把 `max_step_height` 0.10→0.15、`max_ground_residual` 0.06→0.12，重启后看坡道处的判级是否改变；再试"把薄板下缘压到与地面齐平"。

---

## 阶段 6 · 控制器：纯追踪 + Theta* 绕障

阅读：[nav2_a_star_rpp](../src/navigation2/controller/nav2_a_star_rpp)、参数 [theta_rpp.yaml](../src/robot_launch/config/theta_rpp.yaml)、[scan_rpp.yaml](../src/robot_launch/config/scan_rpp.yaml)，外加 [nav2_scan_planner](../src/navigation2/planner/nav2_scan_planner)。

要点：
- RPP 主体：纯追踪 + 曲率限速（`RegulatedPurePursuit`）；
- **魔改部分**：`path_obj_check` 用局部代价地图判全局路径是否受阻 → 受阻则 `run_theta_star_merge_tail` 用 Theta* 绕障并拼接全局尾段（有"汇入点/切片/尾段"三个概念，见控制器源码中 `触发重规划`/`重规划完成`/`跟踪进度` 相关日志）；
- 危险点：绕障后**汇入点落在车身之后**时会进入"原地对准路径"，而进度检查器只统计**平移**（如 `movement_time_allowance=10s`、`required_movement_radius=0.1m`）→ 必然 `Failed to make progress` → abort。这就是"发远点失败"的直接原因。
- 另外 `allow_reversing=false` 让车不能倒车脱困。
- **局部算法也是可切换的**：控制器参数 `local_planner_type`（[regulated_pure_pursuit_controller.cpp#L450-L462](../src/navigation2/controller/nav2_a_star_rpp/src/regulated_pure_pursuit_controller.cpp#L450-L462)）支持两套后端：
  - `theta_star`（默认，用包内 `theta_star.cpp`，即上面说的 Theta\* 绕障）；
  - `scan`：走 [nav2_scan_planner](../src/navigation2/planner/nav2_scan_planner)，核心是 `src/core/dyn_a_star.cpp`（**A\***）＋ `src/core/uniform_bspline.cpp`（均匀 B 样条优化）。
  两套现成 profile：[theta_rpp.yaml](../src/robot_launch/config/theta_rpp.yaml)（`local_planner_type: "theta_star"`）与 [scan_rpp.yaml](../src/robot_launch/config/scan_rpp.yaml)（`local_planner_type: "scan"`）。注意只允许这两个值，取其它值启动时会抛 `local_planner_type must be theta_star or scan`。

**练习 10A**：在 [gazebo_net_nav.yaml](../src/robot_launch/config/gazebo_net_nav.yaml) 的 controller 段显式加上 `local_planner_type: "scan"` 并重启，对比同一目标下两种局部算法的路径与速度曲线。
**练习 10B**：读 `dyn_a_star.cpp`，说清它与全局 A\*（`astarExtended`）在"搜索空间 / 启发式 / 代价"三方面的差别。
**练习 10**：把 `progress_checker` 的 `movement_time_allowance` 调大，看 abort 是否延后；把 `follow_slice_horizon_m` 改小，看是否减少误判障。
**练习 11**：给绕障分支加一道保护——汇入点在车身后时禁止原地对准（读源码后动手改，重新 build）。

---

## 阶段 7 · 状态机与对外接口

阅读：[nav2_net_system](../src/navigation2/system/nav2_net_system)、[NAVIGATION_EXTERNAL_INTERFACE.md](NAVIGATION_EXTERNAL_INTERFACE.md)、[nav_system.md](nav_system.md)。

要点：
- `net_navigator` 状态机：`IDLE → PLANNING → EXECUTING → SUCCEEDED / PLAN_FAILED / EXEC_FAILED`，取消/暂停另走分支。
- 状态查询：`ros2 topic echo /car1/nav_status --once`（0 空闲、1 规划、2 执行、3 成功、4 规划失败、5 执行失败）。
- 控制：`/car1/cancel_navigation`（Empty）、`/car1/pause`（Bool）。
- 生命周期管理由 `nav2_lifecycle_manager` 统一激活；日志记录由 [nav2_log_record](../src/navigation2/system/nav2_log_record) 落盘，产物在 [snn_runs](../snn_runs)。

**练习 12**：用一个完整任务走完 `IDLE→PLANNING→EXECUTING→SUCCEEDED`，同时 `ros2 topic echo /car1/nav_status` 全程记录；再用 `cancel_navigation` 中断一次。
**练习 13**：跑 `python3 tools/monitor_nav.py` 看现成的监控脚本怎么写的。

---

## 阶段 8 · 多场景 / 多平台参数化

阅读：[robot_launch/launch](../src/robot_launch/launch) 全部 9 个 launch 与 [config](../src/robot_launch/config) 全部 yaml。

- 仿真：`gazebo_net_nav`、`gazebo_net_nav_raw_lidar`（纯雷达变体）、`gazebo_square_bridges_nav`；
- 真机：`net_nav2`、`nx_net_nav`、`nx_net_nav_slam`、`3588_net_nav`、航点 `waypoint_nav2`/`nx_waypoint_nav`。
- 入口参数：`namespace`（默认 car1）、`params_file`、`topo_map_path`、`goal_topic`、`cmd_vel_topic`——学会用 `ns:=car2` 起第二台车。

**练习 14**：用 `namespace:=car2` 起第二套栈，确认话题都在 `/car2/...` 下，且两车 TF 前缀正确。
**练习 15**：跑 `gazebo_net_nav_raw_lidar.launch.py`，对比它与默认入口在感知/代价地图上的差异。

---

## 阶段 9 · 工具链与工程化

- **路网编辑器** [map_edit](../map_edit)（Qt）：
```bash
cd /home/lxh/nanhu-research-navigation/map_edit
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/map_edit        # 或 python3 run.py
```
  可画底图、编辑节点/边，导出 yaml。文档在 `map_edit/docs`。
- 脚本 [scripts](../scripts)：`build.sh`、一键启动、地图生成、`pub_gp_path.sh`（发全局路径）等。
- 生成脚本的**自检思想**：从世界几何反推路网，再做净距与往返一致性校验——这是"能自己造这套东西"的关键工程习惯。
- `--symlink-install` 的取舍、日志目录规范、`snn_runs/` 快照流程。

**练习 16**：用 `map_edit` 打开 `topo_map_gazebo.yaml`，加一个节点/边并保存，重启规划器验证生效。
**练习 17**：读 `scripts/build.sh` 与 `scripts/*.sh`，说清每个脚本在整条链路里的位置。

---

## 阶段 10（选修）· 纯视觉/相机导航线

阅读：[camera_boundary_navigation](../src/camera_boundary_navigation)、[plan_env](../src/plan_env)、[traversability_obstacle_cloud](../src/traversability_obstacle_cloud)，文档 [CAMERA_NAVIGATION_LEARNING_GUIDE.md](CAMERA_NAVIGATION_LEARNING_GUIDE.md)、[CAMERA_ONLY_NAVIGATION_PRINCIPLE_REPORT.md](CAMERA_ONLY_NAVIGATION_PRINCIPLE_REPORT.md)、[PURE_VISUAL_NAVIGATION.md](PURE_VISUAL_NAVIGATION.md)。

- 三条线：相机边界跟随（`camera_boundary_navigation`）、纯视觉局部导航、深度网格图感知（`plan_env`）。
- 这套系统不用激光/轮速/SLAM/全局地图，只在"重力对齐的局部坐标系"里生成前进曲线。与雷达路网导航是**两套独立方案**，可对照学习。

**练习 18**：跑 `camera_only_navigation.launch.py` / `gazebo_camera_navigation.launch.py`，记录它输出的话题与雷达方案有何不同。

---

## 毕业项目（Capstone）：从零复刻一套新场景

按顺序做，每步都有验收标准：

| 里程碑 | 做什么 | 验收 |
|---|---|---|
| 1 | 自己写一个 30×30 m 的新 world，含墙、箱、一段 10° 坡 | Gazebo 能起、车能跑 |
| 2 | 给车配好传感器插件与 `map→base_link` 桥接 | `tf2_echo` 正确、`/odom` 有数据 |
| 3 | 用自己的 world 生成拓扑路网（跑生成脚本 + 手改） | 净距自检通过、节点表合理 |
| 4 | 写自己的 `params_file` 与 launch | `ros2 param get topo_map_path` 指向你的地图 |
| 5 | 调通端到端：RViz 发目标 → 车到点 | `nav_status` 走完 0→1→2→3 |
| 6 | 故意造一个故障（障碍物遮住节点 / 坡道被误判），定位到具体层 | 能用日志+RViz 说清是哪一层的问题 |
| 7 | 写一份你自己的场景说明文档 | 别人照你的文档能复现 |

做完 7 步，你就具备"自己做出这个文件夹内容"的能力。

---

## 附录 A · 命令速查

```bash
# ---- 启动仿真 ----
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
ros2 launch vehicle_gazebo_sim simulation.launch.py

# ---- 启动导航栈（另开终端）----
source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source /home/lxh/nanhu-research-navigation/install/setup.bash
ros2 launch robot_launch gazebo_net_nav.launch.py

# ---- 发目标（map 系米制）----
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 27.0, y: 13.0, z: 0.0}, orientation: {w: 1.0}}}'

# ---- 状态 / 控制 ----
ros2 topic echo /car1/nav_status --once
ros2 topic pub --once /car1/cancel_navigation std_msgs/msg/Empty '{}'
ros2 topic pub --once /car1/pause std_msgs/msg/Bool '{data: true}'

# ---- 排查 ----
ros2 topic info /goal_pose -v
ros2 param get /car1/net_planner_action_server topo_map_path
ls -t ~/.ros/log/*.log | head
tail -40 src/logs/navigation.log
rqt_graph
```

## 附录 B · 常见故障速查

| 现象 | 先查 | 常见原因 |
|---|---|---|
| 发目标没反应 | `/goal_pose` 有没有订阅者、`nav_status` | 导航栈没起 / 状态机没激活 |
| `nav_status=4` 规划失败 | 目标到最近路网节点距离 | > `max_snap_distance`(4.0) |
| 路径很绕 | `planned_nodes`、路网节点密度 | 路网太稀疏 |
| 中间停住不动 | `~/.ros/log/controller_server_*.log` | `Failed to make progress`（原地掉头/被障碍冻住） |
| 车"以为自己在障碍里" | RViz 代价地图、`RPP当前位姿碰撞` | 可通行性层误判地形（见阶段 5） |
| 两个终端互相看不见 | `ROS_DOMAIN_ID` | 不一致 |
| 改了配置没生效 | 是否 `--symlink-install`、节点是否重启 | 地图/参数在节点启动时加载 |

## 附录 C · 文档阅读顺序建议

1. [PROJECT_GUIDE.md](PROJECT_GUIDE.md) → 总览与节点职责
2. [GAZEBO_NET_NAV.md](GAZEBO_NET_NAV.md) → 仿真入口怎么跑
3. [nav_system.md](nav_system.md) → 系统结构与算法
4. [NAVIGATION_EXTERNAL_INTERFACE.md](NAVIGATION_EXTERNAL_INTERFACE.md) → 对外话题/服务
5. [GAZEBO_SQUARE_BRIDGES.md](GAZEBO_SQUARE_BRIDGES.md) → 桥梁场景
6. [build-dependencies.md](build-dependencies.md) / [project-setup-record.md](project-setup-record.md) → 构建踩坑
7. [日志记录.md](日志记录.md) → 日志与复现
8. 相机线：[CAMERA_NAVIGATION_LEARNING_GUIDE.md](CAMERA_NAVIGATION_LEARNING_GUIDE.md) → [CAMERA_ONLY_NAVIGATION_PRINCIPLE_REPORT.md](CAMERA_ONLY_NAVIGATION_PRINCIPLE_REPORT.md) → [PURE_VISUAL_NAVIGATION.md](PURE_VISUAL_NAVIGATION.md)
9. [SLAM_NAV_MAP_EDIT_MIGRATION.md](SLAM_NAV_MAP_EDIT_MIGRATION.md) → 换硬件时逐项对照

## 附录 D · 四套规划算法对照与切换方式

本项目在**两个层级**上各有一套可替换的算法，一共四套，容易混淆，务必分清：

**全局层（`nav2_net_planner`，在拓扑图上求解）**

| 算法 | 代码位置 | 当前状态 | 切换方式 |
|---|---|---|---|
| SNN 脉冲波前 | `snn_topo_planner.{hpp,cpp}`，调用点 [topo_map.hpp#L811](../src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp#L811-L820) | **启用中** | — |
| A\* | [topo_map.hpp](../src/navigation2/planner/nav2_net_planner/include/nav2_net_planner/topo_map.hpp#L759-L762) 的 `astar()` / `astarExtended()` | 代码保留、**无任何调用** | 改 `planNodeIds`（代码级），**无参数** |

**局部层（`nav2_a_star_rpp` 控制器，在代价地图上重规划）**

| 算法 | 代码位置 | 当前状态 | 切换方式 |
|---|---|---|---|
| Theta\* + 汇入点/尾段拼接 | `nav2_a_star_rpp/src/theta_star.cpp` + 控制器绕障分支 | **默认**（`local_planner_type` 未显式设置时） | — |
| dyn A\* + 均匀 B 样条 | `nav2_scan_planner/src/core/dyn_a_star.cpp`、`uniform_bspline.cpp` | 可选 | 参数 `local_planner_type: "scan"` |

**切换成本一句话总结**：

- 局部 `theta_star ⇄ scan`：**有参数**，改 yaml 重启即可；
- 全局 `SNN ⇄ A*`：**目前只能改代码**；要做成运行时切换需自行添加 `planner_algorithm` 参数（见阶段 4.3 练习 8C）。

这也解释了为什么运行日志里只有 SNN 的统计（`planned_nodes`/`fired_neurons`/…）：A\* 当前根本不参与运行，只作为对照实现留在仓库里。
