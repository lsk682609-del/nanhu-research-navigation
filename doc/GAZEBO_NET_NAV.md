# Gazebo 雷达路网导航

该入口复用本项目的路网规划器、net_navigator、RPP/Theta* 控制器及 input_bridge。
采用 Gazebo odom 定位、Livox 标准点云避障，通过 `/goal_pose` 发起导航。
不启动 CAN、SLAM、MAVROS 或视觉感知。Gazebo 模型自身的相机仍可发布数据，但导航不订阅。
原有 `3588_net_nav.launch.py`、`nx_net_nav.yaml` 和实车地图不变。

## 场景与坐标

- 本地图只对应 `vehicle_gazebo_sim/worlds/test_world.world`，即 `simulation.launch.py` 的默认场景。
  **不对应** `hui_path.launch.py` 的回字形场景；替换 world 时也要重新制作匹配的路网。
- Gazebo 世界原点 = `map` 原点 = 经纬高 `(30.8277361671667, 120.847804830667, 3.2133)`。
  世界 X 为东、Y 为北、Z 为上；yaw=0 朝东，yaw=π/2 朝北。
- 地理原点沿用 `topo_map_tuihua.yaml`，仿真路网的道路节点和边独立制作。
- 生成节点使用 **WGS84 椭球 ENU → ECEF → LLA**；导航加载时使用现有 C++
  `TopoMap` 的 **LLA → ECEF → ENU**。长半轴 6378137 m，扁率 1/298.257223563。
- 所有节点的 ENU 高度为 0；为保持平面 ENU，反算出的椭球高会有微小变化，不能强行全部改成原点高度。
- 文件虽然以 `.yaml` 结尾，实际采用项目自定义分段格式：
  `node: id type mapId x y z lat lon alt`，`edge: id type mapId from to obt`。
  `lla_mode: 1` 时加载器会从经纬高重新计算 xyz。
- `/goal_pose` 的位置是 **map 系米制坐标**，不是经纬度。它直接进入 net_navigator，
  不经过 bridge 的 GPS 转换和原点握手。bridge 未使用的 GPS 话题隔离在 `unused_gps/` 下。

## 模型与 TF

```text
map                         input_bridge 根据 /odom 发布
└── base_link                导航地面参考点
    └── base_footprint       仿真 launch 发布恒等静态变换
        └── chassis         robot_state_publisher；z=0.10 m
            └── livox_frame  robot_state_publisher；xyz=(0.24, 0, 0.20)
```

雷达相对 `base_link` 的最终外参为 **(0.24, 0, 0.30) m，RPY=(0,0,0)**，
与 3588 导航入口一致。修改位于 Gazebo 工作区的 `simple_car.urdf.xacro` 的 `livox_joint`。
安装目录若不是 symlink-install，修改模型后需重新构建 `vehicle_gazebo_sim`。

车体尺寸已恢复原模型：底盘箱体 `0.40 × 0.30 × 0.10 m`，轮半径 0.10 m、
轮宽 0.05 m，轮心 `x=±0.13、y=±0.205 m`，两对驱动轮距均为 0.41 m。
底盘及轮胎的平面包络约为 `0.46 × 0.46 m`（不包含安装在前方的传感器）。
底盘质量与惯量也恢复原值，雷达和相机外参保持不变；摩擦试验未应用到正式模型。
导航 footprint 仍为 `0.60 × 0.50 m`，保留较保守的碰撞包络，未随模型缩小。

控制器与 LiDAR costmap 参数以 `nx_net_nav.yaml` 为基准同步（包括期望速度 1.0 m/s、
预瞄距离 1.3 m、costmap 更新/发布频率 20/10 Hz）。仿真保留 `use_sim_time=true`、
控制器直接使用 `/odom`；按当前仅 LiDAR 范围不加载实车参数中的视觉观测和视觉边界层。
路网吸附范围、地理原点及 bridge 定位外参仍为仿真场景配置，不属于本次控制/costmap 同步。

保持差速插件 `publish_odom_tf=false`；不能再让其他定位节点发布同一条动态定位链。
原始 `/odom` 的 frame 是 `odom`，实际米制坐标须与 Gazebo 世界一致；bridge 将其发布为 map 系，
将 z 置零，`body_offset_*` 全部为零。它只改 frame 并执行已有补偿，不做世界坐标旋转和平移。
控制器直接订阅原始 `/odom` 的车体速度，不使用 bridge 的世界坐标差分速度。

该入口用于**单车仿真**。导航节点默认在 `/car1` 下，但 TF 使用全局 `/tf` 和 `/tf_static`，
与 Gazebo 模型发布器一致。仅改变 namespace 不构成多车 TF 隔离。

## 避障路径对比

- `/car1/theta_star_path_raw`：Theta* 折线按地图分辨率插值后、进入平滑器前的路径。
- `/car1/theta_star_path_smoothed`：同一次规划的后处理结果；平滑失败或关闭时与原始路径相同。
- 两者均为 `nav_msgs/msg/Path`，使用局部代价地图全局坐标系（当前为 `map`）及同一时间戳，
  不包含拼接后的全局路径尾段。每次 Theta* 规划时发布，保留最近一次结果；它们不是实时执行状态。
  RViz 添加两个 Path、设为不同颜色，Durability 选择 Transient Local 可读取最近一次结果。
- `/car1/received_global_plan` 仍是实际控制器跟踪路径，含截取/拼接，不等同于上述单次规划结果。

## 构建

```bash
cd /home/lxh/nanhu-research-navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to robot_launch \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DUSE_SYSTEM_FMT=ON
```

`USE_SYSTEM_FMT=ON` 用于当前 x86_64 仿真机器，避免链接实车 ARM 库。
需要本项目自定义的 Nav2 包，不能只 source 系统 Nav2 替代。若依赖未安装，先补齐；
本次初始编译曾缺少 `ros-humble-bondcpp`、`ros-humble-diagnostic-updater`、`ros-humble-geographic-msgs`。

## 启动

终端 1：

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
ros2 launch vehicle_gazebo_sim simulation.launch.py
```

无界面时追加 `gui:=false rviz:=false`。默认出生位置使用场景世界原点，导航会根据 odom 读取实际位置。

终端 2：

```bash
cd /home/lxh/nanhu-research-navigation
source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source install/setup.bash
ros2 launch robot_launch gazebo_net_nav.launch.py
```

所有终端使用相同的 `ROS_DOMAIN_ID`。验证时使用了独立域 81；日常启动不必设置为 81。
避免重复启动 Gazebo：ROS 域不同也不自动隔离 Gazebo Classic 默认 master 端口。
看到 `Managed nodes are active` 后再发目标。所有节点及内部 local_costmap 均使用 `/clock`。

支持的参数：

| 参数 | 默认值 | 用途 |
|---|---|---|
| `namespace` | `car1` | 导航节点命名空间 |
| `params_file` | `gazebo_net_nav.yaml` | 完整仿真配置 |
| `topo_map_path` | `topo_map_gazebo.yaml` | 经纬度路网文件 |
| `goal_topic` | `/goal_pose` | 外部 PoseStamped 目标 |
| `cmd_vel_topic` | `/cmd_vel` | Gazebo 速度输入 |

## 发送目标与停止

终端 3 source 同一套 ROS 环境后：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -5.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'
```

后续可测试节点 `(-10,5)`、`(-11,10)`、`(0,14)`、`(9,13)`、`(27,13)` 等。
发布程序应填写仿真时间戳和有效四元数；上述 CLI 示例的零时间戳表示使用最新变换。
路网采用与实车相同的节点吸附，最大吸附距离 4 m（最长边约 6.40 m，覆盖边中部重新规划）。
目标优先选用路网节点，并落在可通行区域。
不会把任意墙后或远离路网的点自动转换成安全目标。

当前 `planNodeIds()` 创建的边投影虚拟节点没有加入 SNN 的拓扑快照，开启
`edge_snap_enabled` 后，车辆位于边中部时可能规划失败。本仿真保持该参数为 false，
通过配置兼容现有算法，未修改规划器源码。

```bash
# 状态：0空闲，1规划，2执行，3成功，4规划失败，5执行失败
ros2 topic echo /car1/nav_status
# 取消任务
ros2 topic pub --once /car1/cancel_navigation std_msgs/msg/Empty '{}'
# 暂停速度输出；恢复时发送 false
ros2 topic pub --once /car1/pause std_msgs/msg/Bool '{data: true}'
```

不要同时运行其他向 `/cmd_vel` 发布的控制器或遥控程序。

## 检查与重新制图

```bash
python3 scripts/generate_gazebo_topo_map.py --check
ros2 param get /car1/local_costmap/local_costmap use_sim_time
ros2 param get /car1/local_costmap/local_costmap plugins
ros2 topic info /car1/cur_lid -v
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo map base_link
```

`plugins` 应只有 `obstacle_layer`、`inflation_layer`；`cur_lid` 的订阅者应为内部 local_costmap。
可在 RViz 中用 `map` 为 Fixed Frame，显示 `/car1/topology_plan`、`/car1/global_path`、
`/car1/cur_lid` 和 `/car1/local_costmap/costmap`，并通过 `/goal_pose` 发目标。

生成脚本中的 `POINTS` 和 `EDGES` 保存人工选定的道路骨架，包含一条西侧通路和东侧环路。
调整后运行脚本，将标准输出保存到 `src/robot_launch/maps/topo_map_gazebo.yaml`。
脚本会检查经纬度保存精度，并针对 world 中的墙体、箱体和圆柱几何按 0.025 m 采样检查路网边净距。
当前地图为 19 节点、19 条双向边；WGS84 序列化往返误差约 3.81e-8 m，最小采样净距 0.936 m。
该几何检查验证静态路网，不代替运行中的碰撞检查；也不自动从 world 提取道路。
`--world` 只指定几何检查文件，不会自动重建 `POINTS/EDGES`。

## 本次验证结果（2026-09-20）

- `robot_launch` 构建、安装和 `--show-args` 检查通过。
- 现有 C++ `TopoMap` 实际加载并重新投影 19 个经纬度节点；19×19 共 361 组节点间路径均可规划。
- 运行中的 local_costmap 使用仿真时间，插件只有雷达障碍层与膨胀层，订阅 `/car1/cur_lid`。
  点云为 `livox_frame`，代价地图为 `map`，代价地图中有非零障碍代价。
- TF 实测 `base_link → livox_frame` 平移 `(0.24, 0, 0.30)`，旋转为零。
- 通过 `/goal_pose` 发送 `(-5,1)` 后收到 `nav_status=3`，到达时误差约 0.193 m。
- 随后发送 `(0,14)`，使用最终节点吸附配置，经过西侧多个节点及转弯后收到 `nav_status=3`，
  到达时误差约 0.199 m，规划路径含 241 个采样点。

以上覆盖默认静态场景的基础闭环与多段跟踪，不表示所有目标、动态障碍和极限避障工况均已验证。

## 文件范围

新增 launch、仿真 YAML、地图、生成脚本和本文；`robot_launch/package.xml` 补充 bridge/log 的运行依赖。
`setup.py` 已使用 glob 自动安装新增 launch/config/maps，不需要修改。
Gazebo 工作区只调整雷达安装外参。未修改实车启动、实车配置和导航 C++ 算法。
