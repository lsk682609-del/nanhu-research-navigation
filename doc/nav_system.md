# 导航系统说明文档

## 1. 系统概述

基于 ROS2 Humble 的轮式机器人自主导航系统，集成 FAST-LIVO2 激光里程计 + Nav2 路网拓扑规划 + RoboSense 激光雷达感知避障。

## 2. 节点架构

```
┌─────────────┐    /car1/rslidar_points   ┌──────────────────┐
│ rslidar_sdk │ ─────────────────────────→ │   input_bridge   │
│  (雷达驱动)  │                            │   (数据桥接)      │
└─────────────┘                            └────────┬─────────┘
                                                    │
                    ┌───────────────────────────────┤
                    │                               │
                    ▼                               ▼
          ┌─────────────┐                  ┌────────────────┐
          │  fast_livo  │                  │   /cur_lid     │
          │   (SLAM)    │                  │  (裁剪点云)     │──→ costmap
          └──────┬──────┘                  └────────────────┘
                 │
          /car1/odom (IMU位姿)
                 │
                 ▼
          ┌─────────────┐    /odom (机器人中心+速度)   ┌──────────────────┐
          │   bridge    │ ──────────────────────────→ │ controller_server│
          │ (odom修正)  │                              │   (路径跟踪)      │
          └─────────────┘                              └────────┬─────────┘
                                                               │
                                                 /topology_plan │
                                                               │
                                          ┌────────────────────┘
                                          ▼
                                ┌──────────────────┐
                                │ net_planner      │
                                │ (路网拓扑规划)    │
                                └──────────────────┘
                                          │
                                          ▼
                                ┌──────────────────┐     ┌───────────┐
                                │  net_navigator   │────→│ ros2can   │
                                │  (导航状态机)     │     │ (CAN底盘)  │
                                └──────────────────┘     └───────────┘
```

## 3. TF 树

```
map ──→ base_link ──→ lidar
 ↑         ↑           ↑
bridge   robot中心     静态TF
发布       (地面投影)    [0.20, 0, 0.30, 0, 0.349, 0]
```

- `map`：全局固定坐标系，原点为 SLAM 初始化位置
- `base_link`：机器人地面投影中心，bridge 将 SLAM 输出的 IMU 位姿修正后发布
- `lidar`：雷达坐标系，相对 base_link 前 20cm、高 30cm、俯仰 20°

## 4. 核心 Topic

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/car1/rslidar_points` | PointCloud2 | rslidar_sdk | 原始雷达点云（XYZIRT） |
| `/cur_lid` | PointCloud2 | input_bridge | 转发给 Nav2 的点云（XYZI，frame=lidar） |
| `/car1/odom` | Odometry | fast_livo | SLAM 输出的 IMU 位姿（无速度） |
| `/odom` | Odometry | input_bridge | 修正后的机器人中心位姿（含速度） |
| `/topology_plan` | Path | net_planner | 路网规划全局路径 |
| `/global_path` | Path | net_navigator | 剩余待跟踪路径 |
| `/cmd_vel` | Twist | controller_server | 速度指令 |
| `/local_costmap/costmap` | OccupancyGrid | costmap | 局部代价地图 |

## 5. 数据流

### 5.1 定位链路

```
雷达点云 → FAST-LIVO2(ESKF) → IMU位姿(/car1/odom)
                                      │
                          input_bridge: 帧间差分算速度
                          input_bridge: 减去body_offset → 机器人中心
                                      │
                                  /odom (pose + twist)
                                      │
                    ┌─────────────────┤
                    ▼                 ▼
              controller_server   costmap (TF: map→base_link)
```

### 5.2 感知链路

```
rslidar_sdk(/car1/rslidar_points, XYZIRT, frame=rslidar)
    → input_bridge 转发(/cur_lid, XYZI, frame=lidar)
    → costmap_2d: TF转map系 → 高度截取(0.2~0.65m) → 二维投影
    → 局部代价地图
```

### 5.3 规划链路

```
目标点(/goal_pose)
    → net_planner: 吸附到路网节点 → A*搜索 → 稀疏路径
    → densify 插值(步长0.1m) → /topology_plan
    → net_navigator: 裁剪已走过部分 → /global_path
    → controller_server(RPP): 纯追踪控制 → /cmd_vel
    → ros2can: CAN总线 → 底盘电机
```

## 6. 配置文件

| 文件 | 说明 |
|---|---|
| `src/robot_launch/config/nx_net_nav.yaml` | Nav2 全部参数 |
| `src/robot_launch/config/slam.yaml` | laser_slam_3d 参数（已废弃） |
| `src/localization/slam/config/car1_airy_ros2.yaml` | FAST-LIVO2 参数 |
| `src/robot_launch/maps/topo_map.yaml` | 路网拓扑图 |
| `src/driver/rs_lidar_driver/rslidar_sdk/config/config.yaml` | 雷达驱动配置 |

## 7. 路网拓扑

`topo_map.yaml` 格式：

```yaml
map:           # 子地图
0 default
node:          # 节点: id type x y z yaw pitch roll
0 0 0 0 0 0 0 0
1 0 5 0 0 0 0 0
edge:          # 边: mapId id1 id2 type cost
0 0 0 1 0 0
```

规划时起点和终点会吸附到最近节点（或边上投影虚拟节点），再通过 A* 搜索最短路径。

## 8. 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `desired_linear_vel` | 0.5 m/s | 期望线速度 |
| `rotate_to_heading_angular_vel` | 0.5 rad/s | 转向角速度 |
| `max_angular_accel` | 1.57 rad/s² | 最大角加速度 |
| `min_obstacle_height` | 0.2 m | 障碍物最低高度（相对base_link） |
| `max_obstacle_height` | 0.65 m | 障碍物最高高度 |
| `max_snap_distance` | 15.0 m | 规划吸附最大距离 |
| `edge_snap_enabled` | false | 是否允许边投影虚拟节点 |
| `lookahead_dist` | 1.3 m | 纯追踪预瞄距离 |
| `body_offset_x` | 0.20 m | IMU→机器人中心前向偏移 |
| `body_offset_z` | 0.30 m | IMU→机器人中心高度偏移 |

## 9. 启动

```bash
source /home/lxh/nanhu-research-navigation/install/setup.bash

# 完整启动（雷达 + SLAM + 导航 + 底盘）
ros2 launch robot_launch nx_net_nav_slam.launch.py

# 关闭 RViz
ros2 launch robot_launch nx_net_nav_slam.launch.py use_rviz:=false

# 关闭雷达（离线测试）
ros2 launch robot_launch nx_net_nav_slam.launch.py use_rslidar:=false
```

## 10. 离线测试

模拟机器人在原点，规划到目标点 x=10m：

```bash
# 发布假定位
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

# 发送规划请求
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, \
    start: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, \
    use_start: true}"

# 查看结果
ros2 topic echo /topology_plan --once
```

## 11. 坐标系关系

```
SLAM 内部:  world → car1/base_link  (= IMU位姿)
bridge 输出: map → base_link          (= 机器人中心，z强制0)
导航使用:   map → base_link → lidar   (统一三层结构)
```

- bridge 负责 frame 重映射（world→map, car1/base_link→base_link）
- bridge 负责 IMU→base_link 平移修正（body_offset_x=0.20m）
- bridge 负责 z 强制归零（2D 导航）
- bridge 负责速度补全（帧间差分）
