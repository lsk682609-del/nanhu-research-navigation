# 术语速查（GLOSSARY）

> 用法：每次学习前花 5 分钟扫当前阶段相关的词条；忘了某个词就回来查。
> 每条格式：**术语（英文）** — 一句话解释 ｜ 对应位置。

---

## 一、ROS 2 基础

| 术语 | 解释 | 对应位置 |
| --- | --- | --- |
| 节点 node | 一个独立进程，提供话题/服务/动作/参数 | `ros2 node list` |
| 话题 topic | 单向数据流，发布者→订阅者 | `ros2 topic list` |
| 服务 service | 请求-响应，一次一答 | `ros2 service list` |
| 动作 action | 长任务，有 goal/feedback/result（导航用它） | `FollowPath`、`ComputePathToPose` |
| 参数 parameter | 节点的运行时配置，可声明/读取/动态修改 | `ros2 param list/get/set` |
| QoS | 通信质量策略；`best_effort` 与 `reliable` **不互通** | `input_bridge` 订阅点云/里程计 |
| `use_sim_time` | 用 `/clock` 仿真时间而非系统时间；所有节点必须一致 | 各 yaml 顶层 |
| 生命周期 lifecycle | 节点状态机 `unconfigured→inactive→active` | lifecycle_manager |
| launch | 启动编排脚本（Python），可传参/条件启动 | `src/robot_launch/launch/` |
| 命名空间 namespace | 节点/话题前缀，如 `/car1/controller_server` | launch 参数 |
| 重映射 remap | 启动时把话题名换掉 | `door_cloud_topic: cur_lid` |
| `source` | 把包的路径/环境变量注入当前 shell；顺序有讲究 | 阶段 0 |
| `--symlink-install` | install 里放软链，改 yaml/launch 免重编 | `scripts/build.sh` |

## 二、TF 与坐标

| 术语 | 解释 | 对应位置 |
| --- | --- | --- |
| TF / tf2 | 坐标系之间的变换树 | `/tf`、`/tf_static` |
| `map` | 全局固定系，与地图对齐，重定位时会跳 | 本项目 Fixed Frame |
| `odom` | 连续但漂移的里程计系；**本项目未使用** | 阶段 3 |
| `base_link` | 车体坐标系（叶子） | footprint 原点 |
| `base_footprint` | 地面投影系，z=0 | URDF 根 link |
| `livox_frame` | 雷达坐标系 | 外参 `(0.24, 0, 0.30)` |
| 静态 TF | 不随时间变，`/tf_static`，`transient_local` | `base_link→base_footprint` |
| 动态 TF | 每帧发布，`/tf` | `map→base_link` |
| 外参 | 传感器相对车体的安装位姿 | `tf2_echo base_link livox_frame` |
| 时间外推 extrapolation | 查询时刻超出 TF 缓存范围，典型 `use_sim_time` 不一致 | 常见报错 |
| ENU / ECEF / WGS84 | 经纬高 ↔ 东-北-天 米制坐标的换算链 | `llaToEnu()`、拓扑地图 origin |

## 三、Gazebo

| 术语 | 解释 | 对应位置 |
| --- | --- | --- |
| SDF / world | Gazebo 的场景描述文件 | `worlds/test_world.world` |
| `<static>` | 静态模型，不参与物理运动 | 门柱、墙 |
| `<collision>` vs `<visual>` | 碰撞体 vs 外观，二者可以不一致 | 阶段 1 |
| 插件 plugin | Gazebo 内运行的 ROS 接口（差速驱动、雷达） | `simple_car.urdf.xacro` |
| `spawn_entity` | 把 URDF/SDF 生成到世界中（失败=全链路空） | `simulation.launch.py` |
| `publish_odom_tf` | 底盘插件是否自己发 TF；本项目设 false | URDF |
| 薄板坡道 | 用倾斜薄板造坡，太薄会被判致命障碍 | 阶段 1/5 |

## 四、Nav2 与导航

| 术语 | 解释 | 对应位置 |
| --- | --- | --- |
| costmap | 栅格代价地图（0–255），规划与避障的基础 | `local_costmap` |
| `lethal_cost` | 判定“致命”的阈值；本项目 253 | yaml |
| 内切半径 inscribed | 车体最小外接圆半径（0.23 m），低于它的格子必撞 | footprint |
| inflation | 障碍周围按距离衰减的代价带 | `inflation_layer` |
| `rolling_window` | 局部地图跟随车移动 | `local_costmap` |
| footprint | 车体多边形 `[[0.275,0.23],...]` | yaml |
| 全局规划 global plan | 起点→目标的一次性路径（本项目=拓扑路网） | `net_planner` |
| 局部规划 local plan | 每周期在代价地图上重算的可执行轨迹 | Theta\* |
| 控制器 controller | 把路径变成 `cmd_vel` | `RegulatedPurePursuitController` |
| `lookahead_dist` | 纯追踪前视距离 | yaml |
| goal checker | 判定到达（`xy_goal_tolerance`） | yaml |
| progress checker | 判定“卡住”（位移/超时） | yaml |
| recovery | 失败的恢复行为（后退/重规划） | 本项目层 1 / 层 2 |
| RTG | rotate-to-goal，终点原地转向 | `use_rotate_to_heading` |

## 五、本项目专有

| 术语 | 解释 | 对应位置 |
| --- | --- | --- |
| 拓扑路网 topo map | 文本描述的节点+边地图，节点间距 < 5 m | `map_edit/`、生成脚本 |
| 吸附 snap | 把任意目标点投影到最近节点/边上 | `max_snap_distance: 4.0` |
| 加密 dense path | 把折线按 0.1 m 插值给控制器 | `dense_path_step` |
| 三点共线 | 门段两侧节点与门心共线，保证门内直线 | 路网约束 |
| `input_bridge` | 把底盘里程计/点云转成导航栈要的 frame 与话题 | `nav2_input_bridge` |
| `cur_lid` | 桥接后的点云话题（代价地图/门区/停车保护都用它） | yaml |
| 门区四阶段 | `PRE_ALIGN → ALIGN → CROSS → EXIT` | 控制器 |
| `door_hold_distance` | 距门框过近后锁存直行（测宽会失真） | yaml 0.50 |
| 层 1 自救 | 控制器内部 `BACKUP` 后退 0.35 m 再重规划 | `updateRecovery` |
| 层 2 重试 | `net_navigator` 回到 PLANNING 重规划，最多 2 次 | `recovery_max_attempts` |
| `front_stop_clearance` | 车头最小净空（0.45 m），低于则停 | yaml |
| `front_slow_clearance` | 开始线性降速的净空（1.05 m） | yaml |
| 253 vs 254 | 253=内切膨胀带（整片走廊都饱和）；254=真障碍 | costmap |
| `start_relax_radius` | 起点被染成致命时向半径 0.8 m 内松弛 | yaml |
| SNN | 备选全局规划算法（脉冲神经网络） | `net_planner` |
| `snn_runs/` | 每次规划的运行快照目录 | 调试 |

## 六、调试与工程

| 术语 | 解释 | 命令/位置 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | DDS 域号，不一致就互相看不见 | `export ROS_DOMAIN_ID=77` |
| `ros2 topic hz` | 看话题频率 | 排查“有没有在发” |
| `ros2 topic info -v` | 看发布/订阅者数量与 QoS | 排查“发了没人理” |
| `tf2_echo` | 查两个 frame 的实时变换 | 阶段 3 |
| `view_frames` | 导出整棵 TF 树 PDF | 阶段 3 |
| `rqt_graph` | 图形化节点/话题连接 | 阶段 0 |
| `~/.ros/log/` | 各节点日志 | `ls -t ~/.ros/log/*.log` |
| `src/logs/navigation.log` | 项目自己的规划/状态日志 | 调试 |
| `colcon build --packages-select` | 只编一个包，节省时间 | 改 cpp 后 |
| `.gitignore` | 排除 `build/install/log` 等 | 仓库根目录 |
