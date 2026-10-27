# camera_boundary_navigation

无 LiDAR、无 odom、无相机里程计条件下的低速单边道路跟随包。节点只使用车辆当前局部
坐标系中的地面边界、地面栅格和障碍点，每帧重新生成局部参考曲线，不维护或伪造全局
位姿。

## 数据流

```text
/traversability/costmap/boundary_constraints --┐
/traversability/grid --------------------├→ camera_boundary_navigation_node
/traversability/costmap/depth_obstacles ------┘       ├→ /camera_navigation/reference_path
                                                 ├→ /camera_navigation/target_point
                                                 ├→ /camera_navigation/status
                                                 └→ /camera_navigation/cmd_vel
```

边界点云 intensity 约定：1=左边界、2=右边界、3=前方边界。节点固定跟随配置的
`reference_side`，用鲁棒二次曲线拟合边界，向道路内部偏移 `boundary.offset` 后通过局部
纯追踪生成速度。车辆原点到偏移边界起点之间使用三次Bé塞尔曲线平滑连接：起点切向
沿车辆前进方向，终点切向与边界偏移曲线一致，避免直接连接产生折角。参考路径必须落在
Ground 栅格内，并与障碍保持 `path_clearance`。

默认启用稳定局部规划策略。边界仅作为距离参考：规划器首先检查车辆正前方的直线路径，
当直线安全且边界距离位于允许死区内时强制保持直行。只有同一方向的边界偏差或直线不安全
持续达到配置帧数后才进入转弯；转弯曲率同时惩罚相邻帧变化，降低分割噪声造成的左右摇摆。

## 安全默认

左右边界在共同连续观测区间均有效时，进入 `CORRIDOR_CENTER`，以两侧中线为目标。
走廊两侧扣除与障碍膨胀一致的安全余量，候选和实际制动轨迹必须位于剩余区间；区间
过窄时返回 `corridor_too_narrow` 并停车。仅一侧有效时使用该侧内偏移，两侧均不足时前行。
拟合采用每侧最长连续观测段，不跨越超过0.20 m的纵向缺口。
`avoidance.preferred_clearance=0.20 m` 是膨胀区之外的舒适净距；不足时主动搜索更大净距
轨迹，优先于稳定直行死区。候选增加净距和走廊居中代价，碰撞半径不变。
诊断新增 `path_clearance`、`corridor_valid`、`corridor_usable`；净距不足搜索原因是
`nominal_clearance_low`。舒适净距是软目标，狭窄但仍安全的通道仍可能低速通过。

默认 `boundaryless.enabled=true`：边界不足时以前方直线为目标，继续用 Ground、障碍和
自适应 VFH 验证路径；遇障绕行，无安全路径则停车。边界恢复后自动切回跟随。
诊断字段 `navigation_mode` 为 `BOUNDARYLESS_FORWARD` 或 `BOUNDARY_FOLLOW`，
`boundary_valid` 保留实际拟合结果。合法空边界与边界话题断流不同，断流仍停车。
此模式要求避障开启；关闭避障时同时设置 `boundaryless.enabled=false`。
无航向反馈，前行方向是当前机体前方，绕障后不能保证恢复最初全局航向。

- `motion.enabled_on_startup=false`，默认只能发布零速；
- 默认速度话题是隔离的 `/camera_navigation/cmd_vel`，不会直接驱动订阅 `/cmd_vel` 的底盘；
- boundary/grid/obstacle 任一超过 `safety.input_timeout` 未更新即停车；
- enable 后必须连续通过 `recovery_valid_frames` 次检查才开始运动；
- pause、近距离障碍、前方有效范围过短、路径离开 Ground 均停车；关闭无边界模式时拟合失败也停车。

## 构建与离线运行

```bash
colcon build --packages-select traversability_obstacle_cloud camera_boundary_navigation
source install/setup.bash

ros2 launch camera_boundary_navigation camera_only_navigation.launch.py \
  depth_input_mode:=compressed_depth \
  enabled_on_startup:=false use_sim_time:=true

ros2 bag play --clock \
  /home/huibo/tdr/navigation/nanhu-research-navigation/rosbag/rosbag2_2026_08_26-16_25_06
```

离线查看：

```bash
ros2 topic echo /camera_navigation/status
ros2 topic echo /camera_navigation/reference_path
ros2 topic echo /camera_navigation/cmd_vel
```

显式使能算法输出：

```bash
ros2 topic pub --once /camera_navigation/enable std_msgs/msg/Bool "{data: true}"
```

## 真车接入

先保持隔离话题完成方向和停车验证。只有在有人监督、底盘已具备独立急停时，才将输出
切到实际底盘话题：

```bash
ros2 launch camera_boundary_navigation camera_only_navigation.launch.py \
  cmd_vel_topic:=/cmd_vel enabled_on_startup:=false
```

随后通过 enable 话题使能。节点没有全局定位能力，不能判断是否到达目标，也不能处理岔路；
它只适用于“单一路径、连续可见边界、低速前进”的受限场景。

## Gazebo纯视觉仿真

`vehicle_gazebo_sim` 已运行时，使用仿真专用组合launch。它加载1280x720相机内参、
相机相对车体的固定25度俯角外参，并关闭IMU依赖；速度默认仍发布到隔离话题：

```bash
source /opt/ros/humble/setup.bash
source /home/lxh/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source /home/lxh/nanhu-research-navigation/install/setup.bash

ros2 launch camera_boundary_navigation gazebo_camera_navigation.launch.py
```

先确认 `/camera_navigation/status` 持续具有有效边界和路径。需要闭环驱动Gazebo车辆时，
重新启动并显式接入底盘话题，之后再发布使能消息：

```bash
ros2 launch camera_boundary_navigation gazebo_camera_navigation.launch.py \
  cmd_vel_topic:=/cmd_vel enabled_on_startup:=false

ros2 topic pub --once /camera_navigation/enable std_msgs/msg/Bool "{data: true}"
```

仿真覆盖参数位于 `config/gazebo_camera_navigation.yaml`。基础参数仍从感知包和导航包的
默认YAML加载，覆盖文件只保存Gazebo差异。仿真配置允许最多0.10秒的未来时间戳，以兼容
Gazebo `/clock` 发布频率低于相机频率的情况；默认实机配置的该容差仍为0。仿真导航订阅
`/traversability/costmap/boundary_constraints` 和 `/traversability/costmap/depth_obstacles`。
两个组合launch均启动 `camera_costmap_cloud_node`：分类边界经剔除视场截断点、平滑和补点后
供导航使用，障碍经相对地面高度、体素和离群点过滤。`processed_perception.yaml` 将适配器
输出保持为 `gravity_aligned_base`，与 Ground 栅格同坐标；近场裁剪下限为0 m。
降采样后的障碍在导航安全栅格内达到1点即视为障碍。单独启动导航节点时需另行启动适配器。

## 深度点云局部避障

默认 `avoidance.enabled=true`。稳定规划器仍生成正常路径；正常路径被挡时，VFH 使用
**边界偏移路径的前视点**作为目标，用已确认障碍栅格构建密度无关的极坐标直方图。
候选由极坐标方向和曲率采样生成前进圆弧，通过机体地面覆盖、障碍膨胀和曲率检查后评分。
直方图是方向代价，最终圆弧的碰撞检查是硬约束，不直接把最优射线当作车辆轨迹。
每个曲率先检查最长可见圆弧；若远端超出可见Ground或遇障，则按路径采样间隔缩短，直到
`avoidance.minimum_path_length`。达到最小长度的安全前缀可以低速执行，并在下一相机帧
重新规划；Ground覆盖和障碍净距不会因缩短而放宽。
无需 LaserScan、odom 或 UWB 输入；处理后的障碍必须是上游已去地面/去自身的障碍点。

状态：`FOLLOW → AVOID → RETURN → FOLLOW`；无可行路径为 `BLOCKED`。避障不等待稳定
规划器的转弯确认帧。左右切换有惩罚但不阻止改选唯一安全的一侧；正常路径连续安全后再
回归。无可行候选、边界失效、Ground 缺失、近场急停时均停止，不回退到被挡的目标方向，
不倒车、不执行无侧后方观测支持的原地旋转。

- 所有点云必须处于 `output_frame`（默认 `gravity_aligned_base`，x前/y左）并具有相同
  时间戳。节点不进行 TF 变换；错误坐标系、陈旧/未来时间戳或非法字段会被拒绝。
  支持 little-endian、紧凑行布局、FLOAT32 x/y（边界和栅格另需 intensity）的 PointCloud2。
  合法空障碍云表示本帧无检测障碍，缺字段的空消息不表示无障碍。
- 新鲜度按源时间戳检查。bag 回放应使用 `use_sim_time:=true` 并 `ros2 bag play ... --clock`，
  不能用历史时间戳配合系统当前时间。恢复计数仅在新同步帧上增长。
- 启用避障时强制要求 Ground 和障碍输入。可见地面以外不是自由空间。近相机
  `minimum_ground_check_x` 以内仍沿用已有地面盲区豁免，但障碍检查不豁免。
- `avoidance.ground_half_width` 是保守圆形机体覆盖半径；`safety.path_clearance` 是障碍
  碰撞半径，额外加上半个栅格对角线补偿。应按车辆完整包络（包括前后悬）与余量设置，
  不能仅用车宽代替整个车体；示例值需按实际底盘复核。Ground 使用更严格的覆盖检查，
  稀疏或有遮挡的场景可能停车。
- 启用避障时 `minimum_front_range` 检查所选路径的连续安全弧长，不再被道路另一侧的
  最短前边界限制；候选仍受边界可见长度和完整 Ground 覆盖限制。
- 速度上限同时受停车距离、路径障碍净距和角速度限制。安全限速在命令滤波后再次应用，
  允许低于 `minimum_linear_velocity`。最终命令按当前曲率预测到停车距离再次检查。
  `braking_deceleration` 须使用底盘实测保守减速度，`reaction_time` 至少覆盖输入超时和
  一个控制周期。该预测不包含运动障碍轨迹，也不替代底盘急停。

新增可视化和诊断：

| 话题 | 内容 |
| --- | --- |
| `/camera_navigation/nominal_path` | 稳定规划器输出的正常参考路径 |
| `/camera_navigation/reference_path` | 实际选中的路径；阻塞时发布空路径清除旧显示 |
| `/camera_navigation/target_point` | 最终路径前视点；阻塞时为原点，结合状态使用 |
| `/camera_navigation/avoidance_markers` | 绿/红候选圆弧及橙色直方图；颜色表示几何可行性 |
| `/camera_navigation/status` | `avoidance_mode/reason/curvature/speed_limit/free_length` |

在 RViz 中固定坐标系设为 `gravity_aligned_base`，添加两个 Path 和 MarkerArray 即可查看。
`avoidance.enabled=false` 可恢复原有稳定跟随/遇障停车模式（输入格式和源时间戳校验仍保留）。

验证命令：

```bash
colcon build --packages-select camera_boundary_navigation
colcon test --packages-select camera_boundary_navigation
colcon test-result --verbose
```

第一版只执行单帧可见区域内的前进圆弧绕行。U形障碍、需要多段倒车/旋转脱困、长时间
遮挡或断续边界可能进入 BLOCKED，尚无地图记忆和全局脱困能力。

ROS 节点合成输入测试（固定使用本机 ROS_DOMAIN_ID=173 和独立测试速度话题，勿同时运行
多个该脚本）：

```bash
source install/setup.bash
python3 src/camera_boundary_navigation/test/test_vfh_node_integration.py \
  install/camera_boundary_navigation/lib/camera_boundary_navigation/camera_boundary_navigation_node
```

该脚本覆盖真实节点的正常跟随、中央障碍、墙体阻塞、恢复，以及断流、错误坐标系、
非法消息、不同步时间戳、过期消息停车。无需相机和底盘。
