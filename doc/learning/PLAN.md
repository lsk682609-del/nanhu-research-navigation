# 从零复刻：分阶段学习计划

> **总目标**：不依赖现有代码，能独立完成「搭 Gazebo 世界 → 建拓扑路网 → 配导航栈 → 调通并解释每一步」。
> **学习对象**：没有任何 ROS 2 / 导航经验也能跟着走；每个阶段都有可勾选的任务、可复制的命令、验收标准与常见坑。
> **配套文件**：[学习中心](README.md) ｜ [进度打卡](PROGRESS.md) ｜ [笔记模板](TEMPLATE.md) ｜ [术语速查](GLOSSARY.md)

---

## 一、毕业标准（全部达到才算复刻成功）

| # | 能力 | 检验方式 |
| --- | --- | --- |
| 1 | 搭一个 Gazebo 世界，含墙、静态障碍、15° 坡道、0.7 m 窄门 | 不看现有 world 文件，写出 `.world` 并让车能进去 |
| 2 | 生成/修改拓扑路网文本，节点间距 < 5 m，门段三点共线 | 用生成脚本产出地图并通过往返一致性校验 |
| 3 | 配一套 Nav2 导航栈（costmap / controller / goal checker / recovery） | 从空 yaml 写出配置，车能走完指定路径 |
| 4 | 解释完整数据流 | 画出 `/odom → TF → costmap → planner → controller → cmd_vel` 并逐段说明 |
| 5 | 独立排查常见故障 | 给定 5 个故障现象，10 分钟内定位到根因 |
| 6 | 改控制器行为并量化验证 | 例如把停车净空从 0.35 改到 0.45，并给出停车间距实测值 |
| 7 | 写可复现的场景测试 | 给出发目标点脚本 + 日志判据，别人按文档能复现同样结果 |
| 8 | 用 Git 规范提交与协作 | 分支 + PR + 规范提交信息 + 冲突处理 |

---

## 二、学习方法（每个阶段固定循环，不要跳）

```text
① 读   ── 读本阶段「要学什么」+ 对应源码/文档（先看结构，不抠细节）
② 跑   ── 按「动手任务」跑通，记录关键命令与输出
③ 改   ── 改一个参数或一行代码，观察行为变化（这一步才真正学会）
④ 记   ── 填笔记（TEMPLATE）+ 打勾 PROGRESS + 写 3 条「下次复习要点」
```

约定：

- **每阶段结束必须留下 3 样东西**：笔记（`notes/stage-XX-*.md`）、一张截图（`notes/assets/`）、一条可复现的验收命令。
- **卡住超过 30 分钟就记录卡点并跳过**，在 PROGRESS 的「卡点」列写清楚，后面阶段会顺带解决。
- **每阶段开始前花 10 分钟复习**：看 [GLOSSARY](GLOSSARY.md) 对应词条 + 上次笔记末尾的「复习要点」。

---

## 三、阶段依赖图

```text
阶段0 环境基本功
   └─► 阶段1 Gazebo世界 ──► 阶段2 RViz可视化
                              └─► 阶段3 定位与TF ──► 阶段4 拓扑路网与全局规划
                                                        ├─► 阶段5 代价地图
                                                        └─► 阶段6 控制器
                                                              └─► 阶段7 状态机与接口
                                                                    └─► 阶段8 参数化与多场景
阶段9 工程化（可与任意阶段并行）
阶段10 视觉（选修，最后做）
Capstone 毕业项目（依赖 1–8）
```

---

## 阶段 0 · 环境与调试基本功

**目标**：能独立把仿真和导航栈拉起来，会用基础命令，看懂日志在哪。

**要学什么**

- [ ] 两个工作区（根目录主工作区、`src/gazebo_sim` 仿真工作区）为什么要分开、谁先构建
- [ ] `colcon build --symlink-install` 的含义（为什么改 yaml 不用重编）
- [ ] `ROS_DOMAIN_ID` 的作用（不一致就互相看不见）
- [ ] `source` 链的顺序：`/opt/ros/humble` → `gazebo setup` → `gazebo_sim install` → 主 `install`
- [ ] 节点 / 话题 / 服务 / 参数 / TF / 生命周期 六类调试入口

**动手任务**

1. 写一个 `navenv` 函数放进 `~/.bashrc`，新终端一条命令完成环境准备。
2. 依次构建仿真工作区、主工作区，记录两条构建命令与实际耗时行数（看最后一行 `Summary`）。
3. 开三个终端：仿真、导航+RViz、发目标点，让车从 (9,8) 走到 (12,8)。
4. 用 `ros2 topic hz /odom`、`tf2_echo map base_link`、`ros2 node list` 各取一份输出粘贴进笔记。

**关键命令**

```bash
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source ~/nanhu-research-navigation/src/gazebo_sim/install/setup.bash
source ~/nanhu-research-navigation/install/setup.bash
export ROS_DOMAIN_ID=77

ros2 launch vehicle_gazebo_sim simulation.launch.py          # 终端 A
ros2 launch robot_launch gazebo_net_nav.launch.py            # 终端 B
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 12.0, y: 8.0, z: 0.0}, orientation: {w: 1.0}}}"   # 终端 C

ros2 node list ; ros2 topic list ; ros2 topic info /goal_pose -v
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
```

**关键文件**：[scripts/build.sh](../scripts/build.sh)、[gazebo_net_nav.launch.py](../robot_launch/launch/gazebo_net_nav.launch.py)、[simulation.launch.py](../gazebo_sim/vehicle_gazebo_sim/launch/simulation.launch.py)

**验收（自测题）**

1. 三个终端分别 source 了哪几个 setup？顺序能不能换？
2. 改 `gazebo_net_nav.yaml` 后要不要重新 build？改 `.cpp` 呢？
3. `ros2 topic info /goal_pose -v` 里 `Publisher count` 和 `Subscription count` 各是谁？
4. 日志三处分别在哪？

**常见坑**：残留 `gzserver` 导致 Gazebo 空白；spawn 失败导致全链路没有 TF；`ROS_DOMAIN_ID` 不一致导致看不到节点。

**产出**：`notes/stage-00-environment.md`、启动成功的两张截图。

---

## 阶段 1 · Gazebo：会造世界

**目标**：能自己写一个包含墙、坡道、窄门的 `.world`，并把车放进去。

**要学什么**

- [ ] SDF 的骨架：`<world>`、`<model>`、`<link>`、`<collision>`、`<visual>`、`<pose>`、`<static>`
- [ ] 静态障碍物：`<static>true</static>`，且不需要末尾 `<state>` 段
- [ ] 坡道怎么造：薄板倾斜（`<pose>` 的 roll/pitch）+ 为什么边沿 0.019 m 离地会被判致命
- [ ] 窄门几何：两根门柱 + 净开宽度（本项目 0.70 m）
- [ ] 世界里的物理与光照参数（`<physics>`、`<light>`）什么时候需要改

**动手任务**

1. 复制 `test_world.world` 为 `my_world.world`，删掉所有障碍，只留地面与墙。
2. 自己加三个东西：一个 0.5 m 立方体障碍、一条 15° 坡道、一组 0.7 m 净宽门柱。
3. 用 `ros2 launch vehicle_gazebo_sim simulation.launch.py world:=/绝对路径/my_world.world x:=... y:=...` 把车放进你的世界。
4. 在 Gazebo 里按 `Ctrl+B`（或 View → Collisions）看碰撞体是否与外观一致。

**关键命令**

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py world:=/path/my_world.world x:=2.0 y:=2.0 yaw:=0.0
gz model -i          # 命令行查 Gazebo 模型列表（可选）
```

**关键文件**：`src/gazebo_sim/vehicle_gazebo_sim/worlds/test_world.world`

**验收**

1. 你的世界里有几面墙、几个障碍、坡道角度是多少（用 `pitch` 反算）？
2. 为什么门柱要用两根而不是一堵带洞的墙？
3. 静态模型为什么不需要 `<state>`？

**常见坑**：`<static>` 忘记写导致模型掉落；collision 尺寸比 visual 小导致“看着能过实际撞”；坡道薄板厚度太小被判致命障碍。

**产出**：`notes/stage-01-gazebo-world.md` + 你的 world 文件 + 一张俯视图截图。

---

## 阶段 2 · RViz：看得见才调得动

**目标**：能配一个 RViz，正确显示地图、路径、点云、车体轮廓、TF。

**要学什么**

- [ ] Fixed Frame 的作用（为什么必须是 `map`）
- [ ] 各显示项：Map / Path / PointCloud2 / Polygon / TF / Grid
- [ ] QoS：`transient_local`（地图）vs `best_effort`（点云）
- [ ] 保存配置为 `.rviz` 文件并在 launch 中加载

**动手任务**

1. 打开现有 [navigation.rviz](../robot_launch/rviz/navigation.rviz)，逐项说明每个 Display 订阅的话题。
2. 自己新建一个 RViz 配置：只留 Map、Path、PointCloud2、TF，保存为 `my_nav.rviz`。
3. 故意把 Fixed Frame 改成 `world`，观察点云是否错位，再改回 `map`。
4. 故意把点云的 QoS 改成 reliable，观察是否收不到数据（体会 QoS 不匹配）。

**验收**

1. `map` 与 `world` 作为 Fixed Frame 的差别是什么？
2. 为什么地图用 `transient_local`，点云用 `best_effort`？
3. RViz 里为什么看不到“车模型”而只有轮廓？（提示：没有 RobotModel 显示项）

**常见坑**：Fixed Frame 错→点云飘；QoS 不匹配→话题在发但 RViz 空白；忘记 `use_sim_time`→时间轴错乱。

**产出**：`notes/stage-02-rviz.md` + 你的 `.rviz` 文件。

---

## 阶段 3 · 定位与 TF

**目标**：说清楚 `map → base_link` 从哪来，能自己排查 TF 类故障。

**要学什么**

- [ ] REP-105 三层坐标：`map`（全局、会跳）、`odom`（连续、会漂）、`base_link`（车体）
- [ ] 本项目为什么**不设 odom 坐标系**，直接发 `map → base_link`
- [ ] `input_bridge` 的七件事：超时、frame 重映射、body offset、位姿差分算速度、发 odom、广播 TF（z=0）、缓存上一帧
- [ ] 静态 TF：`base_link → base_footprint`（恒等）、`base_footprint → chassis/livox_frame`
- [ ] 单发布者原则：底盘插件 `publish_odom_tf=false`
- [ ] 经纬度 ↔ ENU（拓扑地图 `lla_mode` / `origin`）

**动手任务**

1. `tf2_echo base_link livox_frame`，记录平移量，与 URDF 对照。
2. `view_frames` 导出 PDF，把整棵树抄到笔记并标注每条边的发布者。
3. 把 `body_offset_x` 改成 `0.2`，重启导航栈，观察 `map→base_link` 的 x 变化。
4. 停掉 `input_bridge`，记录 `controller_server` 的报错，再恢复。

**关键命令**

```bash
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames
ros2 topic echo /tf --once
ros2 topic echo /tf_static --once
ros2 run tf2_ros tf2_monitor map base_link
```

**验收**

1. `tf2_echo odom base_link` 为什么失败？
2. 哪一行代码保证只有一条 `map→base_link` 链？
3. 静态 TF 忘了 `use_sim_time: true` 会怎样？
4. `lla_mode: 1` 与 `0` 的区别？

**常见坑**：`world` 与 `map` 混用；`car1/base_link` 与 `base_link` 混用；两条链抢同一个子节点导致抖动。

**产出**：`notes/stage-03-tf.md` + `frames_*.pdf` 里的树形图抄本。

---

## 阶段 4 · 拓扑路网与全局规划

**目标**：能自己产出一张可用的路网地图，并解释路径是怎么算出来的。

**要学什么**

- [ ] 路网文本格式：头三行（`lla_mode`、`origin`、`node` 列定义）+ 节点 + 边
- [ ] 节点间距 ≤ 5 m 的原因（吸附半径 4 m）、门段三点共线的原因
- [ ] 目标吸附（`max_snap_distance`）与边吸附（`edge_snap_enabled`）
- [ ] 规划算法：波前/A\*（`net_planner_action_server`）与 SNN 双算法
- [ ] 路径加密（`dense_path_step: 0.1`）与发布话题 `topology_plan`

**动手任务**

1. 用 [generate_gazebo_topo_map.py](../scripts/generate_gazebo_topo_map.py) 从 `test_world.world` 重新生成一次地图，与仓库里的对比差异。
2. 手动在文本里加一个节点和一条边，让车能走到新位置。
3. 把 `max_snap_distance` 从 4.0 改成 1.0，观察靠近走廊中段的目标是否还能规划。
4. 用 `ros2 topic echo /topology_plan --once` 看规划输出的点列。

**关键命令**

```bash
python3 scripts/generate_gazebo_topo_map.py --help
ros2 topic echo /topology_plan --once
ros2 param get /car1.net_planner_action_server max_snap_distance
```

**验收**

1. 为什么节点间距必须小于吸附半径？
2. 门段要求三点共线的几何原因是什么？
3. 目标落在边中间时会发生什么（吸附逻辑）？

**常见坑**：节点落在墙里导致规划穿墙；间距过大导致目标吸不上（“目标无响应”）；经纬度换算与生成脚本不一致导致整体偏移。

**产出**：`notes/stage-04-topo-plan.md` + 你生成的地图文件 + 一张规划路径截图。

---

## 阶段 5 · 代价地图与可通行性

**目标**：能解释每个格子上的代价是怎么来的，知道 253 与 254 的区别。

**要学什么**

- [ ] costmap 结构：`local_costmap` / `global_costmap`、`rolling_window`、`resolution`、`footprint`
- [ ] `SlopeTraversabilityLayer`：点云 → 坡度/台阶/残差 → 标记障碍
- [ ] `InflationLayer`：`inflation_radius`、`cost_scaling_factor`、内切半径
- [ ] 关键数字：内切 0.23 m、膨胀 0.45 m、`lethal_cost: 253`、真障碍 254
- [ ] 为什么窄门里不能靠代价地图测距（整片 253）

**动手任务**

1. 在 RViz 里打开 costmap 显示，找到门洞位置，确认那里是 253 而不是 254。
2. 把 `inflation_radius` 从 0.45 改到 1.0，观察窄门是否被“封死”。
3. 把 `lethal_cost` 从 253 改到 200，观察车是否不敢进走廊。
4. 用 `ros2 topic echo /car1/local_costmap/costmap --once --field info` 看元数据。

**验收**

1. 253 与 254 各代表什么？为什么测距只能用 254？
2. 内切半径怎么从 footprint 算出来？
3. 膨胀半径过大为什么会“封门”？

**常见坑**：`inflation_radius` 过大→两侧膨胀重叠；`lethal_cost` 过低→整条走廊不可通行；footprint 与真实车体不符→贴墙或撞墙。

**产出**：`notes/stage-05-costmap.md` + 门洞处代价截图（含 253 区域）。

---

## 阶段 6 · 控制器（本项目最核心）

**目标**：能解释并修改「纯追踪 + Theta\* 局部规划 + 门区四阶段 + 两级恢复」。

**要学什么**

- [ ] Regulated Pure Pursuit：`lookahead_dist`、速度调节、`min_approach_linear_velocity`
- [ ] Theta\* 局部规划与无解回退（`start_relax_radius`、全局回退）
- [ ] 门区四阶段：`PRE_ALIGN → ALIGN → CROSS → EXIT`，门框测距伺服、`door_hold_distance` 锁存直行
- [ ] 层 1 自救：`BACKUP` 阶段、里程计实测位移锚点、受阻兜底
- [ ] 层 2 全局重试：`net_navigator` 的 `recovery_max_attempts`
- [ ] 前向停车保护：`front_stop_clearance`、降速带、`recovery_front_block_cycles`

**动手任务**

1. 正常穿门跑一次，抓取门区四个阶段的日志时间线。
2. 把 `front_stop_clearance` 从 0.45 改到 0.60，在走廊受阻场景记录停车间距。
3. 把 `recovery_max_cycles` 改成 0，观察不再后退、直接抛错重规划。
4. 把 `door_cross_speed` 从 0.30 降到 0.15，观察穿门时间与稳定性。

**关键命令**

```bash
ros2 param get /car1.controller_server FollowPath.front_stop_clearance
ros2 param get /car1.controller_server FollowPath.door_positions
ros2 param set /car1.controller_server FollowPath.front_stop_clearance 0.60   # 部分参数支持动态修改
```

**验收**

1. 为什么门区要绕过常规碰撞检查？
2. 为什么后退距离必须用里程计实测而不是指令累加？
3. 层 1 与层 2 的职责边界是什么？
4. 停车保护为什么不用代价地图测距？

**常见坑**：门区锁存航向丢失导致绕障分支触发；自救次数用尽仍不抛错→无限撞墙（本项目已修）；RTG 阈值等于 `xy_goal_tolerance` 导致边界死锁。

**产出**：`notes/stage-06-controller.md` + 三段日志摘录（安全/自救/重规划）。

---

## 阶段 7 · 状态机与对外接口

**目标**：知道上层怎么下发目标、怎么知道执行结果、日志从哪来。

**要学什么**

- [ ] `net_navigator`：`FollowPath` 动作调用、进度判定、重试状态机
- [ ] 对外话题：`/goal_pose`、`/car1/nav_status`、`log/event`、`log/record`
- [ ] Nav2 action 模型：`ComputePathToPose`、`FollowPath` 的 goal/feedback/result
- [ ] 生命周期：`configure → activate → deactivate → cleanup`

**动手任务**

1. 发目标后 `ros2 topic echo /car1/nav_status` 记录状态序列。
2. 发一个不可达目标，记录失败状态与日志关键字。
3. 用 `ros2 lifecycle get /car1.controller_server` 查看状态，手动 `deactivate/activate` 一次。

**验收**

1. 一次导航成功会经历哪些状态？
2. `nav_status` 与 action result 的关系？
3. 为什么 `net_navigator` 在 FollowPath 失败后要回到 `PLANNING`？

**产出**：`notes/stage-07-state-machine.md` + 状态序列记录。

---

## 阶段 8 · 参数化与多场景

**目标**：同一套代码能换世界、换起点、换门位置而不改代码。

**要学什么**

- [ ] 参数分层：`hpp` 默认值 / `cpp` `declare_parameter` / `yaml` 三处一致
- [ ] launch 参数化（`world`、`x/y/yaw`、`use_rviz`、`use_sim_time`）
- [ ] 命名空间（`car1`）与话题重映射
- [ ] 多机器人扩展思路

**动手任务**

1. 复制一份 yaml 做“窄门 0.9 m”版本，只改门区参数，验证能否通过。
2. 换出生点 (2,8) 跑同一条路径，确认路网吸附正常。
3. 把 `door_positions` 改成新世界里的门心坐标，验证 PRE_ALIGN 自动启动。

**验收**

1. 参数默认值三处不一致会出什么问题？
2. 换世界时必须同步改哪些文件？

**产出**：`notes/stage-08-params.md` + 你的第二份 yaml。

---

## 阶段 9 · 工程化与协作

**目标**：代码能稳定复现、能给别人用。

**要学什么**

- [ ] `--symlink-install`、分包编译、`BUILD_TESTING=OFF`
- [ ] 脚本化：构建脚本、启动脚本、发目标脚本
- [ ] 日志与可观测性：`~/.ros/log`、项目日志、`snn_runs`
- [ ] Git：分支、规范提交、PR、冲突（见 [Git 协作规范](../git-workflow.md)）

**动手任务**

1. 写一个 `scripts/goal.sh`，一条命令发目标并打印结果。
2. 按 Git 规范提交本阶段全部笔记（`docs(learning): ...`）。
3. 制造一次冲突并解决（用分支同时改同一行）。

**产出**：`notes/stage-09-engineering.md` + 你的脚本 + 一次合规提交记录。

---

## 阶段 10 · 视觉与相机（选修）

**目标**：了解相机导航链路，知道和激光链路如何并存。

**要学什么**：相机话题与内参、图像帧与 TF、视觉目标检测结果如何转成目标点。

**产出**：`notes/stage-10-vision.md`（只写链路图与关键话题即可）。

---

## Capstone · 从零复刻一个新场景（毕业项目）

**任务**：自选一个仓库里没有的场景（例如“L 形走廊 + 两扇窄门 + 一段坡道”），从零完成：

1. 写世界文件（阶段 1 能力）
2. 生成路网（阶段 4 能力，节点间距 < 5 m，门段共线）
3. 写导航栈 yaml（阶段 5、6、8 能力）
4. 跑通三个验收场景：正常通过 / 中途受阻能重规划 / 不可达明确失败
5. 写一份复刻报告：世界几何 → 路网 → 参数 → 日志判据 → 遇到的问题与解决

**毕业判据**：别人只看你的报告，能在这个仓库里复现同样的结果。

**产出**：`notes/capstone-report.md` + 世界/地图/yaml/脚本 四个文件 + 三张关键日志截图。

---

## 四、每阶段的时间盒（不是硬性时长，而是防沉迷）

| 情况 | 处理 |
| --- | --- |
| 30 分钟卡在同一个报错 | 记录到 PROGRESS「卡点」列，跳到下一小步或先做「跑」部分 |
| 某阶段任务全部完成但讲不清 | 不要进入下一阶段，先写笔记并自测 |
| 想跳过阶段 1/4/5 直接学控制器 | 不建议；控制器的问题 80% 根因在世界/路网/代价地图 |

---

## 五、复习机制（隔段时间快速捡回来）

| 频率 | 动作 | 用时 |
| --- | --- | --- |
| 每次开始学习前 | 看 [GLOSSARY](GLOSSARY.md) 里本阶段词条 + 上次笔记末尾「复习要点」 | 5–10 分钟 |
| 每完成 3 个阶段 | 重跑一次该阶段「验收命令」，看是否还能复现 | 20 分钟 |
| 想整体回顾 | 只看 [PROGRESS.md](PROGRESS.md) 的「一句话总结」列 + 各阶段笔记的「关键结论」段 | 30 分钟 |
| 准备面试/汇报 | 用 Capstone 报告当提纲，按「世界→路网→导航栈→调优」讲一遍 | 40 分钟 |
