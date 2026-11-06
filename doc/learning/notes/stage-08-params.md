# 阶段 8 · 参数化与多场景

- 日期：2026-11-05（实验于当日 21:16–21:20 完成）
- 对应计划：[阶段 8](../PLAN.md)
- 状态：已验收（2026-11-05；场景 C 的"门框检测失准"作为阶段结论保留，不再追加调参）
- 相关文件：[gazebo_net_nav.yaml](file:///home/lxh/nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav.yaml) / [test_world.world](file:///home/lxh/nanhu-research-navigation/src/gazebo_sim/vehicle_gazebo_sim/worlds/test_world.world)

## 1. 目标

同一套代码不改一行，靠 world 文件 + yaml 参数 + launch 参数换门宽、换门位、换出生点，并能解释"改了参数为什么不生效"。

## 2. 参数三层（先搞清谁生效）

`recovery_max_attempts` 在三个位置都出现：

| 位置 | 文件 | 值 |
| --- | --- | --- |
| hpp 成员初值 | `net_navigator.hpp#L133` | `{2}` |
| cpp `declare_parameter` 默认 | `net_navigator.cpp#L182` | `2` |
| yaml | `gazebo_net_nav.yaml#L59` | `2` |

**结论：运行时优先级 yaml > cpp declare 默认 > hpp 成员初值。**
hpp 初值只是 C++ 对象初始化，随后会被 `declare_parameter` + `get_parameter` 覆盖，所以"只改 hpp 不生效"是正常现象；yaml 路径写错或节点/插件名前缀写错，也会表现为"参数改了没反应"。

## 3. 三个场景的文件与差异

| 场景 | world | yaml | 与默认的差异 |
| --- | --- | --- | --- |
| A：0.9 m 门 | [test_world_door09.world](file:///home/lxh/nanhu-research-navigation/src/gazebo_sim/vehicle_gazebo_sim/worlds/test_world_door09.world) | [gazebo_net_nav_door09.yaml](file:///home/lxh/nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav_door09.yaml) | 门柱 y 7.15/8.85 → 7.05/8.95（净宽 0.70→0.90 m）；`door_align_angle 0.08→0.10`、`door_cross_speed 0.30→0.40` |
| B：反向穿门 | 默认 `test_world.world` | 默认 `gazebo_net_nav.yaml` | 只改出生点：`x:=12.0 y:=8.0 yaw:=3.1416`（车头朝 -x） |
| C：门挪到 x=13.5 | [test_world_gate135.world](file:///home/lxh/nanhu-research-navigation/src/gazebo_sim/vehicle_gazebo_sim/worlds/test_world_gate135.world) | [gazebo_net_nav_gate135.yaml](file:///home/lxh/nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav_gate135.yaml) | 两门柱 x 10.5→13.5（门宽仍 0.70 m）；`door_positions [10.5,8.0] → [13.5,8.0]` |

判定门宽的算式：柱心 y=7.05/8.95、柱长 1.0 m（y 方向），净缝 = 8.95-0.5-(7.05+0.5) = **0.90 m**；默认 8.85-0.5-(7.15+0.5) = **0.70 m**。

## 4. 实测结果

三场都用 `gui:=false` 无界面跑，启动命令形如：

```bash
ros2 launch vehicle_gazebo_sim simulation.launch.py gui:=false \
  world:=<绝对路径> x:=<...> y:=<...> yaw:=<...>
ros2 launch robot_launch gazebo_net_nav.launch.py use_rviz:=false \
  params_file:=<绝对路径>
```

| 场景 | 参数核对（`param get`） | 状态序列耗时 | 门区实测 | 结果 |
| --- | --- | --- | --- | --- |
| A | `door_cross_speed=0.4`、`door_positions=[10.5,8.0]` | IDLE→PLANNING→EXECUTING→SUCCESS = **6.92 s** | 直接 `NONE->ALIGN`（出生点距门 1.5 m < `door_enter_distance 2.2`，跳过 PRE_ALIGN）；ALIGN 606.728→EXIT 611.628 = **4.90 s**，前移 1.73 m；测宽 0.80→0.70 | SUCCEEDED，`distance_remaining` 3.0005→0.7048 |
| B | `door_cross_speed=0.3`、`door_positions=[10.5,8.0]` | 三态迁移 = **8.52 s** | 反向 `NONE->ALIGN` 689.481→EXIT 695.681 = **6.20 s**，前移 1.70 m；测宽 0.60~0.70 | SUCCEEDED，3.0004→0.6970；`topology_plan` 首点 (12.0004, 8.0007) 与节点 16 完全重合 |
| C | `door_cross_speed=0.3`、`door_positions=[13.5,8.0]` | 三态迁移 = **29.31 s** | PRE_ALIGN 在 along=2.14 m（x≈11.36，符合"门前 2.2 m"）；首次/二次 CROSS 都在 0.05/0.15 m 处被前向保护刹停，两次后退自救用尽 → 交回全局重规划；**最终仍 SUCCEEDED**，7.0005→0.5725 | 见下方分析 |

### 场景 A 补充

- 0.9 m 门在点云里只量到 **0.70–0.80 m**（雷达角分辨率 + 柱厚遮挡），说明不能拿"实测宽度"当真实门宽去标定参数。
- 全程没有 `[安全]`、`[恢复]` 日志，穿门干净。

### 场景 C 分析（本阶段最重要的一条）

1. `PRE_ALIGN` 按 `door_positions` 准时触发（门前 2.14 m），**说明门心坐标改对了**。
2. 但同刻的门框检测把门心估成 `(12.96, 6.44)`，距路线 **1.56 m** > 容差 0.40 m → 日志 `不启用门区伺服`。
3. 随后 ALIGN/CROSS 两次都在门前 0.05–0.15 m 被前向停车拦下，层 1 后退 `0.35 m` 两轮（cycle=1、2），之后 `已达最大自救次数(2)` → `交回全局路径重新规划`。
4. 重规划后门区阶段退出（`已知门不再位于路线前方，退出 PRE_ALIGN`），车辆最终到达 (16,8)。

**结论**：换门位只改 `door_positions` 只能保证"何时进入门区"，保证不了"门框测得准"。门从 x=10.5 挪到 13.5 后，门前助跑长度、雷达观测角度都变了，检测在 2.1 m 外配错左右柱。多场景验收必须同时看：`door_positions`、`door_enter_distance`、门前直线段长度、实际 `[门区诊断]` 输出，不能只看坐标。

## 5. 参数生效的验证方法

```bash
ros2 param list /car1/controller_server | grep door
ros2 param get /car1/controller_server FollowPath.door_cross_speed
ros2 param get /car1/controller_server FollowPath.door_positions
```

插件参数在 controller_server 节点上的名字是 **`FollowPath.<参数>`**，漏了前缀就会得到 `Parameter not set`。

## 6. 卡点（已解决）

| 现象 | 根因 | 解决 |
| --- | --- | --- |
| 第一次跑场景 A：导航栈只到 `Configuring nav2_log_record` 就 `Failed to change state`，goal 被接收但 150 s 不结束 | 上一会话残留的 nav2 节点仍在跑（同名节点、同 ROS_DOMAIN_ID=77），lifecycle 服务被旧节点接走；旧节点时钟已停 | 启动前清理残留：`ps -eo pid,args \| grep "[i]nstall/nav2_" \| awk '{print $1}' \| xargs -r kill -9`，再杀 `gzserver/gzclient`、删 `~/.gazebo` 锁 |
| 新增 yaml 放到 `src/.../config` 后 `params_file` 找不到 | `install/.../config` 指向 `build` 的拷贝，新增文件不重建不会同步 | 直接传 **src 下的绝对路径**，不需要重建 |
| 新 world 改了门柱坐标但仿真没变化 | 启动的是旧 world / 相对路径 | `world:=` 传新文件的绝对路径；world 改动无需编译 |

## 7. 自测作答

1. **参数三处不一致谁生效？** yaml > cpp `declare_parameter` 默认 > hpp 成员初值。表现为三种假象：改了 hpp 没反应；yaml 路径/节点名/插件前缀错导致没加载；改了没重启节点。
2. **换新世界必须同步改什么？** ① world（门位置/坡道/障碍物等几何）；② 拓扑图 yaml（节点、边、`max_snap_distance` 覆盖范围）；③ 参数 yaml（`door_positions`、footprint、`inflation_radius`、门区阈值等）；④ launch（world 绝对路径、出生点 x/y/yaw）。

## 8. 复习要点

1. 运行时参数优先级：yaml > declare 默认 > hpp 初值。
2. `FollowPath.<param>` 才是 controller_server 上插件参数的全名。
3. 换门位 ≠ 只改 `door_positions`：触发位置、检测距离、助跑段都要复核；场景 C 就是反例。
4. 多场景验收统一看：`state_change` 序列 + `[门区]` 阶段日志 + `[安全]/[恢复]` + action result。

## 9. 下次从哪继续

- 阶段 9：工程化与协作（构建脚本、启动脚本、日志、提交与交接）。
- 待补的总账：阶段 5（5 点实测 cost、门洞截图）、阶段 6（两段耗时、停车间距）、阶段 7（`nav_status` 序列、三次异常实验、`controller_server` 恢复 active）。
