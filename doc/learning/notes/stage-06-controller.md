# 阶段 6 · 控制器（Pure Pursuit / 门区 / 自救 / 停车保护）

- 日期：2026-11-05
- 对应计划：[阶段 6](../PLAN.md)
- 状态：进行中（待补两项实测数字）
- 配置：[gazebo_net_nav.yaml](file:///home/lxh/nanhu-research-navigation/src/robot_launch/config/gazebo_net_nav.yaml#L84-L173)
- 代码：[regulated_pure_pursuit_controller.cpp](file:///home/lxh/nanhu-research-navigation/src/navigation2/controller/nav2_a_star_rpp/src/regulated_pure_pursuit_controller.cpp)

## 1. 目标

能解释：常规跟踪怎么算速度、门区为什么单独接管、层 1 自救何时触发、前向停车为什么只看 254。

## 2. 参数表

| 分组 | 参数 | 值 | 作用 |
| --- | --- | --- | --- |
| 跟踪 | `desired_linear_vel` | 1.0 m/s | 目标速度 |
| 跟踪 | `lookahead_dist` | 1.3 m | 前视距离，越大越切角 |
| 跟踪 | `min_approach_linear_velocity` | 0.05 m/s | 接近目标最低速度 |
| 跟踪 | `approach_velocity_scaling_dist` | 1.0 m | 距目标 1 m 内开始减速 |
| 门区 | `door_detect_range` | 2.6 m | 识别门的最大距离 |
| 门区 | `door_enter_distance` | 2.2 m | 进 PRE_ALIGN 的距离 |
| 门区 | `door_align_angle` | 0.08 rad | 对准判据 |
| 门区 | `door_align_rate` | 0.45 | 对准角速度上限系数 |
| 门区 | `door_cross_speed` | 0.30 m/s | 穿门速度 |
| 门区 | `door_lateral_gain` / `door_max_angular` | 1.6 / 0.45 | 横向纠偏增益与角速度上限 |
| 门区 | `door_hold_distance` | 0.50 m | 距门框小于该值锁存直行 |
| 门区 | `door_exit_margin` / `door_exit_hold` | 0.35 m / 0.40 m | 出门判据与保持行程 |
| 自救 | `recovery_backup_distance` | 0.35 m | 后退距离 |
| 自救 | `recovery_backup_speed` | 0.12 m/s | 后退速度 |
| 自救 | `recovery_rear_clearance` | 0.10 m | 车尾所需净空 |
| 自救 | `recovery_stuck_time` / `recovery_stuck_translation` | 1.5 s / 0.05 m | 有指令无位移判据 |
| 自救 | `recovery_collision_cycles` / `recovery_no_path_cycles` | 5 / 3 拍 | 另两条触发 |
| 自救 | `recovery_max_cycles` | 2 | 自救次数上限 |
| 停车保护 | `front_stop_clearance` / `front_slow_clearance` | 0.45 / 1.05 m | 停车阈值 / 开始降速 |
| 停车保护 | `front_stop_half_width` | 0.20 m | 前向探测半宽 |
| 停车保护 | `recovery_front_block_cycles` | 3 拍 | 连续受阻触发自救 |

两个前置问题的答案：

1. **门区为什么要单独一套逻辑？** 门净宽 0.70 m，常规前视 1.3 m 的跟踪在进门前会“抄近道”撞柱；门区改用门柱内缘点云测距直接纠偏。
2. **为什么停车净空取 0.45 m？** 更大则车到不了门前、影响正常接近；更小则车头贴柱，视觉上就是“撞墙”。

## 3. 正常穿门的日志时间线

| 顺序 | 关键字 | 含义 |
| --- | --- | --- |
| 1 | `PRE_ALIGN` | 距门 2.2 m，按已知门心 (10.5, 8) 预对准 |
| 2 | `ALIGN` | 航向误差 < 0.08 rad，进入对准 |
| 3 | `CROSS` | 进洞，速度切到 `door_cross_speed=0.30` |
| 4 | `EXIT`（含 `EXIT->NONE`） | 过门并保持 0.40 m 行程后交回 Pure Pursuit |
| 5 | `Reached the goal` | 到达 (12, 8) |

| 观测项 | 实测值 |
| --- | --- |
| PRE_ALIGN → EXIT 耗时 | 待填（你的秒表/日志时间） |
| 整段 (9,8) → (12,8) 耗时 | 待填 |
| 穿门期间 `cmd_vel.linear.x` | 应从 ≈1.0 降到 ≈0.30，出门后回升 |

## 4. 实验结论

### 4.1 穿门速度 `door_cross_speed`

| 设置 | 现象 |
| --- | --- |
| 0.30（默认） | 穿门稳定，耗时正常 |
| 0.15 | 更稳但耗时约翻倍；过低会逼近进度检查阈值（`movement_time_allowance=10 s`），有被判超时的风险 |

结论：速度与稳定性是权衡，0.3 是当前几何下的折中值。**实验后已还原 0.30。**

### 4.2 前视距离 `lookahead_dist`

| 设置 | 现象 |
| --- | --- |
| 1.3（默认） | 直线稳，门区由门区逻辑兜底 |
| 2.0 | 转弯与进门前明显切角，靠近门柱 |
| 0.8 | 贴线更紧，但角速度需求上升，高速时易“画龙” |

结论：前视距离必须与门区逻辑配合；单独调大前视会显著增加撞柱风险。**实验后已还原 1.3。**

### 4.3 前向受阻（门前 1 m 放 0.5 m 箱）

日志顺序：

| 顺序 | 日志 | 含义 |
| --- | --- | --- |
| 1 | `[安全] 车头前方 0.xx m 有障碍（要求 0.45 m）` | 净空不足，前向钳制为 0 |
| 2 | `[恢复] BACKUP cycle=1` | 层 1 后退自救 |
| 3 | `BACKUP cycle=2` | 第 2 次 |
| 4 | `已达最大自救次数(2)` / `交回全局路径重新规划` | 层 1 用尽，抛 `PlannerException` |
| 5 | `重试 1/2`、`2/2` → `EXEC_FAILED` | 层 2 重规划也用尽 |

| 观测项 | 实测值 |
| --- | --- |
| 停车时车头到障碍近面距离 | 待填（车心 x + 0.275 − 9.75） |
| 是否出现无限撞墙 | 否（本次改造后由停车保护接管） |
| 箱体是否已删除 | 是（`delete_entity` 已调用） |

## 5. 代码 5 问

1. **门区分支为什么排在常规跟踪之前？** 门区需要绕开常规前视与代价地图判定；门区接管后不再走常规碰撞检查（否则门洞的 253 带会直接否决）。
2. **`frontClearDistance()` 为什么只认 254？** 253 是内切膨胀带，窄道里大片区域都是 253，用它测距会把地面/门框当成正面墙；254 才是坡度层标记的真障碍。
3. **`frontStopActive()` 何时抛 `PlannerException`？** 连续受阻且 `recovery_cycles_ >= recovery_max_cycles_` 时抛；由 `controller_server` 捕获 → FollowPath 失败 → `net_navigator` 进入重规划。
4. **层 1 三个触发条件**：① 连续 5 拍预测碰撞（`recovery_collision_cycles`）；② 连续 3 拍局部规划无解（`recovery_no_path_cycles`）；③ 1.5 s 内实测位移 < 0.05 m（有指令无位移）。
5. **为什么后退位移必须用里程计？** 开环累加在车被物理阻挡时仍会“认为退够了”，导致反复无效后退；用里程计实测 + “0.6 s 无增长即中止”才能识别受阻。

## 6. 关键结论

1. 门区是“接管式”阶段机：一旦进入就绕过常规跟踪，必须在 EXIT 后清空绕障态，否则旧路径会触发连线检查导致掉头卡死。
2. 停车保护只看 254，且降速从 1.05 m 开始，避免高速顶墙；受阻 3 拍才触发自救，防止抖动误触发。
3. 层 1（后退自救）与层 2（全局重规划）职责不同：层 1 解决局部顶死，层 2 解决“这条路本身走不通”。
4. 自救上限 2 次是硬闸门；用尽即升级，绝不无限循环。
5. 参数调整必须成对考虑：`lookahead_dist` 与门区逻辑、`door_cross_speed` 与进度检查，都是耦合的。

## 7. 卡点

| 现象 | 误解 | 真因 | 解决 |
| --- | --- | --- | --- |
| 日志没有 `[门区]` | 以为门区逻辑没启用 | 车不在门区半径内 / `door_positions` 与地图不符 | 确认门心 (10.5,8) 与 `door_enter_distance` |
| `[安全]` 刷屏但车不动 | 以为保护逻辑卡死 | 净空不足且自救未达触发拍数 | 连续 3 拍才触发；观察 `recovery_front_block_cycles` |

## 8. 自测作答（参考答案）

1. **四阶段切换条件**：距门 ≤2.2 m 且识别到门框 → PRE_ALIGN；航向误差 <0.08 → ALIGN；对准且进入门洞范围 → CROSS；越过门框 + 保持 0.40 m → EXIT→NONE。
2. **为什么要锁存直行？** 贴近门框时门柱内缘会擦过激光视线，测宽失真；锁存航向可避免在门内被错误测距带偏。
3. **为什么只认 254？** 因为 253 是几何内切带，窄场景饱和，无法表示真实距离。
4. **层 1 / 层 2 边界**：层 1 只做“退一点再试”，层 2 才重新规划整条路径；层 1 次数用尽后抛异常，层 2 重试也用尽才 `EXEC_FAILED`。
5. **开环累加会怎样？** 车被阻挡时仍判定“退满”，随后重复无效动作，表现为原地来回蹭。
6. **前视变大的风险**：转弯切角、进门前偏移增大，容易撞门柱。

## 9. 复习要点

1. 门区 = 接管式阶段机（PRE_ALIGN→ALIGN→CROSS→EXIT），靠点云测门柱距离。
2. 停车保护只认 254，1.05 m 起降速、0.45 m 停；受阻 3 拍 → 层 1，用尽 → 层 2。
3. 后退位移必须用里程计实测，开环会误判。

## 10. 待补（补完即可验收）

- [ ] PRE_ALIGN→EXIT 与整段耗时
- [ ] 停车时车头到障碍近面的实测距离
- [ ] 确认 `door_cross_speed`、`lookahead_dist` 已还原，`git status src/robot_launch/config/` 干净

## 11. 下次从哪继续

- 阶段 7：`net_navigator` 状态机与对外接口（goal/feedback/result、`PLANNING/EXECUTING/RECOVERY`）
