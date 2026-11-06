# 阶段 7 · 状态机与对外接口

- 日期：2026-11-05（核对：原文件为空白模板，以下按源码与实操单基线补全，**实测项标“待实测”**）
- 对应计划：[阶段 7](../PLAN.md)
- 状态：进行中（待补你自己的状态序列与三次失败实验结果）
- 代码：[net_navigator.hpp](file:///home/lxh/nanhu-research-navigation/src/navigation2/system/nav2_net_system/include/net_navigator.hpp#L36-L48) / [net_navigator.cpp](file:///home/lxh/nanhu-research-navigation/src/navigation2/system/nav2_net_system/src/net_navigator.cpp)

## 1. 目标

知道目标怎么下发、结果怎么回、状态与日志从哪来，并能区分 `PLAN_FAILED` / `EXEC_FAILED` / `CANCELED`。

## 2. 状态枚举（源码 L45-48）

| 枚举 | 值 | 含义 | 对应 `navigation_status` |
| --- | --- | --- | --- |
| `IDLE` | 0 | 空闲 | IDLE |
| `PLANNING` | 1 | 调拓扑规划 | PLANNING |
| `EXECUTING` | 2 | 切片下发 FollowPath | **NAVIGATING** |
| `SUCCESS` | 3 | 到达 | SUCCEEDED |
| `PLAN_FAILED` | 4 | 规划失败 | FAILED |
| `EXEC_FAILED` | 5 | 控制失败且重试用尽 | FAILED |
| `CANCELED` | 6 | 被取消 | CANCELED |

**易错点**：状态机里没有独立的 `RECOVERY` 状态。重试是在 `EXECUTING` 内部用 `exec_recovery_attempts_` 计数，失败后回到 `PLANNING`（源码 L1005-1016）。

## 3. 命令与输出

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | `ros2 node list \| grep navigator` | `/car1/net_navigator` |
| 2 | `ros2 action list \| grep navigate` | `/car1/navigate_to_pose` |
| 3 | `ros2 action info /car1/navigate_to_pose` | 1 server、0 client |
| 4 | `ros2 topic info /goal_pose -v` | 订阅者 `/car1/net_navigator` |
| 5 | `ros2 topic echo /car1/nav_status`（std_msgs/Int32） | 整数即枚举值：1→2→3 |
| 6 | `ros2 topic echo /car1/navigation_status`（nav2_msgs/NavigationStatus） | `EXECUTING` 对应 `NAVIGATING` |
| 7 | `ros2 lifecycle get /car1/controller_server` | `active [3]` |
| 8 | `ros2 lifecycle set /car1/controller_server deactivate` 再 activate | FollowPath 不可用 → 导航失败/超时 |

> 日志来源：`publishLog()` 发到 `log/event`（nav2_msgs/Log），`nav2_log_record` 再落盘；状态迁移打的是 `state_change X -> Y`。

## 4. 状态序列（成功一次）

```text
IDLE → PLANNING → EXECUTING → SUCCESS
```

| 实测项 | 结果 |
| --- | --- |
| `nav_status` 数值序列 | 待实测（预期 1 → 2 → 3） |
| T3 里的 `state_change` 行 | 待实测（照抄原文） |
| action `--feedback` 里 `distance_remaining` 起止 | 待实测（起点约 3 m → 0） |
| 最终 status | `SUCCEEDED` |

## 5. 三次异常实验（基线预期，需你用自己的输出覆盖）

| 实验 | 目标/操作 | 预期结果 |
| --- | --- | --- |
| 规划失败 | 目标 (9, 18)，超 `max_snap_distance=4.0`（距节点 8 有 5 m） | `ABORTED`，状态 `PLAN_FAILED` |
| 控制失败 | 门前放 0.5 m 箱，发 (12,8) | 层 1 两次 BACKUP → `[恢复] ... 重新规划重试 1/2`、`2/2` → `EXEC_FAILED` |
| 取消 | 发 (20,8)，feedback 开始后 Ctrl+C | `CANCELED`，状态回 `IDLE` |

## 6. 关键参数

| 参数 | 值 | 作用 |
| --- | --- | --- |
| `follow_slice_horizon_m` | 30.0 | 每次只下发前方 30 m 路径 |
| `executing_freq_hz` | 1.0 | 下发/反馈节流 |
| `goal_match_tolerance` | 0.05 m | 状态机判定到点 |
| `path_progress_min_translation_m` | 0.05 m | 进度：最小位移 |
| `path_progress_max_projection_distance_m` | 1.0 m | 偏离路径判异常 |
| `path_progress_passed_point_margin_m` | 0.10 m | 已过点余量 |
| `recovery_max_attempts` | 2 | 控制失败后重新规划次数 |

## 7. 关键结论

1. 一个状态机、两条入口：话题 `/goal_pose` 便于调试，action `/car1/navigate_to_pose` 提供 feedback/result，是标准对外接口。
2. `nav_status`（Int32）只是状态枚举的“旧版镜像”；对外的正式结果看 action result（SUCCEEDED / ABORTED / CANCELED）与 `navigation_status`（EXECUTING 映射为 NAVIGATING）。
3. `PLAN_FAILED` = 没走出路径（吸附失败/无解）；`EXEC_FAILED` = 有路径但控制失败且 `recovery_max_attempts` 用尽。
4. 重试不是独立状态，而是 `EXECUTING` 内部计数后回 `PLANNING` 重来。
5. 切片（30 m）+ 1 Hz 节流是为了配合控制器 1.3 m 前视：路径太长没意义，切片让重规划切换代价更低。

## 8. 卡点

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| `action list` 找不到 `/car1/navigate_to_pose` | 导航栈未激活 | 等 `Activated (state=IDLE)` |
| 发 goal 立刻 ABORTED | 目标超吸附或 `frame_id` 错 | `frame_id: map`，目标靠近节点 |
| 找不到状态日志 | 看的是 std 输出而非 `log/event` | `ros2 topic echo /car1/log/event` 或查 `src/logs/navigation.log` |

## 9. 自测作答（参考答案）

1. **一次成功导航经历哪些状态？** `IDLE → PLANNING → EXECUTING → SUCCESS`（随后按实现回 `IDLE` 待下一目标）。
2. **`nav_status` 与 action result 的关系？** `nav_status` 是内部状态的整数镜像（0–6）；action result 是标准对外契约，只有 SUCCEEDED/ABORTED/CANCELED 三态加一段说明文字。两者语义对应但不完全同构（4、5 都归 ABORTED）。
3. **FollowPath 失败后为什么回 PLANNING？** 控制器抛 `PlannerException` 表示“当前这条路径不可执行”（前向受阻/无解），此时局部微调无效，必须让全局重新选路；重试上限 2 次，用尽才 `EXEC_FAILED`。

## 10. 复习要点

1. 7 个枚举值；无 RECOVERY 状态，重试在 EXECUTING 内计数。
2. 两条入口：`/goal_pose` 与 `/car1/navigate_to_pose`；`nav_status` 是 Int32 镜像。
3. `PLAN_FAILED`（没路径）≠ `EXEC_FAILED`（走不动且重试用尽）。

## 11. 待补（补完即可验收）

- [ ] 第 4 节四个“待实测”项
- [ ] 第 5 节三次实验的实际输出（尤其是 `EXEC_FAILED` 前的两行 `[恢复]`）
- [ ] 确认 `controller_server` 手动 deactivate/activate 实验后已恢复 `active`

## 12. 下次从哪继续

- 阶段 8：参数化与多场景（新世界 + 新路网 + 三场景验收）
