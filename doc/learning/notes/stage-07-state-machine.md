# 阶段 7 · 状态机与对外接口

- 日期：待填
- 对应计划：[阶段 7](../PLAN.md)
- 状态：待开始

## 1. 目标
知道目标怎么下发、结果怎么回、状态与日志从哪来。

## 2. 命令与输出

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | 发目标后 `ros2 topic echo /car1/nav_status` | 状态序列 |
| 2 | 发不可达目标 | 失败状态与日志关键字 |
| 3 | `ros2 lifecycle get /car1.controller_server` | active |
| 4 | 手动 `deactivate`/`activate` | 观察行为 |

## 3. 关键结论
1. 
2. 
3. 

## 4. 状态序列（成功一次）
```text
PLANNING → ... → SUCCEEDED
```

## 5. 卡点

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| | | |

## 6. 自测
1. 一次导航成功经历哪些状态？
2. `nav_status` 与 action result 的关系？
3. FollowPath 失败后为什么回 PLANNING？

## 7. 复习要点
1. 
2. 
3. 
