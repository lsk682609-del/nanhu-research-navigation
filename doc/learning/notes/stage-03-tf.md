# 阶段 3 · 定位与 TF

- 日期：待填
- 对应计划：[阶段 3](../PLAN.md)
- 状态：待开始

## 1. 目标
说清 `map → base_link` 从哪来，能排查 TF 类故障（无 TF、抖动、外推报错）。

## 2. 命令与输出

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | 期望 `[0.240, 0.000, 0.300]` |
| 2 | `ros2 run tf2_tools view_frames` | 生成 `frames_*.pdf` |
| 3 | `ros2 topic echo /tf --once` | `map → base_link` |
| 4 | `ros2 run tf2_ros tf2_monitor map base_link` | 频率与延迟 |
| 5 | 改 `body_offset_x: 0.2` 后重启 | `map→base_link` 的 x 变化 |

## 3. 关键结论
1. 
2. 
3. 

## 4. TF 树（自己抄一遍并标注发布者）

```text
map → base_link → base_footprint → chassis / livox_frame
```

## 5. 卡点

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| | | |

## 6. 自测
1. `tf2_echo odom base_link` 为什么失败？
2. 哪一行代码保证只有一条 `map→base_link`？
3. 静态 TF 忘记 `use_sim_time` 会怎样？

## 7. 复习要点
1. 
2. 
3. 

## 8. 产出
- [ ] `frames_*.pdf` 抄本
- [ ] 截图 `assets/stage-03-tf-tree.png`
