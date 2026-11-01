# 阶段 2 · RViz 可视化

- 日期：待填
- 对应计划：[阶段 2](../PLAN.md)
- 状态：待开始

## 1. 目标
能配一个 RViz：Map / Path / PointCloud2 / Polygon / TF 正确显示，Fixed Frame 为 `map`。

## 2. 命令与输出

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | `ros2 launch robot_launch gazebo_net_nav.launch.py`（观察 RViz） | |
| 2 | 把 Fixed Frame 改成 `world` | 点云错位（贴现象） |

## 3. 关键结论
1. 
2. 
3. 

## 4. 卡点

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| | | |

## 5. 自测
1. `map` 与 `world` 作 Fixed Frame 差别？
2. 地图为什么 `transient_local`、点云为什么 `best_effort`？
3. 为什么 RViz 里没有 RobotModel？

## 6. 复习要点
1. 
2. 
3. 

## 7. 产出
- [ ] `my_nav.rviz`
- [ ] 截图 `assets/stage-02-rviz.png`
