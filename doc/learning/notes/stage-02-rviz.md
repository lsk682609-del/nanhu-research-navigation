# 阶段 2 · RViz 可视化

- 日期：2026-11-05
- 对应计划：[阶段 2](../PLAN.md)
- 状态：已验收（2026-11-05 完成裁剪与截图）
- 交付物：`doc/learning/rviz/minimal.rviz`、`doc/learning/notes/assets/` 截图

## 1. 目标

读懂项目现有 `navigation.rviz` 的每个显示项，并自己保存一份可复用的最小配置。

## 2. 命令与输出记录

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | `grep -nE "Class: rviz_default_plugins\|Value: /\|Fixed Frame" src/robot_launch/rviz/navigation.rviz` | 解析出 8 个显示项：Grid、Polygon、Map、Topology Path、Controller Path、TF、LiDAR cur_lid、Goal Pose；`Fixed Frame: map` |
| 2 | `ros2 launch vehicle_gazebo_sim simulation.launch.py yaw:=0.0` | `Successfully spawned entity [simple_car]` |
| 3 | `ros2 launch robot_launch gazebo_net_nav.launch.py use_rviz:=false` | `net_navigator: Activated (state=IDLE)`，由自己单独开 RViz |
| 4 | `rviz2` → 设 Fixed Frame=map → Add 显示 → `File → Save Config As` | 生成 `doc/learning/rviz/minimal.rviz`（10 562 字节） |
| 5 | `ros2 topic pub --once /goal_pose ...(12,8)` | 路径延伸、车体沿路径移动、Map 膨胀带随车滚动 |
| 6 | `ls doc/learning/notes/assets/` | 已有 `stage-01-world-top.png`、`stage-02-minimal-rviz.png` |

## 3. 现有配置逐项解读

| 显示名 | 类型 | 话题 | 用途 |
| --- | --- | --- | --- |
| Grid | Grid | — | 地面参考网格 |
| Polygon | Polygon | `/car1/local_costmap/published_footprint` | 车体轮廓，检查窄门是否卡住 |
| Map | Map | `/car1/local_costmap/costmap` + `costmap_updates` | 看 253 内切带 / 254 障碍 |
| Topology Path | Path | `/car1/topology_plan` | 拓扑规划给出的全局折线 |
| Controller Path | Path | `/car1/received_global_plan` | 控制器实际接收并跟踪的路径 |
| TF | TF | `/tf`、`/tf_static` | 坐标系树 |
| LiDAR cur_lid | PointCloud2 | `/car1/cur_lid` | 前半球点云 |
| Goal Pose | Pose | `/goal_pose` | 目标点可视化 |
| Fixed Frame | — | `map` | 全局固定坐标系 |

**三个关键答案**

1. 话题都带 `/car1/` → 节点在 `car1` 命名空间下运行，RViz 显示必须写全名。
2. Map 订阅 `local_costmap` 而不是 `/map` → 本项目只启局部代价地图，不做全局栅格地图，全局规划在拓扑路网上。
3. Polygon 订阅 `published_footprint` → 控制器实际使用的车体轮廓（车头前缘 0.275 m、半宽 0.23 m）。

## 4. minimal.rviz 裁剪结果（返工已完成）

首次保存出来的是现有完整配置的副本（8 个显示项 + 8 个工具），2026-11-05 已裁剪并重建：

| 项目 | 裁剪前 | 裁剪后 |
| --- | --- | --- |
| 显示项 | Grid、Polygon、Map、Topology Path、Controller Path、TF、LiDAR cur_lid、Goal Pose（8 项） | **Map、Topology Path、TF、LiDAR cur_lid（4 项）** |
| 工具 | Interact、MoveCamera、Select、FocusCamera、Measure、SetInitialPose、SetGoal、PublishPoint | 去掉 SetInitialPose、PublishPoint，保留其余 6 个 |
| Fixed Frame | map | map（不变） |
| 文件大小 | 10 562 字节 | 193 行，YAML 解析通过 |

验证命令：

```bash
python3 -c "import yaml;d=yaml.safe_load(open('doc/learning/rviz/minimal.rviz'));print([x['Name'] for x in d['Visualization Manager']['Displays']])"
# ['Map', 'Topology Path', 'TF', 'LiDAR cur_lid']
```

> 裁剪方式：用 PyYAML 读入原配置 → 只保留 `Map`/`TF`/`PointCloud2` 与 `topology_plan` 的 `Path` → 重新 dump。比手工删块更稳，不会破坏缩进。手工删块时最容易踩的就是「块边界判断错，YAML 缩进断裂」。

**截图观察**（`assets/stage-02-minimal-rviz.png`，标题栏显示加载的正是 `doc/learning/rviz/minimal.rviz`）：

- 左侧 Displays 恰为 4 项：Map、Topology Path、TF、LiDAR cur_lid，Fixed Frame=map —— 配置目标达成；
- 画面中可见点云与一条绿色 `Topology Path`；Map 的膨胀带与 TF 坐标轴在当前缩放级别下不明显；
- 这说明“配置里有这一项”不等于“它正在出数据”，需要用放大视角 + 话题频率双重确认（见阶段 3 的 TF 验证）。

## 5. 关键结论

1. Fixed Frame 决定所有显示叠加到哪个坐标系；必须选 `map`，选 `base_link` 会导致地图/路径随车晃动。
2. 话题名必须带命名空间前缀 `/car1/`，否则显示一直为空。
3. PointCloud2 的 QoS 要和发布者匹配：`cur_lid` 由 `SensorDataQoS`（best_effort）发布，RViz 若用 Reliable 会收不到，需改成 `Best Effort`。
4. Map 显示的 253 是内切膨胀带、254 是真障碍；窄门里整片 253 是正常的，不代表门不可通行。
5. `.rviz` 是纯 YAML，保存即生效，不需要编译；`rviz2 -d <file>` 可直接加载。

## 6. 卡点

| 现象 | 一开始的判断 | 真正原因 | 解决 |
| --- | --- | --- | --- |
| `minimal.rviz` 里出现 Polygon / Controller Path / Goal Pose / 多余 Tool | 以为保存时会自动只存当前可见项 | 文件是从现有 `navigation.rviz` 复制而来，未做裁剪 | 按第 4 节 A/B 方案删除多余块后重新保存 |

## 7. 自测题作答（参考答案）

1. Fixed Frame 为什么必须是 `map`？改成 `base_link` 会看到什么？
   **答**：导航所有话题（costmap、path）都以 map 为参考系；改成 base_link 后地图与路径会随车移动/旋转，视觉上“世界在动”。
2. 话题名不带 `/car1/` 前缀会发生什么？
   **答**：RViz 订阅的是 `/local_costmap/costmap`，实际话题是 `/car1/local_costmap/costmap`，显示为空。
3. PointCloud2 有数据但不显示，第一个该怀疑什么？怎么确认？
   **答**：QoS 不兼容。先 `ros2 topic hz /car1/cur_lid` 确认有数据，再把 RViz 的 Reliability 改成 Best Effort。
4. Map 显示里 253 和 254 分别代表什么？
   **答**：253 = 内切/膨胀判定为不可通行的代价；254 = 致命障碍本体（坡度层标记的真实障碍）。
5. 保存的 `.rviz` 改掉 Fixed Frame 后，用 `-d` 加载会怎样？
   **答**：直接以新 Fixed Frame 打开；若该坐标系不存在，显示会全部失效并提示 frame 不存在。

## 8. 复习要点

1. Fixed Frame=map；话题要带 `/car1/`；点云 QoS 用 Best Effort。
2. Map→`/car1/local_costmap/costmap`；Path→`topology_plan` 或 `received_global_plan`。
3. `.rviz` 是 YAML，改了直接生效，`rviz2 -d` 加载。

## 9. 下次从哪继续

- [x] `minimal.rviz` 已裁剪为 4 个显示项（Map / Topology Path / TF / LiDAR cur_lid）
- [x] 截图已归档：`notes/assets/stage-02-minimal-rviz.png`（标题栏确认加载的是 minimal.rviz）
- [ ] 进入阶段 3：定位与 TF（先读 `input_bridge` 的 `onOdom()`）
