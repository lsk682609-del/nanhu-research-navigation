# 阶段 1 · Gazebo：会造世界

- 日期：2026-11-03 ～ 2026-11-04
- 对应计划：[阶段 1](../PLAN.md)
- 状态：已验收
- 交付物：`src/gazebo_sim/vehicle_gazebo_sim/worlds/my_world.world`

## 1. 目标

从零写出一个 20×20 房间，含四面外墙、0.5 m 立方障碍、0.7 m 窄门、15° 坡道，并能把车生成进去。

## 2. 命令与输出记录

| # | 命令 | 关键输出/现象 |
| --- | --- | --- |
| 1 | `cp test_world.world my_world.world` | 得到 1255 行副本，随后删除除 `ground_plane` 外的全部模型 |
| 2 | `python3 -c "import xml.etree.ElementTree as ET; ET.parse('my_world.world'); print('XML OK')"` | `XML OK`（每次改完都跑一次语法自检） |
| 3 | `ros2 launch vehicle_gazebo_sim simulation.launch.py world:=/home/lxh/nanhu-research-navigation/src/gazebo_sim/vehicle_gazebo_sim/worlds/my_world.world x:=9.0 y:=8.0 z:=0.15 yaw:=0.0` | `Successfully spawned entity [simple_car]`；Gazebo 显示房间、红方块、两根黄门柱、坡道 |
| 4 | Gazebo 菜单 `View → Collisions` | 墙 / 门柱 / 方块都有碰撞色块，且与外观位置一致 |
| 5 | T4：`ros2 topic hz /odom` | ≈96 Hz，说明世界加载正常、车体插件工作正常 |

最终 world 文件 4812 字节，`<world name="default">`。

## 3. 我造的世界（实测坐标）

| 模型 | pose (x y z roll pitch yaw) | size (x y z) | 作用 |
| --- | --- | --- | --- |
| wall_south | `10 0 1.25 0 0 0` | `20 0.15 2.5` | 南墙 |
| wall_north | `10 20 1.25 0 0 0` | `20 0.15 2.5` | 北墙 |
| wall_west | `0 10 1.25 0 0 1.5708` | `20 0.15 2.5` | 西墙（转 90°） |
| wall_east | `20 10 1.25 0 0 1.5708` | `20 0.15 2.5` | 东墙（转 90°） |
| box_obstacle | `5 9.5 0.25 0 0 0` | `0.5 0.5 0.5` | 红色立方障碍 |
| gate_west | `10.5 7.15 1.0 0 0 0` | `0.15 1.0 2.0` | 门柱（内缘 y=7.65） |
| gate_east | `10.5 8.85 1.0 0 0 0` | `0.15 1.0 2.0` | 门柱（内缘 y=8.35） |
| slope_15deg_hill | `5 3.5 0 1.5708 0 0` | polyline 拉伸 | 坡道（见下） |

**门净宽推导**：柱心 7.15 / 8.85，柱在 y 方向长 1.0 → 内缘 7.65 / 8.35 → 净宽 **0.70 m**。

**坡道写法（沿用原世界的 polyline，比薄板可靠）**：

```xml
<geometry>
  <polyline>
    <height>2.0</height>
    <point>0 0</point><point>5 0</point>
    <point>3 0.5359</point><point>2 0.5359</point>
  </polyline>
</geometry>
```

- 水平 2 m 升高 0.5359 m → `atan(0.5359/2) = 15°`；
- `roll=1.5708`（90°）把原本沿 y 方向的坡转成沿 **x** 方向，放在 (5, 3.5)，不影响 (9,8)→(10.5,8) 的主通道。

## 4. 关键结论

1. `pose` 六元组是 `x y z roll pitch yaw`，单位米/弧度；墙的 z 取高度的一半（2.5/2=1.25）才能墙底贴地。
2. `<static>true</static>` 的模型不参与物理运动：z 写错也不会掉下来，但会悬空；非静态模型会自由落体。
3. `collision` 决定“能不能撞”，`visual` 决定“长什么样”。只写 visual 会看着有、实际能穿过去。
4. 东西墙必须加 `yaw=1.5708`：`size` 的 x 是长边，不旋转的墙会长在 x 方向，把房间切成两半。
5. 世界文件在 `--symlink-install` 下是软链，**改 world 不需要重新编译**，重启 `simulation.launch.py` 即可。
6. 坡道用 polyline 实体（边沿贴地）比薄板更安全：薄板边沿离地 1–2 cm 会被坡度可通行性层判成致命障碍。

## 5. 卡点

| 现象 | 一开始的判断 | 真正原因 | 解决 |
| --- | --- | --- | --- |
| —（本次未遇到阻塞性问题） | | | |

> 如过程中有报错，把原文补到本表，保留“现象 / 误判 / 真因 / 解决”四列。

## 6. 自测题作答（参考答案，可换成自己的话）

1. `pose` 的 6 个数字分别是什么？墙的 z 为什么取高度的一半？
   **答**：`x y z roll pitch yaw`；墙体几何中心在模型原点，z 取高度一半时底面正好落在 z=0 的地面上。
2. `collision` 和 `visual` 各影响什么？只写 visual 会怎样？
   **答**：collision 参与物理与代价地图，visual 只参与渲染；只写 visual 时能看见但能直接穿过去。
3. 东西墙为什么需要 `yaw=1.5708`？不加会看到什么？
   **答**：`size` 的长边沿 x，转 90° 才能沿 y 延伸；不加会把 20 m 的墙横穿房间。
4. `static=true` 的模型 z 写错为什么不会掉？非静态的会怎样？
   **答**：static 模型物理引擎不积分，z 写错只会悬空或陷入地面；非 static 会重力下落直到接触。
5. 改了 world 文件后，要不要 `colcon build`？为什么？
   **答**：不要。`--symlink-install` 下 `install/` 指向 `src/`，改 world 重启 launch 即生效。

## 7. 复习要点（下次 5 分钟唤醒用）

1. 六元组顺序 `x y z roll pitch yaw`；墙 z=高/2；东西墙靠 yaw=90° 转向。
2. `static` 管物理、`collision` 管碰撞、`visual` 管外观，三者独立。
3. 每改一次 world 先跑 `XML OK` 自检，再重启仿真；不编译。

## 8. 下次从哪继续

- [x] 俯视截图已补：`notes/assets/stage-01-world-top.png`
- [x] 已进入阶段 2：RViz 可视化（见 [stage-02 笔记](stage-02-rviz.md)）
