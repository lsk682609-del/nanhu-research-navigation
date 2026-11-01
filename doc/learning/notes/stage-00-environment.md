# 阶段 0 · 环境与调试基本功

- 日期：2026-10-30

- 对应计划：[阶段 0](../PLAN.md)

- 状态：进行中（任务 1–3 已完成，自测题待作答）

## 1. 目标

能在三个终端独立拉起「仿真 + 导航栈 + 发目标」，并会用 node/topic/tf/param/log 五类调试入口。

## 2. 命令与输出记录

| #  | 命令                                                    | 关键输出/现象                                                                                                                       |
| -- | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| 1  | `lsb_release -ds`                                     | `Ubuntu 22.04 LTS`                                                                                                            |
| 2  | `gazebo --version`                                    | `Gazebo multi-robot simulator, version 11.10.2`                                                                               |
| 3  | `colcon build --symlink-install`（在 `src/gazebo_sim`）  | `Summary: 4 packages finished [1.61s]`                                                                                        |
| 4  | `./scripts/build.sh`（仓库根）                             | 第一次失败：`livox_ros_driver2` 报 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES ... NOTFOUND`；修正脚本后 `Summary: 25 packages finished [8.39s]` |
| 5  | `ros2 launch vehicle_gazebo_sim simulation.launch.py` | `Spawn status: SpawnEntity: Successfully spawned entity [simple_car]`                                                         |
| 6  | `ros2 launch robot_launch gazebo_net_nav.launch.py`   | lifecycle 依次 `Activating → Active`；`net_navigator Activated (state=IDLE)`                                                     |
| 7  | 发目标 `(12.0, 8.0)`                                     | `[门区] CROSS->EXIT 已前进 1.73 m` → `EXIT->NONE，恢复 Pure Pursuit` → `Reached the goal!` → `SUCCESS`                                |
| 8  | `ros2 topic hz /odom`                                 | `average rate: 99.46`（约100 Hz）                                                                                                |
| 9  | `ros2 run tf2_ros tf2_echo map base_link`             | 出生点 `Translation: [8.998, 7.996, 0.000]`；到达后 `[11.829, 8.000, 0.000]`                                                         |
| 10 | `ros2 run tf2_ros tf2_echo base_link livox_frame`     | `Translation: [0.240, 0.000, 0.300]`（与文档外参一致）                                                                                 |
| 11 | `ros2 node list`                                      | 21 个节点，含 `/car1/nav2_input_bridge`、`/car1/controller_server`、`/car1/net_navigator`                                            |

## 3. 关键结论

1. **两个工作区必须按序构建**：先在 `src/gazebo_sim` 构建（提供 `livox_interfaces2`、Gazebo 插件），再构建主工作区；主工作区构建前要 source 仿真工作区的 `install/setup.bash`。
2. **`livox_ros_driver2`** **需要 CMake 变量** **`DISTRO_ROS`**。缺失时会走错分支，把 `LIVOX_INTERFACES_INCLUDE_DIRECTORIES` 解析成 NOTFOUND。已在 `scripts/build.sh` 里统一传入 `${ROS_DISTRO}`。
3. **`--symlink-install`** **只对解释型资源生效**：yaml/launch/地图改动重启节点即可，`.cpp` 改动必须 `--packages-select` 重编。
4. **`ROS_DOMAIN_ID`** **必须三端一致**，否则 `ros2 node list` 看不到对方。
5. **启动顺序有依赖**：先生成车才有 `/odom`，才有 `map→base_link`，costmap 才能激活；否则会刷 `Timed out waiting for transform from base_link to map`。

## 4. 卡点

| 现象                                                                                      | 我一开始的判断                                           | 真正原因                                                               | 解决                                                |
| --------------------------------------------------------------------------------------- | ------------------------------------------------- | ------------------------------------------------------------------ | ------------------------------------------------- |
| 主工作区构建失败：`CMake Generate step failed` + `LIVOX_INTERFACES_INCLUDE_DIRECTORIES NOTFOUND` | 以为是 CMake 缓存脏了，删掉 `build/livox_ros_driver2` 重编仍失败 | 包内 `if(DISTRO_ROS STREQUAL "humble")` 分支未命中，走了 else 分支去读 target 属性 | `scripts/build.sh` 传 `-DDISTRO_ROS=${ROS_DISTRO}` |
| 启动仿真报 `PermissionError: ~/.ros/log/...`                                                 | 以为磁盘权限坏了                                          | 当时的执行环境对 `~/.ros` 没有写权限                                            | 换到有权限的终端执行（本质是环境问题，不是工程问题）                        |
| 激活时刷 `Invalid frame ID "map"`                                                           | 以为 TF 配置错                                         | 桥接节点尚未发布首帧 TF，属正常瞬态                                                | 等待 1–2 秒即自动恢复                                     |

## 5. 自测题作答

1. 三个终端分别 source 了哪几个 setup？顺序能否交换？
   **答**：`/opt/ros/humble` → `/usr/share/gazebo/setup.sh` → `src/gazebo_sim/install` → 主 `install`。**不能交换**：后 source 的覆盖前面的，主工作区必须最后；`gazebo_sim` 必须早于主工作区，否则找不到 `livox_interfaces2` 等依赖。
2. 改 yaml 要不要重新 build？改 cpp 呢？
   **答**：yaml/launch/地图不用重新 build（软链生效，重启节点即可）；cpp 必须 `colcon build --packages-select <包>`。
3. `ros2 topic info /goal_pose -v` 的发布者/订阅者是谁？
   **答**：发布者是发目标的一方（命令行或 RViz），订阅者是 `/car1/net_navigator`；QoS 为 reliable（默认）。
4. 日志三处分别在哪？
   **答**：`~/.ros/log/<时间戳>/`（各节点）、`src/logs/navigation.log`（项目规划与状态）、`snn_runs/`（每次规划快照）。

## 6. 复习要点（下次 5 分钟唤醒用）

1. 构建顺序：`gazebo_sim` → 主工作区；主构建前 source 仿真 install；`DISTRO_ROS` 由 `scripts/build.sh` 自动传入。
2. 三个终端的环境 = `navenv`；`ROS_DOMAIN_ID=77` 必须一致。
3. 启动依赖链：spawn 成功 → `/odom`（≈96 Hz）→ `map→base_link` → costmap 激活 → 可发目标。

## 7. 下次从哪继续

- 补两张截图到 `notes/assets/`（Gazebo 视角、RViz 视角，用 `gui:=true` 和默认 RViz 各截一张）

- 阅读 `scripts/build.sh` 与 `input_bridge` 的 `onOdom()`，为阶段 3 做铺垫

