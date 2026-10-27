# 路网导航外部接口说明

本文说明路网导航系统面向任务调度端、地图编辑器及其他外部节点提供的导航接口。接口由 `nav2_net_system`、`nav2_net_planner` 和 `nav2_msgs` 提供，支持下发/取消导航任务、获取任务状态以及显示稀疏全局路网路径。

## 1. 接口总览

以下名称均为相对名称，会自动处于导航节点的命名空间下。以命名空间 `car1` 为例：

| 类型 | 相对名称 | `car1` 下的完整名称 | 消息类型 | 用途 |
|---|---|---|---|---|
| Service | `navigation_request` | `/car1/navigation_request` | `nav2_msgs/srv/NavigationRequest` | 新建或取消导航任务 |
| Topic | `navigation_status` | `/car1/navigation_status` | `nav2_msgs/msg/NavigationStatus` | 发布标准化任务状态 |
| Topic | `navigation_global_path` | `/car1/navigation_global_path` | `nav_msgs/msg/Path` | 发布用于显示的最简路网路径 |
| Topic | `odom_local` | `/car1/odom_local` | `nav_msgs/msg/Odometry` | `map_edit` 显示车辆当前位置 |

`navigation_status` 和 `navigation_global_path` 均采用 Reliable、Transient Local、深度 1 的 QoS。订阅端晚于发布端启动时，也能收到最近一次数据。

## 2. 导航请求服务

### 2.1 服务定义

文件：`src/navigation2/common/nav2_msgs/srv/NavigationRequest.srv`

```text
string command_id
string cmd
geometry_msgs/PoseStamped target_pose
---
string command_id
bool success
string message
```

字段含义：

- `command_id`：由调用端生成的指令标识，可为空。响应会原样返回，便于关联请求与日志；当前不用于去重或幂等控制。
- `cmd`：支持 `New` 和 `Cancel`，同时兼容小写 `new`、`cancel`。
- `target_pose`：`New` 指令的任务终点。`Cancel` 指令会忽略该字段。
- `success`：表示请求是否被导航节点接受或取消动作是否已执行，不表示机器人已经到达终点。
- `message`：请求处理结果或失败原因。

### 2.2 `New` 行为

收到合法的 `New` 请求后，导航器会：

1. 检查生命周期节点是否处于 Active 状态。
2. 检查 `target_pose.header.frame_id` 非空，位置与四元数各分量为有限数值。
3. 检查路网规划和路径跟踪 Action Server 是否可用。
4. 取消当前仍在执行的旧任务。
5. 接受新目标并进入规划状态。

服务返回 `success=true`、`message="导航目标已受理"` 只表示导航任务已进入异步处理流程。最终结果必须通过 `navigation_status` 判断。

系统当前采用“新任务覆盖旧任务”语义，不提供任务队列。

示例：

```bash
ros2 service call /car1/navigation_request nav2_msgs/srv/NavigationRequest \
  "{command_id: 'mission_001', cmd: 'New', target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 2.3 `Cancel` 行为

`Cancel` 会取消当前导航器持有的规划和控制请求，并将任务状态切换为 `CANCELED`。

```bash
ros2 service call /car1/navigation_request nav2_msgs/srv/NavigationRequest \
  "{command_id: 'cancel_001', cmd: 'Cancel', target_pose: {}}"
```

若不存在活动任务，取消请求仍按取消操作已执行返回。建议上层根据自己的任务编号和最近状态维护任务上下文。

### 2.4 常见拒绝原因

- 导航生命周期节点尚未激活。
- `cmd` 不是 `New` 或 `Cancel`。
- 目标坐标系为空。
- 目标位姿包含 NaN 或无穷值。
- 路网规划或控制 Action Server 尚未就绪。

## 3. 导航状态话题

消息文件：`src/navigation2/common/nav2_msgs/msg/NavigationStatus.msg`

| 状态值 | 常量 | 含义 | 是否为终态 |
|---:|---|---|---|
| 0 | `IDLE` | 当前无导航任务 | 否 |
| 1 | `PLANNING` | 正在请求或更新路网规划 | 否 |
| 2 | `NAVIGATING` | 正在执行路径跟踪 | 否 |
| 3 | `SUCCEEDED` | 最终路径执行成功，任务到达 | 是 |
| 4 | `CANCELED` | 任务已取消或被新任务替换 | 是 |
| 5 | `FAILED` | 规划或执行失败 | 是 |

其他字段：

- `distance_remaining`：导航器维护的剩余路网路径长度，单位为米。无有效距离时为 `-1.0`，成功时为 `0.0`。该值用于反馈显示，不作为最终到达判据。
- `message`：当前状态或失败原因，例如“正在规划”“正在导航”“导航成功”“路径控制失败”。

状态会在发生变化时立即发布，导航执行期间也会约每秒刷新一次。

```bash
ros2 topic echo /car1/navigation_status nav2_msgs/msg/NavigationStatus
```

上层推荐处理方式：

1. 服务返回成功后进入“等待执行结果”，不要立即标记任务完成。
2. 收到 `PLANNING` 或 `NAVIGATING` 时持续等待。
3. 仅收到 `SUCCEEDED` 时标记任务成功。
4. 收到 `CANCELED` 或 `FAILED` 时结束本次任务并记录 `message`。

当前状态消息不携带 `command_id`。若调用端可能高频替换目标，应在调用端串行下发任务，或结合自身任务状态避免把旧任务的终态关联到新任务。

## 4. 最简全局路网路径

`navigation_global_path` 用于地图界面显示任务将经过的路网节点。它从规划结果中提取中间拓扑节点：

- 不包含车辆实时起点。
- 不包含用户下发的精确终点。
- 会合并相邻的重复节点。
- 不进行路径加密，也不包含 Theta* 局部避障轨迹和平滑轨迹。

因此该话题适合显示全局路线走向，不应直接作为控制器输入或精确避障路径使用。实际控制路径仍通过导航内部的规划/控制 Action 链路传递。

```bash
ros2 topic echo /car1/navigation_global_path nav_msgs/msg/Path
```

规划器参数 `navigation_global_path_topic` 可修改该相对话题名称，默认值为 `navigation_global_path`。

## 5. `map_edit` 对接

`map_edit` 中的“导航接口命名空间”同时决定以下接口：

- `/<namespace>/navigation_request`
- `/<namespace>/navigation_status`
- `/<namespace>/navigation_global_path`
- `/<namespace>/odom_local`

命名空间为空时使用根命名空间下的接口。设置项内部仍沿用历史键名 `goal_pose_namespace`，仅用于兼容已有配置。

在地图中右键下发导航时，`map_edit` 会：

1. 生成形如 `map_edit_<毫秒时间戳>` 的 `command_id`。
2. 发送 `cmd=New`、坐标系为 `map` 的目标位姿。
3. 订阅标准状态并显示状态、剩余距离和提示信息。
4. 在规划、取消或失败时清除旧的全局路线显示。
5. 用蓝色线段显示 `navigation_global_path`，用 `odom_local` 更新车辆标记。

## 6. 生命周期与启动要求

服务虽然可能已经被 ROS 发现，但只有 `net_navigator` 进入 Active 状态后才接受任务。外部程序应等待服务可用，并正确处理“导航节点未激活”等拒绝结果。

这些接口是本项目 `nav2_msgs` 的扩展。编译和运行前必须使用本项目工作空间，不能只依赖系统安装的同名 `nav2_msgs`：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to nav2_net_system nav2_net_planner map_edit \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

## 7. 兼容接口与注意事项

现有 `/goal_pose`、`cancel_navigation`、`NavigateToPose` 和旧 `nav_status` 接口仍保留，便于已有程序继续运行。

旧 `nav_status` 直接发布导航器内部枚举，其中 `CANCELED=6`；新 `navigation_status` 使用标准化消息，其中 `CANCELED=4`、`FAILED=5`。新开发的上层程序应只使用 `NavigationStatus` 常量，不要混用两套状态数值。

当前接口的已知限制：

- `command_id` 仅用于请求/响应关联，不校验重复指令。
- 状态消息暂不携带 `command_id`。
- 服务响应成功不等于导航任务成功。
- 最简全局路径只用于可视化，不代表控制器正在执行的完整局部路径。

