# 30 m 正方形桥梁避障场景

场景文件：`src/gazebo_sim/vehicle_gazebo_sim/worlds/square_bridges_30m.world`。
围墙内表面位于 x/y=±15 m，内部净空 30×30 m。全局路网在 x/y=±10 m，
因此距内墙 5 m。ENU x 向东、y 向北，车辆默认出生于 (-10, -10)，朝向东。

| 桥 | 位置与行进方向 | 上/下坡角 | 3 m 坡段对应抬高 | 平台长度 |
|---|---|---:|---:|---:|
| 南侧 | (0, -10)，向东 | 5° | 0.2625 m | 5 m |
| 东侧 | (10, 0)，向北 | 10° | 0.5290 m | 5 m |
| 北侧 | (0, 10)，向西 | 15° | 0.8038 m | 5 m |

坡段按水平投影 3 m 制作，实体长度为 `3/cos(坡角)`；桥面宽 2.4 m。
三座桥的两侧现由实体侧墙从地面封至平台高度；侧墙内表面与桥面边缘齐平，
入口和出口不封堵，桥面净宽仍为 2.4 m。
圆柱与箱体障碍物分别位于南、东、北和西侧路径上，且避开桥面接缝。

路网：`src/robot_launch/maps/topo_map_square_bridges_30m.yaml`。经纬高由与
`gazebo_net_nav.yaml` 相同的参考点按 WGS84 **ENU→ECEF→LLA** 计算；
地图加载时再按 LLA→ECEF→ENU 反算。路网是顺时针单向环线，
`(-10,-10) → (10,-10) → (10,10) → (-10,10) → (-10,-10)`，
边中点也设了路网节点。导航仍是二维，节点高程为 0，桥的实际高程由 Gazebo 物理地形提供。

分别在两个终端启动（两端均先 `source /opt/ros/humble/setup.bash`、
`source /usr/share/gazebo/setup.sh`、`source install/setup.bash`）：

```bash
ros2 launch vehicle_gazebo_sim square_bridges_30m.launch.py gui:=false
ros2 launch robot_launch gazebo_square_bridges_nav.launch.py
```

导航启动默认同时打开 RViz，使用已有的 `robot_launch/rviz/navigation.rviz`。
无图形界面运行导航时添加 `use_rviz:=false`；纯 Gazebo 仿真入口不受此设置影响。

从默认出生点测试完整三桥路径，可向 `/goal_pose` 发布 `map` 帧目标
`(-10, 0)`；单向路网将使路线依次经过南、东、北三座桥后抵达西侧中点。
例如在导航节点激活后执行：

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: -10.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

不要与其他 Gazebo 世界或其他导航 launch 同时使用同一 ROS 域和话题。
