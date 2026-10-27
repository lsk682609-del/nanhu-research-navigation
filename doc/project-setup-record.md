# 项目编译与运行记录

## 1. fmt 版本不匹配问题

**问题**：系统 `libfmt-dev` 是 v9（头文件在 `/usr/include/fmt/`），但 `libspdlog.so` 和整个 Nav2 链路（nav2_util → rcl_logging_spdlog → spdlog）都依赖 `libfmt.so.8`。编译 `nav2_log_record` 时用 v9 头生成 v9 符号，链接时找不到实现。

**解决**：
- 从 apt 下载 `libfmt-dev=8.1.1+ds1-2` 和 `libfmt8=8.1.1+ds1-2` 的 deb 包
- 解压到 `3party/fmt/8.1.1/`（头文件在 `include/fmt/`，so 在 `lib/aarch64-linux-gnu/`）
- 修改 `src/navigation2/system/nav2_log_record/CMakeLists.txt`，添加 `USE_SYSTEM_FMT` 开关（默认 OFF，走本地 v8）

**CMakeLists.txt 关键改动**：
```cmake
option(USE_SYSTEM_FMT "Use system-installed fmt" OFF)

if(NOT USE_SYSTEM_FMT)
  # 在 find_package(nav2_util) 之前手动创建 fmt::fmt 目标
  # 因为 nav2_util → spdlog → find_dependency(fmt) 会先拉系统 v9
  add_library(fmt::fmt SHARED IMPORTED)
  set_target_properties(fmt::fmt PROPERTIES
    IMPORTED_LOCATION             "${FMT8_ROOT}/lib/aarch64-linux-gnu/libfmt.so.8.1.1"
    INTERFACE_INCLUDE_DIRECTORIES "${FMT8_ROOT}/include"
    INTERFACE_COMPILE_DEFINITIONS "FMT_LOCALE;FMT_SHARED"
  )
  set(fmt_FOUND TRUE)  # 阻止 spdlog 内部再次 find_dependency(fmt)
endif()
```

**目录结构**：
```
3party/fmt/8.1.1/
├── include/fmt/      ← v8 头文件
└── lib/aarch64-linux-gnu/
    ├── libfmt.so.8
    └── libfmt.so.8.1.1
```

---

## 2. 启动文件：3588_net_nav.launch.py

**位置**：`src/robot_launch/launch/3588_net_nav.launch.py`

**用途**：纯导航启动，不带 SLAM、不带雷达、不带 CAN 底盘。

**启动的节点**：

| 包 | 可执行文件 |
|---|---|
| `tf2_ros` | `static_transform_publisher` (map→base_link) |
| `nav2_controller` | `controller_server` |
| `nav2_net_planner` | `net_planner_action_server` |
| `nav2_net_system` | `net_navigator_node` |
| `nav2_input_bridge` | `input_bridge_node` |
| `nav2_log_record` | `log_server_node` |
| `nav2_lifecycle_manager` | `lifecycle_manager` |
| `rviz2` | rviz2（可选） |

**启动**：
```bash
source /root/jky_car1/nav/install/setup.bash
ros2 launch robot_launch 3588_net_nav.launch.py use_rviz:=false
```

---

## 3. 其他编译命令

**编译所有（除 driver 目录）**：
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  --packages-skip livox_ros_driver2 ros2can ugv_sdk scout_base scout_description scout_msgs rslidar_sdk rslidar_msg
```

**权限修复**：
```bash
sudo chown -R firefly:firefly /root/jky_car1/nav/build /root/jky_car1/nav/install /root/jky_car1/nav/log
```

---

## 4. 已知问题

- **`ros2 node list` 只显示 4 个节点**：7 个节点实际运行正常（日志确认 lifecycle transitions 成功），生命周期节点 DDS 发现层不可见，不影响功能。
- **`nav2_net_planner` 有 fmt 版本警告**：链接时 warning 但未报错，不影响运行。
- **RViz 无 GPU 崩溃**：无显示器环境 RViz SIGSEGV，启动加 `use_rviz:=false`。
- **`use_sim_time` 默认 false**：3588 launch 已改，无需额外传参。
