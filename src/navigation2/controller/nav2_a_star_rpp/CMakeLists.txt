cmake_minimum_required(VERSION 3.5)
project(nav2_a_star_rpp)

# ---- fmt: 强制使用本地 v8，避免与 spdlog 的 libfmt.so.8 冲突 ----
# 必须在 find_package(nav2_util) 之前创建 fmt::fmt，
# 因为 nav2_util → rcl_logging_spdlog → spdlog → find_dependency(fmt)
option(USE_SYSTEM_FMT "Use system-installed fmt" OFF)

if(NOT USE_SYSTEM_FMT)
  set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../3party"
      CACHE PATH "Third-party directory")
  get_filename_component(THIRD_PARTY_DIR "${THIRD_PARTY_DIR}" ABSOLUTE)
  set(FMT8_ROOT "${THIRD_PARTY_DIR}/fmt/8.1.1")

  add_library(fmt::fmt SHARED IMPORTED)
  set_target_properties(fmt::fmt PROPERTIES
    IMPORTED_LOCATION             "${FMT8_ROOT}/lib/aarch64-linux-gnu/libfmt.so.8.1.1"
    INTERFACE_INCLUDE_DIRECTORIES "${FMT8_ROOT}/include"
    INTERFACE_COMPILE_DEFINITIONS "FMT_LOCALE;FMT_SHARED"
  )
  set(fmt_FOUND TRUE)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_scan_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(USE_SYSTEM_FMT)
  find_package(fmt REQUIRED)
endif()

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(
  include
)

set(dependencies
  nav2_common
  rclcpp
  std_msgs
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  nav2_scan_planner
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  sensor_msgs
)

set(library_name nav2_a_star_rpp)

add_library(${library_name} SHARED
        src/costmap_path_smoother.cpp
        src/regulated_pure_pursuit_controller.cpp
        src/theta_star.cpp)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${library_name}
  fmt::fmt
)

# 本地 fmt .so 的运行时搜索路径
if(NOT USE_SYSTEM_FMT)
  set_target_properties(${library_name} PROPERTIES
    BUILD_RPATH "${FMT8_ROOT}/lib/aarch64-linux-gnu"
    INSTALL_RPATH "${FMT8_ROOT}/lib/aarch64-linux-gnu"
  )
endif()

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core nav2_a_star_rpp.xml)

ament_package()
