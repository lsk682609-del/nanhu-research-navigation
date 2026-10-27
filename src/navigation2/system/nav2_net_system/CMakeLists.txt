cmake_minimum_required(VERSION 3.8)
project(nav2_net_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(fmt REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(net_navigator_node
  src/main.cpp
  src/net_navigator.cpp
)

target_include_directories(net_navigator_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

ament_target_dependencies(net_navigator_node
  nav2_common
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  std_msgs
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(net_navigator_node
  fmt::fmt
)

install(TARGETS net_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  lifecycle_msgs
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  std_msgs
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_package()
