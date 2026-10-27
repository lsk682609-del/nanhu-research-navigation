cmake_minimum_required(VERSION 3.8)
project(nav2_input_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

add_executable(input_bridge_node
  src/main.cpp
  src/input_bridge_node.cpp
)

ament_target_dependencies(input_bridge_node
  geometry_msgs
  nav_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  geographic_msgs
)

target_include_directories(input_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(input_bridge_node
  fmt::fmt
)

install(TARGETS input_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  geometry_msgs
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  geographic_msgs
)

ament_package()
