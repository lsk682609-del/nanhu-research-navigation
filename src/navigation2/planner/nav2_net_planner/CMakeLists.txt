cmake_minimum_required(VERSION 3.8)
project(nav2_net_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fmt REQUIRED)
find_package(nav2_common REQUIRED)

set(library_name nav2_net_planner)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  nav2_util
  nav2_core
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav2_common
)

add_library(${library_name} SHARED src/net_planner.cpp src/snn_topo_planner.cpp)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${library_name} ${dependencies})
pluginlib_export_plugin_description_file(nav2_core global_planner_plugin.xml)

add_executable(net_planner_action_server src/net_planner_action_server.cpp)
target_include_directories(net_planner_action_server PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(net_planner_action_server ${dependencies})
target_link_libraries(net_planner_action_server ${library_name} fmt::fmt)

install(TARGETS ${library_name} net_planner_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES global_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_topo_map test/test_topo_map.cpp)
  target_include_directories(test_topo_map PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
