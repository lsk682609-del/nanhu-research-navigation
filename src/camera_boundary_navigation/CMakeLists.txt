cmake_minimum_required(VERSION 3.16)
project(camera_boundary_navigation)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(camera_boundary_navigation_core
  src/boundary_model.cpp
  src/boundary_controller.cpp
  src/stable_local_planner.cpp
  src/vfh_local_planner.cpp
  src/safety_gate.cpp
)
target_include_directories(camera_boundary_navigation_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(camera_boundary_navigation_core Eigen3::Eigen)

add_executable(camera_boundary_navigation_node
  src/camera_boundary_navigation_node.cpp
)
target_include_directories(camera_boundary_navigation_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
ament_target_dependencies(camera_boundary_navigation_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)
target_link_libraries(camera_boundary_navigation_node camera_boundary_navigation_core)

install(TARGETS
  camera_boundary_navigation_core
  camera_boundary_navigation_node
  EXPORT camera_boundary_navigationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_vfh_local_planner test/test_vfh_local_planner.cpp)
  if(TARGET test_vfh_local_planner)
    target_link_libraries(test_vfh_local_planner camera_boundary_navigation_core)
  endif()
  ament_add_gtest(test_boundary_navigation test/test_boundary_navigation.cpp)
  if(TARGET test_boundary_navigation)
    target_link_libraries(test_boundary_navigation camera_boundary_navigation_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_targets(camera_boundary_navigationTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
