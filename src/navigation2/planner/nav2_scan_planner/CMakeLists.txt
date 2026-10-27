cmake_minimum_required(VERSION 3.8)
project(nav2_scan_planner)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(${PROJECT_NAME} SHARED
  src/scan_backend.cpp src/core/grid_map.cpp src/core/raycast.cpp
  src/core/dyn_a_star.cpp src/core/uniform_bspline.cpp
  src/core/bspline_optimizer.cpp src/core/gradient_descent_optimizer.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_lifecycle nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS ${PROJECT_NAME} EXPORT export_scan
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES LICENSE NOTICE DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_scan HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 rclcpp rclcpp_lifecycle nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_backend test/test_scan_backend.cpp)
  target_link_libraries(test_scan_backend ${PROJECT_NAME})
endif()
ament_package()
