cmake_minimum_required(VERSION 3.8)
project(plan_env)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(plan_env src/grid_map.cpp src/raycast.cpp)
target_include_directories(plan_env PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(plan_env
  cv_bridge message_filters nav_msgs pcl_conversions rclcpp sensor_msgs
)
target_link_libraries(plan_env Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(grid_map_node src/grid_map_node.cpp)
target_link_libraries(grid_map_node plan_env)
ament_target_dependencies(grid_map_node rclcpp)

add_executable(camera_extrinsic_calibrator src/camera_extrinsic_calibrator.cpp)
target_include_directories(camera_extrinsic_calibrator PRIVATE ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_extrinsic_calibrator Eigen3::Eigen ${PCL_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(camera_extrinsic_calibrator
  cv_bridge pcl_conversions rclcpp sensor_msgs)

install(TARGETS plan_env EXPORT export_plan_env
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS grid_map_node camera_extrinsic_calibrator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/plan_env_compressed_decoder
  scripts/plan_env_drivable_inference
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY python/plan_env_perception DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(FILES README.md requirements-conda.txt DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_plan_env HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge Eigen3 message_filters nav_msgs pcl_conversions rclcpp sensor_msgs)
ament_package()
