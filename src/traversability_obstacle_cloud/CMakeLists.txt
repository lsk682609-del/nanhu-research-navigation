cmake_minimum_required(VERSION 3.16)
project(traversability_obstacle_cloud)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(depth_cloud_projector_node
  src/depth_cloud_projector_node.cpp
)

add_executable(rgb_image_decoder_node
  src/rgb_image_decoder_node.cpp
)
target_include_directories(rgb_image_decoder_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(rgb_image_decoder_node
  rclcpp
  sensor_msgs
)
target_link_libraries(rgb_image_decoder_node ${OpenCV_LIBRARIES})
target_include_directories(depth_cloud_projector_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(depth_cloud_projector_node
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(depth_cloud_projector_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(traversability_obstacle_cloud_node
  src/traversability_obstacle_cloud_node.cpp
)
target_include_directories(traversability_obstacle_cloud_node PRIVATE
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(traversability_obstacle_cloud_node
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(traversability_obstacle_cloud_node
  Eigen3::Eigen
  "${cpp_typesupport_target}"
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(camera_costmap_cloud_node
  src/camera_costmap_cloud_node.cpp
)
target_include_directories(camera_costmap_cloud_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(camera_costmap_cloud_node
  pcl_conversions
  rclcpp
  sensor_msgs
)
target_link_libraries(camera_costmap_cloud_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# The bundled RKNN Runtime is an AArch64 library. Keep the rest of this package
# buildable on development PCs and build the detector only for RK3588-class targets.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(RKNN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rknn")
  set(RKNN_INCLUDE_DIR "${RKNN_ROOT}/include")
  set(RKNN_LIBRARY "${RKNN_ROOT}/lib/aarch64/librknnrt.so")
  if(NOT EXISTS "${RKNN_INCLUDE_DIR}/rknn_api.h" OR NOT EXISTS "${RKNN_LIBRARY}")
    message(FATAL_ERROR "Bundled RKNN headers or AArch64 runtime are missing")
  endif()

  add_library(rknnrt SHARED IMPORTED GLOBAL)
  set_target_properties(rknnrt PROPERTIES
    IMPORTED_LOCATION "${RKNN_LIBRARY}"
    IMPORTED_SONAME "librknnrt.so"
  )
  add_library(sbp_yolo_engine STATIC src/sbp_yolo_engine.cpp)
  target_include_directories(sbp_yolo_engine
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    PRIVATE
      "${RKNN_INCLUDE_DIR}"
      ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(sbp_yolo_engine
    PUBLIC rknnrt ${OpenCV_LIBRARIES} ${CMAKE_DL_LIBS})

  add_executable(pothole_detector_node src/pothole_detector_node.cpp)
  target_include_directories(pothole_detector_node PRIVATE
    include "${RKNN_INCLUDE_DIR}" ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(pothole_detector_node
    ament_index_cpp cv_bridge rclcpp sensor_msgs)
  target_link_libraries(pothole_detector_node
    sbp_yolo_engine "${cpp_typesupport_target}" ${OpenCV_LIBRARIES})
  set_target_properties(pothole_detector_node PROPERTIES
    BUILD_RPATH "${RKNN_ROOT}/lib/aarch64"
    INSTALL_RPATH "$ORIGIN/.."
  )
  install(TARGETS pothole_detector_node DESTINATION lib/${PROJECT_NAME})
  install(FILES "${RKNN_LIBRARY}" DESTINATION lib)
endif()

install(TARGETS
  camera_costmap_cloud_node
  depth_cloud_projector_node
  rgb_image_decoder_node
  traversability_obstacle_cloud_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/record_obstacle_events.py
  scripts/extract_bag_color_images.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
