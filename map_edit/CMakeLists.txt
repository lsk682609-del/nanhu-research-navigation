cmake_minimum_required(VERSION 3.16)
project(map_edit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(map_edit
  src/main.cpp
  src/MainWindow.cpp
  src/TopoEditorView.cpp
  src/snn_topo_planner.cpp
  src/region_io.cpp
  src/pgm_io.cpp
  src/map_yaml.cpp
  include/MainWindow.hpp
  include/TopoEditorView.hpp
  include/snn_topo_planner.hpp
)

target_include_directories(map_edit PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(map_edit
  Qt${QT_VERSION_MAJOR}::Widgets
)

ament_target_dependencies(map_edit
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
)
