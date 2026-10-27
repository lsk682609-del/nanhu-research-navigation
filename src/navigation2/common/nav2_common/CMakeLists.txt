cmake_minimum_required(VERSION 3.5)

project(nav2_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(nav2_common)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package(
  CONFIG_EXTRAS "nav2_common-extras.cmake"
)