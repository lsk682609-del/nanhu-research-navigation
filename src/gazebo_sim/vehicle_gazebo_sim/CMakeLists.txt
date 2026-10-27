cmake_minimum_required(VERSION 3.8)
project(vehicle_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
