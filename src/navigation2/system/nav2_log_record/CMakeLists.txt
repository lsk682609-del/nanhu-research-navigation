# Copyright 2026
# Author: Wei Wang -> shaxikai@outlook.com

cmake_minimum_required(VERSION 3.8)
project(nav2_log_record)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- fmt: 通过 USE_SYSTEM_FMT 切换系统版和本地 v8 ----
# 必须在 find_package(nav2_util) 之前创建 fmt::fmt，
# 因为 nav2_util → rcl_logging_spdlog → spdlog → find_dependency(fmt)
option(USE_SYSTEM_FMT "Use system-installed fmt" OFF)

if(NOT USE_SYSTEM_FMT)
  set(THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../3party"
      CACHE PATH "Third-party directory")
  get_filename_component(THIRD_PARTY_DIR "${THIRD_PARTY_DIR}" ABSOLUTE)
  set(FMT8_ROOT "${THIRD_PARTY_DIR}/fmt/8.1.1")

  # 先手动创建目标，后续 spdlog 的 find_dependency 发现已存在就跳过
  add_library(fmt::fmt SHARED IMPORTED)
  set_target_properties(fmt::fmt PROPERTIES
    IMPORTED_LOCATION             "${FMT8_ROOT}/lib/aarch64-linux-gnu/libfmt.so.8.1.1"
    INTERFACE_INCLUDE_DIRECTORIES "${FMT8_ROOT}/include"
    INTERFACE_COMPILE_DEFINITIONS "FMT_LOCALE;FMT_SHARED"
  )
  set(fmt_FOUND TRUE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(spdlog REQUIRED)

if(USE_SYSTEM_FMT)
  find_package(fmt REQUIRED)
endif()

add_executable(log_server_node
  src/main.cpp
  src/log_server_node.cpp
)

target_include_directories(log_server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(log_server_node
  rclcpp
  rclcpp_lifecycle
  nav2_msgs
  nav2_util
)

target_link_libraries(log_server_node
  spdlog::spdlog
  fmt::fmt
)

# 本地 fmt .so 的运行时搜索路径
if(NOT USE_SYSTEM_FMT)
  set_target_properties(log_server_node PROPERTIES
    BUILD_RPATH "${FMT8_ROOT}/lib/aarch64-linux-gnu"
    INSTALL_RPATH "${FMT8_ROOT}/lib/aarch64-linux-gnu"
  )
endif()

install(TARGETS log_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()