cmake_minimum_required(VERSION 3.5)
project(g1_loco_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)

set(
  UNITREE_ROS2_EXAMPLE_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_ros2_example_include
)

#
# Backend library.
#
# At this stage it contains only DryRunBackend and has no Unitree
# client or robot-control capability.
#
add_library(
  g1_loco_bridge_backends
  STATIC
    src/dry_run_backend.cpp
    src/unitree_ros2_backend.cpp
)

target_include_directories(
  g1_loco_bridge_backends
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/common
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/nlohmann
)

ament_target_dependencies(
  g1_loco_bridge_backends
  rclcpp
  unitree_api
)

#
# Existing safety-layer dry-run node.
#
# It is intentionally not connected to the new backend library yet.
#
add_executable(
  g1_cmd_vel_dry_run
  src/g1_cmd_vel_dry_run.cpp
)

ament_target_dependencies(
  g1_cmd_vel_dry_run
  rclcpp
  geometry_msgs
  std_srvs
)

target_link_libraries(
  g1_cmd_vel_dry_run
  g1_loco_bridge_backends
)

install(
  TARGETS
    g1_loco_bridge_backends
    g1_cmd_vel_dry_run
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
