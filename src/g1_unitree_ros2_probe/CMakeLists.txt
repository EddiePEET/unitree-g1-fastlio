cmake_minimum_required(VERSION 3.5)
project(g1_unitree_ros2_probe)

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
find_package(unitree_api REQUIRED)

set(
  UNITREE_ROS2_EXAMPLE_INCLUDE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/unitree_ros2_example_include
)

add_executable(
  g1_unitree_ros2_status_probe
  src/g1_unitree_ros2_status_probe.cpp
)

add_executable(
  g1_unitree_ros2_stop_probe
  src/g1_unitree_ros2_stop_probe.cpp
)

foreach(target
  g1_unitree_ros2_status_probe
  g1_unitree_ros2_stop_probe
)
  target_include_directories(
    ${target}
    PRIVATE
      ${UNITREE_ROS2_EXAMPLE_INCLUDE}
      ${UNITREE_ROS2_EXAMPLE_INCLUDE}/common
      ${UNITREE_ROS2_EXAMPLE_INCLUDE}/nlohmann
  )

  ament_target_dependencies(
    ${target}
    rclcpp
    unitree_api
  )
endforeach()

install(
  TARGETS
    g1_unitree_ros2_status_probe
    g1_unitree_ros2_stop_probe
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  g1_unitree_ros2_start_probe
  src/g1_unitree_ros2_start_probe.cpp
)

target_include_directories(
  g1_unitree_ros2_start_probe
  PRIVATE
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/common
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/nlohmann
)

ament_target_dependencies(
  g1_unitree_ros2_start_probe
  rclcpp
  unitree_api
)

install(
  TARGETS g1_unitree_ros2_start_probe
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  g1_unitree_ros2_velocity_probe
  src/g1_unitree_ros2_velocity_probe.cpp
)

target_include_directories(
  g1_unitree_ros2_velocity_probe
  PRIVATE
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/common
    ${UNITREE_ROS2_EXAMPLE_INCLUDE}/nlohmann
)

ament_target_dependencies(
  g1_unitree_ros2_velocity_probe
  rclcpp
  unitree_api
)

install(
  TARGETS
    g1_unitree_ros2_velocity_probe
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
