cmake_minimum_required(VERSION 3.5)
project(g1_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
