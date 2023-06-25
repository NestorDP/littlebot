cmake_minimum_required(VERSION 3.5)
project(littlebot_navigation2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)

ament_package()