cmake_minimum_required(VERSION 3.20)
project(littlebot_control)


find_package(ament_cmake REQUIRED)


install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
