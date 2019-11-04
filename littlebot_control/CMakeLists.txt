cmake_minimum_required(VERSION 2.8.3)
project(littlebot_control)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS controller_manager diff_drive_controller)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
