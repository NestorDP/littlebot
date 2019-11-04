
cmake_minimum_required(VERSION 2.8.3)
project(littlebot)
find_package(catkin REQUIRED)
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy controller_manager diff_drive_controller
   DEPENDS )
catkin_metapackage()