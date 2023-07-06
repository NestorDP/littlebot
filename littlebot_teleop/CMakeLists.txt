cmake_minimum_required(VERSION 3.5)
project(littlebot_teleop)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# Littlebot teleop
add_library(littlebot_teleop SHARED
  src/teleop.cpp
)

target_compile_definitions(littlebot_teleop
  PRIVATE "LITTLEBOT_TELEOP_CPP_BUILDING_DLL")

ament_target_dependencies(littlebot_teleop
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "sensor_msgs"
)
  
rclcpp_components_register_nodes(littlebot_teleop "littlebot_teleop::Teleop")
set(node_plugins "${node_plugins}littlebot_teleop::Teleop;$<TARGET_FILE:teleop>\n")


## INSTALL
install(TARGETS
  littlebot_teleop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install( 
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
