cmake_minimum_required(VERSION 3.5)
project(littlebot_base)

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
find_package(std_msgs REQUIRED)

include_directories(include)


# create ament index resource which references the libraries in the binary dir
set(node_plugins "")


# Littlebot Listener
add_library(littlebot_listener SHARED
  src/littlebot_listener.cpp
  src/littlebot_communication_protocol.cpp)

target_link_libraries( littlebot_listener
  serial
)

target_compile_definitions(littlebot_listener
  PRIVATE "LITTLEBOT_BASE_CPP_BUILDING_DLL")

ament_target_dependencies(littlebot_listener
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
  
rclcpp_components_register_nodes(littlebot_listener "littlebot_base::Listener")
set(node_plugins "${node_plugins}littlebot_base::Listener;$<TARGET_FILE:littlebot_listener>\n")


install(TARGETS
  littlebot_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
