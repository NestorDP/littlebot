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


# Littlebot base writer
add_library(littlebot_base_writer SHARED
  src/writer.cpp
)

target_link_libraries( littlebot_base_writer
  serial
)

target_compile_definitions(littlebot_base_writer
  PRIVATE "LITTLEBOT_BASE_CPP_BUILDING_DLL")

ament_target_dependencies(littlebot_base_writer
  "rclcpp"
  "rclcpp_components"
  "std_msgs")
  
rclcpp_components_register_nodes(littlebot_base_writer "littlebot_base::Writer")
set(node_plugins "${node_plugins}littlebot_base::Writer;$<TARGET_FILE:writer>\n")


# Littlebot Base Reader
add_library(littlebot_base_reader SHARED
  src/reader.cpp)

target_link_libraries(littlebot_base_reader
  serial
)

target_compile_definitions(littlebot_base_reader
  PRIVATE "LITTLEBOT_NODES_CPP_BUILDING_DLL")

ament_target_dependencies(littlebot_base_reader
  "rclcpp"
  "rclcpp_components"
  "std_msgs")

rclcpp_components_register_nodes(littlebot_base_reader "littlebot_base::Reader")
set(node_plugins "${node_plugins}littlebot_base::Reader;$<TARGET_FILE:reader>\n")


add_executable(littlebot_base_composition
  src/littlebot_base_composition.cpp)
  
target_link_libraries(littlebot_base_composition
  littlebot_base_writer
  littlebot_base_reader)

ament_target_dependencies(littlebot_base_composition
  "rclcpp")

install(TARGETS
  littlebot_base_writer
  littlebot_base_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


install(TARGETS
  littlebot_base_composition
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
