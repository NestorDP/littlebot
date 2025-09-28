cmake_minimum_required(VERSION 3.5)
project(littlebot_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Find libserial or build it if not found
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL libserial)

if(NOT LIBSERIAL_FOUND)
  message(STATUS "libserial not found, checking for library file...")
  find_library(LIBSERIAL_LIBRARY 
    NAMES serial libserial
    PATHS /usr/local/lib /usr/lib
    DOC "libserial library"
  )
  
  find_path(LIBSERIAL_INCLUDE_DIR
    NAMES libserial/SerialPort.h serial/SerialPort.h
    PATHS /usr/local/include /usr/include
    DOC "libserial include directory"
  )
  
  if(LIBSERIAL_LIBRARY AND LIBSERIAL_INCLUDE_DIR)
    message(STATUS "Found libserial library: ${LIBSERIAL_LIBRARY}")
    message(STATUS "Found libserial headers: ${LIBSERIAL_INCLUDE_DIR}")
    set(LIBSERIAL_LIBRARIES ${LIBSERIAL_LIBRARY})
    set(LIBSERIAL_INCLUDE_DIRS ${LIBSERIAL_INCLUDE_DIR})
  else()
    message(FATAL_ERROR 
      "libserial not found! Please install it using:\n"
      "  Option 1: Run the installation script: ./install_dependencies.sh\n"
      "  Option 2: Install manually from: https://github.com/NestorDP/libserial\n"
      "  Option 3: Use vcstool: vcs import src < littlebot.repos"
    )
  endif()
endif()
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(include)

# Print some protobuf info
message(STATUS "Using Protocol Buffers: ${Protobuf_VERSION}")
message(STATUS "Protobuf include directory: ${Protobuf_INCLUDE_DIRS}")
message(STATUS "Protobuf libraries: ${Protobuf_LIBRARIES}")

# Add your proto file
set(PROTO_FILES
    src/littlebot_msg.proto
)

# Generate sources and headers from .proto
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# Add include directories
include_directories(
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # This is where generated files will be
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# Hardware interface
add_library(littlebot_hardware_component SHARED
  src/littlebot_hardware_component.cpp
  src/littlebot_communication.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
  )

ament_target_dependencies(littlebot_hardware_component
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(hardware_interface littlebot_hardware_component.xml)

## INSTALL
install( TARGETS
  littlebot_hardware_component
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install( 
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  littlebot_hardware_component
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
