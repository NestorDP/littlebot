cmake_minimum_required(VERSION 3.5)
project(littlebot_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(littlebot_description REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# Find libserial using pkg-config
pkg_check_modules(LIBSERIAL REQUIRED libserial)

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

# Create serial_port library (standalone)
add_library(serial_port SHARED
  src/serial_port.cpp
)

target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(serial_port PUBLIC
  ${LIBSERIAL_LIBRARIES}
)

# Create littlebot_driver library (depends on serial_port and protobuf)
add_library(littlebot_driver SHARED
  src/littlebot_driver.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

target_include_directories(littlebot_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<INSTALL_INTERFACE:include>
  ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(littlebot_driver PUBLIC
  serial_port
  ${Protobuf_LIBRARIES}
)

# Export the libraries so other packages can find them
ament_export_libraries(serial_port)
ament_export_libraries(littlebot_driver)
ament_export_targets(littlebot_driver_targets HAS_LIBRARY_TARGET)

# Hardware interface
add_library(littlebot_hardware_component SHARED
  src/littlebot_hardware_component.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

ament_target_dependencies(littlebot_hardware_component
  hardware_interface
  pluginlib
  rclcpp)

# Link libserial
target_link_libraries(littlebot_hardware_component
  littlebot_driver
  ${Protobuf_LIBRARIES}
)
target_include_directories(littlebot_hardware_component PRIVATE
  include
  ${LIBSERIAL_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

pluginlib_export_plugin_description_file(hardware_interface littlebot_hardware_component.xml)

## INSTALL
install(TARGETS
  serial_port
  littlebot_driver
  littlebot_hardware_component
  EXPORT littlebot_driver_targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install generated protobuf headers so downstream packages can include them
if(PROTO_HDRS)
  install(FILES ${PROTO_HDRS}
    DESTINATION include)
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(littlebot_hardware_component)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  find_file(LITTLEBOT_URDF_FILE
    NAMES littlebot_description.urdf.xacro
    PATHS
      "${CMAKE_CURRENT_SOURCE_DIR}/../littlebot_description/urdf"
    NO_DEFAULT_PATH
  )

  if(NOT LITTLEBOT_URDF_FILE)
    message(WARNING "Could not find littlebot URDF file")
  endif()

  # Find the xacro executable
  find_program(XACRO_EXECUTABLE xacro
    HINTS ENV PATH /usr/bin /usr/local/bin
  )

  if(NOT XACRO_EXECUTABLE)
    message(FATAL_ERROR "xacro executable not found. Install ROS xacro or make
    'xacro' available in PATH.")
  endif()

  # Convert XACRO to URDF for testing
  set(TEST_URDF_FILE ${CMAKE_CURRENT_BINARY_DIR}/test_littlebot.urdf)

  add_custom_command(
    OUTPUT ${TEST_URDF_FILE}
    COMMAND ${XACRO_EXECUTABLE} ${LITTLEBOT_URDF_FILE} -o ${TEST_URDF_FILE}
    DEPENDS ${LITTLEBOT_URDF_FILE}
    BYPRODUCTS ${TEST_URDF_FILE}
    COMMENT "Converting XACRO to URDF for testing"
  )

  # Add custom target for the URDF generation
  add_custom_target(generate_test_urdf DEPENDS ${TEST_URDF_FILE})

  # Lint tests
  ament_lint_auto_find_test_dependencies()

  # Unit tests for FirmwareComm
  ament_add_gtest(${PROJECT_NAME}_unit_tests
    test/test_littlebot_driver.cpp
    test/test_littlebot_hardware_component.cpp
    test/test_serial_port.cpp
  )

  # # Make test depend on URDF generation
  if(TARGET generate_test_urdf)
    add_dependencies(${PROJECT_NAME}_unit_tests generate_test_urdf)
  endif()

  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
      include
      test
      ${Protobuf_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}
      ${LIBSERIAL_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME}_unit_tests
      littlebot_hardware_component
      littlebot_driver
      serial_port
      ${Protobuf_LIBRARIES}
    )
  endif()
endif()

ament_package()
