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
find_package(example_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)

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

# Hardware interface
add_library(littlebot_hardware_component SHARED
  src/littlebot_hardware_component.cpp
  src/littlebot_driver.cpp
  src/serial_port.cpp
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

ament_target_dependencies(littlebot_hardware_component
  hardware_interface
  pluginlib
  rclcpp)

# Link libserial
target_link_libraries(littlebot_hardware_component
  ${LIBSERIAL_LIBRARIES}
  ${Protobuf_LIBRARIES}
)
target_include_directories(littlebot_hardware_component PRIVATE
  ${LIBSERIAL_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

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

option(ENABLE_COVERAGE "Enable test coverage reporting" OFF)

if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -O0 -g")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -O0 -g")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")

  find_program(LCOV_PATH lcov)
  find_program(GENHTML_PATH genhtml)
  find_program(GCOVR_PATH gcovr)

  if(LCOV_PATH AND GENHTML_PATH)
    message(STATUS "Coverage reporting enabled (lcov found)")
  elseif(GCOVR_PATH)
    message(STATUS "Coverage reporting enabled (gcovr found)")
  else()
    message(WARNING "Coverage reporting enabled but neither lcov nor gcovr found")
  endif()
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # Lint tests
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_unit_tests
    # test/test_main.cpp
    test/test_littlebot_driver.cpp
    test/test_littlebot_hardware_component.cpp
    test/test_serial_port.cpp
    src/littlebot_driver.cpp
    src/serial_port.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
  )

  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
      include
      test
      ${Protobuf_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}
      ${LIBSERIAL_INCLUDE_DIRS}
    )

    target_link_libraries(${PROJECT_NAME}_unit_tests
      ${Protobuf_LIBRARIES}
      ${LIBSERIAL_LIBRARIES}
    )

    ament_target_dependencies(${PROJECT_NAME}_unit_tests
      hardware_interface
      pluginlib
      rclcpp
    )
  endif()

  if(ENABLE_COVERAGE)
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/coverage)

    if(LCOV_PATH AND GENHTML_PATH)
      add_custom_target(coverage_clean
        COMMAND ${LCOV_PATH} --directory . --zerocounters
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Resetting code coverage counters to zero"
      )

      add_custom_target(coverage_run
        COMMAND ${CMAKE_CTEST_COMMAND} --verbose
        DEPENDS ${PROJECT_NAME}_unit_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Running tests for coverage"
      )

      add_custom_target(coverage_capture
        COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info --ignore-errors mismatch,gcov,unused
        COMMAND ${LCOV_PATH} --remove coverage.info
          '/usr/*'
          '${CMAKE_BINARY_DIR}/_deps/*'
          '*/test/*'
          '*/tests/*'
          '*/.conan/*'
          --output-file coverage_filtered.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS coverage_run
        COMMENT "Processing code coverage counters"
      )

      add_custom_target(coverage_html_lcov
        COMMAND ${GENHTML_PATH} -o coverage coverage_filtered.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        DEPENDS coverage_capture
        COMMENT "Generating HTML coverage report with lcov"
      )

      add_custom_target(coverage_lcov
        DEPENDS coverage_clean coverage_html_lcov
        COMMENT "Generate lcov coverage report"
      )
    endif()

    if(GCOVR_PATH)
      add_custom_target(coverage_gcovr
        COMMAND ${CMAKE_CTEST_COMMAND} --verbose
        COMMAND ${GCOVR_PATH}
          --root ${CMAKE_SOURCE_DIR}
          --exclude '.*/test/.*'
          --exclude '.*/tests/.*'
          --exclude '.*/_deps/.*'
          --exclude '/usr/.*'
          --html --html-details
          --output ${CMAKE_BINARY_DIR}/coverage/index.html
        DEPENDS ${PROJECT_NAME}_unit_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating coverage report with gcovr"
      )

      add_custom_target(coverage_xml
        COMMAND ${CMAKE_CTEST_COMMAND} --verbose
        COMMAND ${GCOVR_PATH}
          --root ${CMAKE_SOURCE_DIR}
          --exclude '.*/test/.*'
          --exclude '.*/tests/.*'
          --exclude '.*/_deps/.*'
          --exclude '/usr/.*'
          --xml --output ${CMAKE_BINARY_DIR}/coverage.xml
        DEPENDS ${PROJECT_NAME}_unit_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating XML coverage report"
      )

      add_custom_target(coverage_summary
        COMMAND ${CMAKE_CTEST_COMMAND} --verbose
        COMMAND ${GCOVR_PATH}
          --root ${CMAKE_SOURCE_DIR}
          --exclude '.*/test/.*'
          --exclude '.*/tests/.*'
          --exclude '.*/_deps/.*'
          --exclude '/usr/.*'
        DEPENDS ${PROJECT_NAME}_unit_tests
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating console coverage summary"
      )
    endif()

    # Default coverage target - prefer lcov if available, otherwise gcovr
    if(LCOV_PATH AND GENHTML_PATH)
      add_custom_target(coverage
        DEPENDS coverage_lcov
        COMMENT "Generate coverage report (using lcov)"
      )
    elseif(GCOVR_PATH)
      add_custom_target(coverage
        DEPENDS coverage_gcovr
        COMMENT "Generate coverage report (using gcovr)"
      )
    endif()
  endif()
endif()

ament_package()
