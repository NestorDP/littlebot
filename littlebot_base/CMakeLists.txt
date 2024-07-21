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
  "std_msgs"
  "example_interfaces")
  
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
  "std_msgs"
  "example_interfaces")

rclcpp_components_register_nodes(littlebot_base_reader "littlebot_base::Reader")
set(node_plugins "${node_plugins}littlebot_base::Reader;$<TARGET_FILE:reader>\n")


# Compostion for reader and writer components
add_executable(littlebot_base_composition
  src/littlebot_base_composition.cpp)
  
target_link_libraries(littlebot_base_composition
  littlebot_base_writer
  littlebot_base_reader)

ament_target_dependencies(littlebot_base_composition
  "rclcpp")


# Hardware interface
add_library(littlebot_hardware SHARED
  src/littlebot_hardware.cpp)

ament_target_dependencies( littlebot_hardware
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(hardware_interface littlebot_hardware.xml)

## INSTALL
install( TARGETS
  littlebot_hardware
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  littlebot_base_writer
  littlebot_base_reader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  littlebot_base_composition
  DESTINATION lib/${PROJECT_NAME})

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
  littlebot_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()






























cmake_minimum_required(VERSION 3.8)
project(bravo_seven_driver)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  driver_base
  hardware_interface
  jiro_lifecycle_manager
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros_common_interfaces
  ros2_control_toolbox
  std_msgs
)

foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(bravo_seven_udp_communication SHARED
  src/bravo_seven_udp_communication.cpp
  src/packet_helper.cpp
)

target_include_directories(bravo_seven_udp_communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bravo_seven_udp_communication PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(bravo_seven_driver
  bravo_seven_udp_communication.xml
)



add_library(bravo_seven_hardware_component SHARED
  src/bravo_seven_hardware_component.cpp
)

target_include_directories(bravo_seven_hardware_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bravo_seven_hardware_component PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface
  bravo_seven_hardware_plugin.xml
)


install(
  TARGETS bravo_seven_udp_communication
  EXPORT export_bravo_seven_udp_communication
  LIBRARY DESTINATION lib
)

install(
  TARGETS bravo_seven_hardware_component
  EXPORT export_bravo_seven_hardware_component
  LIBRARY DESTINATION lib
)

if(BUILD_DOCS)
  find_package(ament_doxygen REQUIRED)
  ament_doxygen(DOC_TITLE ${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Next lines implements a local copyrigth checker
  set(ament_cmake_copyright_FOUND TRUE)
  set(DESCRIPTIONS_PYTHON_TESTS test/test_copyright.py)
  get_filename_component(_test_name ${DESCRIPTIONS_PYTHON_TESTS} NAME_WE)

  ament_add_pytest_test(${_test_name} ${DESCRIPTIONS_PYTHON_TESTS}
    PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}"
    APPEND_ENV AMENT_PREFIX_PATH=$ENV{AMENT_PREFIX_PATH}:${CMAKE_INSTALL_PREFIX}
    PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 120
    WERROR ON
  )

  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(MAX_LINE_LENGTH "120")
  set(AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS True)

  ament_add_gmock(test_bravo_seven_hardware
    test/test_bravo_seven_hardware_component.cpp
  )

  target_link_libraries(test_bravo_seven_hardware
    bravo_seven_hardware_component
  )

  ament_target_dependencies(test_bravo_seven_hardware
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  ament_add_gtest(test_bravo_seven_udp_communication
    test/test_bravo_seven_udp_communication.cpp
  )

  ament_target_dependencies(test_bravo_seven_udp_communication
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  target_link_libraries(test_bravo_seven_udp_communication
    bravo_seven_udp_communication
  )

  add_library(fake_bravo_seven_communication SHARED
    test/fake_communication/fake_bravo_seven_communication.cpp
  )

  target_include_directories(fake_bravo_seven_communication PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:test>
  )

  ament_target_dependencies(fake_bravo_seven_communication
    pluginlib
  )

  install(
    TARGETS fake_bravo_seven_communication
    EXPORT export_fake_bravo_seven_communication
    LIBRARY DESTINATION lib
  )

  ament_export_libraries(
    fake_bravo_seven_communication
  )

  ament_export_targets(
    export_fake_bravo_seven_communication
  )

  pluginlib_export_plugin_description_file(bravo_seven_driver
    test/fake_communication/fake_bravo_seven_communication.xml
  )

  ament_add_gmock(test_bravo_seven_communication_loader
    test/test_bravo_seven_communication_loader.cpp
  )

  ament_target_dependencies(test_bravo_seven_communication_loader
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )

  target_link_libraries(test_bravo_seven_communication_loader
    fake_bravo_seven_communication
  )

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_libraries(
  bravo_seven_hardware_component
  bravo_seven_udp_communication
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_bravo_seven_hardware_component
  export_bravo_seven_udp_communication
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()