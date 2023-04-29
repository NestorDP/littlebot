  cmake_minimum_required(VERSION 3.5)
  project(littlebot_description)
  
  find_package(ament_cmake REQUIRED)
  
  install(
    DIRECTORY config gazebo launch meshes ros2_control urdf
    DESTINATION share/${PROJECT_NAME}
  )
  
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
  endif()
  
  ament_package()
  