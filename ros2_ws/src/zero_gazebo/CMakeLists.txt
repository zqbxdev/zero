cmake_minimum_required(VERSION 3.8)
project(zero_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS zero_gazebo/gazebo_drive_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gazebo_drive_adapter
)

install(
  PROGRAMS zero_gazebo/odom_to_tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_to_tf_relay
)

install(
  DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_pytest_test(zero_gazebo_static test)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
