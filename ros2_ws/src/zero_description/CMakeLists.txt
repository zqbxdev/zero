cmake_minimum_required(VERSION 3.8)
project(zero_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_pytest_test(sensor_frames test/test_sensor_frames.py)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
