cmake_minimum_required(VERSION 3.8)
project(zero_mapping)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES maps/README.md
  DESTINATION share/${PROJECT_NAME}/maps
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_pytest_test(package_contract test/test_package_contract.py)
  ament_add_pytest_test(slam_toolbox_config test/test_slam_toolbox_config.py)
  ament_add_pytest_test(mapping_rviz test/test_mapping_rviz.py)
  ament_add_pytest_test(map_output_policy test/test_map_output_policy.py)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
