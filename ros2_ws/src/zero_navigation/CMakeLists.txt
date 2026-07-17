cmake_minimum_required(VERSION 3.8)
project(zero_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_add_pytest_test(package_contract test/test_package_contract.py)
  ament_add_pytest_test(nav2_params test/test_nav2_params.py)
  ament_add_pytest_test(navigation_ownership test/test_navigation_ownership.py)
  ament_add_pytest_test(navigation_rviz test/test_navigation_rviz.py)
  ament_add_pytest_test(resource_visibility test/test_resource_visibility.py)
  ament_add_pytest_test(static_registration test/test_static_registration.py)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
