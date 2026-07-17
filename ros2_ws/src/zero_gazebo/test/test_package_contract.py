import ast
from pathlib import Path
from sys import stdlib_module_names
from typing import Final
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
THIRD_PARTY_TEST_IMPORT_DEPENDENCIES: Final = (
    ("pytest", "ament_cmake_pytest"),
    ("yaml", "python3-yaml"),
)
REQUIRED_RUNTIME_DEPENDENCIES = frozenset(
    {
        "geometry_msgs",
        "nav_msgs",
        "rclpy",
        "tf2_ros",
        "zero_control",
        "zero_interfaces",
    }
)


def test_existing_ament_cmake_build_type_is_preserved() -> None:
    # Given: the zero_gazebo package manifest
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When: the exported build type is read
    build_type = manifest.findtext("./export/build_type")

    # Then: the package remains an ament_cmake package
    assert build_type == "ament_cmake"


def test_existing_resource_directories_are_installed() -> None:
    # Given: the zero_gazebo CMake configuration
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: the resource install rule is inspected
    resource_install_rule = "DIRECTORY config launch models worlds"

    # Then: all existing resource directories keep their shared install rule
    assert resource_install_rule in cmake
    assert "DESTINATION share/${PROJECT_NAME}" in cmake


def test_python_package_and_static_test_dependencies_are_declared_once() -> None:
    # Given: the zero_gazebo manifest and CMake configuration
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: mixed-package declarations and direct test imports are inspected
    buildtool_dependencies = [
        element.text for element in manifest.findall("buildtool_depend")
    ]
    runtime_dependencies = [element.text for element in manifest.findall("exec_depend")]
    test_dependencies = [element.text for element in manifest.findall("test_depend")]
    direct_import_roots: set[str] = set()
    for test_path in sorted((PACKAGE_ROOT / "test").rglob("*.py")):
        tree = ast.parse(test_path.read_text(encoding="utf-8"), filename=str(test_path))
        nodes = tuple(ast.walk(tree))
        direct_import_roots.update(
            alias.name.partition(".")[0]
            for node in nodes
            if isinstance(node, ast.Import)
            for alias in node.names
        )
        direct_import_roots.update(
            node.module.partition(".")[0]
            for node in nodes
            if isinstance(node, ast.ImportFrom)
            and node.level == 0
            and node.module is not None
        )
    third_party_import_roots = {
        import_root
        for import_root in direct_import_roots
        if import_root not in stdlib_module_names
        and not import_root.startswith("zero_")
    }

    # Then: package installation and direct third-party test imports are explicit
    assert buildtool_dependencies.count("ament_cmake_python") == 1
    assert cmake.count("find_package(ament_cmake_python REQUIRED)") == 1
    assert cmake.count("ament_python_install_package(${PROJECT_NAME})") == 1
    assert cmake.count("find_package(ament_cmake_pytest REQUIRED)") == 1
    assert cmake.count("ament_add_pytest_test(zero_gazebo_static test)") == 1
    assert third_party_import_roots == {
        import_root for import_root, _ in THIRD_PARTY_TEST_IMPORT_DEPENDENCIES
    }
    for _, dependency in THIRD_PARTY_TEST_IMPORT_DEPENDENCIES:
        assert test_dependencies.count(dependency) == 1
        assert runtime_dependencies.count(dependency) == 0


def test_required_runtime_dependencies_are_declared_once() -> None:
    # Given: the zero_gazebo package manifest
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When: runtime dependency declarations are counted
    runtime_dependencies = [element.text for element in manifest.findall("exec_depend")]

    # Then: missing and duplicate required dependencies are rejected
    for dependency in REQUIRED_RUNTIME_DEPENDENCIES:
        assert runtime_dependencies.count(dependency) == 1


def test_python_package_module_is_ready_for_installation() -> None:
    # Given: the zero_gazebo package source and CMake configuration
    module = PACKAGE_ROOT / "zero_gazebo" / "__init__.py"
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: the installable Python package contract is inspected
    module_source = module.read_text(encoding="utf-8")

    # Then: a typed package module is installed through ament_cmake_python
    assert "from __future__ import annotations" in module_source
    assert "__all__: tuple[str, ...] = ()" in module_source
    assert cmake.count("ament_python_install_package(${PROJECT_NAME})") == 1


def test_gazebo_drive_adapter_executable_is_declared_for_installation() -> None:
    # Given: the mixed-package CMake configuration and node source
    executable = PACKAGE_ROOT / "zero_gazebo" / "gazebo_drive_adapter.py"
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: the executable install contract is inspected
    expected_source = "PROGRAMS zero_gazebo/gazebo_drive_adapter.py"

    # Then: ros2 run can discover the adapter from lib/zero_gazebo
    assert executable.is_file()
    assert expected_source in cmake
    assert "DESTINATION lib/${PROJECT_NAME}" in cmake
    assert "RENAME gazebo_drive_adapter" in cmake
