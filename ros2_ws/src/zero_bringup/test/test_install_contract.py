from __future__ import annotations

import ast
from pathlib import Path
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
EXPECTED_TEST_DEPENDENCIES = {
    "ament_lint_auto",
    "ament_lint_common",
    "python3-pytest",
    "python3-yaml",
}


def test_existing_bringup_resources_remain_installed() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the package marker, manifest, and existing launch remain installed.
    assert 'f"resource/{package_name}"' in setup_source
    assert '["package.xml"]' in setup_source
    assert '"launch/bringup_fake.launch.py"' in setup_source
    assert setup_source.count('"launch/bringup_fake.launch.py"') == 1


def test_v1_sim_config_is_installed() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the canonical V1 YAML is installed under the package config share.
    assert 'f"share/{package_name}/config"' in setup_source
    assert '["config/v1_sim.yaml"]' in setup_source


def test_control_chain_launch_is_installed_once() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the reusable composition is installed once beside fake bringup.
    assert '"launch/control_chain.launch.py"' in setup_source
    assert setup_source.count('"launch/control_chain.launch.py"') == 1


def test_gazebo_control_launch_is_installed_once() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the sole public simulation composition is installed exactly once.
    assert '"launch/gazebo_control.launch.py"' in setup_source
    assert setup_source.count('"launch/gazebo_control.launch.py"') == 1


def test_gazebo_mapping_launch_is_installed_once() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the sole public mapping composition is installed exactly once.
    assert '"launch/gazebo_mapping.launch.py"' in setup_source
    assert setup_source.count('"launch/gazebo_mapping.launch.py"') == 1


def test_gazebo_navigation_launch_is_installed_once() -> None:
    # Given: the source-backed ament_python setup declaration.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")

    # When: the declaration is parsed without executing setup().
    _ = ast.parse(setup_source, filename="setup.py")

    # Then: the required-map navigation composition is installed exactly once.
    assert '"launch/gazebo_navigation.launch.py"' in setup_source
    assert setup_source.count('"launch/gazebo_navigation.launch.py"') == 1


def test_test_map_fixtures_are_never_installed() -> None:
    # Given: setup metadata and all source-side synthetic map fixtures.
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    test_maps = tuple((PACKAGE_ROOT / "test").rglob("*.yaml"))

    # When/Then: tests own no persistent maps and setup installs no test tree.
    assert test_maps == ()
    assert '"test/' not in setup_source
    assert "'test/" not in setup_source
    assert 'find_packages(exclude=["test", "test.*"])' in setup_source


def test_bringup_runtime_dependencies_are_exact() -> None:
    # Given: zero_bringup's source package manifest.
    package_root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()

    # When: runtime dependencies are collected from package metadata.
    dependencies = {
        element.text for element in package_root.findall("exec_depend")
    }

    # Then: only the launch stack and direct graph/resource owners remain.
    assert dependencies == {
        "ament_index_python",
        "launch",
        "launch_ros",
        "nav2_bringup",
        "robot_state_publisher",
        "ros_gz_bridge",
        "ros_gz_sim",
        "rviz2",
        "slam_toolbox",
        "zero_control",
        "zero_description",
        "zero_gazebo",
        "zero_hardware",
        "zero_mapping",
        "zero_navigation",
        "zero_safety",
    }


def test_bringup_test_dependencies_are_exact_and_unique() -> None:
    # Given: zero_bringup's package-native pytest and lint metadata.
    package_root = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()

    # When: test dependencies are read without invoking setup or ROS tooling.
    dependencies = [element.text for element in package_root.findall("test_depend")]

    # Then: pytest/YAML contracts and package lint are explicit and nonduplicated.
    assert set(dependencies) == EXPECTED_TEST_DEPENDENCIES
    assert len(dependencies) == len(set(dependencies))


def test_test_dependency_contract_rejects_missing_or_duplicate_fixture() -> None:
    # Given: source-derived dependency lists with one missing or duplicate member.
    dependencies = sorted(EXPECTED_TEST_DEPENDENCIES)
    fixtures = (dependencies[:-1], [*dependencies, dependencies[0]])

    # When/Then: neither malformed package-native test contract is acceptable.
    for fixture in fixtures:
        rejected = False
        try:
            assert set(fixture) == EXPECTED_TEST_DEPENDENCIES
            assert len(fixture) == len(set(fixture))
        except AssertionError:
            rejected = True
        assert rejected
