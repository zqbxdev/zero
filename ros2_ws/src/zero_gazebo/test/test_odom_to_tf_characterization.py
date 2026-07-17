from __future__ import annotations

from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


def test_existing_mixed_package_python_install_contract_is_preserved() -> None:
    # Given: the current zero_gazebo mixed CMake/Python package
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When: its Python package and executable install rules are inspected
    package_rule = "ament_python_install_package(${PROJECT_NAME})"
    adapter_rule = "PROGRAMS zero_gazebo/gazebo_drive_adapter.py"

    # Then: the current package and adapter remain installable exactly once
    assert cmake.count(package_rule) == 1
    assert cmake.count(adapter_rule) == 1
    assert cmake.count("RENAME gazebo_drive_adapter") == 1
    assert (PACKAGE_ROOT / "zero_gazebo" / "__init__.py").is_file()


def test_existing_active_tf_paths_do_not_own_odom_to_base_tf() -> None:
    # Given: every current active source/launch/config path related to TF ownership
    model = (PACKAGE_ROOT / "models" / "zero_usv" / "model.sdf").read_text(
        encoding="utf-8",
    )
    bridge = (PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml").read_text(
        encoding="utf-8",
    )
    gazebo_launch = (PACKAGE_ROOT / "launch" / "gazebo_sensors.launch.py").read_text(
        encoding="utf-8",
    )
    display_launch = (
        SOURCE_ROOT / "zero_description" / "launch" / "display.launch.py"
    ).read_text(encoding="utf-8")
    control_launch = (
        SOURCE_ROOT / "zero_bringup" / "launch" / "control_chain.launch.py"
    ).read_text(encoding="utf-8")

    # When: the existing Gazebo, bridge, display, and composed launch boundaries are read
    unbridged_tf_topic = (
        "<tf_topic>/model/zero_usv/diff_drive_tf_unbridged</tf_topic>"
    )

    # Then: Gazebo TF stays unbridged and display-only TF is not composed into control
    assert unbridged_tf_topic in model
    assert "Pose_V" not in bridge
    assert 'ros_topic_name: "/tf"' not in bridge
    assert 'ros_topic_name: "/tf_static"' not in bridge
    assert "static_transform_publisher" not in gazebo_launch
    assert "robot_state_publisher" in gazebo_launch
    assert "static_transform_publisher" in display_launch
    assert "'map'" in display_launch
    assert "'zero_base_link'" in display_launch
    assert "'odom'" not in display_launch
    assert "display.launch.py" not in control_launch
