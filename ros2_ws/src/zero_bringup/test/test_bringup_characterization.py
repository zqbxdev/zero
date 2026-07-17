from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent


def test_bringup_fake_remains_one_fake_hardware_node() -> None:
    # Given: the existing fake-hardware launch source.
    launch_source = (PACKAGE_ROOT / "launch" / "bringup_fake.launch.py").read_text(
        encoding="utf-8"
    )

    # When: its node declarations are inspected without importing ROS modules.
    launch_tree = ast.parse(launch_source, filename="bringup_fake.launch.py")
    node_calls = [
        node
        for node in ast.walk(launch_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]

    # Then: the established launch still owns exactly one fake controller.
    assert len(node_calls) == 1
    keyword_values = {
        keyword.arg: keyword.value
        for keyword in node_calls[0].keywords
        if keyword.arg is not None
    }
    assert ast.literal_eval(keyword_values["package"]) == "zero_hardware"
    assert ast.literal_eval(keyword_values["executable"]) == "fake_motor_controller"
    assert ast.literal_eval(keyword_values["name"]) == "fake_motor_controller"


def test_authoritative_robot_urdf_is_installed_and_read_from_package_share() -> None:
    # Given: zero_description's install declaration and display launch source.
    description_root = SOURCE_ROOT / "zero_description"
    cmake_source = (description_root / "CMakeLists.txt").read_text(encoding="utf-8")
    display_source = (description_root / "launch" / "display.launch.py").read_text(
        encoding="utf-8"
    )

    # When: both files are parsed or inspected without constructing a ROS graph.
    _ = ast.parse(display_source, filename="display.launch.py")

    # Then: the sole authoritative URDF is installed and resolved from package share.
    assert "DIRECTORY launch meshes rviz urdf" in cmake_source
    assert 'get_package_share_directory("zero_description")' in display_source
    assert '"urdf" / "robot.urdf"' in display_source
    assert ".read_text(encoding=\"utf-8\")" in display_source
