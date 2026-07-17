from __future__ import annotations

import ast
from collections import Counter
from pathlib import Path
from typing import Final

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "control_chain.launch.py"
EXPECTED_NODES: Final = (
    ("robot_state_publisher", "robot_state_publisher"),
    ("zero_control", "twist_to_motor_command"),
    ("zero_safety", "command_guard"),
    ("zero_hardware", "fake_motor_controller"),
    ("zero_gazebo", "gazebo_drive_adapter"),
    ("zero_safety", "simulation_safety_initializer"),
)
FORBIDDEN_OWNERS: Final = {
    "amcl",
    "gazebo",
    "nav2_bringup",
    "nav2_controller",
    "robot_localization",
    "ros_gz_bridge",
    "ros_gz_sim",
    "rviz2",
    "slam_toolbox",
    "teleop_twist_keyboard",
}
COPIED_CONFIG_KEYS: Final = {
    "command_timeout_seconds",
    "left_motor_sign",
    "max_angular_radps",
    "max_linear_mps",
    "max_rpm",
    "motor_state_timeout_seconds",
    "right_motor_sign",
    "status_timeout_seconds",
    "track_width_m",
    "wheel_radius_m",
}
VALID_FIXTURE: Final = "\n".join(
    (
        "from pathlib import Path",
        "from ament_index_python.packages import get_package_share_directory",
        "from launch import LaunchDescription",
        "from launch.actions import DeclareLaunchArgument",
        "from launch.substitutions import LaunchConfiguration",
        "from launch_ros.actions import Node",
        "",
        "def generate_launch_description() -> LaunchDescription:",
        '    bringup_share = Path(get_package_share_directory("zero_bringup"))',
        (
            "    description_share = Path("
            'get_package_share_directory("zero_description"))'
        ),
        '    config_file = LaunchConfiguration("config_file")',
        (
            '    robot_description = (description_share / "urdf" / '
            '"robot.urdf").read_text(encoding="utf-8")'
        ),
        "    return LaunchDescription([",
        (
            '        DeclareLaunchArgument("config_file", default_value=str('
            'bringup_share / "config" / "v1_sim.yaml")),'
        ),
        (
            '        Node(package="robot_state_publisher", '
            'executable="robot_state_publisher", name="robot_state_publisher", '
            'parameters=[config_file, {"robot_description": robot_description}]),'
        ),
        (
            '        Node(package="zero_control", '
            'executable="twist_to_motor_command", name="twist_to_motor_command", '
            "parameters=[config_file]),"
        ),
        (
            '        Node(package="zero_safety", executable="command_guard", '
            'name="command_guard", parameters=[config_file]),'
        ),
        (
            '        Node(package="zero_hardware", '
            'executable="fake_motor_controller", name="fake_motor_controller", '
            "parameters=[config_file]),"
        ),
        (
            '        Node(package="zero_gazebo", '
            'executable="gazebo_drive_adapter", name="gazebo_drive_adapter", '
            "parameters=[config_file]),"
        ),
        (
            '        Node(package="zero_safety", '
            'executable="simulation_safety_initializer", '
            'name="simulation_safety_initializer", parameters=[config_file]),'
        ),
        "    ])",
        "",
    )
)


def _named_calls(tree: ast.AST, name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == name
    ]


def _keyword(call: ast.Call, name: str) -> ast.expr:
    matches = [keyword.value for keyword in call.keywords if keyword.arg == name]
    assert len(matches) == 1
    return matches[0]


def _string(expression: ast.expr) -> str:
    assert isinstance(expression, ast.Constant)
    assert isinstance(expression.value, str)
    return expression.value


def _node_contract(call: ast.Call) -> tuple[tuple[str, str], tuple[str, ...]]:
    pair = (_string(_keyword(call, "package")), _string(_keyword(call, "executable")))
    parameters = _keyword(call, "parameters")
    assert isinstance(parameters, ast.List)
    return pair, tuple(ast.unparse(element) for element in parameters.elts)


def assert_control_chain_contract(source: str) -> None:
    tree = ast.parse(source, filename="control_chain.launch.py")
    argument_calls = _named_calls(tree, "DeclareLaunchArgument")
    assert len(argument_calls) == 1
    assert len(argument_calls[0].args) == 1
    assert _string(argument_calls[0].args[0]) == "config_file"
    default_source = ast.unparse(_keyword(argument_calls[0], "default_value"))
    assert "bringup_share" in default_source
    assert "'config'" in default_source
    assert "'v1_sim.yaml'" in default_source

    share_calls = _named_calls(tree, "get_package_share_directory")
    assert [_string(call.args[0]) for call in share_calls] == [
        "zero_bringup",
        "zero_description",
    ]
    assert "description_share / 'urdf' / 'robot.urdf'" in ast.unparse(tree)
    assert ".read_text(encoding='utf-8')" in ast.unparse(tree)

    node_contracts = [_node_contract(call) for call in _named_calls(tree, "Node")]
    node_counts = Counter(pair for pair, _ in node_contracts)
    assert node_counts == Counter(EXPECTED_NODES)
    parameter_sources = dict(node_contracts)
    assert parameter_sources[EXPECTED_NODES[0]] == (
        "config_file",
        "{'robot_description': robot_description}",
    )
    for pair in EXPECTED_NODES[1:]:
        assert parameter_sources[pair] == ("config_file",)

    owners = {owner for pair in node_counts for owner in pair}
    assert owners.isdisjoint(FORBIDDEN_OWNERS)
    assert "IncludeLaunchDescription" not in source
    assert "display.launch.py" not in source
    assert "remappings" not in source
    assert "/zero/motor_command" not in source
    assert all(key not in source for key in COPIED_CONFIG_KEYS)


def test_control_chain_has_exact_single_owner_graph() -> None:
    # Given: the reusable non-Gazebo control-chain launch source.
    assert LAUNCH_PATH.is_file()
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")

    # When/Then: static inspection proves the exact single-owner graph.
    assert_control_chain_contract(launch_source)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"config" / "v1_sim.yaml"', '"config" / "missing.yaml"'),
        ('"urdf" / "robot.urdf"', '"urdf" / "missing.urdf"'),
    ],
)
def test_control_chain_rejects_missing_installed_resource(old: str, new: str) -> None:
    # Given: a launch fixture with one required installed resource replaced.
    malformed_source = VALID_FIXTURE.replace(old, new)

    # When/Then: the static contract fails closed.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(malformed_source)


def test_control_chain_rejects_duplicate_node_owner() -> None:
    # Given: a second robot_state_publisher declaration.
    duplicate_source = VALID_FIXTURE + (
        '\nNode(package="robot_state_publisher", '
        'executable="robot_state_publisher", name="duplicate_rsp", '
        'parameters=[LaunchConfiguration("config_file")])\n'
    )

    # When/Then: duplicate graph ownership is rejected.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(duplicate_source)


def test_control_chain_rejects_wrong_initializer_package() -> None:
    # Given: the initializer executable is assigned to bringup instead of zero_safety.
    malformed_source = VALID_FIXTURE.replace(
        'package="zero_safety", executable="simulation_safety_initializer"',
        'package="zero_bringup", executable="simulation_safety_initializer"',
    )

    # When/Then: package ownership is rejected.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(malformed_source)


def test_control_chain_rejects_copied_tuning_and_direct_bypass() -> None:
    # Given: copied tuning plus an explicit protected-command remapping bypass.
    malformed_source = VALID_FIXTURE.replace(
        'parameters=[config_file]),\n        Node(package="zero_safety"',
        'parameters=[config_file], max_rpm=300.0, '
        + 'remappings=[("/zero/motor_command_raw", "/zero/motor_command")]),\n'
        + '        Node(package="zero_safety"',
        1,
    )

    # When/Then: launch-owned tuning and direct bypass are rejected.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(malformed_source)


@pytest.mark.parametrize("owner", sorted(FORBIDDEN_OWNERS))
def test_control_chain_rejects_forbidden_node_owner(owner: str) -> None:
    # Given: an otherwise valid fixture with one forbidden runtime owner.
    malformed_source = VALID_FIXTURE + (
        f'\nNode(package="{owner}", executable="{owner}", name="forbidden")\n'
    )

    # When/Then: Gazebo, bridge, localization, navigation, teleop, and RViz stay out.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(malformed_source)


def test_control_chain_rejects_display_launch_include() -> None:
    # Given: a display-only launch include is appended to the composition.
    malformed_source = VALID_FIXTURE + (
        '\nIncludeLaunchDescription("display.launch.py")\n'
    )

    # When/Then: display composition is rejected.
    with pytest.raises(AssertionError):
        assert_control_chain_contract(malformed_source)
