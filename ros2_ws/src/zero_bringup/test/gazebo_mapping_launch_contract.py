from __future__ import annotations

import ast
from typing import Final


COPIED_CONFIG_KEYS: Final = {
    "base_frame",
    "command_timeout_seconds",
    "left_motor_sign",
    "map_frame",
    "map_update_interval",
    "max_angular_radps",
    "max_linear_mps",
    "max_rpm",
    "motor_state_timeout_seconds",
    "odom_frame",
    "resolution",
    "right_motor_sign",
    "scan_topic",
    "solver_plugin",
    "status_timeout_seconds",
    "track_width_m",
    "transform_publish_period",
    "wheel_radius_m",
}
FORBIDDEN_SOURCE_FRAGMENTS: Final = {
    "amcl",
    "behavior_server",
    "display.launch.py",
    "gazebo_navigation.launch.py",
    "lifecycle_manager",
    "map_server",
    "nav2_",
    "navigation.rviz",
    "online_sync_launch.py",
    "static_transform_publisher",
    "tf2_ros",
    "transformbroadcaster",
}
VALID_FIXTURE: Final = '''
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("zero_bringup"))
    gazebo_share = Path(get_package_share_directory("zero_gazebo"))
    mapping_share = Path(get_package_share_directory("zero_mapping"))
    slam_toolbox_share = Path(get_package_share_directory("slam_toolbox"))
    world = LaunchConfiguration("world")
    config_file = LaunchConfiguration("config_file")
    rviz = LaunchConfiguration("rviz")
    slam_params_path = mapping_share / "config" / "slam_toolbox.yaml"
    mapping_rviz_path = mapping_share / "rviz" / "mapping.rviz"
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=str(
                gazebo_share / "worlds" / "zero_navigation_world.sdf"
            ),
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=str(bringup_share / "config" / "v1_sim.yaml"),
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "gazebo_control.launch.py")
            ),
            launch_arguments={
                "world": world,
                "config_file": config_file,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    slam_toolbox_share
                    / "launch"
                    / "online_async_launch.py"
                )
            ),
            launch_arguments={
                "slam_params_file": str(slam_params_path),
                "use_sim_time": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="mapping_rviz",
            arguments=["-d", str(mapping_rviz_path)],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(rviz),
        ),
    ])
'''


def _named_calls(tree: ast.AST, name: str) -> tuple[ast.Call, ...]:
    return tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        if isinstance(node.func, ast.Name)
        if node.func.id == name
    )


def _keyword(call: ast.Call, name: str) -> ast.expr:
    matches = [
        keyword.value for keyword in call.keywords if keyword.arg == name
    ]
    assert len(matches) == 1
    return matches[0]


def _string(expression: ast.expr) -> str:
    assert isinstance(expression, ast.Constant)
    assert isinstance(expression.value, str)
    return expression.value


def _dict_entries(expression: ast.expr) -> dict[str, ast.expr]:
    assert isinstance(expression, ast.Dict)
    entries: dict[str, ast.expr] = {}
    for key_node, value_node in zip(
        expression.keys,
        expression.values,
        strict=True,
    ):
        assert key_node is not None
        key = _string(key_node)
        assert key not in entries
        entries[key] = value_node
    return entries


def _string_literals(tree: ast.AST) -> frozenset[str]:
    return frozenset(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    )


def assert_gazebo_mapping_contract(source: str) -> None:
    tree = ast.parse(source, filename="gazebo_mapping.launch.py")
    arguments = _named_calls(tree, "DeclareLaunchArgument")
    assert [_string(call.args[0]) for call in arguments] == [
        "world",
        "config_file",
        "rviz",
    ]
    defaults = [
        ast.unparse(_keyword(call, "default_value")) for call in arguments
    ]
    assert (
        "gazebo_share / 'worlds' / 'zero_navigation_world.sdf'"
        in defaults[0]
    )
    assert "bringup_share / 'config' / 'v1_sim.yaml'" in defaults[1]
    assert defaults[2] == "'false'"

    launch_configurations = _named_calls(tree, "LaunchConfiguration")
    assert [_string(call.args[0]) for call in launch_configurations] == [
        "world",
        "config_file",
        "rviz",
    ]
    share_calls = _named_calls(tree, "get_package_share_directory")
    assert [_string(call.args[0]) for call in share_calls] == [
        "zero_bringup",
        "zero_gazebo",
        "zero_mapping",
        "slam_toolbox",
    ]

    normalized = ast.unparse(tree)
    assert "mapping_share / 'config' / 'slam_toolbox.yaml'" in normalized
    assert "mapping_share / 'rviz' / 'mapping.rviz'" in normalized
    includes = _named_calls(tree, "IncludeLaunchDescription")
    assert len(includes) == 2
    include_sources = [ast.unparse(call.args[0]) for call in includes]
    control_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "gazebo_control.launch.py" in value
    ]
    slam_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "online_async_launch.py" in value
    ]
    assert len(control_indexes) == 1
    assert len(slam_indexes) == 1
    control_arguments = ast.unparse(
        _keyword(includes[control_indexes[0]], "launch_arguments")
    )
    slam_arguments = ast.unparse(
        _keyword(includes[slam_indexes[0]], "launch_arguments")
    )
    assert control_arguments == (
        "{'world': world, 'config_file': config_file}.items()"
    )
    assert slam_arguments == (
        "{'slam_params_file': str(slam_params_path), "
        "'use_sim_time': 'true'}.items()"
    )

    nodes = _named_calls(tree, "Node")
    assert len(nodes) == 1
    rviz_node = nodes[0]
    assert _string(_keyword(rviz_node, "package")) == "rviz2"
    assert _string(_keyword(rviz_node, "executable")) == "rviz2"
    assert _string(_keyword(rviz_node, "name")) == "mapping_rviz"
    assert ast.unparse(_keyword(rviz_node, "arguments")) == (
        "['-d', str(mapping_rviz_path)]"
    )
    rviz_parameters = _keyword(rviz_node, "parameters")
    assert isinstance(rviz_parameters, ast.List)
    assert len(rviz_parameters.elts) == 1
    rviz_entries = _dict_entries(rviz_parameters.elts[0])
    assert set(rviz_entries) == {"use_sim_time"}
    assert isinstance(rviz_entries["use_sim_time"], ast.Constant)
    assert rviz_entries["use_sim_time"].value is True
    assert ast.unparse(_keyword(rviz_node, "condition")) == (
        "IfCondition(rviz)"
    )

    literals = _string_literals(tree)
    assert literals.isdisjoint(COPIED_CONFIG_KEYS)
    normalized_lower = normalized.lower()
    assert all(
        fragment not in normalized_lower
        for fragment in FORBIDDEN_SOURCE_FRAGMENTS
    )
    assert "remappings" not in source
    assert ".read_text(" not in source
