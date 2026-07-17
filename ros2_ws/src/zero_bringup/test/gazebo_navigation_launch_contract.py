from __future__ import annotations

import ast
from typing import Final


COPIED_CONFIG_KEYS: Final = {
    "amcl",
    "bt_navigator",
    "controller_server",
    "global_costmap",
    "local_costmap",
    "map_server",
    "planner_server",
    "robot_base_frame",
    "scan_topic",
}
FORBIDDEN_FRAGMENTS: Final = {
    "mapping.rviz",
    "online_async_launch.py",
    "slam_toolbox",
    "teleop_twist_keyboard",
    "waypoint_follower",
}
VALID_FIXTURE: Final = '''
from pathlib import Path
from typing import Final
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from zero_bringup.map_path import validate_map_path

VALIDATED_MAP_CONFIGURATION: Final = "validated_map"

def _validate_required_map(
    context: LaunchContext,
    map_configuration: LaunchConfiguration,
) -> list[Action]:
    validated_map = validate_map_path(map_configuration.perform(context))
    return [
        SetLaunchConfiguration(
            VALIDATED_MAP_CONFIGURATION,
            str(validated_map),
        )
    ]

def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("zero_bringup"))
    gazebo_share = Path(get_package_share_directory("zero_gazebo"))
    navigation_share = Path(get_package_share_directory("zero_navigation"))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))
    world = LaunchConfiguration("world")
    config_file = LaunchConfiguration("config_file")
    map_path = LaunchConfiguration("map")
    rviz = LaunchConfiguration("rviz")
    validated_map = LaunchConfiguration(VALIDATED_MAP_CONFIGURATION)
    nav2_params_path = navigation_share / "config" / "nav2_params.yaml"
    navigation_rviz_path = navigation_share / "rviz" / "navigation.rviz"
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
        DeclareLaunchArgument(
            "map",
            description="Absolute path to an existing lowercase .yaml map file.",
        ),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_validate_required_map, args=[map_path]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "gazebo_control.launch.py")
            ),
            launch_arguments={"world": world, "config_file": config_file}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(nav2_bringup_share / "launch" / "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "map": validated_map,
                "use_sim_time": "True",
                "params_file": str(nav2_params_path),
                "autostart": "True",
                "use_composition": "True",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="navigation_rviz",
            arguments=["-d", str(navigation_rviz_path)],
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
    matches = [keyword.value for keyword in call.keywords if keyword.arg == name]
    assert len(matches) == 1
    return matches[0]


def _string(expression: ast.expr) -> str:
    assert isinstance(expression, ast.Constant)
    assert isinstance(expression.value, str)
    return expression.value


def _function(tree: ast.AST, name: str) -> ast.FunctionDef:
    matches = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    ]
    assert len(matches) == 1
    return matches[0]


def _string_literals(tree: ast.AST) -> frozenset[str]:
    return frozenset(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    )


def assert_gazebo_navigation_contract(source: str) -> None:
    tree = ast.parse(source, filename="gazebo_navigation.launch.py")
    normalized = ast.unparse(tree)
    arguments = _named_calls(tree, "DeclareLaunchArgument")
    assert [_string(call.args[0]) for call in arguments] == [
        "world",
        "config_file",
        "map",
        "rviz",
    ]
    assert "gazebo_share / 'worlds' / 'zero_navigation_world.sdf'" in ast.unparse(
        _keyword(arguments[0], "default_value")
    )
    assert "bringup_share / 'config' / 'v1_sim.yaml'" in ast.unparse(
        _keyword(arguments[1], "default_value")
    )
    assert all(keyword.arg != "default_value" for keyword in arguments[2].keywords)
    assert "lowercase .yaml" in _string(_keyword(arguments[2], "description"))
    assert _string(_keyword(arguments[3], "default_value")) == "false"

    share_calls = _named_calls(tree, "get_package_share_directory")
    assert [_string(call.args[0]) for call in share_calls] == [
        "zero_bringup",
        "zero_gazebo",
        "zero_navigation",
        "nav2_bringup",
    ]
    assert "navigation_share / 'config' / 'nav2_params.yaml'" in normalized
    assert "navigation_share / 'rviz' / 'navigation.rviz'" in normalized

    opaque_calls = _named_calls(tree, "OpaqueFunction")
    assert len(opaque_calls) == 1
    assert ast.unparse(_keyword(opaque_calls[0], "function")) == "_validate_required_map"
    assert ast.unparse(_keyword(opaque_calls[0], "args")) == "[map_path]"
    callback = ast.unparse(_function(tree, "_validate_required_map"))
    assert "validate_map_path(map_configuration.perform(context))" in callback
    set_calls = _named_calls(tree, "SetLaunchConfiguration")
    assert len(set_calls) == 1
    assert ast.unparse(set_calls[0]) == (
        "SetLaunchConfiguration(VALIDATED_MAP_CONFIGURATION, str(validated_map))"
    )

    includes = _named_calls(tree, "IncludeLaunchDescription")
    assert len(includes) == 2
    include_sources = [ast.unparse(call.args[0]) for call in includes]
    control_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "gazebo_control.launch.py" in value
    ]
    nav2_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "bringup_launch.py" in value
    ]
    assert len(control_indexes) == 1
    assert len(nav2_indexes) == 1
    assert ast.unparse(_keyword(includes[control_indexes[0]], "launch_arguments")) == (
        "{'world': world, 'config_file': config_file}.items()"
    )
    assert ast.unparse(_keyword(includes[nav2_indexes[0]], "launch_arguments")) == (
        "{'slam': 'False', 'map': validated_map, 'use_sim_time': 'True', "
        "'params_file': str(nav2_params_path), 'autostart': 'True', "
        "'use_composition': 'True'}.items()"
    )

    nodes = _named_calls(tree, "Node")
    assert len(nodes) == 1
    rviz_node = nodes[0]
    assert _string(_keyword(rviz_node, "package")) == "rviz2"
    assert _string(_keyword(rviz_node, "executable")) == "rviz2"
    assert _string(_keyword(rviz_node, "name")) == "navigation_rviz"
    assert ast.unparse(_keyword(rviz_node, "arguments")) == (
        "['-d', str(navigation_rviz_path)]"
    )
    assert ast.unparse(_keyword(rviz_node, "parameters")) == (
        "[{'use_sim_time': True}]"
    )
    assert ast.unparse(_keyword(rviz_node, "condition")) == "IfCondition(rviz)"

    literals = _string_literals(tree)
    yaml_literals = {
        value for value in literals if value.lower().endswith((".yaml", ".yml"))
    }
    assert yaml_literals == {"v1_sim.yaml", "nav2_params.yaml"}
    assert literals.isdisjoint(COPIED_CONFIG_KEYS)
    normalized_lower = normalized.lower()
    assert all(fragment not in normalized_lower for fragment in FORBIDDEN_FRAGMENTS)
    assert "remappings" not in source
    assert ".read_text(" not in source
