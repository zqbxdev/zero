from __future__ import annotations

import ast
from collections import Counter
from typing import Final


EXPECTED_NODES: Final = (
    ("ros_gz_bridge", "parameter_bridge"),
    ("zero_gazebo", "odom_to_tf_relay"),
)
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
    "use_sim_time",
    "wheel_radius_m",
}
FORBIDDEN_LITERALS: Final = {
    "/cmd_vel",
    "/tf",
    "/tf_static",
    "Pose_V",
    "TFMessage",
    "display.launch.py",
    "gazebo_sensors.launch.py",
    "robot_description",
    "rviz",
    "rviz2",
    "zero_sim",
}
VALID_FIXTURE: Final = '''
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("zero_bringup"))
    gazebo_share = Path(get_package_share_directory("zero_gazebo"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    world = LaunchConfiguration("world")
    config_file = LaunchConfiguration("config_file")
    model_path = gazebo_share / "models"
    bridge_config_path = gazebo_share / "config" / "ros_gz_bridge.yaml"
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
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=str(model_path)),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=str(model_path)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="zero_gazebo_bridge",
            parameters=[{
                "config_file": str(bridge_config_path),
                "expand_gz_topic_names": False,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / "control_chain.launch.py")
            ),
            launch_arguments={"config_file": config_file}.items(),
        ),
        Node(
            package="zero_gazebo",
            executable="odom_to_tf_relay",
            name="odom_to_tf_relay",
            parameters=[config_file],
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


def _string_literals(tree: ast.AST) -> frozenset[str]:
    return frozenset(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    )


def _dict_entries(expression: ast.expr) -> dict[str, ast.expr]:
    assert isinstance(expression, ast.Dict)
    entries: dict[str, ast.expr] = {}
    for key_node, value_node in zip(expression.keys, expression.values, strict=True):
        assert key_node is not None
        key = _string(key_node)
        assert key not in entries
        entries[key] = value_node
    return entries


def _node_contract(call: ast.Call) -> tuple[tuple[str, str], str, ast.List]:
    pair = (_string(_keyword(call, "package")), _string(_keyword(call, "executable")))
    name = _string(_keyword(call, "name"))
    parameters = _keyword(call, "parameters")
    assert isinstance(parameters, ast.List)
    return pair, name, parameters


def assert_gazebo_control_contract(source: str) -> None:
    tree = ast.parse(source, filename="gazebo_control.launch.py")
    arguments = _named_calls(tree, "DeclareLaunchArgument")
    assert [_string(call.args[0]) for call in arguments] == ["world", "config_file"]
    defaults = [ast.unparse(_keyword(call, "default_value")) for call in arguments]
    assert "gazebo_share / 'worlds' / 'zero_navigation_world.sdf'" in defaults[0]
    assert "bringup_share / 'config' / 'v1_sim.yaml'" in defaults[1]

    share_calls = _named_calls(tree, "get_package_share_directory")
    assert [_string(call.args[0]) for call in share_calls] == [
        "zero_bringup",
        "zero_gazebo",
        "ros_gz_sim",
    ]
    normalized = ast.unparse(tree)
    assert "gazebo_share / 'models'" in normalized
    assert "gazebo_share / 'config' / 'ros_gz_bridge.yaml'" in normalized

    environment = {
        _string(_keyword(call, "name")): ast.unparse(_keyword(call, "value"))
        for call in _named_calls(tree, "SetEnvironmentVariable")
    }
    assert environment == {
        "GZ_SIM_RESOURCE_PATH": "str(model_path)",
        "IGN_GAZEBO_RESOURCE_PATH": "str(model_path)",
    }

    includes = _named_calls(tree, "IncludeLaunchDescription")
    assert len(includes) == 2
    include_sources = [ast.unparse(call.args[0]) for call in includes]
    gazebo_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "gz_sim.launch.py" in value
    ]
    control_indexes = [
        index
        for index, value in enumerate(include_sources)
        if "control_chain.launch.py" in value
    ]
    assert len(gazebo_indexes) == 1
    assert len(control_indexes) == 1
    gazebo_arguments = ast.unparse(
        _keyword(includes[gazebo_indexes[0]], "launch_arguments")
    )
    control_arguments = ast.unparse(
        _keyword(includes[control_indexes[0]], "launch_arguments")
    )
    assert gazebo_arguments == "{'gz_args': ['-r ', world]}.items()"
    assert control_arguments == "{'config_file': config_file}.items()"

    node_contracts = [_node_contract(call) for call in _named_calls(tree, "Node")]
    assert Counter(pair for pair, _, _ in node_contracts) == Counter(EXPECTED_NODES)
    contracts = {pair: (name, parameters) for pair, name, parameters in node_contracts}
    bridge_name, bridge_parameters = contracts[EXPECTED_NODES[0]]
    assert bridge_name == "zero_gazebo_bridge"
    assert len(bridge_parameters.elts) == 1
    bridge_entries = _dict_entries(bridge_parameters.elts[0])
    assert set(bridge_entries) == {"config_file", "expand_gz_topic_names"}
    assert ast.unparse(bridge_entries["config_file"]) == "str(bridge_config_path)"
    assert isinstance(bridge_entries["expand_gz_topic_names"], ast.Constant)
    assert bridge_entries["expand_gz_topic_names"].value is False
    relay_name, relay_parameters = contracts[EXPECTED_NODES[1]]
    assert relay_name == "odom_to_tf_relay"
    assert [ast.unparse(element) for element in relay_parameters.elts] == ["config_file"]

    literals = _string_literals(tree)
    assert literals.isdisjoint(FORBIDDEN_LITERALS)
    assert literals.isdisjoint(COPIED_CONFIG_KEYS)
    assert "remappings" not in source
    assert ".read_text(" not in source
