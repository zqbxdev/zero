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

    return LaunchDescription(
        [
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
                description=(
                    "Absolute path to an existing lowercase .yaml map file."
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            OpaqueFunction(
                function=_validate_required_map,
                args=[map_path],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        bringup_share
                        / "launch"
                        / "gazebo_control.launch.py"
                    )
                ),
                launch_arguments={
                    "world": world,
                    "config_file": config_file,
                }.items(),
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
        ]
    )
