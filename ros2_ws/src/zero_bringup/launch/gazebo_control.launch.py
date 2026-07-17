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
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=str(model_path),
            ),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=str(model_path),
            ),
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
                output="screen",
                parameters=[
                    {
                        "config_file": str(bridge_config_path),
                        "expand_gz_topic_names": False,
                    }
                ],
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
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
