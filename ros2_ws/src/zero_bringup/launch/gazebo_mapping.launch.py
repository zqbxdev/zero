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
        ]
    )
