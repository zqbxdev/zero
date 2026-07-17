from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("zero_bringup"))
    description_share = Path(get_package_share_directory("zero_description"))
    config_file = LaunchConfiguration("config_file")
    robot_description = (description_share / "urdf" / "robot.urdf"
    ).read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=str(bringup_share / "config" / "v1_sim.yaml"),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[config_file, {"robot_description": robot_description}],
            ),
            Node(
                package="zero_control",
                executable="twist_to_motor_command",
                name="twist_to_motor_command",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="zero_safety",
                executable="command_guard",
                name="command_guard",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="zero_hardware",
                executable="fake_motor_controller",
                name="fake_motor_controller",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="zero_gazebo",
                executable="gazebo_drive_adapter",
                name="gazebo_drive_adapter",
                output="screen",
                parameters=[config_file],
            ),
            Node(
                package="zero_safety",
                executable="simulation_safety_initializer",
                name="simulation_safety_initializer",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
