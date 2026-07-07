from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = Path(get_package_share_directory("zero_description"))
    robot_description = (description_share / "urdf" / "robot.urdf").read_text(encoding="utf-8")

    return LaunchDescription([
        # Publish fixed URDF joints only; zero_sim owns odom -> zero_base_link.
        # 只发布 URDF 固定关节；odom -> zero_base_link 由 zero_sim 负责。
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="zero_hardware",
            executable="fake_motor_controller",
            name="fake_motor_controller",
            output="screen",
        ),
        Node(
            package="zero_sim",
            executable="simple_usv_simulator",
            name="simple_usv_simulator",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
        ),
    ])
