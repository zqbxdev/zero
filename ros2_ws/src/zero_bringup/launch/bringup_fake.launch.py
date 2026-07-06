from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        # Reuse zero_hardware's fake node; keep all /zero topic and service names there.
        # 复用 zero_hardware 的 fake 节点；所有 /zero topic 和 service 名称仍由它维护。
        Node(
            package="zero_hardware",
            executable="fake_motor_controller",
            name="fake_motor_controller",
            output="screen",
        ),
    ])
