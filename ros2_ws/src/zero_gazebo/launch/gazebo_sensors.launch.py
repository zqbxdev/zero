from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gazebo_share = Path(get_package_share_directory("zero_gazebo"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    world_path = gazebo_share / "worlds" / "zero_sensor_smoke.sdf"
    model_path = gazebo_share / "models"
    robot_description_path = gazebo_share / "urdf" / "zero_usv_sensors.urdf.xacro"
    bridge_config_path = gazebo_share / "config" / "ros_gz_bridge.yaml"
    robot_description = Command(["xacro", " ", str(robot_description_path)])

    return LaunchDescription([
        # Let Gazebo resolve model://zero_usv from this package's installed models directory.
        # 让 Gazebo 从本包安装后的 models 目录解析 model://zero_usv。
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=str(model_path)),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=str(model_path)),
        # Start Gazebo Fortress with the minimal phase-4 sensor smoke world.
        # 使用阶段 4 的最小传感器 smoke world 启动 Gazebo Fortress。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")),
            launch_arguments={"gz_args": f"-r {world_path}"}.items(),
        ),
        # Publish fixed sensor TF from the xacro-generated robot_description.
        # 从 xacro 生成的 robot_description 发布传感器固定 TF。
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        # Bridge Gazebo sensor topics into ROS 2 using the package YAML config.
        # 使用本包 YAML 配置把 Gazebo 传感器 topic 桥接到 ROS 2。
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="zero_sensor_bridge",
            output="screen",
            parameters=[{"config_file": str(bridge_config_path)}],
        ),
    ])
