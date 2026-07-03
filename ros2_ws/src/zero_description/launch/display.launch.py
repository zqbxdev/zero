import os

# ROS 2 ament package index Python API.
# Resolves an installed package's share directory by package name.
# ROS 2 的 ament 包索引 Python 接口。
# 用于根据包名查找安装后的 share 资源目录。
from ament_index_python.packages import get_package_share_directory

# Describe the complete launch graph returned to ros2 launch.
# 用于描述 ros2 launch 要启动的完整节点图。
from launch import LaunchDescription

# Define ROS2 nodes that should be started by this launch file.
# 用于定义这个 launch 文件需要启动的 ROS2 节点。
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the Zero USV model display nodes. / 启动 Zero 无人船模型显示节点。"""

    # Locate and read the installed URDF before passing it to robot_state_publisher.
    # 先定位并读取安装后的 URDF，再传给 robot_state_publisher。
    pkg_share = get_package_share_directory('zero_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Publish robot_description and fixed joints from the URDF.
        # 从 URDF 发布 robot_description 和固定关节 TF。
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Temporary visualization frame: localization will own map/odom/base frames later.
        # 临时可视化坐标系：后续定位模块会正式管理 map/odom/base 坐标系。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_zero_base_link',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'zero_base_link',
            ],
        ),
        # Start RViz so the model and TF tree can be inspected manually.
        # 启动 RViz，方便人工检查模型和 TF 树。
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
