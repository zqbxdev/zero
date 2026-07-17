from __future__ import annotations

from pathlib import Path
from typing import Final

import pytest

from .gazebo_control_launch_contract import (
    VALID_FIXTURE,
    assert_gazebo_control_contract,
)


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "gazebo_control.launch.py"


def test_gazebo_control_has_exact_installed_single_owner_graph() -> None:
    # Given: every source resource required by the installed composition.
    required_resources = (
        SOURCE_ROOT / "zero_gazebo" / "worlds" / "zero_navigation_world.sdf",
        SOURCE_ROOT / "zero_gazebo" / "config" / "ros_gz_bridge.yaml",
        PACKAGE_ROOT / "config" / "v1_sim.yaml",
        PACKAGE_ROOT / "launch" / "control_chain.launch.py",
        SOURCE_ROOT / "zero_gazebo" / "zero_gazebo" / "odom_to_tf_relay.py",
    )
    assert all(path.is_file() for path in required_resources)
    assert LAUNCH_PATH.is_file()

    # When/Then: source-only inspection proves the exact public graph.
    assert_gazebo_control_contract(LAUNCH_PATH.read_text(encoding="utf-8"))


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("zero_navigation_world.sdf", "missing_world.sdf"),
        ("ros_gz_bridge.yaml", "missing_bridge.yaml"),
        ("v1_sim.yaml", "missing_config.yaml"),
        ("control_chain.launch.py", "display.launch.py"),
        ("gz_sim.launch.py", "gazebo.launch.py"),
    ],
)
def test_gazebo_control_rejects_wrong_installed_resource(old: str, new: str) -> None:
    # Given: one installed resource path is malformed.
    malformed = VALID_FIXTURE.replace(old, new)

    # When/Then: the graph contract fails closed.
    with pytest.raises(AssertionError):
        assert_gazebo_control_contract(malformed)


@pytest.mark.parametrize(
    "extra",
    [
        'DeclareLaunchArgument("rviz", default_value="false")',
        'IncludeLaunchDescription(PythonLaunchDescriptionSource("display.launch.py"))',
        'IncludeLaunchDescription(PythonLaunchDescriptionSource("gazebo_sensors.launch.py"))',
        (
            'Node(package="robot_state_publisher", ' +
            'executable="robot_state_publisher", name="second_rsp", parameters=[{}])'
        ),
        (
            'Node(package="zero_gazebo", executable="odom_to_tf_relay", ' +
            'name="second_relay", parameters=[config_file])'
        ),
        (
            'Node(package="tf2_ros", executable="static_transform_publisher", ' +
            'name="odom_tf", parameters=[{}])'
        ),
        'Node(package="zero_mapping", executable="mapper", name="mapper", parameters=[{}])',
        (
            'Node(package="zero_navigation", executable="navigator", ' +
            'name="navigator", parameters=[{}])'
        ),
        (
            'Node(package="teleop_twist_keyboard", ' +
            'executable="teleop_twist_keyboard", name="teleop", parameters=[{}])'
        ),
        'Node(package="rviz2", executable="rviz2", name="rviz2", parameters=[{}])',
    ],
)
def test_gazebo_control_rejects_extra_argument_include_or_owner(extra: str) -> None:
    # Given: an otherwise valid graph with one duplicate or forbidden owner.
    malformed = VALID_FIXTURE + f"\n{extra}\n"

    # When/Then: no extra public surface or graph owner is accepted.
    with pytest.raises(AssertionError):
        assert_gazebo_control_contract(malformed)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"expand_gz_topic_names": False', '"expand_gz_topic_names": True'),
        ('"expand_gz_topic_names": False', '"expand_gz_topic_names": False, "use_sim_time": True'),
        (
            '"expand_gz_topic_names": False',
            '"expand_gz_topic_names": False, ' +
            '"config_file": str(bridge_config_path)',
        ),
        ('"expand_gz_topic_names": False', '"expand_gz_topic_names": False, "topic": "/cmd_vel"'),
        (
            '"expand_gz_topic_names": False',
            '"expand_gz_topic_names": False, "gz_type_name": "Pose_V"',
        ),
        ('parameters=[config_file]', 'parameters=[config_file, {"use_sim_time": True}]'),
    ],
)
def test_gazebo_control_rejects_bridge_drift_or_copied_config(old: str, new: str) -> None:
    # Given: bridge/config propagation is duplicated, changed, or copied into the parent.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: only installed YAML plus the shared child config are accepted.
    with pytest.raises(AssertionError):
        assert_gazebo_control_contract(malformed)
