from __future__ import annotations

from pathlib import Path
from typing import Final

import pytest

from .gazebo_control_launch_contract import assert_gazebo_control_contract
from .gazebo_navigation_launch_contract import (
    VALID_FIXTURE,
    assert_gazebo_navigation_contract,
)


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "gazebo_navigation.launch.py"
CONTROL_LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "gazebo_control.launch.py"


def test_gazebo_navigation_has_exact_required_map_single_owner_graph() -> None:
    # Given: every installed package resource used by navigation composition.
    required_resources = (
        SOURCE_ROOT / "zero_gazebo" / "worlds" / "zero_navigation_world.sdf",
        PACKAGE_ROOT / "config" / "v1_sim.yaml",
        CONTROL_LAUNCH_PATH,
        SOURCE_ROOT / "zero_navigation" / "config" / "nav2_params.yaml",
        SOURCE_ROOT / "zero_navigation" / "rviz" / "navigation.rviz",
    )
    assert all(path.is_file() for path in required_resources)
    assert LAUNCH_PATH.is_file()

    # When/Then: parent and child sources prove the exact ownership graph.
    assert_gazebo_navigation_contract(LAUNCH_PATH.read_text(encoding="utf-8"))
    assert_gazebo_control_contract(CONTROL_LAUNCH_PATH.read_text(encoding="utf-8"))


@pytest.mark.parametrize(
    ("old", "new"),
    [
        (
            'description="Absolute path to an existing lowercase .yaml map file.",',
            'default_value="/tmp/fake.yaml",\n'
            '            description="Absolute path to an existing lowercase .yaml map file.",',
        ),
        ("map_configuration.perform(context)", '"/tmp/fake.yaml"'),
        ('"map": validated_map,', '"map": map_path,'),
        ('"map": validated_map,', '"map": "relative.yaml",'),
    ],
)
def test_gazebo_navigation_rejects_default_or_fabricated_map(
    old: str,
    new: str,
) -> None:
    # Given: the required external map is defaulted or bypasses validation.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: the source-only contract fails closed.
    with pytest.raises(AssertionError):
        assert_gazebo_navigation_contract(malformed)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"world": world,', '"world_name": world,'),
        ('"config_file": config_file', '"params_file": config_file'),
        ('"slam": "False"', '"slam": "True"'),
        ('"use_sim_time": "True"', '"use_sim_time": "False"'),
        ('"autostart": "True"', '"autostart": "False"'),
        ('"use_composition": "True"', '"composition": "True"'),
        (
            '"use_composition": "True",',
            '"use_composition": "True", "use_keepout_zones": "True",',
        ),
    ],
)
def test_gazebo_navigation_rejects_wrong_or_rolling_include_argument(
    old: str,
    new: str,
) -> None:
    # Given: a child argument is renamed, changed, or expanded beyond Humble.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: only the pinned Humble contract is accepted.
    with pytest.raises(AssertionError):
        assert_gazebo_navigation_contract(malformed)


@pytest.mark.parametrize(
    "extra",
    [
        (
            'IncludeLaunchDescription(PythonLaunchDescriptionSource('
            '"gazebo_control.launch.py"), launch_arguments={}.items())'
        ),
        (
            'IncludeLaunchDescription(PythonLaunchDescriptionSource('
            '"bringup_launch.py"), launch_arguments={}.items())'
        ),
        'Node(package="rviz2", executable="rviz2", name="second_rviz")',
        'Node(package="nav2_map_server", executable="map_server")',
        'Node(package="nav2_amcl", executable="amcl")',
        'Node(package="nav2_planner", executable="planner_server")',
        'Node(package="nav2_controller", executable="controller_server")',
        'Node(package="nav2_behaviors", executable="behavior_server")',
        'Node(package="nav2_bt_navigator", executable="bt_navigator")',
        (
            'Node(package="nav2_lifecycle_manager", '
            'executable="lifecycle_manager")'
        ),
        'Node(package="slam_toolbox", executable="async_slam_toolbox_node")',
        'Node(package="zero_navigation", executable="waypoint_follower")',
        (
            'Node(package="teleop_twist_keyboard", '
            'executable="teleop_twist_keyboard")'
        ),
    ],
)
def test_gazebo_navigation_rejects_duplicate_or_direct_owner(extra: str) -> None:
    # Given: composition duplicates an include or directly owns a forbidden node.
    malformed = VALID_FIXTURE + f"\n{extra}\n"

    # When/Then: Gazebo and Nav2 remain the only lifecycle graph owners.
    with pytest.raises(AssertionError):
        assert_gazebo_navigation_contract(malformed)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('DeclareLaunchArgument("rviz", default_value="false")',
         'DeclareLaunchArgument("rviz", default_value="true")'),
        ("navigation.rviz", "mapping.rviz"),
        ("condition=IfCondition(rviz)", "condition=None"),
        ('parameters=[{"use_sim_time": True}]',
         'parameters=[{"use_sim_time": False}]'),
    ],
)
def test_gazebo_navigation_rejects_rviz_ownership_drift(
    old: str,
    new: str,
) -> None:
    # Given: navigation RViz defaults, config, condition, or clock drift.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: RViz remains optional, navigation-specific, and simulated.
    with pytest.raises(AssertionError):
        assert_gazebo_navigation_contract(malformed)


@pytest.mark.parametrize(
    "copied_key",
    ["map_server", "amcl", "planner_server", "controller_server"],
)
def test_gazebo_navigation_rejects_copied_nav2_tuning(copied_key: str) -> None:
    # Given: server tuning is copied into the orchestration launch.
    malformed = VALID_FIXTURE + f'\nCOPIED = {{"{copied_key}": "value"}}\n'

    # When/Then: tuning remains solely in zero_navigation's installed YAML.
    with pytest.raises(AssertionError):
        assert_gazebo_navigation_contract(malformed)
