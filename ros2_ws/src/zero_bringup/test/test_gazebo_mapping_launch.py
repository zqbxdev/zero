from __future__ import annotations

from pathlib import Path
from typing import Final

import pytest

from .gazebo_control_launch_contract import assert_gazebo_control_contract
from .gazebo_mapping_launch_contract import (
    VALID_FIXTURE,
    assert_gazebo_mapping_contract,
)


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
LAUNCH_PATH: Final = PACKAGE_ROOT / "launch" / "gazebo_mapping.launch.py"
CONTROL_LAUNCH_PATH: Final = (
    PACKAGE_ROOT / "launch" / "gazebo_control.launch.py"
)


def test_gazebo_mapping_has_exact_installed_single_owner_graph() -> None:
    # Given: every installed resource used by the mapping composition.
    required_resources = (
        SOURCE_ROOT / "zero_gazebo" / "worlds" / "zero_navigation_world.sdf",
        PACKAGE_ROOT / "config" / "v1_sim.yaml",
        CONTROL_LAUNCH_PATH,
        SOURCE_ROOT / "zero_mapping" / "config" / "slam_toolbox.yaml",
        SOURCE_ROOT / "zero_mapping" / "rviz" / "mapping.rviz",
    )
    assert all(path.is_file() for path in required_resources)
    assert LAUNCH_PATH.is_file()

    # When/Then: parent and child prove the exact TF ownership graph.
    assert_gazebo_mapping_contract(LAUNCH_PATH.read_text(encoding="utf-8"))
    assert_gazebo_control_contract(
        CONTROL_LAUNCH_PATH.read_text(encoding="utf-8")
    )


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("zero_navigation_world.sdf", "missing_world.sdf"),
        ("v1_sim.yaml", "missing_config.yaml"),
        ("slam_toolbox.yaml", "copied_slam.yaml"),
        ("mapping.rviz", "navigation.rviz"),
        ("gazebo_control.launch.py", "control_chain.launch.py"),
        ("online_async_launch.py", "online_sync_launch.py"),
    ],
)
def test_gazebo_mapping_rejects_wrong_installed_resource(
    old: str,
    new: str,
) -> None:
    # Given: one installed path or include is malformed.
    malformed = VALID_FIXTURE.replace(old, new)

    # When/Then: the source-only contract fails closed.
    with pytest.raises(AssertionError):
        assert_gazebo_mapping_contract(malformed)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ('"world": world,', '"world_name": world,'),
        ('"config_file": config_file,', '"params_file": config_file,'),
        (
            '"slam_params_file": str(slam_params_path),',
            '"params_file": str(slam_params_path),',
        ),
        ('"use_sim_time": "true",', '"use_sim_time": "false",'),
        (
            '"use_sim_time": "true",',
            '"use_sim_time": "true", "autostart": "true",',
        ),
    ],
)
def test_gazebo_mapping_rejects_wrong_or_rolling_include_argument(
    old: str,
    new: str,
) -> None:
    # Given: a child argument is renamed, changed, or expanded beyond Humble.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: only the pinned Humble argument contract is accepted.
    with pytest.raises(AssertionError):
        assert_gazebo_mapping_contract(malformed)


@pytest.mark.parametrize(
    "extra",
    [
        (
            'IncludeLaunchDescription(PythonLaunchDescriptionSource('
            '"gazebo_control.launch.py"), launch_arguments={}.items())'
        ),
        (
            'IncludeLaunchDescription(PythonLaunchDescriptionSource('
            '"online_async_launch.py"), launch_arguments={}.items())'
        ),
        'Node(package="rviz2", executable="rviz2", name="second_rviz")',
        'Node(package="amcl", executable="amcl", name="amcl")',
        (
            'Node(package="nav2_map_server", executable="map_server", '
            'name="map_server")'
        ),
        (
            'Node(package="nav2_lifecycle_manager", '
            'executable="lifecycle_manager", name="manager")'
        ),
        (
            'Node(package="tf2_ros", executable="static_transform_publisher", '
            'name="map_to_odom")'
        ),
    ],
)
def test_gazebo_mapping_rejects_duplicate_or_forbidden_owner(
    extra: str,
) -> None:
    # Given: a second or forbidden localization/navigation owner.
    malformed = VALID_FIXTURE + f"\n{extra}\n"

    # When/Then: ownership remains singular and mapping-only.
    with pytest.raises(AssertionError):
        assert_gazebo_mapping_contract(malformed)


@pytest.mark.parametrize(
    ("old", "new"),
    [
        (
            'DeclareLaunchArgument("rviz", default_value="false")',
            'DeclareLaunchArgument("rviz", default_value="true")',
        ),
        ("condition=IfCondition(rviz)", "condition=None"),
        (
            'parameters=[{"use_sim_time": True}]',
            'parameters=[{"use_sim_time": False}]',
        ),
        (
            'arguments=["-d", str(mapping_rviz_path)]',
            'arguments=["-d", "navigation.rviz"]',
        ),
    ],
)
def test_gazebo_mapping_rejects_rviz_ownership_drift(
    old: str,
    new: str,
) -> None:
    # Given: optional mapping RViz defaults, condition, clock, or config drift.
    malformed = VALID_FIXTURE.replace(old, new, 1)

    # When/Then: one disabled-by-default mapping RViz contract is enforced.
    with pytest.raises(AssertionError):
        assert_gazebo_mapping_contract(malformed)


@pytest.mark.parametrize(
    "copied_key",
    ["map_frame", "odom_frame", "scan_topic", "max_rpm"],
)
def test_gazebo_mapping_rejects_copied_node_tuning(copied_key: str) -> None:
    # Given: tuning is copied into the parent instead of installed YAML.
    malformed = VALID_FIXTURE + f'\nCOPIED = {{"{copied_key}": "value"}}\n'

    # When/Then: the parent remains orchestration-only.
    with pytest.raises(AssertionError):
        assert_gazebo_mapping_contract(malformed)
