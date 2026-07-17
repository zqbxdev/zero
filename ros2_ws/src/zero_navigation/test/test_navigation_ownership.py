from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Final

from .nav2_params_contract import PACKAGE_ROOT, YamlMap, mapping, parameters, strings
from .nav2_params_server_contract import source_document

BT_PLUGIN_LIBRARIES: Final = (
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_smooth_path_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_assisted_teleop_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_drive_on_heading_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_is_stuck_condition_bt_node",
    "nav2_goal_reached_condition_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_globally_updated_goal_condition_bt_node",
    "nav2_is_path_valid_condition_bt_node",
    "nav2_initial_pose_received_condition_bt_node",
    "nav2_reinitialize_global_localization_service_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_distance_controller_bt_node",
    "nav2_speed_controller_bt_node",
    "nav2_truncate_path_action_bt_node",
    "nav2_truncate_path_local_action_bt_node",
    "nav2_goal_updater_node_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
    "nav2_transform_available_condition_bt_node",
    "nav2_time_expired_condition_bt_node",
    "nav2_path_expiring_timer_condition",
    "nav2_distance_traveled_condition_bt_node",
    "nav2_single_trigger_bt_node",
    "nav2_goal_updated_controller_bt_node",
    "nav2_is_battery_low_condition_bt_node",
    "nav2_navigate_through_poses_action_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_planner_selector_bt_node",
    "nav2_controller_selector_bt_node",
    "nav2_goal_checker_selector_bt_node",
    "nav2_controller_cancel_bt_node",
    "nav2_path_longer_on_approach_bt_node",
    "nav2_wait_cancel_bt_node",
    "nav2_spin_cancel_bt_node",
    "nav2_back_up_cancel_bt_node",
    "nav2_assisted_teleop_cancel_bt_node",
    "nav2_drive_on_heading_cancel_bt_node",
    "nav2_is_battery_charging_condition_bt_node",
)
BEHAVIOR_PLUGINS: Final = {
    "spin": "nav2_behaviors/Spin",
    "backup": "nav2_behaviors/BackUp",
    "drive_on_heading": "nav2_behaviors/DriveOnHeading",
    "assisted_teleop": "nav2_behaviors/AssistedTeleop",
    "wait": "nav2_behaviors/Wait",
}
REQUIRED_README_POLICY: Final = (
    "installed nav2 packaged default navigate-to-pose behavior tree",
    "no copied bt xml",
    "no custom waypoint follower",
    "no custom recovery server",
    "one lifecycle manager",
    "zero_bringup",
)


def _assert_ownership_contract(document: YamlMap) -> None:
    bt = parameters(document, "bt_navigator")
    assert strings(bt, "plugin_lib_names") == BT_PLUGIN_LIBRARIES
    assert "default_nav_to_pose_bt_xml" not in bt
    assert "default_nav_through_poses_bt_xml" not in bt

    behavior = parameters(document, "behavior_server")
    assert strings(behavior, "behavior_plugins") == tuple(BEHAVIOR_PLUGINS)
    for plugin_id, class_name in BEHAVIOR_PLUGINS.items():
        assert mapping(behavior, plugin_id) == {"plugin": class_name}

    assert all(not root.startswith("lifecycle_manager") for root in document)


def _assert_rejected(document: YamlMap) -> None:
    rejected = False
    try:
        _assert_ownership_contract(document)
    except (AssertionError, KeyError, TypeError):
        rejected = True
    assert rejected


def _assert_no_custom_sources(
    xml_files: set[Path],
    runtime_sources: set[Path],
) -> None:
    assert xml_files == {Path("package.xml")}
    assert runtime_sources == set()


def test_source_uses_installed_default_bt_and_explicit_humble_plugins() -> None:
    # Given: the source Nav2 parameters parsed without starting Nav2.
    document = source_document()

    # When/Then: installed defaults own BT XML and plugin libraries are explicit.
    _assert_ownership_contract(document)


def test_contract_rejects_local_bt_missing_plugins_and_custom_library() -> None:
    # Given: source-derived fixtures violating one BT ownership rule at a time.
    local_bt = deepcopy(source_document())
    parameters(local_bt, "bt_navigator")["default_nav_to_pose_bt_xml"] = (
        "behavior_trees/custom.xml"
    )
    missing_plugins = deepcopy(source_document())
    _ = parameters(missing_plugins, "bt_navigator").pop("plugin_lib_names", None)
    custom_library = deepcopy(source_document())
    parameters(custom_library, "bt_navigator")["plugin_lib_names"] = [
        *BT_PLUGIN_LIBRARIES,
        "zero_custom_recovery_bt_node",
    ]

    # When/Then: copied XML, implicit libraries, and nonexistent plugins fail.
    for fixture in (local_bt, missing_plugins, custom_library):
        _assert_rejected(fixture)


def test_contract_rejects_missing_behavior_plugin_or_lifecycle_duplication() -> None:
    # Given: fixtures with an incomplete behavior server or local lifecycle owner.
    missing_behavior = deepcopy(source_document())
    behavior = parameters(missing_behavior, "behavior_server")
    behavior["behavior_plugins"] = ["spin", "backup", "wait"]
    duplicate_lifecycle = deepcopy(source_document())
    duplicate_lifecycle["lifecycle_manager_navigation"] = {
        "ros__parameters": {"autostart": True, "node_names": ["bt_navigator"]}
    }

    # When/Then: explicit behavior completeness and bringup ownership are enforced.
    for fixture in (missing_behavior, duplicate_lifecycle):
        _assert_rejected(fixture)


def test_package_contains_no_bt_copy_or_custom_navigation_server_source() -> None:
    # Given: all package XML documents and non-test runtime source candidates.
    xml_files = {path.relative_to(PACKAGE_ROOT) for path in PACKAGE_ROOT.rglob("*.xml")}
    runtime_sources = {
        path.relative_to(PACKAGE_ROOT)
        for suffix in ("*.cpp", "*.hpp", "*.py")
        for path in PACKAGE_ROOT.rglob(suffix)
        if "test" not in path.parts
    }

    # When/Then: Nav2 owns BT/follower/recovery code; only package.xml remains local.
    _assert_no_custom_sources(xml_files, runtime_sources)


def test_contract_rejects_bt_copy_waypoint_or_recovery_source_fixture() -> None:
    # Given: synthetic resource inventories claiming package-local Nav2 ownership.
    fixtures: tuple[tuple[set[Path], set[Path]], ...] = (
        ({Path("package.xml"), Path("behavior_trees/custom.xml")}, set[Path]()),
        ({Path("package.xml")}, {Path("zero_navigation/waypoint_follower.py")}),
        ({Path("package.xml")}, {Path("src/custom_recovery_server.cpp")}),
    )

    # When/Then: copied BT and custom navigation server source are rejected.
    for xml_files, runtime_sources in fixtures:
        rejected = False
        try:
            _assert_no_custom_sources(xml_files, runtime_sources)
        except AssertionError:
            rejected = True
        assert rejected


def test_readme_records_navigation_server_and_lifecycle_ownership_policy() -> None:
    # Given: the installed README source normalized for policy inspection.
    readme = " ".join(
        (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8").lower().split()
    )

    # When/Then: every ownership decision is explicit and reviewable.
    for policy in REQUIRED_README_POLICY:
        assert policy in readme
