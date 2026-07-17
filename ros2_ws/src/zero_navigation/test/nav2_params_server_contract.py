from __future__ import annotations

from math import isfinite

import yaml

from .nav2_params_contract import (
    AMCL_KEYS,
    BEHAVIOR_PLUGINS,
    CONFIG_PATH,
    CONTROLLER_KEYS,
    CRITICS,
    EXPECTED_ROOTS,
    FOLLOW_PATH_KEYS,
    V1_MAX_ANGULAR,
    V1_MAX_LINEAR,
    YamlMap,
    assert_costmaps,
    mapping,
    parameters,
    strings,
)


def _assert_navigation_servers(document: YamlMap) -> None:
    planner = parameters(document, "planner_server")
    assert set(planner) == {
        "GridBased", "expected_planner_frequency", "planner_plugins", "use_sim_time",
    }
    assert strings(planner, "planner_plugins") == ("GridBased",)
    assert mapping(planner, "GridBased") == {
        "allow_unknown": True,
        "plugin": "nav2_navfn_planner/NavfnPlanner",
        "tolerance": 0.5,
        "use_astar": False,
    }
    controller = parameters(document, "controller_server")
    assert set(controller) == CONTROLLER_KEYS
    assert "progress_checker_plugins" not in controller
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert strings(controller, "goal_checker_plugins") == ("general_goal_checker",)
    assert strings(controller, "controller_plugins") == ("FollowPath",)
    assert mapping(controller, "progress_checker")["plugin"] == (
        "nav2_controller::SimpleProgressChecker"
    )
    assert mapping(controller, "general_goal_checker")["plugin"] == (
        "nav2_controller::SimpleGoalChecker"
    )
    follow_path = mapping(controller, "FollowPath")
    assert set(follow_path) == FOLLOW_PATH_KEYS
    assert follow_path["plugin"] == "dwb_core::DWBLocalPlanner"
    assert strings(follow_path, "default_critic_namespaces") == ("dwb_critics",)
    assert strings(follow_path, "critics") == tuple(CRITICS)
    for key, ceiling in (
        ("max_vel_x", V1_MAX_LINEAR), ("max_speed_xy", V1_MAX_LINEAR),
        ("max_vel_theta", V1_MAX_ANGULAR), ("acc_lim_x", V1_MAX_LINEAR),
        ("acc_lim_theta", V1_MAX_ANGULAR),
    ):
        value = follow_path[key]
        assert isinstance(value, float)
        assert isfinite(value)
        assert 0.0 < value <= ceiling
    for key, ceiling in (
        ("decel_lim_x", V1_MAX_LINEAR), ("decel_lim_theta", V1_MAX_ANGULAR),
    ):
        value = follow_path[key]
        assert isinstance(value, float)
        assert isfinite(value)
        assert 0.0 < abs(value) <= ceiling


def _assert_localization_and_behavior(document: YamlMap) -> None:
    amcl = parameters(document, "amcl")
    assert set(amcl) == AMCL_KEYS
    assert amcl["global_frame_id"] == "map"
    assert amcl["odom_frame_id"] == "odom"
    assert amcl["base_frame_id"] == "zero_base_link"
    assert amcl["scan_topic"] == "/scan"
    assert amcl["robot_model_type"] == "nav2_amcl::DifferentialMotionModel"
    assert parameters(document, "map_server") == {
        "use_sim_time": True,
        "yaml_filename": "",
    }
    behavior = parameters(document, "behavior_server")
    assert set(behavior) == {
        *BEHAVIOR_PLUGINS, "behavior_plugins", "costmap_topic", "cycle_frequency",
        "footprint_topic", "global_frame", "max_rotational_vel",
        "min_rotational_vel", "robot_base_frame", "rotational_acc_lim",
        "simulate_ahead_time", "transform_tolerance", "use_sim_time",
    }
    assert strings(behavior, "behavior_plugins") == tuple(BEHAVIOR_PLUGINS)
    for key, plugin in BEHAVIOR_PLUGINS.items():
        assert mapping(behavior, key) == {"plugin": plugin}
    assert behavior["global_frame"] == "odom"
    assert behavior["robot_base_frame"] == "zero_base_link"
    for key in ("min_rotational_vel", "max_rotational_vel", "rotational_acc_lim"):
        value = behavior[key]
        assert isinstance(value, float)
        assert 0.0 < value <= V1_MAX_ANGULAR


def assert_nav2_contract(document: YamlMap) -> None:
    assert set(document) == EXPECTED_ROOTS
    for root in EXPECTED_ROOTS - {"global_costmap", "local_costmap"}:
        assert parameters(document, root)["use_sim_time"] is True
    bt = parameters(document, "bt_navigator")
    assert set(bt) == {
        "bt_loop_duration", "default_server_timeout", "global_frame", "odom_topic",
        "plugin_lib_names", "robot_base_frame", "use_sim_time",
        "wait_for_service_timeout",
    }
    assert bt["global_frame"] == "map"
    assert bt["robot_base_frame"] == "zero_base_link"
    assert bt["odom_topic"] == "/odom"
    for child in (
        "bt_navigator_navigate_through_poses_rclcpp_node",
        "bt_navigator_navigate_to_pose_rclcpp_node",
    ):
        assert parameters(document, child) == {"use_sim_time": True}
    _assert_localization_and_behavior(document)
    _assert_navigation_servers(document)
    assert_costmaps(document)


def source_document() -> YamlMap:
    document: YamlMap = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    return document


def assert_contract_rejected(document: YamlMap) -> None:
    rejected = False
    try:
        assert_nav2_contract(document)
    except (AssertionError, KeyError, TypeError):
        rejected = True
    assert rejected
