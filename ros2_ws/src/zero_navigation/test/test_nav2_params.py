from __future__ import annotations

from copy import deepcopy

from .nav2_params_contract import (
    CONFIG_PATH,
    PACKAGE_ROOT,
    YamlMap,
    YamlValue,
    costmap_parameters,
    mapping,
    parameters,
    remove_string,
)
from .nav2_params_server_contract import (
    assert_contract_rejected,
    assert_nav2_contract,
    source_document,
)


def test_source_yaml_matches_exact_humble_nav2_contract() -> None:
    # Given: the source parameter file parsed without starting ROS or Nav2.
    document = source_document()

    # When/Then: all server, plugin, kinematic, footprint, and QoS rules hold.
    assert_nav2_contract(document)


def test_contract_rejects_rolling_keys_wrong_plugins_and_missing_members() -> None:
    # Given: source-derived fixtures with one Humble identity rule broken.
    fixtures: list[YamlMap] = []
    wrong_navfn = deepcopy(source_document())
    mapping(parameters(wrong_navfn, "planner_server"), "GridBased")["plugin"] = (
        "nav2_navfn_planner::NavfnPlanner"
    )
    fixtures.append(wrong_navfn)
    rolling_checker = deepcopy(source_document())
    checker = parameters(rolling_checker, "controller_server")
    checker["progress_checker_plugins"] = [checker.pop("progress_checker_plugin")]
    fixtures.append(rolling_checker)
    missing_critic = deepcopy(source_document())
    remove_string(
        mapping(parameters(missing_critic, "controller_server"), "FollowPath"),
        "critics",
        "GoalDist",
    )
    fixtures.append(missing_critic)
    missing_layer = deepcopy(source_document())
    remove_string(
        costmap_parameters(missing_layer, "global_costmap"),
        "plugins",
        "obstacle_layer",
    )
    fixtures.append(missing_layer)
    wrong_behavior_key = deepcopy(source_document())
    behavior = parameters(wrong_behavior_key, "behavior_server")
    behavior["recovery_plugins"] = behavior.pop("behavior_plugins")
    fixtures.append(wrong_behavior_key)

    # When/Then: every plugin, checker, critic, layer, and key mutation is rejected.
    for fixture in fixtures:
        assert_contract_rejected(fixture)


def test_contract_rejects_bad_footprints_qos_limits_and_map_paths() -> None:
    # Given: malformed safety, geometry, static-map, and generated-map fixtures.
    fixtures: list[YamlMap] = []
    robot_radius = deepcopy(source_document())
    costmap_parameters(robot_radius, "local_costmap")["robot_radius"] = 0.3
    fixtures.append(robot_radius)
    padded_hull = deepcopy(source_document())
    costmap_parameters(padded_hull, "global_costmap")["footprint"] = (
        "[[-0.65,-0.325],[-0.65,0.325],[0.65,0.325],[0.65,-0.325]]"
    )
    fixtures.append(padded_hull)
    unquoted_hull = deepcopy(source_document())
    parsed_sequence: list[YamlValue] = []
    costmap_parameters(unquoted_hull, "local_costmap")["footprint"] = parsed_sequence
    fixtures.append(unquoted_hull)
    missing_qos = deepcopy(source_document())
    del mapping(
        costmap_parameters(missing_qos, "global_costmap"), "static_layer"
    )["map_subscribe_transient_local"]
    fixtures.append(missing_qos)
    for value in (0.0, 1.01):
        invalid_limit = deepcopy(source_document())
        follow_path = mapping(
            parameters(invalid_limit, "controller_server"), "FollowPath"
        )
        follow_path["max_vel_x"] = value
        fixtures.append(invalid_limit)
    hard_coded_map = deepcopy(source_document())
    parameters(hard_coded_map, "map_server")["yaml_filename"] = "maps/generated.yaml"
    fixtures.append(hard_coded_map)

    # When/Then: fallback radius, double padding, zero/excess limits, and map paths fail.
    for fixture in fixtures:
        assert_contract_rejected(fixture)


def test_contract_rejects_frame_topic_sim_time_and_cardinality_drift() -> None:
    # Given: malformed authoritative-frame, sensor-topic, clock, and extra-key fixtures.
    fixtures: list[YamlMap] = []
    for root, key, value in (
        ("amcl", "base_frame_id", "base_link"),
        ("amcl", "scan_topic", "scan"),
        ("behavior_server", "global_frame", "map"),
    ):
        fixture = deepcopy(source_document())
        parameters(fixture, root)[key] = value
        fixtures.append(fixture)
    disabled_sim_time = deepcopy(source_document())
    costmap_parameters(disabled_sim_time, "local_costmap")["use_sim_time"] = False
    fixtures.append(disabled_sim_time)
    missing_namespace = deepcopy(source_document())
    del mapping(
        parameters(missing_namespace, "controller_server"), "FollowPath"
    )["default_critic_namespaces"]
    fixtures.append(missing_namespace)
    extra_root = deepcopy(source_document())
    extra_root["waypoint_follower"] = {"ros__parameters": {"use_sim_time": True}}
    fixtures.append(extra_root)

    # When/Then: authoritative identity, exact keys, and exact roots cannot drift.
    for fixture in fixtures:
        assert_contract_rejected(fixture)


def test_package_contains_no_generated_map_references_or_outputs() -> None:
    # Given: every map-like artifact and textual YAML reference in the resource package.
    map_outputs = {
        *PACKAGE_ROOT.rglob("*.pgm"),
        *PACKAGE_ROOT.rglob("*.png"),
        *(path for path in PACKAGE_ROOT.rglob("*.yaml") if path != CONFIG_PATH),
    }
    source = CONFIG_PATH.read_text(encoding="utf-8")

    # When/Then: only the empty launch-rewritten map slot exists; no map is generated.
    assert map_outputs == set()
    assert "yaml_filename: ''" in source
    assert "maps/" not in source
