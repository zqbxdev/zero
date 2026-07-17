from __future__ import annotations

from math import isclose
from pathlib import Path
from typing import Final, TypeAlias

import yaml

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "nav2_params.yaml"
HULL_TEXT: Final = "[[-0.60,-0.275],[-0.60,0.275],[0.60,0.275],[0.60,-0.275]]"
HULL_POINTS: Final = [
    [-0.60, -0.275],
    [-0.60, 0.275],
    [0.60, 0.275],
    [0.60, -0.275],
]
FOOTPRINT_PADDING: Final = 0.05
V1_MAX_LINEAR: Final = 1.0
V1_MAX_ANGULAR: Final = 1.0
EXPECTED_ROOTS: Final = {
    "amcl",
    "behavior_server",
    "bt_navigator",
    "bt_navigator_navigate_through_poses_rclcpp_node",
    "bt_navigator_navigate_to_pose_rclcpp_node",
    "controller_server",
    "global_costmap",
    "local_costmap",
    "map_server",
    "planner_server",
}
AMCL_KEYS: Final = {
    "alpha1", "alpha2", "alpha3", "alpha4", "alpha5", "base_frame_id",
    "beam_skip_distance", "beam_skip_error_threshold", "beam_skip_threshold",
    "do_beamskip", "global_frame_id", "lambda_short",
    "laser_likelihood_max_dist", "laser_max_range", "laser_min_range",
    "laser_model_type", "max_beams", "max_particles", "min_particles",
    "odom_frame_id", "pf_err", "pf_z", "recovery_alpha_fast",
    "recovery_alpha_slow", "resample_interval", "robot_model_type",
    "save_pose_rate", "scan_topic", "sigma_hit", "tf_broadcast",
    "transform_tolerance", "update_min_a", "update_min_d", "use_sim_time",
    "z_hit", "z_max", "z_rand", "z_short",
}
CONTROLLER_KEYS: Final = {
    "FollowPath", "controller_frequency", "controller_plugins",
    "failure_tolerance", "general_goal_checker", "goal_checker_plugins",
    "min_theta_velocity_threshold", "min_x_velocity_threshold",
    "min_y_velocity_threshold", "progress_checker",
    "progress_checker_plugin", "use_sim_time",
}
FOLLOW_PATH_KEYS: Final = {
    "BaseObstacle.scale", "GoalAlign.forward_point_distance",
    "GoalAlign.scale", "GoalDist.scale", "PathAlign.forward_point_distance",
    "PathAlign.scale", "PathDist.scale", "RotateToGoal.lookahead_time",
    "RotateToGoal.scale", "RotateToGoal.slowing_factor", "acc_lim_theta",
    "acc_lim_x", "acc_lim_y", "angular_granularity", "critics",
    "debug_trajectory_details", "decel_lim_theta", "decel_lim_x",
    "decel_lim_y", "default_critic_namespaces", "linear_granularity",
    "max_speed_xy", "max_vel_theta", "max_vel_x", "max_vel_y",
    "min_speed_theta", "min_speed_xy", "min_vel_x", "min_vel_y", "plugin",
    "short_circuit_trajectory_evaluation", "sim_time", "stateful",
    "trans_stopped_velocity", "transform_tolerance", "vtheta_samples",
    "vx_samples", "vy_samples", "xy_goal_tolerance",
}
CRITICS: Final = [
    "RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign",
    "PathDist", "GoalDist",
]
BEHAVIOR_PLUGINS: Final = {
    "spin": "nav2_behaviors/Spin",
    "backup": "nav2_behaviors/BackUp",
    "drive_on_heading": "nav2_behaviors/DriveOnHeading",
    "assisted_teleop": "nav2_behaviors/AssistedTeleop",
    "wait": "nav2_behaviors/Wait",
}
YamlScalar: TypeAlias = bool | int | float | str
YamlValue: TypeAlias = YamlScalar | list["YamlValue"] | dict[str, "YamlValue"]
YamlMap: TypeAlias = dict[str, YamlValue]


def mapping(container: YamlMap, key: str) -> YamlMap:
    value = container[key]
    assert isinstance(value, dict)
    return value


def strings(container: YamlMap, key: str) -> tuple[str, ...]:
    value = container[key]
    assert isinstance(value, list)
    result: list[str] = []
    for item in value:
        assert isinstance(item, str)
        result.append(item)
    return tuple(result)


def remove_string(container: YamlMap, key: str, item: str) -> None:
    value = container[key]
    assert isinstance(value, list)
    value.remove(item)


def parameters(document: YamlMap, root: str) -> YamlMap:
    node = mapping(document, root)
    assert set(node) == {"ros__parameters"}
    return mapping(node, "ros__parameters")


def costmap_parameters(document: YamlMap, root: str) -> YamlMap:
    outer = mapping(document, root)
    assert set(outer) == {root}
    node = mapping(outer, root)
    assert set(node) == {"ros__parameters"}
    return mapping(node, "ros__parameters")


def _assert_footprint(costmap: YamlMap) -> None:
    footprint = costmap["footprint"]
    padding = costmap["footprint_padding"]
    assert isinstance(footprint, str)
    assert isinstance(padding, float)
    assert footprint == HULL_TEXT
    assert padding == FOOTPRINT_PADDING
    parsed: list[list[float]] = yaml.safe_load(footprint)
    assert parsed == HULL_POINTS
    width = max(point[0] for point in parsed) - min(point[0] for point in parsed)
    height = max(point[1] for point in parsed) - min(point[1] for point in parsed)
    assert isclose(width, 1.20)
    assert isclose(height, 0.55)
    assert padding > 0.0


def _assert_obstacle_layer(costmap: YamlMap) -> None:
    obstacle = mapping(costmap, "obstacle_layer")
    assert set(obstacle) == {"enabled", "observation_sources", "plugin", "scan"}
    assert obstacle["plugin"] == "nav2_costmap_2d::ObstacleLayer"
    assert obstacle["enabled"] is True
    assert obstacle["observation_sources"] == "scan"
    scan = mapping(obstacle, "scan")
    assert set(scan) == {
        "clearing", "data_type", "marking", "max_obstacle_height",
        "obstacle_max_range", "obstacle_min_range", "raytrace_max_range",
        "raytrace_min_range", "topic",
    }
    assert scan["topic"] == "/scan"
    assert scan["data_type"] == "LaserScan"
    assert scan["clearing"] is True
    assert scan["marking"] is True


def assert_costmaps(document: YamlMap) -> None:
    global_map = costmap_parameters(document, "global_costmap")
    local_map = costmap_parameters(document, "local_costmap")
    assert set(global_map) == {
        "always_send_full_costmap", "footprint", "footprint_padding",
        "global_frame", "inflation_layer", "obstacle_layer", "plugins",
        "publish_frequency", "resolution", "robot_base_frame", "static_layer",
        "track_unknown_space", "update_frequency", "use_sim_time",
    }
    assert set(local_map) == {
        "always_send_full_costmap", "footprint", "footprint_padding",
        "global_frame", "height", "inflation_layer", "obstacle_layer", "plugins",
        "publish_frequency", "resolution", "robot_base_frame", "rolling_window",
        "update_frequency", "use_sim_time", "width",
    }
    assert strings(global_map, "plugins") == (
        "static_layer", "obstacle_layer", "inflation_layer",
    )
    assert strings(local_map, "plugins") == ("obstacle_layer", "inflation_layer")
    assert global_map["global_frame"] == "map"
    assert local_map["global_frame"] == "odom"
    for costmap in (global_map, local_map):
        assert costmap["robot_base_frame"] == "zero_base_link"
        assert costmap["use_sim_time"] is True
        assert "robot_radius" not in costmap
        _assert_footprint(costmap)
        _assert_obstacle_layer(costmap)
        assert mapping(costmap, "inflation_layer")["plugin"] == (
            "nav2_costmap_2d::InflationLayer"
        )
    assert mapping(global_map, "static_layer") == {
        "map_subscribe_transient_local": True,
        "plugin": "nav2_costmap_2d::StaticLayer",
    }
