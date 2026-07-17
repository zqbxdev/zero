from __future__ import annotations

from copy import deepcopy
from math import isfinite
from pathlib import Path
from typing import Final

import yaml


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "slam_toolbox.yaml"
ParameterValue = str | float | bool | list[float]
ParameterMap = dict[str, ParameterValue]
SlamDocument = dict[str, dict[str, ParameterMap]]
EXPECTED_PARAMETERS: Final[ParameterMap] = {
    "solver_plugin": "solver_plugins::CeresSolver",
    "odom_frame": "odom",
    "map_frame": "map",
    "base_frame": "zero_base_link",
    "scan_topic": "/scan",
    "mode": "mapping",
    "resolution": 0.05,
    "transform_publish_period": 0.02,
    "map_update_interval": 5.0,
    "use_sim_time": True,
}
FORBIDDEN_KEYS: Final = frozenset(
    {
        "amcl",
        "autostart",
        "lifecycle_nodes",
        "map_file_name",
        "map_server",
        "map_start_at_dock",
        "map_start_pose",
        "nav2",
        "node_name",
        "params_file",
        "use_lifecycle_manager",
        "yaml_filename",
    }
)


def _parameters(document: SlamDocument) -> ParameterMap:
    assert set(document) == {"slam_toolbox"}
    node = document["slam_toolbox"]
    assert set(node) == {"ros__parameters"}
    parameters = node["ros__parameters"]
    assert set(parameters) == set(EXPECTED_PARAMETERS)
    return parameters


def _assert_positive_finite(value: ParameterValue) -> None:
    assert isinstance(value, float)
    assert isfinite(value)
    assert value > 0.0


def _assert_slam_contract(document: SlamDocument) -> None:
    parameters = _parameters(document)
    assert parameters == EXPECTED_PARAMETERS
    assert FORBIDDEN_KEYS.isdisjoint(parameters)
    for key in ("resolution", "transform_publish_period", "map_update_interval"):
        _assert_positive_finite(parameters[key])


def _assert_contract_rejected(document: SlamDocument) -> None:
    rejected = False
    try:
        _assert_slam_contract(document)
    except (AssertionError, KeyError, TypeError):
        rejected = True
    assert rejected


def _source_document() -> SlamDocument:
    document = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    return document


def test_source_yaml_matches_exact_humble_async_mapping_contract() -> None:
    # Given: the tracked source YAML parsed without starting ROS or slam_toolbox.
    document = _source_document()

    # When/Then: its node root and complete parameter contract are exact.
    _assert_slam_contract(document)


def test_contract_rejects_wrong_root_base_frame_and_scan_topic() -> None:
    # Given: source-derived fixtures with one identity field changed at a time.
    fixtures: list[SlamDocument] = []
    wrong_root = deepcopy(_source_document())
    wrong_root["/**"] = wrong_root.pop("slam_toolbox")
    fixtures.append(wrong_root)
    for key, value in (("base_frame", "base_link"), ("scan_topic", "scan")):
        fixture = deepcopy(_source_document())
        fixture["slam_toolbox"]["ros__parameters"][key] = value
        fixtures.append(fixture)

    # When/Then: wildcard roots and frame/topic drift are rejected.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_absent_or_false_sim_time() -> None:
    # Given: source-derived fixtures with simulation time missing or disabled.
    missing = deepcopy(_source_document())
    del missing["slam_toolbox"]["ros__parameters"]["use_sim_time"]
    disabled = deepcopy(_source_document())
    disabled["slam_toolbox"]["ros__parameters"]["use_sim_time"] = False

    # When/Then: both fixtures fail the simulation-time contract.
    for fixture in (missing, disabled):
        _assert_contract_rejected(fixture)


def test_contract_rejects_nonfinite_or_nonpositive_numeric_contract() -> None:
    # Given: every timing/resolution key mutated across invalid numeric classes.
    fixtures: list[SlamDocument] = []
    for key in ("resolution", "transform_publish_period", "map_update_interval"):
        for value in (0.0, -0.01, float("inf"), float("nan")):
            fixture = deepcopy(_source_document())
            fixture["slam_toolbox"]["ros__parameters"][key] = value
            fixtures.append(fixture)

    # When/Then: zero, negative, infinite, and NaN values are all rejected.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_navigation_localization_and_map_ownership() -> None:
    # Given: AMCL, Nav2, map_server, and localization ownership mutations.
    fixtures: list[SlamDocument] = []
    for key, value in (
        ("amcl", True),
        ("nav2", True),
        ("map_server", True),
        ("mode", "localization"),
    ):
        fixture = deepcopy(_source_document())
        fixture["slam_toolbox"]["ros__parameters"][key] = value
        fixtures.append(fixture)

    # When/Then: zero_mapping remains async mapping configuration only.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_rolling_lifecycle_and_launch_only_forms() -> None:
    # Given: lifecycle-manager and launch-argument keys copied from newer integrations.
    fixtures: list[SlamDocument] = []
    lifecycle_keys = (
        "autostart",
        "lifecycle_nodes",
        "node_name",
        "params_file",
        "use_lifecycle_manager",
    )
    for key in lifecycle_keys:
        fixture = deepcopy(_source_document())
        fixture["slam_toolbox"]["ros__parameters"][key] = True
        fixtures.append(fixture)

    # When/Then: launch orchestration cannot leak into the Humble parameter file.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_pose_graph_and_occupancy_map_file_misuse() -> None:
    # Given: pose-graph continuation and Nav2 map-output keys.
    fixtures: list[SlamDocument] = []
    for key, value in (
        ("map_file_name", "map.yaml"),
        ("map_start_pose", [0.0, 0.0, 0.0]),
        ("map_start_at_dock", True),
        ("yaml_filename", "map.yaml"),
    ):
        fixture = deepcopy(_source_document())
        fixture["slam_toolbox"]["ros__parameters"][key] = value
        fixtures.append(fixture)

    # When/Then: a serialized pose graph is never treated as an occupancy-map result.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_package_contains_no_generated_map_outputs() -> None:
    # Given: every map-like artifact below the tracked package root.
    candidates = {
        *PACKAGE_ROOT.rglob("*.pgm"),
        *PACKAGE_ROOT.rglob("*.png"),
        *PACKAGE_ROOT.rglob("*.yaml"),
    }

    # When: the one owned parameter file is excluded.
    generated_outputs = candidates - {CONFIG_PATH}

    # Then: no fabricated or generated occupancy-map outputs are tracked.
    assert generated_outputs == set()
