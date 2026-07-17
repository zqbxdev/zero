from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Final, TypeAlias

import yaml

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
RVIZ_PATH: Final = PACKAGE_ROOT / "rviz" / "navigation.rviz"
EXPECTED_DISPLAYS: Final = {
    "Grid": ("rviz_default_plugins/Grid", None),
    "TF": ("rviz_default_plugins/TF", None),
    "Map": ("rviz_default_plugins/Map", "/map"),
    "LaserScan": ("rviz_default_plugins/LaserScan", "/scan"),
    "RobotModel": ("rviz_default_plugins/RobotModel", None),
    "Global Costmap": (
        "rviz_default_plugins/Map",
        "/global_costmap/costmap",
    ),
    "Local Costmap": (
        "rviz_default_plugins/Map",
        "/local_costmap/costmap",
    ),
    "Planned Path": ("rviz_default_plugins/Path", "/plan"),
}
REQUIRED_TOOLS: Final = {
    "rviz_default_plugins/SetInitialPose",
    "nav2_rviz_plugins/GoalTool",
}
YamlScalar: TypeAlias = bool | int | float | str
YamlValue: TypeAlias = YamlScalar | list["YamlValue"] | dict[str, "YamlValue"]
YamlMap: TypeAlias = dict[str, YamlValue]


def _mapping(container: YamlMap, key: str) -> YamlMap:
    value = container[key]
    assert isinstance(value, dict)
    return value


def _sequence(container: YamlMap, key: str) -> list[YamlValue]:
    value = container[key]
    assert isinstance(value, list)
    return value


def _source_document() -> YamlMap:
    document: YamlMap = yaml.safe_load(RVIZ_PATH.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    return document


def _assert_navigation_rviz(document: YamlMap) -> None:
    manager = _mapping(document, "Visualization Manager")
    options = _mapping(manager, "Global Options")
    assert options["Fixed Frame"] == "map"

    displays = _sequence(manager, "Displays")
    by_name: dict[str, YamlMap] = {}
    for value in displays:
        assert isinstance(value, dict)
        name = value["Name"]
        assert isinstance(name, str)
        by_name[name] = value
    assert set(by_name) == set(EXPECTED_DISPLAYS)
    for name, (class_name, topic) in EXPECTED_DISPLAYS.items():
        display = by_name[name]
        assert display["Class"] == class_name
        assert display["Enabled"] is True
        if topic is not None:
            assert _mapping(display, "Topic")["Value"] == topic

    robot_model = by_name["RobotModel"]
    assert robot_model["Description Source"] == "Topic"
    assert _mapping(robot_model, "Description Topic")["Value"] == (
        "/robot_description"
    )

    tools = _sequence(manager, "Tools")
    tool_classes: set[str] = set()
    for tool in tools:
        assert isinstance(tool, dict)
        class_name = tool["Class"]
        assert isinstance(class_name, str)
        tool_classes.add(class_name)
    assert REQUIRED_TOOLS <= tool_classes

    owned_text = "\n".join(
        [
            *(f"{name} {display['Class']}" for name, display in by_name.items()),
            *(str(tool_class) for tool_class in tool_classes),
        ]
    ).lower()
    assert "waypoint" not in owned_text


def _assert_rejected(document: YamlMap) -> None:
    rejected = False
    try:
        _assert_navigation_rviz(document)
    except (AssertionError, KeyError, TypeError):
        rejected = True
    assert rejected


def test_source_rviz_owns_exact_navigation_visualization_contract() -> None:
    # Given: the source RViz document parsed without starting RViz or ROS.
    document = _source_document()

    # When/Then: navigation displays, topics, frame, and tools are exact.
    _assert_navigation_rviz(document)


def test_contract_rejects_wrong_frame_topics_or_missing_navigation_display() -> None:
    # Given: source-derived fixtures with one navigation display rule broken.
    source = _source_document()
    wrong_frame = deepcopy(source)
    _mapping(_mapping(wrong_frame, "Visualization Manager"), "Global Options")[
        "Fixed Frame"
    ] = "odom"
    wrong_scan = deepcopy(source)
    for display in _sequence(
        _mapping(wrong_scan, "Visualization Manager"), "Displays"
    ):
        if isinstance(display, dict) and display.get("Name") == "LaserScan":
            _mapping(display, "Topic")["Value"] = "scan"
    wrong_map = deepcopy(source)
    for display in _sequence(
        _mapping(wrong_map, "Visualization Manager"), "Displays"
    ):
        if isinstance(display, dict) and display.get("Name") == "Map":
            _mapping(display, "Topic")["Value"] = "/generated_map"
    wrong_global_costmap = deepcopy(source)
    for display in _sequence(
        _mapping(wrong_global_costmap, "Visualization Manager"), "Displays"
    ):
        if isinstance(display, dict) and display.get("Name") == "Global Costmap":
            _mapping(display, "Topic")["Value"] = "/costmap"
    missing_path = deepcopy(source)
    manager = _mapping(missing_path, "Visualization Manager")
    manager["Displays"] = [
        display
        for display in _sequence(manager, "Displays")
        if isinstance(display, dict) and display.get("Name") != "Planned Path"
    ]
    missing_costmap = deepcopy(source)
    manager = _mapping(missing_costmap, "Visualization Manager")
    manager["Displays"] = [
        display
        for display in _sequence(manager, "Displays")
        if isinstance(display, dict) and display.get("Name") != "Local Costmap"
    ]

    # When/Then: mapping-only, wrong-frame, and wrong-topic fixtures fail.
    for fixture in (
        wrong_frame,
        wrong_scan,
        wrong_map,
        wrong_global_costmap,
        missing_path,
        missing_costmap,
    ):
        _assert_rejected(fixture)


def test_contract_rejects_missing_nav2_tools_or_legacy_waypoint_display() -> None:
    # Given: fixtures without a goal tool or with stale waypoint ownership.
    source = _source_document()
    missing_goal = deepcopy(source)
    manager = _mapping(missing_goal, "Visualization Manager")
    manager["Tools"] = [
        tool
        for tool in _sequence(manager, "Tools")
        if isinstance(tool, dict) and tool.get("Class") != "nav2_rviz_plugins/GoalTool"
    ]
    legacy_waypoint = deepcopy(source)
    _sequence(_mapping(legacy_waypoint, "Visualization Manager"), "Displays").append(
        {
            "Class": "rviz_default_plugins/Path",
            "Enabled": True,
            "Name": "Legacy Waypoint Path",
            "Topic": {"Value": "/waypoints"},
        }
    )

    # When/Then: both ownership gaps are rejected.
    for fixture in (missing_goal, legacy_waypoint):
        _assert_rejected(fixture)
