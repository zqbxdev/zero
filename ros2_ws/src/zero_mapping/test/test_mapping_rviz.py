from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Final

import yaml

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
RVIZ_PATH: Final = PACKAGE_ROOT / "rviz" / "mapping.rviz"
EXPECTED_DISPLAY_CLASSES: Final = frozenset(
    {
        "rviz_default_plugins/Grid",
        "rviz_default_plugins/LaserScan",
        "rviz_default_plugins/Map",
        "rviz_default_plugins/RobotModel",
        "rviz_default_plugins/TF",
    }
)
FORBIDDEN_CLASS_FRAGMENTS: Final = (
    "Costmap",
    "Goal",
    "InitialPose",
    "Nav2",
    "Path",
    "PoseWithCovariance",
)


@dataclass(frozen=True, slots=True)
class DisplaySpec:
    class_name: str
    enabled: bool
    topic: str | None
    description_source: str | None
    description_topic: str | None


@dataclass(frozen=True, slots=True)
class RvizContract:
    fixed_frame: str
    displays: tuple[DisplaySpec, ...]
    tool_classes: tuple[str, ...]


def _optional(value: str) -> str | None:
    return value or None


def _source_contract() -> RvizContract:
    document = yaml.safe_load(RVIZ_PATH.read_text(encoding="utf-8"))
    assert isinstance(document, dict)
    manager = document["Visualization Manager"]
    assert isinstance(manager, dict)
    global_options = manager["Global Options"]
    assert isinstance(global_options, dict)
    fixed_frame = global_options["Fixed Frame"]
    assert isinstance(fixed_frame, str)
    raw_displays = manager["Displays"]
    assert isinstance(raw_displays, list)
    displays = tuple(
        DisplaySpec(
            class_name=display["Class"],
            enabled=display["Enabled"],
            topic=_optional(display.get("Topic", {}).get("Value", "")),
            description_source=_optional(display.get("Description Source", "")),
            description_topic=_optional(
                display.get("Description Topic", {}).get("Value", "")
            ),
        )
        for display in raw_displays
    )
    raw_tools = manager["Tools"]
    assert isinstance(raw_tools, list)
    tool_classes = tuple(tool["Class"] for tool in raw_tools)
    return RvizContract(
        fixed_frame=fixed_frame,
        displays=displays,
        tool_classes=tool_classes,
    )


def _assert_mapping_rviz_contract(contract: RvizContract) -> None:
    assert contract.fixed_frame == "map"
    displays = {display.class_name: display for display in contract.displays}
    assert len(displays) == len(contract.displays)
    assert frozenset(displays) == EXPECTED_DISPLAY_CLASSES
    assert all(display.enabled for display in displays.values())
    assert displays["rviz_default_plugins/Map"].topic == "/map"
    assert displays["rviz_default_plugins/LaserScan"].topic == "/scan"

    robot_model = displays["rviz_default_plugins/RobotModel"]
    assert robot_model.description_source == "Topic"
    assert robot_model.description_topic == "/robot_description"

    owned_classes = (*displays, *contract.tool_classes)
    for owned_class in owned_classes:
        assert all(fragment not in owned_class for fragment in FORBIDDEN_CLASS_FRAGMENTS)


def _assert_contract_rejected(contract: RvizContract) -> None:
    rejected = False
    try:
        _assert_mapping_rviz_contract(contract)
    except (AssertionError, KeyError):
        rejected = True
    assert rejected


def _with_display_topic(
    contract: RvizContract,
    display_class: str,
    topic: str,
) -> RvizContract:
    displays = tuple(
        replace(display, topic=topic)
        if display.class_name == display_class
        else display
        for display in contract.displays
    )
    return replace(contract, displays=displays)


def test_source_rviz_owns_exact_mapping_visualization_contract() -> None:
    # Given: the tracked RViz document parsed without starting RViz or ROS.
    contract = _source_contract()

    # When/Then: mapping frames, topics, and display ownership are exact.
    _assert_mapping_rviz_contract(contract)


def test_contract_rejects_wrong_fixed_frame_or_mapping_topics() -> None:
    # Given: source-derived fixtures with one mapping identity changed at a time.
    source = _source_contract()
    fixtures = [
        replace(source, fixed_frame="odom"),
        _with_display_topic(
            source,
            "rviz_default_plugins/Map",
            "/global_costmap/costmap",
        ),
        _with_display_topic(source, "rviz_default_plugins/LaserScan", "scan"),
    ]

    # When/Then: frame drift and noncanonical mapping topics are rejected.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_navigation_display_or_goal_tool_ownership() -> None:
    # Given: source-derived fixtures claiming path, costmap, or goal ownership.
    source = _source_contract()
    fixtures = [
        replace(
            source,
            displays=(
                *source.displays,
                DisplaySpec(display_class, True, None, None, None),
            ),
        )
        for display_class in (
            "rviz_default_plugins/Path",
            "nav2_rviz_plugins/Costmap",
        )
    ]
    fixtures.append(
        replace(
            source,
            tool_classes=(*source.tool_classes, "rviz_default_plugins/SetGoal"),
        )
    )

    # When/Then: navigation visualization and goal interaction remain out of scope.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)
