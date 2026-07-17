from __future__ import annotations

from pathlib import Path
from typing import Final


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
BRIDGE_CONFIG_PATH: Final = PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml"
CMAKE_PATH: Final = PACKAGE_ROOT / "CMakeLists.txt"
DEFAULT_QUEUE: Final = 10
RouteValue = str | int
BridgeRoute = dict[str, RouteValue]
EXISTING_ROUTES: Final[tuple[BridgeRoute, ...]] = (
    {
        "ros_topic_name": "/scan",
        "gz_topic_name": "/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "ignition.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": DEFAULT_QUEUE,
        "publisher_queue": DEFAULT_QUEUE,
    },
    {
        "ros_topic_name": "/imu",
        "gz_topic_name": "/imu",
        "ros_type_name": "sensor_msgs/msg/Imu",
        "gz_type_name": "ignition.msgs.IMU",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": DEFAULT_QUEUE,
        "publisher_queue": DEFAULT_QUEUE,
    },
    {
        "ros_topic_name": "/clock",
        "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock",
        "gz_type_name": "ignition.msgs.Clock",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": DEFAULT_QUEUE,
        "publisher_queue": DEFAULT_QUEUE,
    },
)


def _load_routes() -> list[BridgeRoute]:
    routes: list[BridgeRoute] = []
    current: BridgeRoute | None = None
    for raw_line in BRIDGE_CONFIG_PATH.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if raw_line.startswith("- "):
            if current is not None:
                routes.append(current)
            current = {}
            entry = line[2:]
        else:
            assert raw_line.startswith("  ")
            entry = line
        assert current is not None
        key, separator, raw_value = entry.partition(":")
        assert separator == ":"
        assert key not in current
        value_text = raw_value.strip()
        assert value_text
        quoted = value_text.startswith('"') and value_text.endswith('"')
        value = value_text[1:-1] if quoted else value_text
        current[key] = int(value) if value.isdecimal() else value
    assert current is not None
    routes.append(current)
    return routes


def test_existing_sensor_and_clock_routes_keep_effective_queue_contract() -> None:
    # Given: the source bridge configuration before adding drive routes.
    routes = _load_routes()

    # When/Then: each existing route keeps its pinned one-way effective contract.
    for expected in EXISTING_ROUTES:
        matches = [
            route
            for route in routes
            if route["ros_topic_name"] == expected["ros_topic_name"]
            and route["gz_topic_name"] == expected["gz_topic_name"]
        ]
        assert len(matches) == 1
        route = matches[0]
        actual = {
            "ros_topic_name": route["ros_topic_name"],
            "gz_topic_name": route["gz_topic_name"],
            "ros_type_name": route["ros_type_name"],
            "gz_type_name": route["gz_type_name"],
            "direction": route["direction"],
            "subscriber_queue": route.get("subscriber_queue", DEFAULT_QUEUE),
            "publisher_queue": route.get("publisher_queue", DEFAULT_QUEUE),
        }
        assert actual == expected


def test_bridge_config_remains_in_the_shared_resource_install_rule() -> None:
    # Given: the current zero_gazebo CMake resource installation block.
    cmake = CMAKE_PATH.read_text(encoding="utf-8")

    # When/Then: config remains installed with the existing resource directories.
    assert cmake.count("DIRECTORY config launch models worlds") == 1
    assert cmake.count("DESTINATION share/${PROJECT_NAME}") == 1
