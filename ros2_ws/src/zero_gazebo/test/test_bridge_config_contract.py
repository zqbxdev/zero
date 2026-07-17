from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Final

import yaml


BRIDGE_CONFIG_PATH: Final = (
    Path(__file__).resolve().parents[1] / "config" / "ros_gz_bridge.yaml"
)
QUEUE_DEPTH: Final = 10
RouteValue = str | int
BridgeRoute = dict[str, RouteValue]
BridgeContract = list[BridgeRoute]
ALLOWED_KEYS: Final = frozenset(
    {
        "ros_topic_name",
        "gz_topic_name",
        "ros_type_name",
        "gz_type_name",
        "direction",
        "subscriber_queue",
        "publisher_queue",
    }
)
EXPECTED_ROUTES: Final[BridgeContract] = [
    {
        "ros_topic_name": "/scan",
        "gz_topic_name": "/scan",
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "ignition.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    },
    {
        "ros_topic_name": "/imu",
        "gz_topic_name": "/imu",
        "ros_type_name": "sensor_msgs/msg/Imu",
        "gz_type_name": "ignition.msgs.IMU",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    },
    {
        "ros_topic_name": "/clock",
        "gz_topic_name": "/clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock",
        "gz_type_name": "ignition.msgs.Clock",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    },
    {
        "ros_topic_name": "/zero/gazebo/cmd_vel",
        "gz_topic_name": "/model/zero_usv/cmd_vel",
        "ros_type_name": "geometry_msgs/msg/Twist",
        "gz_type_name": "ignition.msgs.Twist",
        "direction": "ROS_TO_GZ",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    },
    {
        "ros_topic_name": "/odom",
        "gz_topic_name": "/model/zero_usv/odometry",
        "ros_type_name": "nav_msgs/msg/Odometry",
        "gz_type_name": "ignition.msgs.Odometry",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    },
]


def _dump(routes: BridgeContract) -> str:
    return yaml.safe_dump(routes, sort_keys=False)


def _parse(config_text: str) -> BridgeContract:
    routes: BridgeContract = []
    current: BridgeRoute | None = None
    for raw_line in config_text.splitlines():
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


def _assert_bridge_contract(config_text: str) -> None:
    routes = _parse(config_text)
    assert len(routes) == len(EXPECTED_ROUTES)
    for route in routes:
        assert isinstance(route, dict)
        assert set(route) == ALLOWED_KEYS
    identities = [
        (route["ros_topic_name"], route["gz_topic_name"]) for route in routes
    ]
    assert len(identities) == len(set(identities))
    assert routes == EXPECTED_ROUTES
    assert all(route["direction"] in {"GZ_TO_ROS", "ROS_TO_GZ"} for route in routes)


def _assert_contract_rejected(config_text: str) -> None:
    rejected = False
    try:
        _assert_bridge_contract(config_text)
    except AssertionError:
        rejected = True
    assert rejected


def test_source_bridge_config_matches_exact_five_route_contract() -> None:
    # Given: the source YAML loaded without constructing a bridge node.
    source = BRIDGE_CONFIG_PATH.read_text(encoding="utf-8")

    # When/Then: all five pinned one-way routes and explicit queues are exact.
    _assert_bridge_contract(source)


def test_contract_rejects_malformed_schema_and_yaml() -> None:
    # Given: non-list and syntactically malformed YAML fixtures.
    fixtures = ("ros_topic_name: /scan\n", "- [unterminated\n")

    # When/Then: neither fixture can represent the bridge array contract.
    for fixture in fixtures:
        _assert_contract_rejected(fixture)


def test_contract_rejects_duplicate_route_identity() -> None:
    # Given: an odometry slot replaced by a duplicate command route.
    routes = deepcopy(EXPECTED_ROUTES)
    routes[-1] = deepcopy(routes[-2])

    # When/Then: duplicate ROS/Gazebo topic identity is rejected.
    _assert_contract_rejected(_dump(routes))


def test_contract_rejects_direction_inversion_and_bidirectional_route() -> None:
    # Given: command direction inversion and unsupported bidirectional fixtures.
    fixtures: list[BridgeContract] = []
    for route_index, direction in ((3, "GZ_TO_ROS"), (4, "ROS_TO_GZ"), (0, "BIDIRECTIONAL")):
        routes = deepcopy(EXPECTED_ROUTES)
        routes[route_index]["direction"] = direction
        fixtures.append(routes)

    # When/Then: every direction mutation is rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_dump(fixture))


def test_contract_rejects_wrong_and_newer_only_types() -> None:
    # Given: one wrong ROS or Gazebo type in each source-derived fixture.
    fixtures: list[BridgeContract] = []
    for route_index, key, value in (
        (3, "ros_type_name", "geometry_msgs/msg/TwistStamped"),
        (3, "gz_type_name", "gz.msgs.Twist"),
        (4, "ros_type_name", "geometry_msgs/msg/Pose"),
        (4, "gz_type_name", "ignition.msgs.OdometryWithCovariance"),
    ):
        routes = deepcopy(EXPECTED_ROUTES)
        routes[route_index][key] = value
        fixtures.append(routes)

    # When/Then: Humble/Fortress type drift is rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_dump(fixture))


def test_contract_rejects_direct_cmd_vel_and_wrong_odometry_topics() -> None:
    # Given: public topic mutations that bypass the approved adapter contract.
    fixtures: list[BridgeContract] = []
    for route_index, key, value in (
        (3, "ros_topic_name", "/cmd_vel"),
        (3, "gz_topic_name", "/cmd_vel"),
        (4, "ros_topic_name", "/model/zero_usv/odometry"),
        (4, "gz_topic_name", "/odom"),
    ):
        routes = deepcopy(EXPECTED_ROUTES)
        routes[route_index][key] = value
        fixtures.append(routes)

    # When/Then: every topic mutation is rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_dump(fixture))


def test_contract_rejects_private_diff_drive_tf_pose_v_bridge() -> None:
    # Given: a private DiffDrive TF route replacing the approved odometry route.
    routes = deepcopy(EXPECTED_ROUTES)
    routes[-1] = {
        "ros_topic_name": "/tf",
        "gz_topic_name": "/model/zero_usv/diff_drive_tf_unbridged",
        "ros_type_name": "tf2_msgs/msg/TFMessage",
        "gz_type_name": "ignition.msgs.Pose_V",
        "direction": "GZ_TO_ROS",
        "subscriber_queue": QUEUE_DEPTH,
        "publisher_queue": QUEUE_DEPTH,
    }

    # When/Then: Pose_V and the private TF topic remain outside the bridge.
    _assert_contract_rejected(_dump(routes))


def test_contract_rejects_qos_and_unsupported_keys() -> None:
    # Given: unsupported Humble QoS and arbitrary schema-key fixtures.
    fixtures: list[BridgeContract] = []
    for key, value in (("qos_profile", "SENSOR_DATA"), ("unsupported_option", 1)):
        routes = deepcopy(EXPECTED_ROUTES)
        routes[0][key] = value
        fixtures.append(routes)

    # When/Then: only the approved seven YAML keys are accepted.
    for fixture in fixtures:
        _assert_contract_rejected(_dump(fixture))


def test_contract_rejects_any_missing_explicit_queue() -> None:
    # Given: each route with either required queue field removed in isolation.
    fixtures: list[BridgeContract] = []
    for route_index in range(len(EXPECTED_ROUTES)):
        for queue_key in ("subscriber_queue", "publisher_queue"):
            routes = deepcopy(EXPECTED_ROUTES)
            del routes[route_index][queue_key]
            fixtures.append(routes)

    # When/Then: implicit queue defaults are not accepted by the final contract.
    for fixture in fixtures:
        _assert_contract_rejected(_dump(fixture))
