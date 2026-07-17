from __future__ import annotations

from pathlib import Path
from typing import Final
from xml.etree import ElementTree
from xml.etree.ElementTree import Element


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
MODEL_DIR: Final = PACKAGE_ROOT / "models" / "zero_usv"
MODEL_PATH: Final = MODEL_DIR / "model.sdf"
MODEL_CONFIG_PATH: Final = MODEL_DIR / "model.config"
SMOKE_WORLD_PATH: Final = PACKAGE_ROOT / "worlds" / "zero_sensor_smoke.sdf"


def _required_element(parent: Element[str], path: str) -> Element[str]:
    element = parent.find(path)
    assert element is not None
    return element


def test_existing_hull_geometry_and_inertial_contract_is_preserved() -> None:
    # Given: the source Gazebo model rather than a stale installed copy.
    root = ElementTree.parse(MODEL_PATH).getroot()

    # When: the model identity and existing hull link are inspected.
    model = _required_element(root, "model")
    hull = _required_element(model, "./link[@name='zero_base_link']")

    # Then: the simplified dynamic hull contract remains unchanged.
    assert root.attrib == {"version": "1.8"}
    assert model.attrib == {"name": "zero_usv"}
    assert model.findtext("pose") == "0 0 0 0 0 0"
    assert model.findtext("static") == "false"
    assert hull.findtext("inertial/mass") == "12.0"
    assert hull.findtext("inertial/inertia/ixx") == "0.82"
    assert hull.findtext("inertial/inertia/iyy") == "1.46"
    assert hull.findtext("inertial/inertia/izz") == "2.10"
    assert hull.findtext("collision/geometry/box/size") == "1.20 0.55 0.20"
    assert hull.findtext("visual/geometry/box/size") == "1.20 0.55 0.20"
    assert hull.findtext("visual/material/diffuse") == "0.86 0.88 0.90 1"


def test_existing_sensor_links_poses_and_payloads_are_preserved() -> None:
    # Given: the current source model's sensor links.
    model = _required_element(ElementTree.parse(MODEL_PATH).getroot(), "model")

    # When: the LiDAR and IMU contracts are inspected.
    lidar = _required_element(model, "./link[@name='lidar_link']")
    imu = _required_element(model, "./link[@name='imu_link']")
    lidar_sensor = _required_element(lidar, "sensor")
    imu_sensor = _required_element(imu, "sensor")

    # Then: names, poses, topics, frames, rates, and geometry stay stable.
    assert _required_element(lidar, "pose").attrib == {
        "relative_to": "zero_base_link",
    }
    assert lidar.findtext("pose") == "-0.00015 -0.00040397 0.1277 0 0 0"
    assert lidar_sensor.attrib == {"name": "zero_lidar", "type": "gpu_lidar"}
    assert lidar_sensor.findtext("topic") == "/scan"
    assert lidar_sensor.findtext("gz_frame_id") == "laser_frame"
    assert lidar_sensor.findtext("update_rate") == "6"
    assert lidar_sensor.findtext("ray/scan/horizontal/samples") == "500"
    assert lidar_sensor.findtext("ray/range/min") == "0.10"
    assert lidar_sensor.findtext("ray/range/max") == "8.0"
    assert _required_element(imu, "pose").attrib == {
        "relative_to": "zero_base_link",
    }
    assert imu.findtext("pose") == "0.15 0 0.13 0 0 0"
    assert imu_sensor.attrib == {"name": "zero_imu", "type": "imu"}
    assert imu_sensor.findtext("topic") == "/imu"
    assert imu_sensor.findtext("gz_frame_id") == "imu_link"
    assert imu_sensor.findtext("update_rate") == "100"


def test_existing_fixed_joints_and_sensor_frame_are_preserved() -> None:
    # Given: the model's existing fixed relationships.
    model = _required_element(ElementTree.parse(MODEL_PATH).getroot(), "model")

    # When: the named joints and attached frame are inspected.
    lidar_joint = _required_element(
        model,
        "./joint[@name='zero_lidar_fixed_joint']",
    )
    imu_joint = _required_element(
        model,
        "./joint[@name='zero_imu_fixed_joint']",
    )
    laser_frame = _required_element(model, "./frame[@name='laser_frame']")

    # Then: additional drive joints cannot disturb sensor ownership or frames.
    assert lidar_joint.attrib["type"] == "fixed"
    assert lidar_joint.findtext("parent") == "zero_base_link"
    assert lidar_joint.findtext("child") == "lidar_link"
    assert imu_joint.attrib["type"] == "fixed"
    assert imu_joint.findtext("parent") == "zero_base_link"
    assert imu_joint.findtext("child") == "imu_link"
    assert laser_frame.attrib == {
        "name": "laser_frame",
        "attached_to": "lidar_link",
    }
    assert laser_frame.findtext("pose") == "0 0 0 0 0 0"


def test_existing_plugin_and_model_resource_contract_is_preserved() -> None:
    # Given: the source model metadata and sensor smoke world.
    model = _required_element(ElementTree.parse(MODEL_PATH).getroot(), "model")
    config = ElementTree.parse(MODEL_CONFIG_PATH).getroot()
    world = _required_element(
        ElementTree.parse(SMOKE_WORLD_PATH).getroot(),
        "world",
    )

    # When: plugin identifiers, mesh policy, and model resolution are read.
    world_plugins = [
        (plugin.attrib["filename"], plugin.attrib["name"])
        for plugin in world.findall("plugin")
    ]
    model_uris = [uri.text for uri in world.findall("include/uri")]

    # Then: existing Fortress systems and primitive resources remain intact.
    assert world_plugins == [
        (
            "ignition-gazebo-physics-system",
            "ignition::gazebo::systems::Physics",
        ),
        (
            "ignition-gazebo-user-commands-system",
            "ignition::gazebo::systems::UserCommands",
        ),
        (
            "ignition-gazebo-scene-broadcaster-system",
            "ignition::gazebo::systems::SceneBroadcaster",
        ),
        (
            "ignition-gazebo-sensors-system",
            "ignition::gazebo::systems::Sensors",
        ),
        ("ignition-gazebo-imu-system", "ignition::gazebo::systems::Imu"),
    ]
    assert model.findall(".//mesh") == []
    assert config.findtext("name") == "zero_usv"
    assert config.findtext("version") == "0.1.0"
    assert _required_element(config, "sdf").attrib == {"version": "1.8"}
    assert config.findtext("sdf") == "model.sdf"
    assert model_uris == ["model://zero_usv"]
