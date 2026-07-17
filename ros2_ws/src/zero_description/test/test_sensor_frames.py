from __future__ import annotations

from pathlib import Path
from xml.etree import ElementTree
from xml.etree.ElementTree import Element


DESCRIPTION_DIR = Path(__file__).resolve().parents[1]
GAZEBO_DIR = DESCRIPTION_DIR.parent / "zero_gazebo"
URDF_PATH = DESCRIPTION_DIR / "urdf" / "robot.urdf"
SDF_PATH = GAZEBO_DIR / "models" / "zero_usv" / "model.sdf"


def _required_element(parent: Element[str], path: str) -> Element[str]:
    element = parent.find(path)
    assert element is not None
    return element


def _assert_sensor_contract(urdf_text: str, sdf_text: str) -> None:
    robot = ElementTree.fromstring(urdf_text)
    model = _required_element(ElementTree.fromstring(sdf_text), "model")

    expected_joints = {
        "zero_lidar_fixed_joint": ("zero_base_link", "lidar_link", "-0.00015 -0.00040397 0.1277"),
        "zero_laser_frame_fixed_joint": ("lidar_link", "laser_frame", "0 0 0"),
        "zero_imu_fixed_joint": ("zero_base_link", "imu_link", "0.15 0 0.13"),
    }
    link_names = {link.attrib["name"] for link in robot.findall("link")}
    assert {"imu_link", "laser_frame"} <= link_names

    for joint_name, (parent_name, child_name, xyz) in expected_joints.items():
        joint = _required_element(robot, f"./joint[@name='{joint_name}']")
        assert joint.attrib["type"] == "fixed"
        assert _required_element(joint, "parent").attrib["link"] == parent_name
        assert _required_element(joint, "child").attrib["link"] == child_name
        assert _required_element(joint, "origin").attrib == {"xyz": xyz, "rpy": "0 0 0"}

    lidar_pose = model.findtext("./link[@name='lidar_link']/pose")
    imu_pose = model.findtext("./link[@name='imu_link']/pose")
    laser_pose = model.findtext("./frame[@name='laser_frame']/pose")
    assert lidar_pose == f"{expected_joints['zero_lidar_fixed_joint'][2]} 0 0 0"
    assert imu_pose == f"{expected_joints['zero_imu_fixed_joint'][2]} 0 0 0"
    assert laser_pose == "0 0 0 0 0 0"
    assert model.findtext("./link[@name='lidar_link']/sensor/gz_frame_id") == "laser_frame"
    assert model.findtext("./link[@name='imu_link']/sensor/gz_frame_id") == "imu_link"


def test_authoritative_urdf_preserves_base_geometry_resources() -> None:
    # Given: the source-tree robot description, never a stale install tree.
    robot = ElementTree.parse(URDF_PATH).getroot()

    # When: the authoritative base link and all mesh references are inspected.
    base_link = robot.find("./link[@name='zero_base_link']")
    mesh_references = [mesh.attrib["filename"] for mesh in robot.findall(".//mesh")]

    # Then: the exported hull geometry and package resource references are unchanged.
    assert robot.attrib["name"] == "zero_usv"
    assert base_link is not None
    assert _required_element(base_link, "./visual/origin").attrib == {
        "xyz": "0 0 0",
        "rpy": "0 0 1.5708",
    }
    assert _required_element(base_link, "./collision/origin").attrib == {
        "xyz": "0 0 0",
        "rpy": "0 0 1.5708",
    }
    assert mesh_references == [
        "package://zero_description/meshes/hull/zero_base_link.STL",
        "package://zero_description/meshes/hull/zero_base_link.STL",
        "package://zero_description/meshes/sensors/lidar.STL",
        "package://zero_description/meshes/sensors/lidar.STL",
        "package://zero_description/meshes/propeller/left_motor_link.STL",
        "package://zero_description/meshes/propeller/left_motor_link.STL",
        "package://zero_description/meshes/propeller/right_motor_link.STL",
        "package://zero_description/meshes/propeller/right_motor_link.STL",
    ]
    for reference in set(mesh_references):
        resource_path = DESCRIPTION_DIR / reference.removeprefix("package://zero_description/")
        assert resource_path.is_file()


def test_sdf_sensor_pose_and_frame_data_is_characterized() -> None:
    # Given: the current Gazebo source model.
    model = _required_element(ElementTree.parse(SDF_PATH).getroot(), "model")

    # When: sensor links, sensor names, poses, and ROS frame IDs are inspected.
    lidar_link = _required_element(model, "./link[@name='lidar_link']")
    imu_link = _required_element(model, "./link[@name='imu_link']")
    laser_frame = _required_element(model, "./frame[@name='laser_frame']")

    # Then: the source SDF sensor contract is explicit.
    assert lidar_link.findtext("pose") == "-0.00015 -0.00040397 0.1277 0 0 0"
    assert _required_element(lidar_link, "sensor").attrib == {
        "name": "zero_lidar",
        "type": "gpu_lidar",
    }
    assert lidar_link.findtext("sensor/gz_frame_id") == "laser_frame"
    assert imu_link.findtext("pose") == "0.15 0 0.13 0 0 0"
    assert _required_element(imu_link, "sensor").attrib == {"name": "zero_imu", "type": "imu"}
    assert imu_link.findtext("sensor/gz_frame_id") == "imu_link"
    assert laser_frame.attrib == {"name": "laser_frame", "attached_to": "lidar_link"}
    assert laser_frame.findtext("pose") == "0 0 0 0 0 0"


def test_urdf_and_sdf_sensor_frames_have_matching_fixed_relationships() -> None:
    # Given: both source robot-description formats.
    urdf_text = URDF_PATH.read_text(encoding="utf-8")
    sdf_text = SDF_PATH.read_text(encoding="utf-8")

    # When/Then: their fixed sensor frame contracts are compared directly.
    _assert_sensor_contract(urdf_text, sdf_text)


def test_sensor_contract_rejects_missing_frame() -> None:
    # Given: a URDF fixture with the required laser frame removed.
    urdf_text = URDF_PATH.read_text(encoding="utf-8").replace('<link name="laser_frame" />', "")

    # When/Then: the static contract rejects the incomplete description.
    rejected = False
    try:
        _assert_sensor_contract(urdf_text, SDF_PATH.read_text(encoding="utf-8"))
    except AssertionError:
        rejected = True
    assert rejected


def test_sensor_contract_rejects_mismatched_pose() -> None:
    # Given: an SDF fixture whose IMU pose disagrees with the URDF joint.
    sdf_text = SDF_PATH.read_text(encoding="utf-8").replace(
        "0.15 0 0.13 0 0 0",
        "0.16 0 0.13 0 0 0",
    )

    # When/Then: the static contract rejects the pose drift.
    rejected = False
    try:
        _assert_sensor_contract(URDF_PATH.read_text(encoding="utf-8"), sdf_text)
    except AssertionError:
        rejected = True
    assert rejected


def test_sensor_xml_rejects_malformed_fixture() -> None:
    # Given: malformed XML that cannot represent a robot description.
    malformed_xml = "<robot><link name='zero_base_link'></robot>"

    # When/Then: the standard XML boundary parser rejects it.
    rejected = False
    try:
        _ = ElementTree.fromstring(malformed_xml)
    except ElementTree.ParseError:
        rejected = True
    assert rejected
