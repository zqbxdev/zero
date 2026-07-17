from __future__ import annotations

from copy import deepcopy
from math import isclose
from pathlib import Path
from typing import Final
from xml.etree import ElementTree
from xml.etree.ElementTree import Element


MODEL_PATH: Final = Path(__file__).resolve().parents[1].joinpath(
    "models", "zero_usv", "model.sdf"
)
LEFT_LINK: Final = "left_virtual_wheel_link"
RIGHT_LINK: Final = "right_virtual_wheel_link"
LEFT_JOINT: Final = "left_virtual_wheel_joint"
RIGHT_JOINT: Final = "right_virtual_wheel_joint"
PLUGIN_CHILDREN: Final = (
    ("left_joint", LEFT_JOINT),
    ("right_joint", RIGHT_JOINT),
    ("wheel_separation", "0.407"),
    ("wheel_radius", "0.05"),
    ("odom_publish_frequency", "50"),
    ("topic", "/model/zero_usv/cmd_vel"),
    ("odom_topic", "/model/zero_usv/odometry"),
    ("tf_topic", "/model/zero_usv/diff_drive_tf_unbridged"),
    ("frame_id", "odom"),
    ("child_frame_id", "zero_base_link"),
    ("min_linear_velocity", "-1.0"),
    ("max_linear_velocity", "1.0"),
    ("min_angular_velocity", "-1.0"),
    ("max_angular_velocity", "1.0"),
    ("min_linear_acceleration", "-1.0"),
    ("max_linear_acceleration", "1.0"),
    ("min_angular_acceleration", "-1.0"),
    ("max_angular_acceleration", "1.0"),
)


def _required_element(parent: Element[str], path: str) -> Element[str]:
    element = parent.find(path)
    assert element is not None
    return element


def _required_text(parent: Element[str], path: str) -> str:
    text = _required_element(parent, path).text
    assert text is not None
    return text


def _model(root: Element[str]) -> Element[str]:
    models = root.findall("model")
    assert len(models) == 1
    return models[0]


def _source_root() -> Element[str]:
    return ElementTree.parse(MODEL_PATH).getroot()


def _xml(root: Element[str]) -> str:
    return ElementTree.tostring(root, encoding="unicode")


def _assert_virtual_wheel(
    model: Element[str], link_name: str, expected_y: str
) -> None:
    links = model.findall(f"./link[@name='{link_name}']")
    assert len(links) == 1
    link = links[0]
    pose = _required_element(link, "pose")
    assert pose.attrib == {"relative_to": "zero_base_link"}
    assert pose.text == f"0 {expected_y} -0.13 0 0 0"
    assert link.findall("visual") == []

    collisions = link.findall("collision")
    assert len(collisions) == 1
    collision = collisions[0]
    assert collision.findtext("pose") == "0 0 0 1.57079632679 0 0"
    assert collision.findtext("geometry/cylinder/radius") == "0.05"
    assert collision.findtext("geometry/cylinder/length") == "0.02"

    inertial = _required_element(link, "inertial")
    assert float(_required_text(inertial, "mass")) > 0.0
    diagonal = [
        float(_required_text(inertial, f"inertia/{name}"))
        for name in ("ixx", "iyy", "izz")
    ]
    products = [
        float(_required_text(inertial, f"inertia/{name}"))
        for name in ("ixy", "ixz", "iyz")
    ]
    assert all(value > 0.0 for value in diagonal)
    assert products == [0.0, 0.0, 0.0]


def _assert_virtual_joint(
    model: Element[str], joint_name: str, child_link: str
) -> None:
    joints = model.findall(f"./joint[@name='{joint_name}']")
    assert len(joints) == 1
    joint = joints[0]
    assert joint.attrib == {"name": joint_name, "type": "revolute"}
    assert joint.findtext("parent") == "zero_base_link"
    assert joint.findtext("child") == child_link
    axes = joint.findall("axis")
    assert len(axes) == 1
    axis_text = _required_text(axes[0], "xyz")
    assert axis_text == "0 1 0"
    axis_values = [float(value) for value in axis_text.split()]
    assert isclose(sum(value * value for value in axis_values), 1.0)


def _assert_diff_drive_contract(sdf_text: str) -> None:
    model = _model(ElementTree.fromstring(sdf_text))
    _assert_virtual_wheel(model, LEFT_LINK, "0.2035")
    _assert_virtual_wheel(model, RIGHT_LINK, "-0.2035")
    _assert_virtual_joint(model, LEFT_JOINT, LEFT_LINK)
    _assert_virtual_joint(model, RIGHT_JOINT, RIGHT_LINK)

    left_pose = _required_text(model, f"./link[@name='{LEFT_LINK}']/pose")
    right_pose = _required_text(model, f"./link[@name='{RIGHT_LINK}']/pose")
    left_y = float(left_pose.split()[1])
    right_y = float(right_pose.split()[1])
    assert isclose(left_y - right_y, 0.407, abs_tol=1e-12)

    plugins = model.findall("plugin")
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.attrib == {
        "filename": "libignition-gazebo-diff-drive-system.so",
        "name": "ignition::gazebo::systems::DiffDrive",
    }
    assert [(child.tag, child.text) for child in plugin] == list(
        PLUGIN_CHILDREN
    )
    assert [child.text for child in plugin.findall("left_joint")] == [
        LEFT_JOINT
    ]
    assert [child.text for child in plugin.findall("right_joint")] == [
        RIGHT_JOINT
    ]


def _assert_contract_rejected(sdf_text: str) -> None:
    rejected = False
    try:
        _assert_diff_drive_contract(sdf_text)
    except (AssertionError, ValueError):
        rejected = True
    assert rejected


def test_source_model_has_exact_hidden_fortress_diff_drive_contract() -> None:
    # Given: the source zero_usv SDF rather than a generated install-tree copy.
    sdf_text = MODEL_PATH.read_text(encoding="utf-8")

    # When/Then: the complete virtual-wheel Fortress contract is checked.
    _assert_diff_drive_contract(sdf_text)


def test_contract_rejects_wrong_newer_diff_drive_identifiers() -> None:
    # Given: source-derived fixtures with newer gz-sim identifiers.
    fixtures: list[Element[str]] = []
    for attribute_name, wrong_value in (
        ("filename", "gz-sim-diff-drive-system"),
        ("name", "gz::sim::systems::DiffDrive"),
    ):
        root = _source_root()
        plugin = _required_element(_model(root), "plugin")
        plugin.attrib[attribute_name] = wrong_value
        fixtures.append(root)

    # When/Then: the Fortress-pinned contract rejects each identifier.
    for fixture in fixtures:
        _assert_contract_rejected(_xml(fixture))


def test_contract_rejects_wrong_topics_frames_and_limits() -> None:
    # Given: source-derived fixtures with one incorrect DiffDrive value each.
    fixtures: list[Element[str]] = []
    for tag, wrong_value in (
        ("topic", "/cmd_vel"),
        ("odom_topic", "/odom"),
        ("tf_topic", "/model/zero_usv/tf"),
        ("frame_id", "zero_usv/odom"),
        ("child_frame_id", "zero_usv/zero_base_link"),
        ("max_linear_velocity", "1.1"),
        ("max_angular_velocity", "1.1"),
        ("max_linear_acceleration", "1.1"),
        ("max_angular_acceleration", "1.1"),
    ):
        root = _source_root()
        plugin = _required_element(_model(root), "plugin")
        _required_element(plugin, tag).text = wrong_value
        fixtures.append(root)

    # When/Then: defaults, public topics, and limit drift are rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_xml(fixture))


def test_contract_rejects_missing_plugin_value() -> None:
    # Given: source-derived plugins each missing one required explicit value.
    fixtures: list[Element[str]] = []
    for tag, _ in PLUGIN_CHILDREN:
        root = _source_root()
        plugin = _required_element(_model(root), "plugin")
        plugin.remove(_required_element(plugin, tag))
        fixtures.append(root)

    # When/Then: implicit defaults and missing wheel joints are rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_xml(fixture))


def test_contract_rejects_wrong_first_pair_and_duplicate_joint() -> None:
    # Given: an extra left joint inserted before the approved odometry pair.
    root = _source_root()
    plugin = _required_element(_model(root), "plugin")
    wrong_first = Element("left_joint")
    wrong_first.text = "unapproved_left_joint"
    plugin.insert(0, wrong_first)

    # When/Then: first-pair ordering and duplicate side entries are rejected.
    _assert_contract_rejected(_xml(root))


def test_contract_rejects_missing_or_duplicate_model_joint() -> None:
    # Given: source fixtures with a missing and duplicate revolute joint.
    fixtures: list[Element[str]] = []
    for joint_name in (LEFT_JOINT, RIGHT_JOINT):
        missing_root = _source_root()
        missing_model = _model(missing_root)
        missing_joint = _required_element(
            missing_model, f"./joint[@name='{joint_name}']"
        )
        missing_model.remove(missing_joint)
        fixtures.append(missing_root)

        duplicate_root = _source_root()
        duplicate_model = _model(duplicate_root)
        duplicate_joint = _required_element(
            duplicate_model, f"./joint[@name='{joint_name}']"
        )
        duplicate_model.append(deepcopy(duplicate_joint))
        fixtures.append(duplicate_root)

    # When/Then: both malformed joint inventories are rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_xml(fixture))


def test_contract_rejects_geometry_visibility_and_second_plugin() -> None:
    # Given: source violations of geometry, visibility, and ownership.
    fixtures: list[Element[str]] = []
    for path, wrong_value in (
        (
            f"./link[@name='{LEFT_LINK}']/collision/geometry/cylinder/radius",
            "0.051",
        ),
        ("./plugin/wheel_radius", "0.051"),
        (
            f"./link[@name='{RIGHT_LINK}']/pose",
            "0 -0.204 -0.13 0 0 0",
        ),
    ):
        root = _source_root()
        _required_element(_model(root), path).text = wrong_value
        fixtures.append(root)

    visible_root = _source_root()
    visible_link = _required_element(
        _model(visible_root), f"./link[@name='{LEFT_LINK}']"
    )
    visible_link.append(Element("visual"))
    fixtures.append(visible_root)

    second_plugin_root = _source_root()
    second_plugin_model = _model(second_plugin_root)
    second_plugin = _required_element(second_plugin_model, "plugin")
    second_plugin_model.append(deepcopy(second_plugin))
    fixtures.append(second_plugin_root)

    # When/Then: every deterministic mutation is rejected.
    for fixture in fixtures:
        _assert_contract_rejected(_xml(fixture))


def test_contract_rejects_malformed_xml() -> None:
    # Given: truncated XML that cannot represent an SDF model.
    malformed_sdf = "<sdf version='1.8'><model name='zero_usv'>"

    # When/Then: the XML parser rejects it before contract inspection.
    rejected = False
    try:
        _assert_diff_drive_contract(malformed_sdf)
    except ElementTree.ParseError:
        rejected = True
    assert rejected
