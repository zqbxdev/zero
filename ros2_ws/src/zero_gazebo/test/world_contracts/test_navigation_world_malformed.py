from __future__ import annotations

from copy import deepcopy
from xml.etree import ElementTree

from .navigation_world_contract import analyze_world
from .navigation_world_geometry import load_world_root


def _world(root: ElementTree.Element[str]) -> ElementTree.Element[str]:
    world = root.find("world")
    assert world is not None
    return world


def _assert_invalid_world(root: ElementTree.Element[str]) -> None:
    try:
        _ = analyze_world(root)
    except AssertionError:
        return
    raise AssertionError(
        "malformed world unexpectedly passed geometry contract"
    )


def test_duplicate_geometry_name_is_rejected() -> None:
    # Given: a world with a duplicated boundary model and component names.
    root = load_world_root()
    world = _world(root)
    boundary = world.find("./model[@name='boundary_north']")
    assert boundary is not None
    world.append(deepcopy(boundary))

    # When / Then: deterministic name validation rejects the duplicate.
    _assert_invalid_world(root)


def test_malformed_pose_is_rejected() -> None:
    # Given: a world model pose with fewer than six finite values.
    root = load_world_root()
    pose = _world(root).find("./model[@name='box_east']/pose")
    assert pose is not None
    pose.text = "5.5 -1.8"

    # When / Then: geometry parsing rejects the malformed pose.
    _assert_invalid_world(root)


def test_nonpositive_size_is_rejected() -> None:
    # Given: a fixed obstacle with an invalid negative box dimension.
    root = load_world_root()
    size = _world(root).find(
        "./model[@name='box_east']/link/collision/geometry/box/size"
    )
    assert size is not None
    size.text = "-0.8 0.8 0.8"

    # When / Then: primitive dimension validation rejects the obstacle.
    _assert_invalid_world(root)


def test_intersecting_spawn_is_rejected() -> None:
    # Given: the padded zero_usv spawn moved into bypass_obstacle_alpha.
    root = load_world_root()
    pose = _world(root).find("include/pose")
    assert pose is not None
    pose.text = "0 -1.5 0.18 0 0 0"

    # When / Then: padded spawn separation rejects the intersection.
    _assert_invalid_world(root)


def test_missing_boundary_is_rejected() -> None:
    # Given: the bounded world with its east boundary removed.
    root = load_world_root()
    world = _world(root)
    boundary = world.find("./model[@name='boundary_east']")
    assert boundary is not None
    world.remove(boundary)

    # When / Then: the four-sided boundary contract rejects the world.
    _assert_invalid_world(root)


def test_narrow_corridor_is_rejected() -> None:
    # Given: bypass_obstacle_beta shifted into the center corridor.
    root = load_world_root()
    pose = _world(root).find("./model[@name='bypass_obstacle_beta']/pose")
    assert pose is not None
    pose.text = "2.2 1.2 0.50 0 0 0"

    # When / Then: the padded-hull corridor threshold rejects the route.
    _assert_invalid_world(root)


def test_malformed_xml_is_rejected() -> None:
    # Given: a truncated SDF document.
    malformed_sdf = "<sdf version='1.8'><world name='zero_navigation_world'>"

    # When / Then: ElementTree rejects the malformed source boundary.
    try:
        _ = ElementTree.fromstring(malformed_sdf)
    except ElementTree.ParseError:
        return
    raise AssertionError("malformed XML unexpectedly parsed")
