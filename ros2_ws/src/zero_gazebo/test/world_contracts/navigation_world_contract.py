from __future__ import annotations

import math
from dataclasses import dataclass
from xml.etree.ElementTree import Element

from .navigation_world_geometry import (
    BOUNDARY_NAMES,
    BYPASS_NAMES,
    CORRIDOR_PAIRS,
    LANDMARK_PREFIXES,
    LIDAR_RANGE_M,
    MIN_CORRIDOR_WIDTH_M,
    MIN_SPAWN_CLEARANCE_M,
    PADDED_HULL_LENGTH_M,
    PADDED_HULL_WIDTH_M,
    Geometry,
    aabb_distance,
    gap_x,
    geometry_by_name,
    numbers,
    parse_model,
    required,
    route_widths,
)


@dataclass(frozen=True, slots=True)
class WorldAnalysis:
    geometries: tuple[Geometry, ...]
    spawn: Geometry
    boundary_count: int
    box_count: int
    column_count: int
    buoy_count: int
    spawn_clearance_m: float
    corridor_widths_m: tuple[float, ...]
    bypass_routes_m: tuple[tuple[str, float, float], ...]
    asymmetry_metric: float
    landmark_ranges_m: tuple[tuple[str, float], ...]


def _validate_world_metadata(world: Element[str]) -> None:
    assert world.attrib == {"name": "zero_navigation_world"}
    named_elements = tuple(
        element.attrib["name"]
        for element in world.iter()
        if "name" in element.attrib
    )
    assert len(named_elements) == len(set(named_elements))
    assert world.findall(".//mesh") == []
    assert world.findall(".//wind") == []
    plugin_text = " ".join(
        " ".join(plugin.attrib.values()).lower()
        for plugin in world.findall("plugin")
    )
    assert not any(
        forbidden in plugin_text
        for forbidden in ("wave", "wind", "current", "hydrodynamic")
    )


def _validate_boundaries(geometries: tuple[Geometry, ...]) -> int:
    boundaries = tuple(
        geometry
        for geometry in geometries
        if geometry.name in BOUNDARY_NAMES
    )
    assert {boundary.name for boundary in boundaries} == BOUNDARY_NAMES
    expected_boundaries = (
        ("boundary_west", -9.0, 0.0, 0.30, 14.0),
        ("boundary_east", 9.0, 0.0, 0.30, 14.0),
        ("boundary_south", 0.0, -7.0, 18.0, 0.30),
        ("boundary_north", 0.0, 7.0, 18.0, 0.30),
    )
    for name, x, y, size_x, size_y in expected_boundaries:
        boundary = geometry_by_name(geometries, name)
        assert (boundary.x, boundary.y) == (x, y)
        assert (boundary.size_x, boundary.size_y) == (size_x, size_y)
    return len(boundaries)


def _obstacles(geometries: tuple[Geometry, ...]) -> tuple[Geometry, ...]:
    obstacles = tuple(
        geometry
        for geometry in geometries
        if geometry.kind != "plane" and geometry.name not in BOUNDARY_NAMES
    )
    assert all(
        obstacle.min_x >= -8.85
        and obstacle.max_x <= 8.85
        and obstacle.min_y >= -6.85
        and obstacle.max_y <= 6.85
        for obstacle in obstacles
    )
    return obstacles


def _count_prefix(obstacles: tuple[Geometry, ...], prefix: str) -> int:
    return sum(obstacle.name.startswith(prefix) for obstacle in obstacles)


def _spawn_analysis(
    world: Element[str],
    obstacles: tuple[Geometry, ...],
) -> tuple[Geometry, float]:
    includes = world.findall("include")
    assert len(includes) == 1
    assert includes[0].findtext("uri") == "model://zero_usv"
    assert includes[0].findtext("name") == "zero_usv"
    spawn_values = numbers(includes[0].findtext("pose"), 6)
    assert spawn_values[3:] == (0.0, 0.0, 0.0)
    spawn = Geometry(
        "zero_usv",
        "spawn",
        spawn_values[0],
        spawn_values[1],
        spawn_values[2],
        PADDED_HULL_LENGTH_M,
        PADDED_HULL_WIDTH_M,
        0.20,
    )
    clearance = min(aabb_distance(spawn, obstacle) for obstacle in obstacles)
    assert clearance >= MIN_SPAWN_CLEARANCE_M
    return spawn, clearance


def _corridor_widths(
    geometries: tuple[Geometry, ...],
) -> tuple[float, ...]:
    widths = tuple(
        gap_x(
            geometry_by_name(geometries, left_name),
            geometry_by_name(geometries, right_name),
        )
        for left_name, right_name in CORRIDOR_PAIRS
    )
    assert min(widths) >= MIN_CORRIDOR_WIDTH_M
    return widths


def _bypass_routes(
    geometries: tuple[Geometry, ...],
) -> tuple[tuple[str, float, float], ...]:
    blockers = tuple(
        geometry for geometry in geometries if geometry.kind != "plane"
    )
    routes = tuple(
        (
            name,
            *route_widths(geometry_by_name(geometries, name), blockers),
        )
        for name in BYPASS_NAMES
    )
    assert all(
        left >= MIN_CORRIDOR_WIDTH_M and right >= MIN_CORRIDOR_WIDTH_M
        for _name, left, right in routes
    )
    return routes


def _asymmetry_metric(obstacles: tuple[Geometry, ...]) -> float:
    mirrored_count = sum(
        any(
            candidate.name != obstacle.name
            and math.isclose(candidate.x, -obstacle.x, abs_tol=1e-9)
            and math.isclose(candidate.y, -obstacle.y, abs_tol=1e-9)
            and candidate.size_x == obstacle.size_x
            and candidate.size_y == obstacle.size_y
            for candidate in obstacles
        )
        for obstacle in obstacles
    )
    metric = 1.0 - (mirrored_count / len(obstacles))
    assert metric >= 0.75
    return metric


def _landmark_ranges(
    obstacles: tuple[Geometry, ...],
    spawn: Geometry,
) -> tuple[tuple[str, float], ...]:
    landmarks = tuple(
        obstacle
        for obstacle in obstacles
        if obstacle.name.startswith(LANDMARK_PREFIXES)
    )
    ranges = tuple(
        (
            landmark.name,
            math.hypot(landmark.x - spawn.x, landmark.y - spawn.y),
        )
        for landmark in landmarks
    )
    assert len(ranges) >= 9
    assert max(distance for _name, distance in ranges) <= LIDAR_RANGE_M
    return ranges


def analyze_world(root: Element[str]) -> WorldAnalysis:
    assert root.tag == "sdf"
    assert root.attrib == {"version": "1.8"}
    world = required(root, "world")
    _validate_world_metadata(world)
    geometries = tuple(parse_model(model) for model in world.findall("model"))
    assert len(geometries) == len({geometry.name for geometry in geometries})
    boundary_count = _validate_boundaries(geometries)
    obstacles = _obstacles(geometries)
    box_count = _count_prefix(obstacles, "box_")
    column_count = _count_prefix(obstacles, "column_")
    buoy_count = _count_prefix(obstacles, "buoy_")
    assert min(box_count, column_count, buoy_count) >= 3
    assert _count_prefix(obstacles, "shoreline_") >= 2
    assert _count_prefix(obstacles, "dock_") >= 2
    spawn, spawn_clearance = _spawn_analysis(world, obstacles)
    corridor_widths = _corridor_widths(geometries)
    bypass_routes = _bypass_routes(geometries)
    asymmetry_metric = _asymmetry_metric(obstacles)
    landmark_ranges = _landmark_ranges(obstacles, spawn)
    return WorldAnalysis(
        geometries=geometries,
        spawn=spawn,
        boundary_count=boundary_count,
        box_count=box_count,
        column_count=column_count,
        buoy_count=buoy_count,
        spawn_clearance_m=spawn_clearance,
        corridor_widths_m=corridor_widths,
        bypass_routes_m=bypass_routes,
        asymmetry_metric=asymmetry_metric,
        landmark_ranges_m=landmark_ranges,
    )
