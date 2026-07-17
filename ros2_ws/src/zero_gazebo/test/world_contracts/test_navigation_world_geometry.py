from __future__ import annotations

import math

from .navigation_world_contract import analyze_world
from .navigation_world_geometry import (
    LIDAR_RANGE_M,
    MIN_CORRIDOR_WIDTH_M,
    MIN_SPAWN_CLEARANCE_M,
    load_world_root,
)


def test_navigation_world_satisfies_the_complete_geometry_contract() -> None:
    # Given: the deterministic source navigation world.
    root = load_world_root()

    # When: every static geometry predicate is evaluated.
    analysis = analyze_world(root)

    # Then: the bounded world has required classes and safe navigation space.
    assert analysis.boundary_count == 4
    assert analysis.box_count >= 3
    assert analysis.column_count >= 3
    assert analysis.buoy_count >= 3
    assert analysis.spawn_clearance_m >= MIN_SPAWN_CLEARANCE_M
    assert min(analysis.corridor_widths_m) >= MIN_CORRIDOR_WIDTH_M
    assert all(
        left >= MIN_CORRIDOR_WIDTH_M and right >= MIN_CORRIDOR_WIDTH_M
        for _name, left, right in analysis.bypass_routes_m
    )
    assert analysis.asymmetry_metric >= 0.75
    assert max(
        distance for _name, distance in analysis.landmark_ranges_m
    ) <= LIDAR_RANGE_M


def test_navigation_world_geometry_values_are_deterministic() -> None:
    # Given: two independent parses of the source navigation world.
    first_root = load_world_root()
    second_root = load_world_root()

    # When: both documents are analyzed.
    first = analyze_world(first_root)
    second = analyze_world(second_root)

    # Then: poses, sizes, clearances, routes, and ranges are identical.
    assert first == second
    assert math.isclose(first.spawn.x, -2.0)
    assert math.isclose(first.spawn.y, -3.8)
