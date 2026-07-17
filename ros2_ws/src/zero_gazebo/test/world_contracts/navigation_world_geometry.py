from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Final
from xml.etree import ElementTree
from xml.etree.ElementTree import Element


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[2]
WORLD_PATH: Final = PACKAGE_ROOT / "worlds" / "zero_navigation_world.sdf"
HULL_LENGTH_M: Final = 1.20
HULL_WIDTH_M: Final = 0.55
FOOTPRINT_PADDING_M: Final = 0.15
EXTRA_CORRIDOR_CLEARANCE_M: Final = 0.40
PADDED_HULL_LENGTH_M: Final = HULL_LENGTH_M + (2 * FOOTPRINT_PADDING_M)
PADDED_HULL_WIDTH_M: Final = HULL_WIDTH_M + (2 * FOOTPRINT_PADDING_M)
MIN_CORRIDOR_WIDTH_M: Final = (
    PADDED_HULL_WIDTH_M + EXTRA_CORRIDOR_CLEARANCE_M
)
MIN_SPAWN_CLEARANCE_M: Final = 0.25
LIDAR_RANGE_M: Final = 8.0
BOUNDARY_NAMES: Final = frozenset(
    {
        "boundary_north",
        "boundary_south",
        "boundary_east",
        "boundary_west",
    }
)
CORRIDOR_PAIRS: Final = (
    ("dock_finger", "column_center"),
    ("column_center", "bypass_obstacle_beta"),
    ("bypass_obstacle_alpha", "box_east"),
)
BYPASS_NAMES: Final = (
    "bypass_obstacle_alpha",
    "bypass_obstacle_beta",
)
LANDMARK_PREFIXES: Final = ("box_", "column_", "buoy_")


@dataclass(frozen=True, slots=True)
class Geometry:
    name: str
    kind: str
    x: float
    y: float
    z: float
    size_x: float
    size_y: float
    size_z: float

    @property
    def min_x(self) -> float:
        return self.x - (self.size_x / 2)

    @property
    def max_x(self) -> float:
        return self.x + (self.size_x / 2)

    @property
    def min_y(self) -> float:
        return self.y - (self.size_y / 2)

    @property
    def max_y(self) -> float:
        return self.y + (self.size_y / 2)


def numbers(text: str | None, count: int) -> tuple[float, ...]:
    assert text is not None
    values = tuple(float(token) for token in text.split())
    assert len(values) == count
    assert all(math.isfinite(value) for value in values)
    return values


def required(parent: Element[str], path: str) -> Element[str]:
    element = parent.find(path)
    assert element is not None
    return element


def parse_model(model: Element[str]) -> Geometry:
    name = model.attrib["name"]
    assert model.findtext("static") == "true"
    x, y, z, _roll, _pitch, _yaw = numbers(model.findtext("pose"), 6)
    links = model.findall("link")
    assert len(links) == 1
    collisions = links[0].findall("collision")
    visuals = links[0].findall("visual")
    assert len(collisions) == 1
    assert len(visuals) == 1
    collision_geometry = required(collisions[0], "geometry")
    visual_geometry = required(visuals[0], "geometry")
    box = collision_geometry.find("box")
    cylinder = collision_geometry.find("cylinder")
    plane = collision_geometry.find("plane")
    assert sum(item is not None for item in (box, cylinder, plane)) == 1

    if box is not None:
        size_x, size_y, size_z = numbers(box.findtext("size"), 3)
        assert min(size_x, size_y, size_z) > 0
        assert visual_geometry.findtext("box/size") == box.findtext("size")
        return Geometry(name, "box", x, y, z, size_x, size_y, size_z)

    if cylinder is not None:
        radius = numbers(cylinder.findtext("radius"), 1)[0]
        length = numbers(cylinder.findtext("length"), 1)[0]
        assert radius > 0
        assert length > 0
        assert visual_geometry.findtext("cylinder/radius") == str(radius)
        assert visual_geometry.findtext("cylinder/length") == str(length)
        return Geometry(
            name,
            "cylinder",
            x,
            y,
            z,
            2 * radius,
            2 * radius,
            length,
        )

    assert plane is not None
    size_x, size_y = numbers(plane.findtext("size"), 2)
    assert min(size_x, size_y) > 0
    assert visual_geometry.findtext("plane/size") == plane.findtext("size")
    return Geometry(name, "plane", x, y, z, size_x, size_y, 0.0)


def geometry_by_name(
    geometries: tuple[Geometry, ...],
    name: str,
) -> Geometry:
    matches = tuple(
        geometry for geometry in geometries if geometry.name == name
    )
    assert len(matches) == 1
    return matches[0]


def gap_x(left: Geometry, right: Geometry) -> float:
    assert left.max_x <= right.min_x
    return right.min_x - left.max_x


def _overlaps_y(first: Geometry, second: Geometry) -> bool:
    return first.min_y < second.max_y and second.min_y < first.max_y


def route_widths(
    obstacle: Geometry,
    blockers: tuple[Geometry, ...],
) -> tuple[float, float]:
    left_edges = tuple(
        blocker.max_x
        for blocker in blockers
        if blocker.name != obstacle.name
        and blocker.max_x <= obstacle.min_x
        and _overlaps_y(blocker, obstacle)
    )
    right_edges = tuple(
        blocker.min_x
        for blocker in blockers
        if blocker.name != obstacle.name
        and blocker.min_x >= obstacle.max_x
        and _overlaps_y(blocker, obstacle)
    )
    assert left_edges
    assert right_edges
    left_width = obstacle.min_x - max(left_edges)
    right_width = min(right_edges) - obstacle.max_x
    return left_width, right_width


def aabb_distance(first: Geometry, second: Geometry) -> float:
    gap_x_m = max(
        second.min_x - first.max_x,
        first.min_x - second.max_x,
        0.0,
    )
    gap_y_m = max(
        second.min_y - first.max_y,
        first.min_y - second.max_y,
        0.0,
    )
    return math.hypot(gap_x_m, gap_y_m)


def load_world_root() -> Element[str]:
    return ElementTree.parse(WORLD_PATH).getroot()
