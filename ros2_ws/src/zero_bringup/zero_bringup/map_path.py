from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Final, NewType


MapYamlPath = NewType("MapYamlPath", Path)
YAML_SUFFIX: Final = ".yaml"


class MapPathViolation(str, Enum):
    EMPTY = "map path is required"
    RELATIVE = "map path must be absolute"
    SUFFIX = "map path must use the lowercase .yaml suffix"
    MISSING = "map path does not exist"
    NOT_FILE = "map path must identify a file"


@dataclass(frozen=True, slots=True)
class InvalidMapPathError(ValueError):
    raw_path: str
    violation: MapPathViolation

    def __str__(self) -> str:
        return f"invalid map path {self.raw_path!r}: {self.violation.value}"


def validate_map_path(raw_path: str) -> MapYamlPath:
    normalized_path = raw_path.strip()
    if not normalized_path:
        raise InvalidMapPathError(raw_path, MapPathViolation.EMPTY)

    map_path = Path(normalized_path)
    if not map_path.is_absolute():
        raise InvalidMapPathError(raw_path, MapPathViolation.RELATIVE)
    if map_path.suffix != YAML_SUFFIX:
        raise InvalidMapPathError(raw_path, MapPathViolation.SUFFIX)
    if not map_path.exists():
        raise InvalidMapPathError(raw_path, MapPathViolation.MISSING)
    if not map_path.is_file():
        raise InvalidMapPathError(raw_path, MapPathViolation.NOT_FILE)
    return MapYamlPath(map_path)
