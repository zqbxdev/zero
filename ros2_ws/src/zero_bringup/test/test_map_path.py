from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest


def _validator() -> Callable[[str], Path]:
    from zero_bringup.map_path import validate_map_path

    return validate_map_path


@pytest.mark.parametrize("raw_path", ["", "   ", "maps/site.yaml"])
def test_map_path_rejects_empty_or_relative_input(raw_path: str) -> None:
    # Given: a missing, blank, or relative map argument.
    validate_map_path = _validator()

    # When/Then: validation rejects it before Nav2 composition.
    with pytest.raises(ValueError):
        validate_map_path(raw_path)


@pytest.mark.parametrize("suffix", [".yml", ".YAML", ".txt"])
def test_map_path_rejects_non_lowercase_yaml_suffix(
    tmp_path: Path,
    suffix: str,
) -> None:
    # Given: an absolute existing file without the exact lowercase .yaml suffix.
    map_path = tmp_path / f"map{suffix}"
    map_path.write_text("image: map.pgm\n", encoding="utf-8")
    validate_map_path = _validator()

    # When/Then: the explicit lowercase extension policy rejects it.
    with pytest.raises(ValueError):
        validate_map_path(str(map_path))


def test_map_path_rejects_nonexistent_absolute_yaml(tmp_path: Path) -> None:
    # Given: an absolute lowercase YAML path that does not exist.
    missing_path = tmp_path / "missing.yaml"
    validate_map_path = _validator()

    # When/Then: validation rejects the nonexistent external map.
    with pytest.raises(ValueError):
        validate_map_path(str(missing_path))


def test_map_path_rejects_directory_with_yaml_suffix(tmp_path: Path) -> None:
    # Given: an absolute existing directory named with a YAML suffix.
    directory_path = tmp_path / "directory.yaml"
    directory_path.mkdir()
    validate_map_path = _validator()

    # When/Then: only regular-file-compatible paths are accepted.
    with pytest.raises(ValueError):
        validate_map_path(str(directory_path))


def test_map_path_accepts_absolute_existing_lowercase_yaml(tmp_path: Path) -> None:
    # Given: a synthetic map YAML fixture isolated under pytest's temp root.
    map_path = tmp_path / "valid.yaml"
    map_path.write_text("image: map.pgm\n", encoding="utf-8")
    validate_map_path = _validator()

    # When: the external path is parsed at the launch boundary.
    validated = validate_map_path(str(map_path))

    # Then: the original absolute path is returned without creating resources.
    assert validated == map_path
