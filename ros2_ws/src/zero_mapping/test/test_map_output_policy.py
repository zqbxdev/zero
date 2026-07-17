from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Final


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT: Final = PACKAGE_ROOT.parents[2]
MAPS_ROOT: Final = PACKAGE_ROOT / "maps"
MAPS_README: Final = MAPS_ROOT / "README.md"
GITIGNORE_PATH: Final = REPOSITORY_ROOT / ".gitignore"
GIT_COMMAND: Final = ("env", "GIT_MASTER=1", "git")
MAP_SAVER_COMMAND: Final = (
    "ros2 run nav2_map_server map_saver_cli -f <external-prefix>"
)
GENERATED_IGNORE_RULES: Final = frozenset(
    {
        "/ros2_ws/src/zero_mapping/maps/*.pgm",
        "/ros2_ws/src/zero_mapping/maps/*.png",
        "/ros2_ws/src/zero_mapping/maps/*.yaml",
    }
)
REQUIRED_README_EXCEPTIONS: Final = frozenset(
    {
        "!/ros2_ws/src/zero_mapping/README.md",
        "!/ros2_ws/src/zero_mapping/maps/README.md",
    }
)
FORBIDDEN_BROAD_IGNORE_RULES: Final = frozenset(
    {
        "*.pgm",
        "*.png",
        "*.yaml",
        "ros2_ws/src/zero_mapping/",
        "/ros2_ws/src/zero_mapping/",
        "ros2_ws/src/zero_mapping/maps/",
        "/ros2_ws/src/zero_mapping/maps/",
        "/ros2_ws/src/zero_mapping/maps/*",
        "/ros2_ws/src/zero_mapping/maps/**",
    }
)


def _ignore_rules(text: str) -> frozenset[str]:
    return frozenset(
        line
        for raw_line in text.splitlines()
        if (line := raw_line.strip()) and not line.startswith("#")
    )


def _assert_ignore_contract(rules: frozenset[str]) -> None:
    assert GENERATED_IGNORE_RULES <= rules
    assert REQUIRED_README_EXCEPTIONS <= rules
    assert FORBIDDEN_BROAD_IGNORE_RULES.isdisjoint(rules)


def _assert_ignore_contract_rejected(rules: frozenset[str]) -> None:
    rejected = False
    try:
        _assert_ignore_contract(rules)
    except AssertionError:
        rejected = True
    assert rejected


def _is_ignored(relative_path: Path) -> bool:
    result = subprocess.run(
        [
            *GIT_COMMAND,
            "-c",
            f"safe.directory={REPOSITORY_ROOT}",
            "-C",
            str(REPOSITORY_ROOT),
            "check-ignore",
            "--quiet",
            "--no-index",
            str(relative_path),
        ],
        check=False,
    )
    assert result.returncode in {0, 1}
    return result.returncode == 0


def _assert_maps_readme_contract(text: str) -> None:
    lowered = text.lower()
    assert MAP_SAVER_COMMAND in text
    assert "future runtime procedure only" in lowered
    assert "do not run" in lowered
    assert "external" in lowered
    assert "occupancy map" in lowered
    assert "yaml metadata" in lowered
    assert ".pgm" in lowered
    assert ".png" in lowered
    assert "slam_toolbox pose-graph serialization" in lowered
    assert "separate" in lowered
    assert "does not create a pose graph" in lowered


def _assert_maps_readme_contract_rejected(text: str) -> None:
    rejected = False
    try:
        _assert_maps_readme_contract(text)
    except AssertionError:
        rejected = True
    assert rejected


def test_source_ignore_rules_are_exactly_scoped_to_generated_map_outputs() -> None:
    # Given: the repository ignore rules after zero_mapping became source-visible.
    rules = _ignore_rules(GITIGNORE_PATH.read_text(encoding="utf-8"))

    # When/Then: only generated map result suffixes receive package-local ignores.
    _assert_ignore_contract(rules)


def test_ignore_contract_rejects_broad_package_or_global_patterns() -> None:
    # Given: the required narrow rules plus one unsafe broad pattern at a time.
    required_rules = GENERATED_IGNORE_RULES | REQUIRED_README_EXCEPTIONS

    # When/Then: package, maps-directory, and global suffix ignores are rejected.
    for broad_rule in FORBIDDEN_BROAD_IGNORE_RULES:
        _assert_ignore_contract_rejected(required_rules | {broad_rule})


def test_git_ignores_generated_fixtures_but_preserves_source_resources() -> None:
    # Given: representative generated outputs and required package source paths.
    generated_paths = [
        Path(f"ros2_ws/src/zero_mapping/maps/generated-map{suffix}")
        for suffix in (".pgm", ".png", ".yaml")
    ]
    source_paths = [
        Path("ros2_ws/src/zero_mapping/CMakeLists.txt"),
        Path("ros2_ws/src/zero_mapping/README.md"),
        Path("ros2_ws/src/zero_mapping/config/slam_toolbox.yaml"),
        Path("ros2_ws/src/zero_mapping/maps/README.md"),
        Path("ros2_ws/src/zero_mapping/package.xml"),
        Path("ros2_ws/src/zero_mapping/rviz/mapping.rviz"),
        Path("ros2_ws/src/zero_mapping/test/test_mapping_rviz.py"),
    ]

    # When/Then: generated results are ignored while every source class stays visible.
    assert all(_is_ignored(path) for path in generated_paths)
    assert not any(_is_ignored(path) for path in source_paths)


def test_maps_directory_contains_only_the_installed_policy_readme() -> None:
    # Given: every regular file under the package-local maps convention directory.
    files = {
        path.relative_to(MAPS_ROOT)
        for path in MAPS_ROOT.rglob("*")
        if path.is_file()
    }

    # When/Then: no generated occupancy map or pose graph is packaged as a result.
    assert files == {Path("README.md")}


def test_maps_readme_documents_future_external_occupancy_export_only() -> None:
    # Given: the source map-output policy document.
    text = MAPS_README.read_text(encoding="utf-8")

    # When/Then: its command, warning, output pair, and pose-graph boundary are exact.
    _assert_maps_readme_contract(text)


def test_maps_readme_rejects_pose_graph_confusion_or_missing_future_warning() -> None:
    # Given: source-derived policies missing one required distinction at a time.
    source = MAPS_README.read_text(encoding="utf-8")
    fixtures = [
        source.replace("Future Runtime Procedure Only", "Runtime Procedure"),
        source.replace("Do Not Run", "Run"),
        source.replace("slam_toolbox pose-graph serialization", "map serialization"),
        source.replace("does not create a pose graph", "creates all map formats"),
    ]

    # When/Then: future-only safety and occupancy/pose-graph separation are mandatory.
    for fixture in fixtures:
        _assert_maps_readme_contract_rejected(fixture)
