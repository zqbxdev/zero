from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Final

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT: Final = PACKAGE_ROOT.parents[2]
GIT_COMMAND: Final = ("env", "GIT_MASTER=1", "git")
REQUIRED_RESOURCES: Final = {
    Path("CMakeLists.txt"),
    Path("README.md"),
    Path("config/nav2_params.yaml"),
    Path("package.xml"),
    Path("rviz/navigation.rviz"),
}


def test_navigation_resources_are_regular_files_visible_to_git() -> None:
    # Given: the source package and repository Git visibility boundary.
    required_paths = {PACKAGE_ROOT / path for path in REQUIRED_RESOURCES}
    repository_result = subprocess.run(
        [
            *GIT_COMMAND,
            "-c",
            f"safe.directory={REPOSITORY_ROOT}",
            "-C",
            str(PACKAGE_ROOT),
            "rev-parse",
            "--show-toplevel",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    repository_root = Path(repository_result.stdout.strip())
    assert repository_root == REPOSITORY_ROOT
    package_relative = PACKAGE_ROOT.relative_to(repository_root)

    # When: tracked and non-ignored untracked package resources are enumerated.
    visible_result = subprocess.run(
        [
            *GIT_COMMAND,
            "-c",
            f"safe.directory={repository_root}",
            "-C",
            str(repository_root),
            "ls-files",
            "--cached",
            "--others",
            "--exclude-standard",
            "--",
            str(package_relative),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    visible_resources = {
        Path(line).relative_to(package_relative)
        for line in visible_result.stdout.splitlines()
    }

    # Then: build inputs, README, and navigation RViz are present and visible.
    assert all(path.is_file() for path in required_paths)
    assert REQUIRED_RESOURCES <= visible_resources
