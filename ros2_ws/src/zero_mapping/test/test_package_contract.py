from __future__ import annotations

from pathlib import Path
import subprocess
from typing import Final
from xml.etree import ElementTree


PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
REPOSITORY_ROOT: Final = PACKAGE_ROOT.parents[2]
GIT_COMMAND: Final = ("env", "GIT_MASTER=1", "git")
EXPECTED_PACKAGES: Final = frozenset(
    {
        "zero_bringup",
        "zero_control",
        "zero_description",
        "zero_gazebo",
        "zero_hardware",
        "zero_interfaces",
        "zero_mapping",
        "zero_navigation",
        "zero_safety",
    }
)
EXPECTED_RUNTIME_DEPENDENCIES: Final = ["slam_toolbox"]
EXPECTED_TEST_DEPENDENCIES: Final = [
    "ament_cmake_pytest",
    "ament_lint_auto",
    "ament_lint_common",
    "python3-yaml",
]
REQUIRED_SOURCE_RESOURCES: Final = frozenset(
    {
        Path("CMakeLists.txt"),
        Path("README.md"),
        Path("config/slam_toolbox.yaml"),
        Path("maps/README.md"),
        Path("package.xml"),
        Path("rviz/mapping.rviz"),
    }
)


def _package_names(manifest_paths: list[Path]) -> list[str]:
    names: list[str] = []
    for manifest_path in manifest_paths:
        name = ElementTree.parse(manifest_path).findtext("name")
        assert name is not None
        names.append(name)
    return names


def _assert_unique_package_names(names: list[str]) -> None:
    assert len(names) == len(set(names))


def test_package_manifest_has_exact_identity_and_dependency_contract() -> None:
    # Given: the source zero_mapping package manifest.
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When: identity, build type, and dependency declarations are inspected.
    buildtool_dependencies = [item.text for item in manifest.findall("buildtool_depend")]
    runtime_dependencies = [item.text for item in manifest.findall("exec_depend")]
    test_dependencies = [item.text for item in manifest.findall("test_depend")]

    # Then: this remains a minimal ament_cmake slam_toolbox resource package.
    assert manifest.findtext("name") == "zero_mapping"
    assert manifest.findtext("./export/build_type") == "ament_cmake"
    assert buildtool_dependencies == ["ament_cmake"]
    assert runtime_dependencies == EXPECTED_RUNTIME_DEPENDENCIES
    assert test_dependencies == EXPECTED_TEST_DEPENDENCIES


def test_cmake_installs_exact_resources_and_registers_static_tests() -> None:
    # Given: the source CMake configuration.
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When/Then: only source resources are installed and all static tests are registered.
    assert cmake.count("project(zero_mapping)") == 1
    assert cmake.count("DIRECTORY config rviz") == 1
    assert "DIRECTORY maps" not in cmake
    assert cmake.count("FILES README.md") == 1
    assert cmake.count("FILES maps/README.md") == 1
    assert cmake.count("DESTINATION share/${PROJECT_NAME}") == 3
    assert cmake.count("DESTINATION share/${PROJECT_NAME}/maps") == 1
    assert cmake.count("find_package(ament_cmake_pytest REQUIRED)") == 1
    assert cmake.count("ament_add_pytest_test(package_contract") == 1
    assert cmake.count("ament_add_pytest_test(slam_toolbox_config") == 1
    assert cmake.count("ament_add_pytest_test(mapping_rviz") == 1
    assert cmake.count("ament_add_pytest_test(map_output_policy") == 1
    assert cmake.count("ament_lint_auto_find_test_dependencies()") == 1


def test_authoritative_source_inventory_contains_exactly_nine_packages() -> None:
    # Given: package manifests from the authoritative source tree only.
    manifest_paths = sorted(SOURCE_ROOT.glob("*/package.xml"))

    # When: package names are parsed without consulting stale build/install trees.
    package_names = _package_names(manifest_paths)

    # Then: adding zero_mapping produces the exact nine-package inventory.
    _assert_unique_package_names(package_names)
    assert len(package_names) == 9
    assert frozenset(package_names) == EXPECTED_PACKAGES


def test_duplicate_package_name_fixture_is_rejected() -> None:
    # Given: a source-derived inventory fixture with one duplicate name.
    package_names = sorted(EXPECTED_PACKAGES)
    package_names[-1] = package_names[0]

    # When/Then: duplicate package identity cannot satisfy the inventory contract.
    rejected = False
    try:
        _assert_unique_package_names(package_names)
    except AssertionError:
        rejected = True
    assert rejected


def test_required_source_resources_exist() -> None:
    # Given: the package's tracked resource contract.
    required_paths = [PACKAGE_ROOT / path for path in REQUIRED_SOURCE_RESOURCES]

    # When/Then: every required source resource is a regular file.
    assert all(path.is_file() for path in required_paths)


def test_required_source_resources_are_visible_to_git() -> None:
    # Given: the Git repository containing the source package.
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

    # When: tracked and non-ignored untracked package files are enumerated.
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

    # Then: every source resource needed to build and install the package is visible.
    assert REQUIRED_SOURCE_RESOURCES <= visible_resources
