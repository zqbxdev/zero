from __future__ import annotations

from pathlib import Path
from typing import Final
from xml.etree import ElementTree

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
REPOSITORY_ROOT: Final = PACKAGE_ROOT.parents[2]
EXPECTED_DESCRIPTION: Final = (
    "Nav2-oriented resource package for the Zero unmanned surface vessel."
)
EXPECTED_TEST_DEPENDENCIES: Final = [
    "ament_cmake_pytest",
    "ament_lint_auto",
    "ament_lint_common",
    "python3-yaml",
]
EXPECTED_RUNTIME_DEPENDENCIES: Final = [
    "dwb_core",
    "dwb_critics",
    "dwb_plugins",
    "nav2_amcl",
    "nav2_behaviors",
    "nav2_bringup",
    "nav2_bt_navigator",
    "nav2_controller",
    "nav2_costmap_2d",
    "nav2_map_server",
    "nav2_navfn_planner",
    "nav2_planner",
    "nav2_rviz_plugins",
    "rviz2",
    "rviz_default_plugins",
]
README_EXCEPTION: Final = "!/ros2_ws/src/zero_navigation/README.md"
LEGACY_EXECUTABLE: Final = "waypoint" + "_follower"
LEGACY_TOPIC: Final = "/cmd" + "_vel"
LEGACY_DESCRIPTION: Final = "Odom" + "-only"
LEGACY_ENTRY_POINT_GROUP: Final = "console" + "_scripts"
LEGACY_BUILD_TYPE: Final = "ament_" + "python"


def _assert_unique_package_names(package_names: list[str]) -> None:
    assert len(package_names) == len(set(package_names))


def _assert_manifest_contract(manifest: ElementTree.Element) -> None:
    build_types = [item.text for item in manifest.findall("./export/build_type")]
    buildtool_dependencies = [
        item.text for item in manifest.findall("buildtool_depend")
    ]
    runtime_dependencies = [item.text for item in manifest.findall("exec_depend")]
    test_dependencies = [item.text for item in manifest.findall("test_depend")]

    assert manifest.findtext("name") == "zero_navigation"
    assert manifest.findtext("description") == EXPECTED_DESCRIPTION
    assert build_types == ["ament_cmake"]
    assert buildtool_dependencies == ["ament_cmake"]
    assert runtime_dependencies == EXPECTED_RUNTIME_DEPENDENCIES
    assert len(runtime_dependencies) == len(set(runtime_dependencies))
    assert test_dependencies == EXPECTED_TEST_DEPENDENCIES


def _assert_forbidden_symbols_absent(source_text: str) -> None:
    for forbidden in (
        LEGACY_EXECUTABLE,
        LEGACY_TOPIC,
        LEGACY_DESCRIPTION,
        LEGACY_ENTRY_POINT_GROUP,
    ):
        assert forbidden not in source_text


def test_manifest_declares_minimal_resource_package_contract() -> None:
    # Given: the authoritative zero_navigation source manifest.
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()

    # When/Then: identity, build type, description, and dependencies are exact.
    _assert_manifest_contract(manifest)


def test_malformed_manifest_fixture_is_rejected() -> None:
    # Given: a duplicate-build-type legacy manifest fixture.
    malformed_manifest = ElementTree.fromstring(
        "".join(
            (
                "<package><name>zero_navigation</name>",
                f"<description>{LEGACY_DESCRIPTION} fixture</description>",
                f"<buildtool_depend>{LEGACY_BUILD_TYPE}</buildtool_depend>",
                f"<export><build_type>{LEGACY_BUILD_TYPE}</build_type>",
                "<build_type>ament_cmake</build_type></export></package>",
            )
        )
    )

    # When/Then: it cannot satisfy the converted package contract.
    rejected = False
    try:
        _assert_manifest_contract(malformed_manifest)
    except AssertionError:
        rejected = True
    assert rejected


def test_manifest_contract_rejects_missing_or_duplicate_runtime_dependency() -> None:
    # Given: source-derived manifests with one missing or duplicate runtime dependency.
    source_manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    fixtures: list[ElementTree.Element] = []
    missing_dependency = ElementTree.fromstring(
        ElementTree.tostring(source_manifest, encoding="unicode")
    )
    first_runtime = missing_dependency.find("exec_depend")
    assert first_runtime is not None
    missing_dependency.remove(first_runtime)
    fixtures.append(missing_dependency)
    duplicate_dependency = ElementTree.fromstring(
        ElementTree.tostring(source_manifest, encoding="unicode")
    )
    ElementTree.SubElement(duplicate_dependency, "exec_depend").text = (
        EXPECTED_RUNTIME_DEPENDENCIES[0]
    )
    fixtures.append(duplicate_dependency)

    # When/Then: exact direct runtime ownership rejects both malformed manifests.
    for fixture in fixtures:
        rejected = False
        try:
            _assert_manifest_contract(fixture)
        except AssertionError:
            rejected = True
        assert rejected


def test_cmake_installs_resources_and_registers_static_tests() -> None:
    # Given: the source CMake configuration.
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When/Then: the package installs its README, Nav2 parameters, and RViz config.
    assert cmake.count("project(zero_navigation)") == 1
    assert cmake.count("find_package(ament_cmake REQUIRED)") == 1
    assert cmake.count("DIRECTORY config rviz") == 1
    assert cmake.count("FILES README.md") == 1
    assert cmake.count("DESTINATION share/${PROJECT_NAME}") == 2
    assert "DIRECTORY launch" not in cmake

    # Then: pytest and package lint are registered without runtime launch tests.
    assert cmake.count("find_package(ament_cmake_pytest REQUIRED)") == 1
    assert cmake.count("ament_add_pytest_test(package_contract") == 1
    assert cmake.count("ament_add_pytest_test(nav2_params") == 1
    assert cmake.count("ament_add_pytest_test(navigation_ownership") == 1
    assert cmake.count("ament_add_pytest_test(navigation_rviz") == 1
    assert cmake.count("ament_add_pytest_test(resource_visibility") == 1
    assert cmake.count("ament_lint_auto_find_test_dependencies()") == 1
    assert cmake.count("ament_package()") == 1


def test_source_inventory_contains_no_legacy_python_package_or_tests() -> None:
    # Given: every active path in the source package.
    non_test_python = sorted(
        path.relative_to(PACKAGE_ROOT)
        for path in PACKAGE_ROOT.rglob("*.py")
        if "test" not in path.parts and "__pycache__" not in path.parts
    )
    test_python = sorted(
        path.relative_to(PACKAGE_ROOT)
        for path in (PACKAGE_ROOT / "test").glob("*.py")
    )

    # When/Then: only the package and Nav2 parameter contracts are Python source.
    assert non_test_python == []
    assert test_python == [
        Path("test/__init__.py"),
        Path("test/nav2_params_contract.py"),
        Path("test/nav2_params_server_contract.py"),
        Path("test/test_nav2_params.py"),
        Path("test/test_navigation_ownership.py"),
        Path("test/test_navigation_rviz.py"),
        Path("test/test_package_contract.py"),
        Path("test/test_resource_visibility.py"),
        Path("test/test_static_registration.py"),
    ]
    assert not (PACKAGE_ROOT / "setup.py").exists()
    assert not (PACKAGE_ROOT / "setup.cfg").exists()
    assert not (PACKAGE_ROOT / "resource").exists()
    assert not (PACKAGE_ROOT / "zero_navigation").exists()


def test_active_resource_sources_contain_no_forbidden_legacy_contract() -> None:
    # Given: only active package metadata and installed resource source.
    active_sources = (
        PACKAGE_ROOT / "CMakeLists.txt",
        PACKAGE_ROOT / "README.md",
        PACKAGE_ROOT / "package.xml",
    )

    # When: their text is combined for a static forbidden-symbol scan.
    source_text = "\n".join(
        path.read_text(encoding="utf-8") for path in active_sources
    )

    # Then: no executable, topic publisher, stale description, or entry point remains.
    _assert_forbidden_symbols_absent(source_text)


def test_legacy_source_fixture_is_rejected() -> None:
    # Given: a synthetic source fixture representing the removed publisher.
    legacy_source = (
        f"name={LEGACY_EXECUTABLE}\n"
        f"topic={LEGACY_TOPIC}\n"
        f"entry_points={LEGACY_ENTRY_POINT_GROUP}"
    )

    # When/Then: the forbidden-symbol contract rejects the stale fixture.
    rejected = False
    try:
        _assert_forbidden_symbols_absent(legacy_source)
    except AssertionError:
        rejected = True
    assert rejected


def test_authoritative_source_has_one_unique_navigation_package() -> None:
    # Given: source manifests only, excluding stale build and install trees.
    package_names = [
        name
        for manifest_path in sorted(SOURCE_ROOT.glob("*/package.xml"))
        if (name := ElementTree.parse(manifest_path).findtext("name")) is not None
    ]

    # When/Then: package identities are unique and zero_navigation appears once.
    _assert_unique_package_names(package_names)
    assert package_names.count("zero_navigation") == 1


def test_duplicate_package_name_fixture_is_rejected() -> None:
    # Given: a source-derived package fixture with a duplicate navigation identity.
    package_names = ["zero_navigation", "zero_navigation"]

    # When/Then: duplicate package discovery cannot satisfy the contract.
    rejected = False
    try:
        _assert_unique_package_names(package_names)
    except AssertionError:
        rejected = True
    assert rejected


def test_readme_has_exact_git_visibility_exception() -> None:
    # Given: the repository ignore policy and required package README.
    ignore_lines = (REPOSITORY_ROOT / ".gitignore").read_text(
        encoding="utf-8"
    ).splitlines()

    # When/Then: only the exact navigation README exception is required here.
    assert ignore_lines.count(README_EXCEPTION) == 1
    assert (PACKAGE_ROOT / "README.md").is_file()
