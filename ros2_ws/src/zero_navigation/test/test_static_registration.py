from __future__ import annotations

from pathlib import Path
import re
from typing import Final

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
PYTEST_REGISTRATION: Final = re.compile(
    r"ament_add_pytest_test\((?P<name>[^\s]+)\s+(?P<path>[^)]+)\)"
)


def _registered_tests(cmake: str) -> tuple[tuple[str, Path], ...]:
    return tuple(
        (match.group("name"), Path(match.group("path")))
        for match in PYTEST_REGISTRATION.finditer(cmake)
    )


def _assert_static_registration(
    cmake: str,
    expected_paths: set[Path],
) -> None:
    registrations = _registered_tests(cmake)
    names = [name for name, _ in registrations]
    paths = [path for _, path in registrations]
    assert len(names) == len(set(names))
    assert len(paths) == len(set(paths))
    assert set(paths) == expected_paths
    assert "launch_testing" not in cmake
    assert "DIRECTORY test" not in cmake


def test_cmake_registers_every_static_pytest_exactly_once() -> None:
    # Given: every collectable package test and the package CMake source.
    expected_paths = {
        path.relative_to(PACKAGE_ROOT)
        for path in (PACKAGE_ROOT / "test").glob("test_*.py")
    }
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    # When/Then: every static test is registered once and no test tree is installed.
    _assert_static_registration(cmake, expected_paths)


def test_static_registration_rejects_missing_or_duplicate_fixture() -> None:
    # Given: CMake fixtures missing one test or registering one path twice.
    malformed_fixtures = (
        "ament_add_pytest_test(first test/test_first.py)\n",
        (
            "ament_add_pytest_test(first test/test_first.py)\n"
            "ament_add_pytest_test(duplicate test/test_first.py)\n"
            "ament_add_pytest_test(second test/test_second.py)\n"
        ),
    )
    expected_paths = {Path("test/test_first.py"), Path("test/test_second.py")}

    # When/Then: package-native static registration fails closed.
    for malformed in malformed_fixtures:
        rejected = False
        try:
            _assert_static_registration(malformed, expected_paths)
        except AssertionError:
            rejected = True
        assert rejected
