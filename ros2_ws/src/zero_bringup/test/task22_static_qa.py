from __future__ import annotations

import ast
from pathlib import Path
from tempfile import TemporaryDirectory

from zero_bringup.map_path import InvalidMapPathError, validate_map_path

from .gazebo_control_launch_contract import assert_gazebo_control_contract
from .gazebo_navigation_launch_contract import (
    assert_gazebo_navigation_contract,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "gazebo_navigation.launch.py"
CONTROL_LAUNCH_PATH = PACKAGE_ROOT / "launch" / "gazebo_control.launch.py"


def _calls(tree: ast.AST, name: str) -> tuple[ast.Call, ...]:
    return tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        if isinstance(node.func, ast.Name)
        if node.func.id == name
    )


def _optional_keyword(call: ast.Call, name: str) -> ast.expr | None:
    matches = [keyword.value for keyword in call.keywords if keyword.arg == name]
    assert len(matches) <= 1
    return matches[0] if matches else None


def _required_keyword(call: ast.Call, name: str) -> ast.expr:
    value = _optional_keyword(call, name)
    assert value is not None
    return value


def _string(expression: ast.expr) -> str:
    assert isinstance(expression, ast.Constant)
    assert isinstance(expression.value, str)
    return expression.value


def _include_keys(call: ast.Call) -> tuple[str, ...]:
    arguments = _required_keyword(call, "launch_arguments")
    assert isinstance(arguments, ast.Call)
    assert isinstance(arguments.func, ast.Attribute)
    entries = arguments.func.value
    assert isinstance(entries, ast.Dict)
    return tuple(sorted(_string(key) for key in entries.keys if key is not None))


def _print_map_validation() -> None:
    with TemporaryDirectory(prefix="task22-map-") as temp_dir:
        fixture_root = Path(temp_dir)
        non_yaml = fixture_root / "map.yml"
        non_yaml.write_text("image: map.pgm\n", encoding="utf-8")
        missing = fixture_root / "missing.yaml"
        directory = fixture_root / "directory.yaml"
        directory.mkdir()
        valid = fixture_root / "valid.yaml"
        valid.write_text("image: map.pgm\n", encoding="utf-8")
        rejected_cases = (
            ("EMPTY", ""),
            ("RELATIVE", "relative.yaml"),
            ("NON_YAML", str(non_yaml)),
            ("MISSING", str(missing)),
            ("NOT_FILE", str(directory)),
        )
        for label, raw_path in rejected_cases:
            try:
                validate_map_path(raw_path)
            except InvalidMapPathError as error:
                print(f"MAP_VALIDATION_{label}=REJECT:{error.violation.name}")
            else:
                raise AssertionError(f"validator accepted {label}")
        first = validate_map_path(str(valid))
        second = validate_map_path(str(valid))
        assert first == second == valid
        print(f"MAP_VALIDATION_VALID=ACCEPT:{first}")
        print("MAP_VALIDATION_REPEAT=STATELESS")


def run_static_qa(install_root: Path) -> None:
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    control_source = CONTROL_LAUNCH_PATH.read_text(encoding="utf-8")
    assert_gazebo_navigation_contract(source)
    assert_gazebo_control_contract(control_source)
    tree = ast.parse(source, filename=str(LAUNCH_PATH))

    arguments = _calls(tree, "DeclareLaunchArgument")
    for argument in arguments:
        name = _string(argument.args[0])
        default = _optional_keyword(argument, "default_value")
        rendered_default = "<required>" if default is None else ast.unparse(default)
        print(f"LAUNCH_ARG_{name}={rendered_default}")

    includes = _calls(tree, "IncludeLaunchDescription")
    include_sources = [ast.unparse(call.args[0]) for call in includes]
    control_includes = [
        call
        for call, source_text in zip(includes, include_sources, strict=True)
        if "gazebo_control.launch.py" in source_text
    ]
    nav2_includes = [
        call
        for call, source_text in zip(includes, include_sources, strict=True)
        if "bringup_launch.py" in source_text
    ]
    assert len(control_includes) == 1
    assert len(nav2_includes) == 1
    print("GAZEBO_CONTROL_INCLUDE_COUNT=1")
    print(
        "GAZEBO_CONTROL_FORWARD_KEYS="
        + ",".join(_include_keys(control_includes[0]))
    )
    print("NAV2_BRINGUP_INCLUDE_COUNT=1")
    print("NAV2_FORWARD_KEYS=" + ",".join(_include_keys(nav2_includes[0])))

    normalized = ast.unparse(tree)
    assert "navigation_share / 'config' / 'nav2_params.yaml'" in normalized
    assert "navigation_share / 'rviz' / 'navigation.rviz'" in normalized
    print("NAV2_PARAMS_PATH=zero_navigation/config/nav2_params.yaml")
    print("NAVIGATION_RVIZ_PATH=zero_navigation/rviz/navigation.rviz")

    nodes = _calls(tree, "Node")
    packages = [_string(_required_keyword(node, "package")) for node in nodes]
    executables = [
        _string(_required_keyword(node, "executable")) for node in nodes
    ]
    direct_nav2 = [package for package in packages if package.startswith("nav2_")]
    lifecycle_owners = [
        executable for executable in executables if "lifecycle" in executable
    ]
    slam_owners = [package for package in packages if "slam" in package]
    assert packages == ["rviz2"]
    assert direct_nav2 == lifecycle_owners == slam_owners == []
    print("DIRECT_NODE_COUNT=1:rviz2/rviz2")
    print("DIRECT_NAV2_NODE_COUNT=0")
    print("DIRECT_LIFECYCLE_OWNER_COUNT=0")
    print("DIRECT_SLAM_OWNER_COUNT=0")

    bringup_prefix = install_root / "zero_bringup"
    installed_fixtures = [
        path
        for path in bringup_prefix.rglob("*")
        if "test" in path.parts or ".pytest_cache" in path.parts
    ]
    installed_extra_maps = [
        path
        for path in (bringup_prefix / "share" / "zero_bringup").rglob("*.yaml")
        if path.name != "v1_sim.yaml"
    ]
    assert installed_fixtures == installed_extra_maps == []
    print("INSTALLED_TEST_FIXTURE_COUNT=0")
    print("INSTALLED_EXTRA_MAP_COUNT=0")
    _print_map_validation()
    print("TASK22_STATIC_QA=PASS:EXACT_GRAPH")
