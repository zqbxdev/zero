from __future__ import annotations

import ast
from pathlib import Path
from typing import Final

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
CONFIG_PATH = PACKAGE_ROOT / "config" / "v1_sim.yaml"
FRAMEWORK_PARAMETERS: Final = {"use_sim_time"}
PRODUCTION_SOURCES: Final = {
    "robot_state_publisher": None,
    "twist_to_motor_command": SOURCE_ROOT
    / "zero_control"
    / "zero_control"
    / "twist_to_motor_command.py",
    "command_guard": SOURCE_ROOT / "zero_safety" / "zero_safety" / "command_guard.py",
    "fake_motor_controller": SOURCE_ROOT
    / "zero_hardware"
    / "zero_hardware"
    / "fake_motor_controller.py",
    "gazebo_drive_adapter": SOURCE_ROOT
    / "zero_gazebo"
    / "zero_gazebo"
    / "gazebo_drive_adapter.py",
    "odom_to_tf_relay": SOURCE_ROOT
    / "zero_gazebo"
    / "zero_gazebo"
    / "odom_to_tf_relay.py",
    "simulation_safety_initializer": SOURCE_ROOT
    / "zero_safety"
    / "zero_safety"
    / "simulation_safety_initializer.py",
}


def _named_method_calls(tree: ast.AST, method_name: str) -> list[ast.Call]:
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == method_name
    ]


def _parameter_name(call: ast.Call) -> str:
    assert call.args
    name = call.args[0]
    assert isinstance(name, ast.Constant)
    assert isinstance(name.value, str)
    return name.value


def _production_parameter_usage(path: Path) -> tuple[set[str], set[str]]:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    parents = {
        child: parent
        for parent in ast.walk(tree)
        for child in ast.iter_child_nodes(parent)
    }
    declarations = _named_method_calls(tree, "declare_parameter")
    declared = {_parameter_name(call) for call in declarations}
    consumed_inline = {
        _parameter_name(call)
        for call in declarations
        if not isinstance(parents[call], ast.Expr)
    }
    consumed_by_lookup = {
        _parameter_name(call) for call in _named_method_calls(tree, "get_parameter")
    }
    return declared, consumed_inline | consumed_by_lookup


def _configured_parameter_names(path: Path) -> dict[str, set[str]]:
    configured: dict[str, set[str]] = {}
    current_node: str | None = None
    in_parameters = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        stripped = line.lstrip()
        indentation = len(line) - len(stripped)
        if indentation == 0:
            assert stripped.endswith(":")
            current_node = stripped.removesuffix(":")
            configured[current_node] = set()
            in_parameters = False
        elif indentation == 2:
            assert stripped == "ros__parameters:"
            assert current_node is not None
            in_parameters = True
        else:
            assert indentation == 4
            assert current_node is not None and in_parameters
            parameter_name, separator, _ = stripped.partition(":")
            assert separator == ":"
            configured[current_node].add(parameter_name)
    return configured


def test_every_v1_parameter_is_declared_and_consumed_by_its_production_node() -> None:
    # Given: the sole public V1 YAML and each production node source it configures.
    configured_by_node = _configured_parameter_names(CONFIG_PATH)
    assert set(configured_by_node) == set(PRODUCTION_SOURCES)

    # When: configured keys are compared with declared and consumed parameter names.
    mismatches: dict[str, dict[str, set[str]]] = {}
    for node_name, source_path in PRODUCTION_SOURCES.items():
        configured = configured_by_node[node_name] - FRAMEWORK_PARAMETERS
        if source_path is None:
            declared: set[str] = set()
            consumed: set[str] = set()
        else:
            declared, consumed = _production_parameter_usage(source_path)
        if configured - declared or configured - consumed:
            mismatches[node_name] = {
                "undeclared": configured - declared,
                "unconsumed": configured - consumed,
            }

    # Then: every public configured value reaches its owning production node.
    assert mismatches == {}
