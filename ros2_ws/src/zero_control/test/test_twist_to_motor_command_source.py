from __future__ import annotations

import ast
from pathlib import Path


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "zero_control"
    / "twist_to_motor_command.py"
)


def _load_tree() -> ast.Module:
    return ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))


def _method(tree: ast.Module, name: str) -> ast.FunctionDef:
    methods = (
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    return next(methods)


def _called_attributes(function: ast.FunctionDef) -> set[str]:
    return {
        call.func.attr
        for call in ast.walk(function)
        if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute)
    }


def test_node_loads_the_shared_watchdog_and_signed_kinematics_parameters() -> None:
    # Given
    tree = _load_tree()

    # When
    string_values = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }

    # Then
    assert {
        "max_rpm",
        "left_motor_sign",
        "right_motor_sign",
        "command_timeout_seconds",
        "publish_rate_hz",
    } <= string_values


def test_subscription_callback_only_caches_the_typed_command() -> None:
    # Given
    tree = _load_tree()

    # When
    calls = _called_attributes(_method(tree, "_on_twist"))

    # Then
    assert "accept_command" in calls
    assert "publish" not in calls


def test_timer_converts_and_publishes_the_watchdog_output() -> None:
    # Given
    tree = _load_tree()

    # When
    calls = _called_attributes(_method(tree, "_publish_tick"))

    # Then
    assert {"command_at", "convert", "publish"} <= calls


def test_node_preserves_integer_ros_clock_nanoseconds_and_exact_sign_types() -> None:
    # Given
    source = SOURCE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)

    # When
    nanosecond_calls = tuple(
        call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "RosTimeNanoseconds"
    )
    integer_value_reads = tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute) and node.attr == "integer_value"
    )

    # Then
    assert "1e-9" not in source
    assert len(nanosecond_calls) == 2
    assert all(
        len(call.args) == 1
        and isinstance(call.args[0], ast.Attribute)
        and call.args[0].attr == "nanoseconds"
        for call in nanosecond_calls
    )
    assert len(integer_value_reads) == 2
