from __future__ import annotations

import ast
from pathlib import Path

from .static_test_scanner import (
    GIT_VISIBILITY_TESTS,
    SOURCE_ROOT,
    call_name,
    runtime_violations,
    selected_test_paths,
)


def test_selected_static_tests_contain_no_runtime_process_or_ros_graph_code() -> None:
    # Given: every Python test/helper selected across all nine phase packages.
    test_paths = selected_test_paths()

    # When: executable AST constructs are inspected without importing test modules.
    violations = tuple(
        violation
        for path in test_paths
        for violation in runtime_violations(
            path,
            path.read_text(encoding="utf-8"),
        )
    )

    # Then: no runtime launch, node, process, command, or ROS graph API is reachable.
    assert violations == ()


def test_selected_static_tests_cover_all_nine_phase_packages() -> None:
    # Given: the exact phase package inventory and every source test module.
    package_names = {
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
    expected_paths = {
        path
        for package_name in package_names
        for path in (SOURCE_ROOT / package_name / "test").rglob("*.py")
    }

    # When: the static scanner enumerates its complete package test boundary.
    selected_paths = set(selected_test_paths())

    # Then: every Python test/helper in all nine packages is selected exactly once.
    assert selected_paths == expected_paths


def test_mapping_rviz_contract_is_selected_and_uses_no_runtime_process() -> None:
    # Given: the mapping RViz contract that previously escaped package selection.
    mapping_test = SOURCE_ROOT / "zero_mapping" / "test" / "test_mapping_rviz.py"

    # When: its executable AST and scanner membership are inspected.
    violations = runtime_violations(
        mapping_test,
        mapping_test.read_text(encoding="utf-8"),
    )

    # Then: it is selected and contains no subprocess or runtime boundary.
    assert violations == ()
    assert mapping_test in selected_test_paths()


def test_git_visibility_subprocesses_are_exactly_constrained() -> None:
    # Given: the explicit tests allowed to query repository visibility via Git.
    for path, expected_run_count in GIT_VISIBILITY_TESTS:
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        assignments = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "GIT_COMMAND"
        ]
        run_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and call_name(node) == "subprocess.run"
        ]

        # When/Then: every allowed call has one immutable Git prefix and no shell.
        assert len(assignments) == 1
        command = assignments[0].value
        assert isinstance(command, ast.Tuple)
        assert [ast.literal_eval(element) for element in command.elts] == [
            "env",
            "GIT_MASTER=1",
            "git",
        ]
        assert len(run_calls) == expected_run_count
        for call in run_calls:
            assert call.args
            arguments = call.args[0]
            assert isinstance(arguments, ast.List)
            assert arguments.elts
            prefix = arguments.elts[0]
            assert isinstance(prefix, ast.Starred)
            assert isinstance(prefix.value, ast.Name)
            assert prefix.value.id == "GIT_COMMAND"
            assert all(keyword.arg != "shell" for keyword in call.keywords)


def test_runtime_scanner_rejects_malformed_static_test_fixture() -> None:
    # Given: a fixture containing every prohibited runtime boundary class.
    malformed = """
import launch_testing
import os
import rclpy
import subprocess

subprocess.Popen(["ros2", "node", "list"])
os.system("gz sim")
rclpy.init()
node.get_node_names()
"""

    # When/Then: structured AST checks reject imports, processes, and graph access.
    violations = runtime_violations(Path("malformed_fixture.py"), malformed)
    assert len(violations) == 8
    assert any("launch_testing" in violation for violation in violations)
    assert any("subprocess.Popen" in violation for violation in violations)
    assert any("os.system" in violation for violation in violations)
    assert any("rclpy.init" in violation for violation in violations)
    assert any("get_node_names" in violation for violation in violations)
    assert any(":command:ros2" in violation for violation in violations)
    assert any(":command:gz" in violation for violation in violations)


def test_static_test_safety_contract_rejects_non_git_subprocess_fixture() -> None:
    # Given: a source-only test fixture using subprocess.run for a ROS command.
    malformed = 'import subprocess\nsubprocess.run(["ros2", "topic", "list"])\n'

    # When/Then: subprocess.run outside the justified Git test is rejected.
    assert runtime_violations(Path("fixture.py"), malformed) != ()


def test_runtime_scanner_rejects_process_spawn_and_ros_command_fixtures() -> None:
    # Given: static-test fixtures using alternate process APIs and ROS/Gazebo tools.
    malformed = """
import asyncio
import anyio
import os
import pty

os.popen("ros2 node list")
pty.spawn(["ign", "gazebo"])
asyncio.create_subprocess_exec("gz", "sim")
anyio.run_process(["ros2", "topic", "list"])
"""

    # When: the structured scanner checks executable calls and command tokens.
    violations = runtime_violations(Path("spawn_fixture.py"), malformed)

    # Then: each spawn boundary and each prohibited executable is rejected.
    assert sum(":call:" in violation for violation in violations) == 4
    assert sum(":command:" in violation for violation in violations) == 4


def test_runtime_scanner_rejects_mapping_subprocess_regression_fixture() -> None:
    # Given: the former mapping RViz subprocess shape using the current interpreter.
    malformed = """
import subprocess
import sys

subprocess.run([sys.executable, "-c", "print('map')"], check=True)
"""

    # When/Then: a non-Git subprocess is rejected even without a ROS command token.
    violations = runtime_violations(Path("test_mapping_rviz.py"), malformed)
    assert any(":process:subprocess.run" in violation for violation in violations)
