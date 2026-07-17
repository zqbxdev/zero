from __future__ import annotations

import ast
from pathlib import Path
from typing import Final

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
SOURCE_ROOT: Final = PACKAGE_ROOT.parent
PHASE_PACKAGE_NAMES: Final = (
    "zero_bringup",
    "zero_control",
    "zero_description",
    "zero_gazebo",
    "zero_hardware",
    "zero_interfaces",
    "zero_mapping",
    "zero_navigation",
    "zero_safety",
)
GIT_VISIBILITY_TESTS: Final = (
    (
        SOURCE_ROOT / "zero_mapping" / "test" / "test_map_output_policy.py",
        1,
    ),
    (
        SOURCE_ROOT / "zero_mapping" / "test" / "test_package_contract.py",
        2,
    ),
    (
        SOURCE_ROOT / "zero_navigation" / "test" / "test_resource_visibility.py",
        2,
    ),
)
ALLOWED_GIT_VISIBILITY_TESTS: Final = frozenset(
    path for path, _ in GIT_VISIBILITY_TESTS
)
PROHIBITED_IMPORT_ROOTS: Final = {"launch_testing", "rclpy"}
PROCESS_CALLS: Final = {
    "ExecuteProcess",
    "anyio.open_process",
    "anyio.run_process",
    "asyncio.create_subprocess_exec",
    "asyncio.create_subprocess_shell",
    "multiprocessing.Process",
    "os.execv",
    "os.execve",
    "os.execvp",
    "os.execvpe",
    "os.spawnl",
    "os.spawnle",
    "os.spawnlp",
    "os.spawnlpe",
    "os.spawnv",
    "os.spawnve",
    "os.spawnvp",
    "os.spawnvpe",
    "os.popen",
    "os.system",
    "pty.spawn",
    "subprocess.call",
    "subprocess.check_call",
    "subprocess.check_output",
    "subprocess.getoutput",
    "subprocess.getstatusoutput",
    "subprocess.Popen",
    "subprocess.run",
}
PROHIBITED_CALLS: Final = (PROCESS_CALLS - {"subprocess.run"}) | {
    "rclpy.init",
    "rclpy.spin",
    "rclpy.spin_once",
    "rclpy.spin_until_future_complete",
}
PROHIBITED_COMMANDS: Final = {"gz", "ign", "ros2"}
ROS_GRAPH_METHODS: Final = {
    "create_client",
    "create_node",
    "create_publisher",
    "create_service",
    "create_subscription",
    "get_node_names",
    "get_node_names_and_namespaces",
    "get_topic_names_and_types",
    "wait_for_service",
}


def call_name(call: ast.Call) -> str:
    return ast.unparse(call.func)


def _command_tokens(call: ast.Call) -> tuple[str, ...]:
    command_nodes = [*call.args[:1]]
    command_nodes.extend(
        keyword.value
        for keyword in call.keywords
        if keyword.arg in {"args", "cmd", "command"}
    )
    return tuple(
        token
        for node in command_nodes
        for descendant in ast.walk(node)
        if isinstance(descendant, ast.Constant)
        and isinstance(descendant.value, str)
        for token in descendant.value.split()
    )


class _RuntimeScanner(ast.NodeVisitor):
    def __init__(self, path: Path) -> None:
        self._path: Path = path
        self._violations: list[str] = []

    @property
    def violations(self) -> tuple[str, ...]:
        return tuple(self._violations)

    def visit_Import(self, node: ast.Import) -> None:
        for alias in node.names:
            if alias.name.split(".")[0] in PROHIBITED_IMPORT_ROOTS:
                self._violations.append(
                    f"{self._path}:{node.lineno}:import:{alias.name}"
                )

    def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
        module = node.module
        if module is not None and module.split(".")[0] in PROHIBITED_IMPORT_ROOTS:
            self._violations.append(
                f"{self._path}:{node.lineno}:import:{module}"
            )

    def visit_Call(self, node: ast.Call) -> None:
        name = call_name(node)
        if name in PROHIBITED_CALLS:
            self._violations.append(
                f"{self._path}:{node.lineno}:call:{name}"
            )
        method_name = name.rsplit(".", maxsplit=1)[-1]
        if method_name in ROS_GRAPH_METHODS:
            self._violations.append(
                f"{self._path}:{node.lineno}:graph:{method_name}"
            )
        if name == "subprocess.run" and self._path not in ALLOWED_GIT_VISIBILITY_TESTS:
            self._violations.append(
                f"{self._path}:{node.lineno}:process:{name}"
            )
        if name in PROCESS_CALLS:
            for command in sorted(PROHIBITED_COMMANDS & set(_command_tokens(node))):
                self._violations.append(
                    f"{self._path}:{node.lineno}:command:{command}"
                )
        self.generic_visit(node)


def runtime_violations(path: Path, source: str) -> tuple[str, ...]:
    scanner = _RuntimeScanner(path)
    scanner.visit(ast.parse(source, filename=str(path)))
    return scanner.violations


def selected_test_paths() -> tuple[Path, ...]:
    return tuple(
        path
        for package_name in PHASE_PACKAGE_NAMES
        for path in sorted((SOURCE_ROOT / package_name / "test").rglob("*.py"))
    )
