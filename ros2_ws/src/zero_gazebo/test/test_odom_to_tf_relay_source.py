from __future__ import annotations

import ast
from pathlib import Path
from re import DOTALL, search
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
NODE_PATH = PACKAGE_ROOT / "zero_gazebo" / "odom_to_tf_relay.py"
MODEL_PATH = PACKAGE_ROOT / "zero_gazebo" / "odom_to_tf_model.py"


def _parse(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _method_calls(tree: ast.Module, method_name: str) -> tuple[ast.Call, ...]:
    return tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == method_name
    )


def _named_calls(tree: ast.Module, callable_name: str) -> tuple[ast.Call, ...]:
    return tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == callable_name
    )


def _string_literals(tree: ast.AST) -> frozenset[str]:
    return frozenset(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    )


def _active_paths() -> tuple[Path, ...]:
    paths: set[Path] = set()
    for package_manifest in SOURCE_ROOT.glob("*/package.xml"):
        package_root = package_manifest.parent
        package_name = package_root.name
        paths.update((package_root / "launch").glob("*.py"))
        paths.update((package_root / "config").glob("*.yaml"))
        paths.update((package_root / "config").glob("*.yml"))
        paths.update((package_root / package_name).glob("*.py"))
        paths.update((package_root / "models").rglob("*.sdf"))
        paths.update((package_root / "src").rglob("*.cpp"))
        paths.update((package_root / "src").rglob("*.hpp"))
    return tuple(sorted(path for path in paths if "__pycache__" not in path.parts))


def test_relay_model_and_executable_are_installed_with_minimal_dependencies() -> None:
    # Given: the zero_gazebo mixed-package build files
    cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When: the relay source, executable rule, and runtime dependencies are inspected
    runtime_dependencies = [element.text for element in manifest.findall("exec_depend")]

    # Then: the pure model ships with one discoverable relay and only required ROS types
    assert MODEL_PATH.is_file()
    assert NODE_PATH.is_file()
    assert cmake.count("PROGRAMS zero_gazebo/odom_to_tf_relay.py") == 1
    assert cmake.count("RENAME odom_to_tf_relay") == 1
    for dependency in ("geometry_msgs", "nav_msgs", "rclpy", "tf2_ros"):
        assert runtime_dependencies.count(dependency) == 1


def test_relay_owns_one_odom_input_and_one_transform_broadcaster() -> None:
    # Given: the relay node AST
    tree = _parse(NODE_PATH)
    source = NODE_PATH.read_text(encoding="utf-8")

    # When: ROS graph calls, broadcaster construction, and state fields are collected
    subscriptions = _method_calls(tree, "create_subscription")
    publishers = _method_calls(tree, "create_publisher")
    broadcasters = tuple(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "TransformBroadcaster"
    )
    sent_transforms = _method_calls(tree, "sendTransform")
    assigned_self_fields = {
        target.attr
        for node in ast.walk(tree)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (node.targets if isinstance(node, ast.Assign) else (node.target,))
        if isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == "self"
    }

    # Then: the node is a stateless one-input/one-output TF relay
    assert len(subscriptions) == 1
    assert _string_literals(subscriptions[0]) == frozenset({"/odom"})
    assert not publishers
    assert len(broadcasters) == 1
    assert len(sent_transforms) == 1
    assert assigned_self_fields == {"_transform_broadcaster"}
    assert "create_timer" not in source
    assert "get_clock" not in source
    assert not any(isinstance(node, ast.AugAssign) for node in ast.walk(tree))


def test_relay_copies_only_parser_output_into_one_transform() -> None:
    # Given: the relay node source and callback AST
    tree = _parse(NODE_PATH)
    source = NODE_PATH.read_text(encoding="utf-8")

    # When: required ROS boundary types and output field reads are inspected
    result_reads = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "result"
    }

    # Then: source odometry is parsed once and the accepted copy is broadcast unchanged
    assert "from nav_msgs.msg import Odometry" in source
    assert "from geometry_msgs.msg import TransformStamped" in source
    assert "from tf2_ros import TransformBroadcaster" in source
    assert source.count("parse_odom_transform(") == 1
    assert {
        "stamp",
        "parent_frame_id",
        "child_frame_id",
        "translation",
        "rotation",
    } <= result_reads
    assert "Odometry(" not in source
    assert ".publish(" not in source


def test_active_source_has_one_odom_to_base_owner_and_no_tf_bridge() -> None:
    # Given: active package source, launch, and config paths without docs or tests
    active_paths = _active_paths()
    sources = {path: path.read_text(encoding="utf-8") for path in active_paths}

    # When: dynamic/static TF owners, odom publishers, and bridge surfaces are scanned
    dynamic_owners = tuple(
        path
        for path, source in sources.items()
        if "TransformBroadcaster" in source
        and "StaticTransformBroadcaster" not in source
    )
    odom_publishers = tuple(
        path
        for path, source in sources.items()
        if (
            path.suffix == ".py"
            and any(
                "/odom" in _string_literals(call)
                for call in _method_calls(_parse(path), "create_publisher")
            )
        )
        or (
            path.suffix in {".cpp", ".hpp"}
            and search(
                r"create_publisher[^;]*[\"']/odom[\"']",
                source,
                flags=DOTALL,
            )
            is not None
        )
    )
    bridge_sources = tuple(
        source
        for path, source in sources.items()
        if path.suffix in {".yaml", ".yml"}
    )

    # Then: the relay is sole owner and no competing TF/odom path is active
    assert dynamic_owners == (NODE_PATH,)
    assert not odom_publishers
    assert all("Pose_V" not in source for source in bridge_sources)
    assert all(
        search(
            r"ros_topic_name:\s*[\"']?/tf(?:_static)?[\"']?",
            source,
        )
        is None
        for source in bridge_sources
    )
    for path, source in sources.items():
        if path.suffix == ".sdf" and "<tf_topic>" in source:
            assert "<tf_topic>/model/zero_usv/diff_drive_tf_unbridged</tf_topic>" in source
    for path, source in sources.items():
        if path == SOURCE_ROOT / "zero_description" / "launch" / "display.launch.py":
            continue
        assert "display.launch.py" not in source
    for path in active_paths:
        if path.suffix != ".py" or path.parent.name != "launch":
            continue
        tree = _parse(path)
        for call in _named_calls(tree, "Node"):
            literals = _string_literals(call)
            assert not (
                "static_transform_publisher" in literals
                and "odom" in literals
                and "zero_base_link" in literals
            )
