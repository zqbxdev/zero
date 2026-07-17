from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = PACKAGE_ROOT / "zero_gazebo" / "gazebo_drive_adapter_model.py"
NODE_PATH = PACKAGE_ROOT / "zero_gazebo" / "gazebo_drive_adapter.py"
MOTOR_STATE_FIELDS = frozenset(
    {
        "left_target_rpm",
        "right_target_rpm",
        "left_actual_rpm",
        "right_actual_rpm",
        "left_encoder_count",
        "right_encoder_count",
        "left_pwm_duty",
        "right_pwm_duty",
        "left_enabled",
        "right_enabled",
        "fault",
        "fault_message",
    }
)


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


def _string_literals(tree: ast.Module) -> frozenset[str]:
    return frozenset(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    )


def test_model_is_ros_independent_and_reuses_shared_inverse() -> None:
    # Given: the pure adapter model source
    tree = _parse(MODEL_PATH)

    # When: imports, calls, and state-field reads are inspected
    imported_roots = {
        alias.name.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.Import)
        for alias in node.names
    }
    imported_roots.update(
        node.module.split(".")[0]
        for node in ast.walk(tree)
        if isinstance(node, ast.ImportFrom) and node.module is not None
    )
    called_names = {
        node.func.id
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    function_names = {
        node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
    }
    state_reads = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "state"
    }

    # Then: ROS stays at the boundary and only actual RPM drives shared conversion
    assert imported_roots.isdisjoint({"geometry_msgs", "rclpy", "zero_interfaces"})
    assert "motor_rpm_to_twist" in called_names
    assert {"left_actual_rpm", "right_actual_rpm", "fault"} <= state_reads
    assert {"left_target_rpm", "right_target_rpm"}.isdisjoint(state_reads)
    assert "_rpm_to_velocity" not in function_names


def test_node_owns_only_motor_state_input_and_gazebo_twist_output() -> None:
    # Given: the ROS adapter node source
    tree = _parse(NODE_PATH)

    # When: ROS graph calls and exact topic literals are inspected
    subscriptions = _method_calls(tree, "create_subscription")
    publishers = _method_calls(tree, "create_publisher")
    literals = _string_literals(tree)

    # Then: the node owns exactly the dedicated one-way adapter topics
    assert len(subscriptions) == 1
    assert len(publishers) == 1
    assert "/zero/motor_state" in literals
    assert "/zero/gazebo/cmd_vel" in literals
    assert "/cmd_vel" not in literals
    assert "/odom" not in literals


def test_node_parses_the_complete_motor_state_at_the_boundary() -> None:
    # Given: the ROS adapter node AST
    tree = _parse(NODE_PATH)

    # When: all attributes read directly from the MotorState callback value are collected
    message_fields = {
        node.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Attribute)
        and isinstance(node.value, ast.Name)
        and node.value.id == "msg"
    }

    # Then: every MotorState payload field is parsed before entering the pure model
    assert MOTOR_STATE_FIELDS <= message_fields
    assert "stamp" in message_fields


def test_node_uses_integer_clock_nanoseconds_without_pose_or_tf_state() -> None:
    # Given: the ROS adapter node source and AST
    source = NODE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(NODE_PATH))

    # When: forbidden integration surfaces and self-owned fields are inspected
    assigned_self_fields = {
        target.attr
        for node in ast.walk(tree)
        if isinstance(node, (ast.Assign, ast.AnnAssign))
        for target in (
            node.targets if isinstance(node, ast.Assign) else (node.target,)
        )
        if isinstance(target, ast.Attribute)
        and isinstance(target.value, ast.Name)
        and target.value.id == "self"
    }

    # Then: time is integer nanoseconds and no odometry/TF integration state exists
    assert ".nanoseconds" in source
    assert "* 1e-9" not in source
    assert "TransformBroadcaster" not in source
    assert "Odometry" not in source
    assert {"_x", "_y", "_yaw", "_pose"}.isdisjoint(assigned_self_fields)
    assert not any(isinstance(node, ast.AugAssign) for node in ast.walk(tree))
