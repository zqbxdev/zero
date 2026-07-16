from __future__ import annotations

import ast
from pathlib import Path
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "zero_safety" / "simulation_safety_initializer.py"
GUARD_PATH = PACKAGE_ROOT / "zero_safety" / "command_guard.py"
QOS_PATH = PACKAGE_ROOT / "zero_safety" / "safety_qos.py"


def test_initializer_node_has_safe_topics_services_and_qos() -> None:
    # Given
    source = NODE_PATH.read_text(encoding="utf-8")
    qos_source = QOS_PATH.read_text(encoding="utf-8")

    # When
    _ = ast.parse(source, filename=str(NODE_PATH))

    # Then
    assert 'create_publisher(Bool, "/zero/e_stop", E_STOP_QOS)' in source
    assert "create_publisher(" in source
    assert 'String, "/zero/safety_initializer_status", INITIALIZER_STATUS_QOS' in source
    assert 'create_subscription(UsvStatus, "/zero/status"' in source
    assert 'create_subscription(MotorState, "/zero/motor_state"' in source
    assert 'create_subscription(UsvStatus, "/zero/safety_status"' in source
    assert 'create_client(SetControlMode, "/zero/set_control_mode")' in source
    assert 'create_client(Trigger, "/zero/release_e_stop")' in source
    assert "ReliabilityPolicy.RELIABLE" in qos_source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in qos_source
    assert "HistoryPolicy.KEEP_LAST" in qos_source
    assert "depth=1" in qos_source
    assert "INITIALIZER_STATUS_QOS" in qos_source


def test_terminal_status_is_published_once_with_exact_payload() -> None:
    # Given
    source = NODE_PATH.read_text(encoding="utf-8")

    # When
    _ = ast.parse(source, filename=str(NODE_PATH))

    # Then
    assert "from std_msgs.msg import Bool, String" in source
    assert "self._terminal_published = False" in source
    assert "if terminal is None or self._terminal_published:" in source
    assert "msg = String()" in source
    assert "msg.data = terminal.value" in source
    assert "self._terminal_status_pub.publish(msg)" in source
    assert "self._terminal_published = True" in source


def test_initializer_node_parses_complete_authoritative_statuses() -> None:
    # Given
    source = NODE_PATH.read_text(encoding="utf-8")
    motor_fields = (
        "left_target_rpm=float(msg.left_target_rpm)",
        "right_target_rpm=float(msg.right_target_rpm)",
        "left_actual_rpm=float(msg.left_actual_rpm)",
        "right_actual_rpm=float(msg.right_actual_rpm)",
        "left_encoder_count=int(msg.left_encoder_count)",
        "right_encoder_count=int(msg.right_encoder_count)",
        "left_pwm_duty=float(msg.left_pwm_duty)",
        "right_pwm_duty=float(msg.right_pwm_duty)",
        "left_enabled=bool(msg.left_enabled)",
        "right_enabled=bool(msg.right_enabled)",
        "fault=bool(msg.fault)",
        "fault_message=str(msg.fault_message)",
    )

    # Then
    assert "UsvStatusObservation(" in source
    assert "mode=parse_control_mode(int(msg.mode))" in source
    assert "fault_code=int(msg.fault_code)" in source
    assert "SafetyStatusObservation(" in source
    assert "MotorStateObservation(" in source
    assert all(field in source for field in motor_fields)


def test_initializer_never_requires_or_publishes_commands() -> None:
    # Given
    source = NODE_PATH.read_text(encoding="utf-8")

    # When
    tree = ast.parse(source, filename=str(NODE_PATH))
    imports = [node for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))]

    # Then
    assert imports
    assert "MotorCommand" not in source
    assert "geometry_msgs" not in source
    assert "/zero/motor_command" not in source
    assert "/zero/gazebo" not in source
    assert "/cmd_vel" not in source


def test_guard_uses_the_same_transient_local_e_stop_qos() -> None:
    # Given
    source = GUARD_PATH.read_text(encoding="utf-8")

    # Then
    assert 'create_subscription(Bool, "/zero/e_stop", self._on_e_stop, E_STOP_QOS)' in source


def test_initializer_console_script_and_dependencies_are_installable() -> None:
    # Given
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When
    _ = ast.parse(setup_source, filename="setup.py")
    dependencies = [element.text for element in manifest.findall("exec_depend")]

    # Then
    assert (
        '"simulation_safety_initializer = '
        'zero_safety.simulation_safety_initializer:main"'
        in setup_source
    )
    assert dependencies == ["rclpy", "std_msgs", "std_srvs", "zero_interfaces"]
    assert dependencies.count("std_msgs") == 1
