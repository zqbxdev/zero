from __future__ import annotations

import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE_ROOT / "zero_safety" / "command_guard.py"


def _node_source() -> str:
    return NODE_PATH.read_text(encoding="utf-8")


def test_node_declares_dual_status_timeouts_and_subscriptions() -> None:
    # Given
    source = _node_source()

    # When
    _ = ast.parse(source, filename=str(NODE_PATH))

    # Then
    assert 'declare_parameter("status_timeout_seconds", 0.5)' in source
    assert 'declare_parameter("motor_state_timeout_seconds", 0.5)' in source
    assert 'create_subscription(UsvStatus, "/zero/status", self._on_status, 10)' in source
    assert (
        'create_subscription(MotorState, "/zero/motor_state", self._on_motor_state, 10)'
        in source
    )
    assert source.count('"/zero/safety_status"') == 1
    assert 'create_publisher(UsvStatus, "/zero/safety_status", 10)' in source


def test_node_consumes_public_max_rpm_as_internal_safety_limit() -> None:
    # Given
    source = _node_source()

    # When
    _ = ast.parse(source, filename=str(NODE_PATH))

    # Then
    assert 'max_command_rpm=float(self.declare_parameter("max_rpm", 300.0).value)' in source
    assert 'declare_parameter("max_command_rpm"' not in source


def test_node_parses_complete_usv_status_at_receive_time() -> None:
    # Given
    source = _node_source()

    # Then
    assert "UsvStatusInput(" in source
    assert "mode=parse_control_mode(int(msg.mode))" in source
    assert "fault=bool(msg.fault)" in source
    assert "fault_code=int(msg.fault_code)" in source
    assert "fault_message=str(msg.fault_message)" in source
    assert "self._model.accept_usv_status(status, self._now_seconds())" in source


def test_node_parses_complete_motor_state_at_receive_time() -> None:
    # Given
    source = _node_source()
    fields = (
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
    assert "MotorStateInput(" in source
    assert all(field in source for field in fields)
    assert "self._model.accept_motor_state(state, self._now_seconds())" in source
