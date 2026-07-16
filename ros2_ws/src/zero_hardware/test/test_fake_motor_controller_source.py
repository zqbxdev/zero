from __future__ import annotations

import ast
from pathlib import Path


CONTROLLER_PATH = (
    Path(__file__).resolve().parents[1]
    / "zero_hardware"
    / "fake_motor_controller.py"
)


def test_controller_passes_ros_receive_and_update_times_to_the_pure_model() -> None:
    # Given
    source = CONTROLLER_PATH.read_text(encoding="utf-8")

    # When
    _ = ast.parse(source, filename=str(CONTROLLER_PATH))

    # Then
    assert "received_at_seconds=self._now_seconds()" in source
    assert "now_seconds=now_seconds" in source
    assert "def _now_seconds(self) -> float:" in source


def test_controller_declares_v1_fake_motor_limits_and_timeout() -> None:
    # Given
    source = CONTROLLER_PATH.read_text(encoding="utf-8")

    # When
    _ = ast.parse(source, filename=str(CONTROLLER_PATH))

    # Then
    assert 'self.declare_parameter("max_rpm", 300.0)' in source
    assert 'self.declare_parameter("command_timeout_seconds", 0.5)' in source


def test_controller_publishes_model_faults_on_motor_and_status_messages() -> None:
    # Given
    source = CONTROLLER_PATH.read_text(encoding="utf-8")

    # When
    _ = ast.parse(source, filename=str(CONTROLLER_PATH))

    # Then
    assert source.count("msg.fault = feedback.fault") == 2
    assert source.count("msg.fault_message = feedback.fault_message") == 2
    assert "msg.fault_code = 1 if feedback.fault else 0" in source
