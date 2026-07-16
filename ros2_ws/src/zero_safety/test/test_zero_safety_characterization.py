from __future__ import annotations

import ast
from pathlib import Path
from xml.etree import ElementTree

from zero_safety.command_guard_model import (
    CommandGuardModel,
    ControlMode,
    MotorStateInput,
    SafetyFaultCode,
    UsvStatusInput,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _accept_healthy_statuses(model: CommandGuardModel, received_at_seconds: float) -> None:
    model.accept_usv_status(
        UsvStatusInput(
            mode=ControlMode.AUTO,
            fault=False,
            fault_code=0,
            fault_message="",
        ),
        received_at_seconds,
    )
    model.accept_motor_state(
        MotorStateInput(
            left_target_rpm=0.0,
            right_target_rpm=0.0,
            left_actual_rpm=0.0,
            right_actual_rpm=0.0,
            left_encoder_count=0,
            right_encoder_count=0,
            left_pwm_duty=0.0,
            right_pwm_duty=0.0,
            left_enabled=True,
            right_enabled=True,
            fault=False,
            fault_message="",
        ),
        received_at_seconds,
    )


def test_existing_console_script_and_install_directories_are_characterized() -> None:
    # Given
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    setup_cfg = (PACKAGE_ROOT / "setup.cfg").read_text(encoding="utf-8")

    # When
    _ = ast.parse(setup_source, filename="setup.py")

    # Then
    assert '"command_guard = zero_safety.command_guard:main"' in setup_source
    assert "script_dir=$base/lib/zero_safety" in setup_cfg
    assert "install_scripts=$base/lib/zero_safety" in setup_cfg


def test_existing_runtime_dependencies_are_characterized() -> None:
    # Given
    manifest = ElementTree.parse(PACKAGE_ROOT / "package.xml")

    # When
    dependencies = [element.text for element in manifest.findall("exec_depend")]

    # Then
    assert dependencies == ["rclpy", "std_msgs", "std_srvs", "zero_interfaces"]


def test_guard_reports_false_input_with_latch_still_active_before_release() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(True)
    model.set_e_stop(False)
    _accept_healthy_statuses(model, received_at_seconds=1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.status.mode is ControlMode.AUTO
    assert decision.status.fault is True
    assert decision.status.fault_code == int(SafetyFaultCode.E_STOP_LATCHED)


def test_guard_release_needs_no_command_and_status_confirms_latch_clearance() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(True)
    model.set_e_stop(False)
    _accept_healthy_statuses(model, received_at_seconds=1.0)

    # When
    release = model.release(now_seconds=1.1)
    decision = model.decide(now_seconds=1.1)

    # Then
    assert release.accepted is True
    assert decision.status.mode is ControlMode.AUTO
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)
