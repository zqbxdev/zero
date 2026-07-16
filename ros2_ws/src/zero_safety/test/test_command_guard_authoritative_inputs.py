from __future__ import annotations

from math import inf, nan

import zero_safety.command_guard_model as guard_model
from zero_safety.command_guard_model import (
    CommandGuardModel,
    ControlMode,
    SafetyFaultCode,
    SafetyTuning,
)


def _usv_status(
    mode: ControlMode = ControlMode.AUTO,
    *,
    fault: bool = False,
    fault_code: int = 0,
) -> guard_model.UsvStatusInput:
    return guard_model.UsvStatusInput(
        mode=mode,
        fault=fault,
        fault_code=fault_code,
        fault_message="usv fault" if fault else "",
    )


def _motor_state(*, fault: bool = False) -> guard_model.MotorStateInput:
    return guard_model.MotorStateInput(
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
        fault=fault,
        fault_message="motor fault" if fault else "",
    )


def _accept_healthy_inputs(model: CommandGuardModel, received_at_seconds: float) -> None:
    model.set_e_stop(False)
    model.accept_usv_status(_usv_status(), received_at_seconds)
    model.accept_motor_state(_motor_state(), received_at_seconds)
    model.accept_command(70.0, 60.0, received_at_seconds)


def test_missing_usv_status_fails_closed() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(False)
    model.accept_motor_state(_motor_state(), 1.0)
    model.accept_command(70.0, 60.0, 1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.USV_STATUS_UNKNOWN)


def test_missing_motor_state_fails_closed() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(False)
    model.accept_usv_status(_usv_status(), 1.0)
    model.accept_command(70.0, 60.0, 1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MOTOR_STATE_UNKNOWN)


def test_usv_status_timeout_boundary_is_inclusive() -> None:
    # Given
    model = CommandGuardModel(
        SafetyTuning(
            command_timeout_seconds=1.0,
            status_timeout_seconds=0.5,
            motor_state_timeout_seconds=1.0,
        ),
    )
    _accept_healthy_inputs(model, 1.0)

    # When
    boundary = model.decide(now_seconds=1.5)
    stale = model.decide(now_seconds=1.500001)

    # Then
    assert boundary.status.fault_code == int(SafetyFaultCode.NONE)
    assert stale.command.left_target_rpm == 0.0
    assert stale.status.fault_code == int(SafetyFaultCode.USV_STATUS_STALE)


def test_motor_state_timeout_boundary_is_inclusive() -> None:
    # Given
    model = CommandGuardModel(
        SafetyTuning(
            command_timeout_seconds=1.0,
            status_timeout_seconds=1.0,
            motor_state_timeout_seconds=0.5,
        ),
    )
    _accept_healthy_inputs(model, 1.0)

    # When
    boundary = model.decide(now_seconds=1.5)
    stale = model.decide(now_seconds=1.500001)

    # Then
    assert boundary.status.fault_code == int(SafetyFaultCode.NONE)
    assert stale.command.right_target_rpm == 0.0
    assert stale.status.fault_code == int(SafetyFaultCode.MOTOR_STATE_STALE)


def test_usv_status_fault_and_nonzero_fault_code_fail_closed() -> None:
    # Given
    fault_flag_model = CommandGuardModel()
    fault_flag_model.set_e_stop(False)
    fault_flag_model.accept_usv_status(_usv_status(fault=True), 1.0)
    fault_flag_model.accept_motor_state(_motor_state(), 1.0)
    fault_flag_model.accept_command(70.0, 60.0, 1.0)

    fault_code_model = CommandGuardModel()
    fault_code_model.set_e_stop(False)
    fault_code_model.accept_usv_status(_usv_status(fault_code=42), 1.0)
    fault_code_model.accept_motor_state(_motor_state(), 1.0)
    fault_code_model.accept_command(70.0, 60.0, 1.0)

    # When
    fault_flag = fault_flag_model.decide(now_seconds=1.1)
    fault_code = fault_code_model.decide(now_seconds=1.1)

    # Then
    assert fault_flag.status.fault_code == int(SafetyFaultCode.USV_STATUS_FAULT)
    assert fault_code.status.fault_code == int(SafetyFaultCode.USV_STATUS_FAULT)
    assert fault_flag.command.left_target_rpm == 0.0
    assert fault_code.command.right_target_rpm == 0.0


def test_motor_state_fault_fails_closed() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(False)
    model.accept_usv_status(_usv_status(), 1.0)
    model.accept_motor_state(_motor_state(fault=True), 1.0)
    model.accept_command(70.0, 60.0, 1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MOTOR_STATE_FAULT)


def test_mode_fault_and_invalid_mode_fail_closed_with_distinct_codes() -> None:
    # Given
    mode_fault_model = CommandGuardModel()
    mode_fault_model.set_e_stop(False)
    mode_fault_model.accept_usv_status(_usv_status(ControlMode.FAULT), 1.0)
    mode_fault_model.accept_motor_state(_motor_state(), 1.0)
    mode_fault_model.accept_command(70.0, 60.0, 1.0)

    invalid_mode_model = CommandGuardModel()
    invalid_mode_model.set_e_stop(False)
    invalid_mode_model.accept_usv_status(
        _usv_status(guard_model.parse_control_mode(99)),
        1.0,
    )
    invalid_mode_model.accept_motor_state(_motor_state(), 1.0)
    invalid_mode_model.accept_command(70.0, 60.0, 1.0)

    # When
    mode_fault = mode_fault_model.decide(now_seconds=1.1)
    invalid_mode = invalid_mode_model.decide(now_seconds=1.1)

    # Then
    assert mode_fault.status.fault_code == int(SafetyFaultCode.MODE_FAULT)
    assert invalid_mode.status.fault_code == int(SafetyFaultCode.MODE_UNKNOWN)
    assert mode_fault.command.left_target_rpm == 0.0
    assert invalid_mode.command.right_target_rpm == 0.0


def test_nonfinite_receive_timestamps_invalidate_status_streams() -> None:
    # Given
    usv_model = CommandGuardModel()
    usv_model.set_e_stop(False)
    usv_accepted = usv_model.accept_usv_status(_usv_status(), nan)
    usv_model.accept_motor_state(_motor_state(), 1.0)
    usv_model.accept_command(70.0, 60.0, 1.0)

    motor_model = CommandGuardModel()
    motor_model.set_e_stop(False)
    motor_model.accept_usv_status(_usv_status(), 1.0)
    motor_accepted = motor_model.accept_motor_state(_motor_state(), inf)
    motor_model.accept_command(70.0, 60.0, 1.0)

    # When
    usv_decision = usv_model.decide(now_seconds=1.1)
    motor_decision = motor_model.decide(now_seconds=1.1)

    # Then
    assert usv_accepted is False
    assert motor_accepted is False
    assert usv_decision.status.fault_code == int(SafetyFaultCode.USV_STATUS_UNKNOWN)
    assert motor_decision.status.fault_code == int(SafetyFaultCode.MOTOR_STATE_UNKNOWN)


def test_clock_rollback_invalidates_each_stream_until_new_sample() -> None:
    # Given
    model = CommandGuardModel()
    _accept_healthy_inputs(model, 10.0)
    healthy = model.decide(now_seconds=10.1)

    # When
    rolled_back = model.decide(now_seconds=10.0)
    model.accept_usv_status(_usv_status(), 10.0)
    usv_recovered = model.decide(now_seconds=10.0)
    model.accept_motor_state(_motor_state(), 10.0)
    statuses_recovered = model.decide(now_seconds=10.0)
    model.accept_command(70.0, 60.0, 10.0)
    fully_recovered = model.decide(now_seconds=10.0)

    # Then
    assert healthy.status.fault_code == int(SafetyFaultCode.NONE)
    assert rolled_back.status.fault_code == int(SafetyFaultCode.USV_STATUS_STALE)
    assert usv_recovered.status.fault_code == int(SafetyFaultCode.MOTOR_STATE_STALE)
    assert statuses_recovered.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)
    assert fully_recovered.status.fault_code == int(SafetyFaultCode.NONE)


def test_nonfinite_decision_time_invalidates_freshness_until_new_samples() -> None:
    # Given
    model = CommandGuardModel()
    _accept_healthy_inputs(model, 1.0)

    # When
    invalid_time = model.decide(now_seconds=nan)
    still_invalid = model.decide(now_seconds=1.1)

    # Then
    assert invalid_time.status.fault_code == int(SafetyFaultCode.USV_STATUS_STALE)
    assert still_invalid.status.fault_code == int(SafetyFaultCode.USV_STATUS_STALE)
