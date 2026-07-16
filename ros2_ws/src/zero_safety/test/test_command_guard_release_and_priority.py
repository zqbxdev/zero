from __future__ import annotations

import zero_safety.command_guard_model as guard_model
from zero_safety.command_guard_model import CommandGuardModel, ControlMode, SafetyFaultCode


def _usv_status(
    mode: ControlMode = ControlMode.AUTO,
    *,
    fault: bool = False,
) -> guard_model.UsvStatusInput:
    return guard_model.UsvStatusInput(
        mode=mode,
        fault=fault,
        fault_code=1 if fault else 0,
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


def _accept_statuses(
    model: CommandGuardModel,
    received_at_seconds: float,
    *,
    usv: guard_model.UsvStatusInput | None = None,
    motor: guard_model.MotorStateInput | None = None,
) -> None:
    model.accept_usv_status(_usv_status() if usv is None else usv, received_at_seconds)
    model.accept_motor_state(_motor_state() if motor is None else motor, received_at_seconds)


def test_release_succeeds_without_command_then_reports_command_timeout() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(True)
    model.set_e_stop(False)
    _accept_statuses(model, 1.0)

    # When
    result = model.release(now_seconds=1.1)
    decision = model.decide(now_seconds=1.1)

    # Then
    assert result.accepted is True
    assert result.message == "E-stop latch released"
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)


def test_release_rejects_unknown_and_active_e_stop_inputs() -> None:
    # Given
    unknown_model = CommandGuardModel()
    _accept_statuses(unknown_model, 1.0)

    active_model = CommandGuardModel()
    active_model.set_e_stop(True)
    _accept_statuses(active_model, 1.0)

    # When
    unknown = unknown_model.release(now_seconds=1.1)
    active = active_model.release(now_seconds=1.1)

    # Then
    assert unknown.accepted is False
    assert unknown.message == "Cannot release while e-stop state is unknown"
    assert active.accepted is False
    assert active.message == "Cannot release while e-stop input is active"


def test_release_rejects_non_auto_stale_or_faulted_statuses() -> None:
    # Given
    mode_model = CommandGuardModel()
    mode_model.set_e_stop(True)
    mode_model.set_e_stop(False)
    _accept_statuses(mode_model, 1.0, usv=_usv_status(ControlMode.MANUAL))

    stale_model = CommandGuardModel()
    stale_model.set_e_stop(True)
    stale_model.set_e_stop(False)
    _accept_statuses(stale_model, 1.0)

    usv_fault_model = CommandGuardModel()
    usv_fault_model.set_e_stop(True)
    usv_fault_model.set_e_stop(False)
    _accept_statuses(usv_fault_model, 1.0, usv=_usv_status(fault=True))

    motor_fault_model = CommandGuardModel()
    motor_fault_model.set_e_stop(True)
    motor_fault_model.set_e_stop(False)
    _accept_statuses(motor_fault_model, 1.0, motor=_motor_state(fault=True))

    # When
    results = (
        mode_model.release(now_seconds=1.1),
        stale_model.release(now_seconds=1.6),
        usv_fault_model.release(now_seconds=1.1),
        motor_fault_model.release(now_seconds=1.1),
    )

    # Then
    assert tuple(result.accepted for result in results) == (False, False, False, False)


def test_fault_clearance_never_clears_latch() -> None:
    # Given
    model = CommandGuardModel()
    model.set_e_stop(True)
    model.set_e_stop(False)
    _accept_statuses(model, 1.0, usv=_usv_status(fault=True))
    model.accept_usv_status(_usv_status(), 1.1)
    model.accept_motor_state(_motor_state(), 1.1)
    model.accept_command(70.0, 60.0, 1.1)

    # When
    decision = model.decide(now_seconds=1.2)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.E_STOP_LATCHED)


def test_fault_priority_is_deterministic() -> None:
    # Given
    unknown_e_stop = CommandGuardModel()
    _accept_statuses(
        unknown_e_stop,
        1.0,
        usv=_usv_status(fault=True),
        motor=_motor_state(fault=True),
    )

    active_e_stop = CommandGuardModel()
    active_e_stop.set_e_stop(True)
    _accept_statuses(
        active_e_stop,
        1.0,
        usv=_usv_status(fault=True),
        motor=_motor_state(fault=True),
    )

    latched_e_stop = CommandGuardModel()
    latched_e_stop.set_e_stop(True)
    latched_e_stop.set_e_stop(False)
    _accept_statuses(
        latched_e_stop,
        1.0,
        usv=_usv_status(fault=True),
        motor=_motor_state(fault=True),
    )

    missing_usv = CommandGuardModel()
    missing_usv.set_e_stop(False)

    missing_motor = CommandGuardModel()
    missing_motor.set_e_stop(False)
    missing_motor.accept_usv_status(_usv_status(), 1.0)

    mode_fault = CommandGuardModel()
    mode_fault.set_e_stop(False)
    _accept_statuses(
        mode_fault,
        1.0,
        usv=_usv_status(ControlMode.FAULT, fault=True),
        motor=_motor_state(fault=True),
    )

    usv_fault = CommandGuardModel()
    usv_fault.set_e_stop(False)
    _accept_statuses(usv_fault, 1.0, usv=_usv_status(fault=True), motor=_motor_state(fault=True))

    motor_fault = CommandGuardModel()
    motor_fault.set_e_stop(False)
    _accept_statuses(motor_fault, 1.0, motor=_motor_state(fault=True))

    blocked_mode = CommandGuardModel()
    blocked_mode.set_e_stop(False)
    _accept_statuses(blocked_mode, 1.0, usv=_usv_status(ControlMode.STOP))

    missing_command = CommandGuardModel()
    missing_command.set_e_stop(False)
    _accept_statuses(missing_command, 1.0)

    # When
    codes = tuple(
        model.decide(now_seconds=1.1).status.fault_code
        for model in (
            unknown_e_stop,
            active_e_stop,
            latched_e_stop,
            missing_usv,
            missing_motor,
            mode_fault,
            usv_fault,
            motor_fault,
            blocked_mode,
            missing_command,
        )
    )

    # Then
    assert codes == (
        int(SafetyFaultCode.E_STOP_UNKNOWN),
        int(SafetyFaultCode.E_STOP_INPUT_ACTIVE),
        int(SafetyFaultCode.E_STOP_LATCHED),
        int(SafetyFaultCode.USV_STATUS_UNKNOWN),
        int(SafetyFaultCode.MOTOR_STATE_UNKNOWN),
        int(SafetyFaultCode.MODE_FAULT),
        int(SafetyFaultCode.USV_STATUS_FAULT),
        int(SafetyFaultCode.MOTOR_STATE_FAULT),
        int(SafetyFaultCode.MODE_BLOCKED),
        int(SafetyFaultCode.COMMAND_TIMEOUT),
    )
