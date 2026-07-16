from math import inf, nan

from zero_safety.command_guard_model import (
    CommandGuardModel,
    CommandSource,
    ControlMode,
    InvalidSafetyTuningError,
    MotorStateInput,
    SafetyFaultCode,
    SafetyTuning,
    UsvStatusInput,
    parse_command_source,
)


def _accept_statuses(
    model: CommandGuardModel,
    mode: ControlMode = ControlMode.AUTO,
    received_at_seconds: float = 1.0,
) -> None:
    model.accept_usv_status(
        UsvStatusInput(mode=mode, fault=False, fault_code=0, fault_message=""),
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


def test_default_state_outputs_zero_until_inputs_are_known() -> None:
    model = CommandGuardModel()

    decision = model.decide(now_seconds=0.0)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault is True
    assert decision.status.fault_code == int(SafetyFaultCode.E_STOP_UNKNOWN)


def test_auto_command_passes_through_when_mode_e_stop_and_command_are_valid() -> None:
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=120.0, right_target_rpm=-80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 120.0
    assert decision.command.right_target_rpm == -80.0
    assert decision.status.fault is False
    assert decision.status.fault_code == int(SafetyFaultCode.NONE)


def test_rpm_limit_clamps_output_and_reports_intervention() -> None:
    model = CommandGuardModel(SafetyTuning(max_command_rpm=100.0))
    _accept_statuses(model)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=150.0, right_target_rpm=-250.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 100.0
    assert decision.command.right_target_rpm == -100.0
    assert decision.status.fault is True
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_CLAMPED)


def test_stop_mode_forces_zero_output() -> None:
    model = CommandGuardModel()
    _accept_statuses(model, ControlMode.STOP)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=120.0, right_target_rpm=120.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MODE_BLOCKED)


def test_manual_mode_rejects_auto_source_command() -> None:
    model = CommandGuardModel()
    _accept_statuses(model, ControlMode.MANUAL)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MODE_BLOCKED)


def test_manual_mode_allows_manual_or_debug_source_command() -> None:
    for source in (CommandSource.MANUAL, CommandSource.DEBUG):
        model = CommandGuardModel(SafetyTuning(input_source=source))
        _accept_statuses(model, ControlMode.MANUAL)
        model.set_e_stop(False)
        model.accept_command(left_target_rpm=80.0, right_target_rpm=60.0, now_seconds=1.0)

        decision = model.decide(now_seconds=1.1)

        assert decision.command.left_target_rpm == 80.0
        assert decision.command.right_target_rpm == 60.0
        assert decision.status.fault is False


def test_timeout_forces_zero_output() -> None:
    model = CommandGuardModel(
        SafetyTuning(
            command_timeout_seconds=0.5,
            status_timeout_seconds=1.0,
            motor_state_timeout_seconds=1.0,
        ),
    )
    _accept_statuses(model)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.6)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)


def test_e_stop_latches_and_blocks_output_until_release() -> None:
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)
    model.set_e_stop(True)

    latched_decision = model.decide(now_seconds=1.1)
    model.set_e_stop(False)
    still_latched_decision = model.decide(now_seconds=1.2)

    assert latched_decision.command.left_target_rpm == 0.0
    assert latched_decision.status.fault_code == int(SafetyFaultCode.E_STOP_ACTIVE)
    assert still_latched_decision.command.right_target_rpm == 0.0
    assert still_latched_decision.status.fault_code == int(SafetyFaultCode.E_STOP_LATCHED)


def test_release_fails_while_e_stop_input_is_active() -> None:
    model = CommandGuardModel()
    model.set_e_stop(True)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    result = model.release(now_seconds=1.1)

    assert result.accepted is False
    assert result.message == "Cannot release while e-stop input is active"


def test_release_requires_fresh_statuses_and_auto_mode() -> None:
    missing_status_model = CommandGuardModel()
    missing_status_model.set_e_stop(False)

    stale_model = CommandGuardModel()
    _accept_statuses(stale_model)
    stale_model.set_e_stop(False)

    blocked_model = CommandGuardModel()
    _accept_statuses(blocked_model, ControlMode.MANUAL)
    blocked_model.set_e_stop(False)

    assert missing_status_model.release(now_seconds=1.1).accepted is False
    assert stale_model.release(now_seconds=1.6).accepted is False
    assert blocked_model.release(now_seconds=1.1).accepted is False


def test_release_discards_non_zero_command_and_waits_for_new_command() -> None:
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(True)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    result = model.release(now_seconds=1.1)
    decision = model.decide(now_seconds=1.2)

    assert result.accepted is True
    assert result.message == "E-stop latch released"
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)


def test_release_clears_latch_and_waits_for_new_command() -> None:
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(True)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=0.0, right_target_rpm=0.0, now_seconds=1.0)

    result = model.release(now_seconds=1.1)
    decision = model.decide(now_seconds=1.2)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.3)
    resumed_decision = model.decide(now_seconds=1.4)

    assert result.accepted is True
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)
    assert resumed_decision.command.left_target_rpm == 90.0
    assert resumed_decision.command.right_target_rpm == 90.0
    assert resumed_decision.status.fault is False


def test_non_finite_rpm_input_forces_zero_and_reports_fault() -> None:
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(False)

    model.accept_command(left_target_rpm=nan, right_target_rpm=80.0, now_seconds=1.0)
    nan_decision = model.decide(now_seconds=1.1)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=inf, now_seconds=1.2)
    infinite_decision = model.decide(now_seconds=1.3)

    assert nan_decision.command.left_target_rpm == 0.0
    assert nan_decision.status.fault_code == int(SafetyFaultCode.INVALID_COMMAND)
    assert infinite_decision.command.right_target_rpm == 0.0
    assert infinite_decision.status.fault_code == int(SafetyFaultCode.INVALID_COMMAND)


def test_characterization_active_e_stop_forces_zero() -> None:
    # Given
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(True)
    model.accept_command(left_target_rpm=70.0, right_target_rpm=60.0, now_seconds=1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.E_STOP_ACTIVE)


def test_characterization_mode_blocks_fresh_command() -> None:
    # Given
    model = CommandGuardModel()
    _accept_statuses(model, ControlMode.STOP)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=70.0, right_target_rpm=60.0, now_seconds=1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MODE_BLOCKED)


def test_characterization_command_timeout_boundary_is_inclusive() -> None:
    # Given
    model = CommandGuardModel(
        SafetyTuning(
            command_timeout_seconds=0.5,
            status_timeout_seconds=1.0,
            motor_state_timeout_seconds=1.0,
        ),
    )
    _accept_statuses(model)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=70.0, right_target_rpm=60.0, now_seconds=1.0)

    # When
    boundary = model.decide(now_seconds=1.5)
    stale = model.decide(now_seconds=1.500001)

    # Then
    assert boundary.command.left_target_rpm == 70.0
    assert boundary.status.fault_code == int(SafetyFaultCode.NONE)
    assert stale.command.left_target_rpm == 0.0
    assert stale.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)


def test_characterization_latch_persists_after_input_clears() -> None:
    # Given
    model = CommandGuardModel()
    _accept_statuses(model)
    model.set_e_stop(True)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=0.0, right_target_rpm=0.0, now_seconds=1.0)

    # When
    decision = model.decide(now_seconds=1.1)

    # Then
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault is True


def test_invalid_tuning_values_are_rejected() -> None:
    try:
        _ = SafetyTuning(max_command_rpm=0.0)
    except InvalidSafetyTuningError as error:
        assert str(error) == "max_command_rpm must be positive and finite"
    else:
        raise AssertionError("expected invalid rpm limit to be rejected")

    try:
        _ = parse_command_source("operator")
    except InvalidSafetyTuningError as error:
        assert str(error) == "input_source must be one of: auto, debug, manual"
    else:
        raise AssertionError("expected invalid source to be rejected")
