from math import inf, nan

from zero_safety.command_guard_model import (
    CommandGuardModel,
    CommandSource,
    ControlMode,
    InvalidSafetyTuningError,
    SafetyFaultCode,
    SafetyTuning,
    parse_command_source,
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
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=120.0, right_target_rpm=-80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 120.0
    assert decision.command.right_target_rpm == -80.0
    assert decision.status.fault is False
    assert decision.status.fault_code == int(SafetyFaultCode.NONE)


def test_rpm_limit_clamps_output_and_reports_intervention() -> None:
    model = CommandGuardModel(SafetyTuning(max_command_rpm=100.0))
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=150.0, right_target_rpm=-250.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 100.0
    assert decision.command.right_target_rpm == -100.0
    assert decision.status.fault is True
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_CLAMPED)


def test_stop_mode_forces_zero_output() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.STOP))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=120.0, right_target_rpm=120.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MODE_BLOCKED)


def test_manual_mode_rejects_auto_source_command() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.MANUAL))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.1)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.MODE_BLOCKED)


def test_manual_mode_allows_manual_or_debug_source_command() -> None:
    for source in (CommandSource.MANUAL, CommandSource.DEBUG):
        model = CommandGuardModel(SafetyTuning(input_source=source))
        model.set_mode_value(int(ControlMode.MANUAL))
        model.set_e_stop(False)
        model.accept_command(left_target_rpm=80.0, right_target_rpm=60.0, now_seconds=1.0)

        decision = model.decide(now_seconds=1.1)

        assert decision.command.left_target_rpm == 80.0
        assert decision.command.right_target_rpm == 60.0
        assert decision.status.fault is False


def test_timeout_forces_zero_output() -> None:
    model = CommandGuardModel(SafetyTuning(command_timeout_seconds=0.5))
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=80.0, now_seconds=1.0)

    decision = model.decide(now_seconds=1.6)

    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.COMMAND_TIMEOUT)


def test_e_stop_latches_and_blocks_output_until_release() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)
    model.set_e_stop(True)

    latched_decision = model.decide(now_seconds=1.1)
    model.set_e_stop(False)
    still_latched_decision = model.decide(now_seconds=1.2)

    assert latched_decision.command.left_target_rpm == 0.0
    assert latched_decision.status.fault_code == int(SafetyFaultCode.E_STOP_ACTIVE)
    assert still_latched_decision.command.right_target_rpm == 0.0
    assert still_latched_decision.status.fault_code == int(SafetyFaultCode.E_STOP_ACTIVE)


def test_release_fails_while_e_stop_input_is_active() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(True)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    result = model.release(now_seconds=1.1)

    assert result.accepted is False
    assert result.message == "Cannot release while e-stop input is active"


def test_release_requires_fresh_command_and_allowed_mode() -> None:
    no_command_model = CommandGuardModel()
    no_command_model.set_mode_value(int(ControlMode.AUTO))
    no_command_model.set_e_stop(False)

    stale_model = CommandGuardModel(SafetyTuning(command_timeout_seconds=0.5))
    stale_model.set_mode_value(int(ControlMode.AUTO))
    stale_model.set_e_stop(False)
    stale_model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    blocked_model = CommandGuardModel()
    blocked_model.set_mode_value(int(ControlMode.MANUAL))
    blocked_model.set_e_stop(False)
    blocked_model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    assert no_command_model.release(now_seconds=1.1).accepted is False
    assert stale_model.release(now_seconds=1.6).accepted is False
    assert blocked_model.release(now_seconds=1.1).accepted is False


def test_release_rejects_non_zero_command() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(True)
    model.set_e_stop(False)
    model.accept_command(left_target_rpm=90.0, right_target_rpm=90.0, now_seconds=1.0)

    result = model.release(now_seconds=1.1)
    decision = model.decide(now_seconds=1.2)

    assert result.accepted is False
    assert result.message == "Cannot release while current command is non-zero"
    assert decision.command.left_target_rpm == 0.0
    assert decision.command.right_target_rpm == 0.0
    assert decision.status.fault_code == int(SafetyFaultCode.E_STOP_ACTIVE)


def test_release_clears_latch_and_waits_for_new_command() -> None:
    model = CommandGuardModel()
    model.set_mode_value(int(ControlMode.AUTO))
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
    model.set_mode_value(int(ControlMode.AUTO))
    model.set_e_stop(False)

    model.accept_command(left_target_rpm=nan, right_target_rpm=80.0, now_seconds=1.0)
    nan_decision = model.decide(now_seconds=1.1)
    model.accept_command(left_target_rpm=80.0, right_target_rpm=inf, now_seconds=1.2)
    infinite_decision = model.decide(now_seconds=1.3)

    assert nan_decision.command.left_target_rpm == 0.0
    assert nan_decision.status.fault_code == int(SafetyFaultCode.INVALID_COMMAND)
    assert infinite_decision.command.right_target_rpm == 0.0
    assert infinite_decision.status.fault_code == int(SafetyFaultCode.INVALID_COMMAND)


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
