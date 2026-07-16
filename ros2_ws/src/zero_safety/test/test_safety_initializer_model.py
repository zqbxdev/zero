from __future__ import annotations

from math import inf, nan

import pytest

from zero_safety.command_guard_model import ControlMode, SafetyFaultCode
from zero_safety.safety_initializer_model import (
    InitializerState,
    InitializerTuning,
    ModeRequestResult,
    MotorStateObservation,
    ReleaseRequestResult,
    SafetyInitializerModel,
    SafetyStatusObservation,
    TerminalResult,
    UsvStatusObservation,
)


def _tuning(
    status_timeout_ns: int = 10,
    motor_state_timeout_ns: int = 10,
) -> InitializerTuning:
    return InitializerTuning(
        status_timeout_ns=status_timeout_ns,
        motor_state_timeout_ns=motor_state_timeout_ns,
    )


def _usv(
    at_ns: int,
    mode: ControlMode = ControlMode.AUTO,
    *,
    fault: bool = False,
) -> UsvStatusObservation:
    return UsvStatusObservation(
        received_at_ns=at_ns,
        mode=mode,
        fault=fault,
        fault_code=1 if fault else 0,
        fault_message="usv fault" if fault else "",
    )


def _motor(at_ns: int, *, fault: bool = False, actual_rpm: float = 0.0) -> MotorStateObservation:
    return MotorStateObservation(
        received_at_ns=at_ns,
        left_target_rpm=0.0,
        right_target_rpm=0.0,
        left_actual_rpm=actual_rpm,
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


def _safety(
    at_ns: int,
    fault_code: SafetyFaultCode,
    *,
    fault: bool | None = None,
) -> SafetyStatusObservation:
    return SafetyStatusObservation(
        received_at_ns=at_ns,
        mode=ControlMode.AUTO,
        fault=fault_code is not SafetyFaultCode.NONE if fault is None else fault,
        fault_code=int(fault_code),
        fault_message=fault_code.name,
    )


def _waiting_for_health(tuning: InitializerTuning | None = None) -> SafetyInitializerModel:
    model = SafetyInitializerModel.start(_tuning() if tuning is None else tuning, now_ns=0)
    model = model.observe_safety_status(_safety(1, SafetyFaultCode.E_STOP_INPUT_ACTIVE))
    auto_step = model.advance(now_ns=1)
    model = auto_step.model.accept_mode_result(
        ModeRequestResult(received_at_ns=2, accepted=True, current_mode=ControlMode.AUTO),
    )
    return model.advance(now_ns=2).model


def _waiting_for_release_response() -> SafetyInitializerModel:
    model = _waiting_for_health()
    model = model.observe_usv_status(_usv(3))
    model = model.observe_motor_state(_motor(3))
    model = model.advance(now_ns=3).model
    model = model.observe_safety_status(_safety(4, SafetyFaultCode.E_STOP_LATCHED))
    return model.advance(now_ns=4).model


def test_success_sequence_holds_targets_and_requests_release_once() -> None:
    # Given
    model = SafetyInitializerModel.start(_tuning(), now_ns=0)

    # When
    initial = model.advance(now_ns=0)
    model = initial.model.observe_safety_status(_safety(1, SafetyFaultCode.E_STOP_INPUT_ACTIVE))
    auto = model.advance(now_ns=1)
    duplicate_auto = auto.model.advance(now_ns=1)
    model = duplicate_auto.model.accept_mode_result(
        ModeRequestResult(received_at_ns=2, accepted=True, current_mode=ControlMode.AUTO),
    )
    model = model.advance(now_ns=2).model.observe_usv_status(_usv(3))
    model = model.observe_motor_state(_motor(3))
    lower = model.advance(now_ns=3)
    model = lower.model.observe_safety_status(_safety(4, SafetyFaultCode.E_STOP_LATCHED))
    release = model.advance(now_ns=4)
    duplicate_release = release.model.advance(now_ns=4)
    model = duplicate_release.model.accept_release_result(
        ReleaseRequestResult(received_at_ns=5, accepted=True),
    )
    model = model.advance(now_ns=5).model
    model = model.observe_safety_status(_safety(6, SafetyFaultCode.NONE))
    terminal = model.advance(now_ns=6)

    # Then
    assert initial.publish_e_stop is True
    assert auto.request_auto is True
    assert duplicate_auto.request_auto is False
    assert lower.publish_e_stop is False
    assert release.request_release is True
    assert duplicate_release.request_release is False
    assert terminal.model.state is InitializerState.SUCCESS
    assert terminal.terminal is TerminalResult.SUCCESS


def test_statuses_sampled_before_auto_response_at_same_nanosecond_are_rejected() -> None:
    # Given
    model = SafetyInitializerModel.start(_tuning(), now_ns=0)
    model = model.observe_safety_status(_safety(1, SafetyFaultCode.E_STOP_INPUT_ACTIVE))
    model = model.advance(now_ns=1).model
    model = model.observe_usv_status(_usv(2)).observe_motor_state(_motor(2))
    model = model.accept_mode_result(ModeRequestResult(2, True, ControlMode.AUTO))

    # When
    waiting = model.advance(now_ns=2).model
    same_nanosecond = waiting.advance(now_ns=2)
    fresh = same_nanosecond.model.observe_usv_status(_usv(3)).observe_motor_state(_motor(3))
    confirmed = fresh.advance(now_ns=3)

    # Then
    assert same_nanosecond.model.state is InitializerState.WAIT_HEALTHY_STATUS
    assert confirmed.model.state is InitializerState.WAIT_ESTOP_FALSE_LATCHED


def test_safety_status_sampled_before_release_response_at_same_nanosecond_is_rejected() -> None:
    # Given
    model = _waiting_for_release_response()
    model = model.observe_safety_status(_safety(5, SafetyFaultCode.NONE))
    model = model.accept_release_result(ReleaseRequestResult(5, True))

    # When
    waiting = model.advance(now_ns=5).model
    same_nanosecond = waiting.advance(now_ns=5)
    fresh = same_nanosecond.model.observe_safety_status(_safety(6, SafetyFaultCode.NONE))
    confirmed = fresh.advance(now_ns=6)

    # Then
    assert same_nanosecond.model.state is InitializerState.WAIT_LATCH_RELEASE
    assert same_nanosecond.terminal is None
    assert confirmed.terminal is TerminalResult.SUCCESS


@pytest.mark.parametrize(
    "fault_code",
    [SafetyFaultCode.NONE, SafetyFaultCode.COMMAND_TIMEOUT, SafetyFaultCode.COMMAND_CLAMPED],
)
def test_faulted_safety_status_never_confirms_release(fault_code: SafetyFaultCode) -> None:
    # Given
    model = _waiting_for_release_response()
    model = model.accept_release_result(ReleaseRequestResult(5, True)).advance(5).model
    model = model.observe_safety_status(_safety(6, fault_code, fault=True))

    # When
    decision = model.advance(now_ns=6)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert decision.publish_e_stop is True
    assert decision.terminal is TerminalResult.FAILED


@pytest.mark.parametrize(
    "accepted,current_mode",
    [(False, ControlMode.STOP), (True, ControlMode.MANUAL)],
)
def test_auto_rejection_or_mismatch_fails_safe(accepted: bool, current_mode: ControlMode) -> None:
    # Given
    model = SafetyInitializerModel.start(_tuning(), now_ns=0)
    model = model.observe_safety_status(_safety(1, SafetyFaultCode.E_STOP_INPUT_ACTIVE))
    model = model.advance(now_ns=1).model

    # When
    model = model.accept_mode_result(ModeRequestResult(2, accepted, current_mode))
    decision = model.advance(now_ns=2)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert decision.publish_e_stop is True
    assert decision.terminal is TerminalResult.FAILED


@pytest.mark.parametrize("wait_kind", ["auto", "release"])
def test_each_service_wait_is_bounded(wait_kind: str) -> None:
    # Given
    model = SafetyInitializerModel.start(_tuning(), now_ns=0)
    if wait_kind == "auto":
        model = model.observe_safety_status(_safety(1, SafetyFaultCode.E_STOP_INPUT_ACTIVE))
        model = model.advance(now_ns=1).model
        timeout_ns = 12
    else:
        model = _waiting_for_release_response()
        timeout_ns = 15

    # When
    decision = model.advance(now_ns=timeout_ns)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert decision.publish_e_stop is True


@pytest.mark.parametrize(
    "usv",
    [
        _usv(3, ControlMode.UNKNOWN),
        _usv(3, ControlMode.FAULT),
        _usv(3, fault=True),
    ],
)
def test_each_unsafe_usv_status_fails_safe(usv: UsvStatusObservation) -> None:
    # Given
    model = _waiting_for_health().observe_usv_status(usv).observe_motor_state(_motor(3))

    # When
    decision = model.advance(now_ns=3)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert decision.publish_e_stop is True


@pytest.mark.parametrize("motor", [_motor(3, fault=True), _motor(3, actual_rpm=inf)])
def test_faulted_or_malformed_motor_state_fails_safe(motor: MotorStateObservation) -> None:
    # Given
    model = _waiting_for_health().observe_usv_status(_usv(3)).observe_motor_state(motor)

    # When
    decision = model.advance(now_ns=3)

    # Then
    assert decision.model.state is InitializerState.FAILED


@pytest.mark.parametrize("stale_stream", ["usv", "motor"])
def test_each_status_stream_staleness_fails_safe(stale_stream: str) -> None:
    # Given
    tuning = _tuning(5, 20) if stale_stream == "usv" else _tuning(20, 5)
    model = _waiting_for_health(tuning).observe_usv_status(_usv(3)).observe_motor_state(_motor(3))

    # When
    decision = model.advance(now_ns=9)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert "stale" in decision.model.failure_reason.lower()


@pytest.mark.parametrize("missing_stream", ["usv", "motor"])
def test_missing_status_stream_times_out_safe(missing_stream: str) -> None:
    # Given
    model = _waiting_for_health()
    model = (
        model.observe_motor_state(_motor(3))
        if missing_stream == "usv"
        else model.observe_usv_status(_usv(3))
    )

    # When
    decision = model.advance(now_ns=13)

    # Then
    assert decision.model.state is InitializerState.FAILED
    assert decision.publish_e_stop is True


def test_release_failure_and_pre_release_latch_mismatch_fail_safe() -> None:
    # Given
    release_wait = _waiting_for_release_response()
    mismatch = _waiting_for_health().observe_usv_status(_usv(3)).observe_motor_state(_motor(3))
    mismatch = mismatch.advance(now_ns=3).model
    mismatch = mismatch.observe_safety_status(_safety(4, SafetyFaultCode.COMMAND_TIMEOUT))

    # When
    rejected = release_wait.accept_release_result(ReleaseRequestResult(5, False)).advance(5)
    mismatched = mismatch.advance(4)

    # Then
    assert rejected.model.state is InitializerState.FAILED
    assert mismatched.model.state is InitializerState.FAILED


def test_clock_rollback_and_nonfinite_time_fail_safe() -> None:
    # Given
    rollback = SafetyInitializerModel.start(_tuning(), now_ns=10)
    malformed = SafetyInitializerModel.start(_tuning(), now_ns=0)

    # When
    rolled_back = rollback.advance(now_ns=9)
    nonfinite = malformed.advance(now_ns=nan)

    # Then
    assert rolled_back.model.state is InitializerState.FAILED
    assert nonfinite.model.state is InitializerState.FAILED


def test_terminal_state_ignores_repeated_inputs_and_reports_once() -> None:
    # Given
    model = SafetyInitializerModel.start(_tuning(), now_ns=0).advance(now_ns=11).model
    first = model.advance(now_ns=11)

    # When
    repeated = first.model.observe_usv_status(_usv(12)).accept_release_result(
        ReleaseRequestResult(12, True),
    ).advance(now_ns=12)

    # Then
    assert first.model.state is InitializerState.FAILED
    assert repeated.model == first.model
    assert repeated.request_auto is False
    assert repeated.request_release is False
    assert repeated.terminal is None
