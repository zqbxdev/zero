from __future__ import annotations

from math import nan

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
    UsvStatusObservation,
)


def _model() -> SafetyInitializerModel:
    return SafetyInitializerModel.start(InitializerTuning(10, 10), now_ns=0)


def _usv(at_ns: int | float) -> UsvStatusObservation:
    return UsvStatusObservation(at_ns, ControlMode.AUTO, False, 0, "")


def _motor(at_ns: int | float) -> MotorStateObservation:
    return MotorStateObservation(
        at_ns,
        0.0,
        0.0,
        0.0,
        0.0,
        0,
        0,
        0.0,
        0.0,
        True,
        True,
        False,
        "",
    )


def _safety(at_ns: int | float) -> SafetyStatusObservation:
    return SafetyStatusObservation(
        at_ns,
        ControlMode.AUTO,
        True,
        int(SafetyFaultCode.E_STOP_INPUT_ACTIVE),
        "latched",
    )


def test_usv_status_is_stored_directly_with_validated_timestamp() -> None:
    # Given
    sample = _usv(1)

    # When
    observed = _model().observe_usv_status(sample)

    # Then
    stored = observed.usv_status
    assert stored is not None and stored is sample
    assert type(stored.received_at_ns) is int


def test_motor_state_is_stored_directly_with_validated_timestamp() -> None:
    # Given
    sample = _motor(1)

    # When
    observed = _model().observe_motor_state(sample)

    # Then
    stored = observed.motor_state
    assert stored is not None and stored is sample
    assert type(stored.received_at_ns) is int


def test_safety_status_is_stored_directly_with_validated_timestamp() -> None:
    # Given
    sample = _safety(1)

    # When
    observed = _model().observe_safety_status(sample)

    # Then
    stored = observed.safety_status
    assert stored is not None and stored is sample
    assert type(stored.received_at_ns) is int


def test_mode_result_is_stored_directly_with_validated_timestamp() -> None:
    # Given
    result = ModeRequestResult(1, True, ControlMode.AUTO)

    # When
    observed = _model().accept_mode_result(result)

    # Then
    stored = observed.mode_result
    assert stored is not None and stored is result
    assert type(stored.received_at_ns) is int


def test_release_result_is_stored_directly_with_validated_timestamp() -> None:
    # Given
    result = ReleaseRequestResult(1, True)

    # When
    observed = _model().accept_release_result(result)

    # Then
    stored = observed.release_result
    assert stored is not None and stored is result
    assert type(stored.received_at_ns) is int


@pytest.mark.parametrize("received_at_ns", [True, nan])
def test_invalid_usv_status_timestamp_is_rejected_without_storing(
    received_at_ns: int | float,
) -> None:
    # Given
    previous = _usv(1)
    model = _model().observe_usv_status(previous)

    # When
    observed = model.observe_usv_status(_usv(received_at_ns))

    # Then
    assert observed.usv_status is previous
    assert observed.state is InitializerState.FAILED


@pytest.mark.parametrize("received_at_ns", [True, nan])
def test_invalid_motor_state_timestamp_is_rejected_without_storing(
    received_at_ns: int | float,
) -> None:
    # Given
    previous = _motor(1)
    model = _model().observe_motor_state(previous)

    # When
    observed = model.observe_motor_state(_motor(received_at_ns))

    # Then
    assert observed.motor_state is previous
    assert observed.state is InitializerState.FAILED


@pytest.mark.parametrize("received_at_ns", [True, nan])
def test_invalid_safety_status_timestamp_is_rejected_without_storing(
    received_at_ns: int | float,
) -> None:
    # Given
    previous = _safety(1)
    model = _model().observe_safety_status(previous)

    # When
    observed = model.observe_safety_status(_safety(received_at_ns))

    # Then
    assert observed.safety_status is previous
    assert observed.state is InitializerState.FAILED


@pytest.mark.parametrize("received_at_ns", [True, nan])
def test_invalid_mode_result_timestamp_is_rejected_without_storing(
    received_at_ns: int | float,
) -> None:
    # Given
    previous = ModeRequestResult(1, True, ControlMode.AUTO)
    model = _model().accept_mode_result(previous)

    # When
    observed = model.accept_mode_result(ModeRequestResult(received_at_ns, True, ControlMode.AUTO))

    # Then
    assert observed.mode_result is previous
    assert observed.state is InitializerState.FAILED


@pytest.mark.parametrize("received_at_ns", [True, nan])
def test_invalid_release_result_timestamp_is_rejected_without_storing(
    received_at_ns: int | float,
) -> None:
    # Given
    previous = ReleaseRequestResult(1, True)
    model = _model().accept_release_result(previous)

    # When
    observed = model.accept_release_result(ReleaseRequestResult(received_at_ns, True))

    # Then
    assert observed.release_result is previous
    assert observed.state is InitializerState.FAILED
