from dataclasses import replace
from math import inf, nan

import pytest
from zero_control import BodyTwist, RosTimeNanoseconds

from zero_gazebo.gazebo_drive_adapter_model import (
    GazeboDriveAdapterModel,
    GazeboDriveAdapterTuning,
    MotorStateInput,
)


ZERO_TWIST = BodyTwist(linear_x=0.0, angular_z=0.0)
DEFAULT_STATE = MotorStateInput(
    stamp_nanoseconds=RosTimeNanoseconds(0),
    left_target_rpm=0.0,
    right_target_rpm=0.0,
    left_actual_rpm=60.0,
    right_actual_rpm=60.0,
    left_encoder_count=0,
    right_encoder_count=0,
    left_pwm_duty=0.0,
    right_pwm_duty=0.0,
    left_enabled=True,
    right_enabled=True,
    fault=False,
    fault_message="",
)


def _time(seconds: int, nanoseconds: int = 0) -> RosTimeNanoseconds:
    return RosTimeNanoseconds(seconds * 1_000_000_000 + nanoseconds)


def test_missing_feedback_outputs_zero() -> None:
    # Given: an adapter with no accepted motor state
    model = GazeboDriveAdapterModel()

    # When: output is requested
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: missing state fails closed
    assert twist == ZERO_TWIST


def test_exact_timeout_is_fresh_and_one_nanosecond_late_is_stale() -> None:
    # Given: one valid sample and a half-second timeout
    tuning = GazeboDriveAdapterTuning(motor_state_timeout_seconds=0.5)
    model = GazeboDriveAdapterModel(tuning)
    _ = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(10))

    # When: output is checked at and just beyond the timeout
    at_timeout = model.twist_at(now_nanoseconds=_time(10, 500_000_000))
    one_nanosecond_late = model.twist_at(now_nanoseconds=_time(10, 500_000_001))

    # Then: freshness uses inclusive age less than or equal to timeout
    assert at_timeout != ZERO_TWIST
    assert one_nanosecond_late == ZERO_TWIST


@pytest.mark.parametrize(
    ("left_actual_rpm", "right_actual_rpm"),
    (
        (300.000001, 0.0),
        (-300.000001, 0.0),
        (0.0, 300.000001),
        (0.0, -300.000001),
    ),
)
def test_out_of_range_actual_rpm_is_invalid_not_clamped(
    left_actual_rpm: float,
    right_actual_rpm: float,
) -> None:
    # Given: actual feedback outside either inclusive RPM bound
    model = GazeboDriveAdapterModel()
    overspeed = replace(
        DEFAULT_STATE,
        left_actual_rpm=left_actual_rpm,
        right_actual_rpm=right_actual_rpm,
    )

    # When: the overspeed sample arrives
    accepted = model.accept_motor_state(overspeed, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: the entire sample is rejected rather than RPM-clamped
    assert not accepted
    assert twist == ZERO_TWIST


@pytest.mark.parametrize(
    ("left_actual_rpm", "right_actual_rpm"),
    (
        (nan, 0.0),
        (inf, 0.0),
        (-inf, 0.0),
        (0.0, nan),
        (0.0, inf),
        (0.0, -inf),
    ),
)
def test_nonfinite_actual_rpm_invalidates_previous_feedback(
    left_actual_rpm: float,
    right_actual_rpm: float,
) -> None:
    # Given: a previously valid sample followed by malformed actual RPM
    model = GazeboDriveAdapterModel()
    _ = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(1))
    invalid = replace(
        DEFAULT_STATE,
        left_actual_rpm=left_actual_rpm,
        right_actual_rpm=right_actual_rpm,
    )

    # When: malformed feedback replaces the previous state
    accepted = model.accept_motor_state(invalid, received_at_nanoseconds=_time(1, 1))
    twist = model.twist_at(now_nanoseconds=_time(1, 2))

    # Then: malformed feedback cannot replay cached motion
    assert not accepted
    assert twist == ZERO_TWIST


def test_fault_feedback_invalidates_previous_feedback() -> None:
    # Given: valid motion followed by a controller fault
    model = GazeboDriveAdapterModel()
    _ = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(1))
    fault = replace(DEFAULT_STATE, fault=True, fault_message="simulated fault")

    # When: the fault sample arrives
    accepted = model.accept_motor_state(fault, received_at_nanoseconds=_time(1, 1))
    twist = model.twist_at(now_nanoseconds=_time(1, 2))

    # Then: fault state publishes zero without replaying prior RPM
    assert not accepted
    assert twist == ZERO_TWIST


def test_clock_rollback_requires_a_later_new_sample() -> None:
    # Given: valid feedback observed on a later ROS time epoch
    model = GazeboDriveAdapterModel()
    _ = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(10))
    before_rollback = model.twist_at(now_nanoseconds=_time(10, 100_000_000))

    # When: time rolls back and a later sample in the new epoch arrives
    rolled_back = model.twist_at(now_nanoseconds=_time(5))
    before_recovery = model.twist_at(now_nanoseconds=_time(5, 100_000_000))
    recovery_state = replace(DEFAULT_STATE, left_actual_rpm=-60.0, right_actual_rpm=-60.0)
    recovered_accept = model.accept_motor_state(
        recovery_state,
        received_at_nanoseconds=_time(5, 200_000_000),
    )
    recovered = model.twist_at(now_nanoseconds=_time(5, 200_000_000))

    # Then: cached motion is cleared and only the new sample restores output
    assert before_rollback != ZERO_TWIST
    assert rolled_back == ZERO_TWIST
    assert before_recovery == ZERO_TWIST
    assert recovered_accept
    assert recovered.linear_x < 0.0


def test_regressed_sample_is_rejected_before_recovery() -> None:
    # Given: valid feedback and an observed later time
    model = GazeboDriveAdapterModel()
    _ = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(10))
    _ = model.twist_at(now_nanoseconds=_time(10, 100_000_000))

    # When: an older sample arrives, followed by a later sample in that epoch
    regressed = model.accept_motor_state(DEFAULT_STATE, received_at_nanoseconds=_time(5))
    after_regression = model.twist_at(now_nanoseconds=_time(5))
    recovered = model.accept_motor_state(
        DEFAULT_STATE,
        received_at_nanoseconds=_time(5, 1),
    )

    # Then: rollback is zero and recovery needs genuinely newer feedback
    assert not regressed
    assert after_regression == ZERO_TWIST
    assert recovered


def test_timeout_is_stored_as_exact_integer_nanoseconds() -> None:
    # Given/When: a decimal timeout with an exact nanosecond representation
    tuning = GazeboDriveAdapterTuning(motor_state_timeout_seconds=0.5)

    # Then: the threshold is integer nanoseconds, not floating-point seconds
    assert tuning.motor_state_timeout_nanoseconds == 500_000_000
    assert type(tuning.motor_state_timeout_nanoseconds) is int
