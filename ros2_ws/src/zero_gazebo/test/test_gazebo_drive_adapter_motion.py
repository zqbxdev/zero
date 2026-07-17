from dataclasses import replace
from math import isclose

import pytest
from zero_control import BodyTwist, RosTimeNanoseconds, TwistToMotorTuning

from zero_gazebo.gazebo_drive_adapter_model import (
    GazeboDriveAdapterModel,
    GazeboDriveAdapterTuning,
    MotorStateInput,
)


DEFAULT_STATE = MotorStateInput(
    stamp_nanoseconds=RosTimeNanoseconds(0),
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
)


def _time(seconds: int, nanoseconds: int = 0) -> RosTimeNanoseconds:
    return RosTimeNanoseconds(seconds * 1_000_000_000 + nanoseconds)


def _assert_twist_close(actual: BodyTwist, expected: BodyTwist) -> None:
    assert isclose(actual.linear_x, expected.linear_x)
    assert isclose(actual.angular_z, expected.angular_z)


@pytest.mark.parametrize(
    ("left_actual_rpm", "right_actual_rpm", "expected"),
    (
        (60.0, 60.0, BodyTwist(linear_x=0.3141592653589793, angular_z=0.0)),
        (-60.0, -60.0, BodyTwist(linear_x=-0.3141592653589793, angular_z=0.0)),
        (-30.0, 30.0, BodyTwist(linear_x=0.0, angular_z=0.771890086877098)),
    ),
)
def test_actual_rpm_maps_forward_reverse_and_spin(
    left_actual_rpm: float,
    right_actual_rpm: float,
    expected: BodyTwist,
) -> None:
    # Given: fresh fault-free actual motor feedback
    model = GazeboDriveAdapterModel()
    state = replace(
        DEFAULT_STATE,
        left_actual_rpm=left_actual_rpm,
        right_actual_rpm=right_actual_rpm,
    )

    # When: the adapter receives and evaluates the sample
    accepted = model.accept_motor_state(state, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: signed body motion follows the actual RPM pair
    assert accepted
    _assert_twist_close(twist, expected)


def test_target_only_feedback_cannot_drive_output() -> None:
    # Given: nonzero targets paired with zero actual RPM
    model = GazeboDriveAdapterModel()
    target_only = replace(
        DEFAULT_STATE,
        left_target_rpm=300.0,
        right_target_rpm=-300.0,
    )

    # When: the target-only sample is evaluated
    accepted = model.accept_motor_state(target_only, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: targets cannot create Gazebo motion
    assert accepted
    assert twist == BodyTwist(linear_x=0.0, angular_z=0.0)


def test_actual_feedback_wins_when_targets_disagree() -> None:
    # Given: targets disagree with finite actual forward RPM
    model = GazeboDriveAdapterModel()
    mismatch = replace(
        DEFAULT_STATE,
        left_target_rpm=-300.0,
        right_target_rpm=300.0,
        left_actual_rpm=60.0,
        right_actual_rpm=60.0,
    )

    # When: the complete state is accepted
    _ = model.accept_motor_state(mismatch, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: output follows actual forward RPM only
    _assert_twist_close(twist, BodyTwist(linear_x=0.3141592653589793, angular_z=0.0))


@pytest.mark.parametrize(
    ("left_actual_rpm", "right_actual_rpm", "expected"),
    (
        (300.0, 300.0, BodyTwist(linear_x=1.0, angular_z=0.0)),
        (-300.0, -300.0, BodyTwist(linear_x=-1.0, angular_z=0.0)),
        (-300.0, 300.0, BodyTwist(linear_x=0.0, angular_z=1.0)),
        (300.0, -300.0, BodyTwist(linear_x=0.0, angular_z=-1.0)),
    ),
)
def test_exact_rpm_bounds_are_valid_and_output_is_clamped(
    left_actual_rpm: float,
    right_actual_rpm: float,
    expected: BodyTwist,
) -> None:
    # Given: actual RPM exactly on the inclusive shared bounds
    model = GazeboDriveAdapterModel()
    state = replace(
        DEFAULT_STATE,
        left_actual_rpm=left_actual_rpm,
        right_actual_rpm=right_actual_rpm,
    )

    # When: the boundary sample is converted
    accepted = model.accept_motor_state(state, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: the sample is accepted and body limits are enforced
    assert accepted
    assert twist == expected


@pytest.mark.parametrize(
    ("state", "expected"),
    (
        (
            replace(DEFAULT_STATE, left_actual_rpm=60.0, right_actual_rpm=60.0),
            BodyTwist(linear_x=0.2, angular_z=0.0),
        ),
        (
            replace(DEFAULT_STATE, left_actual_rpm=-60.0, right_actual_rpm=60.0),
            BodyTwist(linear_x=0.0, angular_z=0.25),
        ),
    ),
)
def test_valid_inverse_output_uses_configured_body_clamps(
    state: MotorStateInput,
    expected: BodyTwist,
) -> None:
    # Given: body limits lower than the valid inverse-conversion output
    tuning = GazeboDriveAdapterTuning(
        kinematics=TwistToMotorTuning(max_linear_mps=0.2, max_angular_radps=0.25),
    )
    model = GazeboDriveAdapterModel(tuning)

    # When: valid in-range RPM is converted
    accepted = model.accept_motor_state(state, received_at_nanoseconds=_time(1))
    twist = model.twist_at(now_nanoseconds=_time(1))

    # Then: conversion succeeds and clamps only the resulting Twist
    assert accepted
    assert twist == expected
