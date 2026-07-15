from math import copysign, isclose

import pytest

from zero_control import InvalidTuningError, TwistToMotorTuning
from zero_control import motor_rpm_to_twist, twist_to_motor_targets


def test_forward_conversion_applies_motor_signs_after_logical_kinematics() -> None:
    # Given
    tuning = TwistToMotorTuning(left_motor_sign=-1, right_motor_sign=1)

    # When
    targets = twist_to_motor_targets(0.5, 0.0, tuning)

    # Then
    assert targets.left_target_rpm < 0.0 < targets.right_target_rpm
    assert isclose(targets.left_target_rpm, -targets.right_target_rpm)


def test_inverse_conversion_removes_motor_signs_before_logical_kinematics() -> None:
    # Given
    tuning = TwistToMotorTuning(left_motor_sign=-1, right_motor_sign=1)

    # When
    twist = motor_rpm_to_twist(-60.0, 60.0, tuning)

    # Then
    assert twist.linear_x > 0.0
    assert twist.angular_z == 0.0


def test_signed_forward_inverse_round_trip_is_consistent() -> None:
    # Given
    tuning = TwistToMotorTuning(
        max_rpm=300.0,
        left_motor_sign=-1,
        right_motor_sign=1,
    )

    # When
    targets = twist_to_motor_targets(0.3, -0.4, tuning)
    restored = motor_rpm_to_twist(targets.left_target_rpm, targets.right_target_rpm, tuning)

    # Then
    assert isclose(restored.linear_x, 0.3)
    assert isclose(restored.angular_z, -0.4)


def test_signed_saturation_preserves_motor_polarity() -> None:
    # Given
    tuning = TwistToMotorTuning(
        max_rpm=50.0,
        track_width_m=0.4,
        left_motor_sign=-1,
        right_motor_sign=1,
    )

    # When
    targets = twist_to_motor_targets(0.4, 1.0, tuning)

    # Then
    assert targets.left_target_rpm < 0.0
    assert targets.right_target_rpm == 50.0


def test_signed_zero_uses_canonical_positive_zero() -> None:
    # Given
    tuning = TwistToMotorTuning(left_motor_sign=-1, right_motor_sign=1)

    # When
    targets = twist_to_motor_targets(0.0, 0.0, tuning)

    # Then
    assert copysign(1.0, targets.left_target_rpm) == 1.0
    assert copysign(1.0, targets.right_target_rpm) == 1.0


@pytest.mark.parametrize(
    ("left_motor_sign", "right_motor_sign", "field_name"),
    (
        (0, 1, "left_motor_sign"),
        (2, 1, "left_motor_sign"),
        (True, 1, "left_motor_sign"),
        (1.0, 1, "left_motor_sign"),
        (-1.0, 1, "left_motor_sign"),
        (1, 0, "right_motor_sign"),
        (1, -2, "right_motor_sign"),
        (1, True, "right_motor_sign"),
        (1, 1.0, "right_motor_sign"),
        (1, -1.0, "right_motor_sign"),
    ),
)
def test_motor_signs_are_restricted_to_plus_or_minus_one(
    left_motor_sign: int,
    right_motor_sign: int,
    field_name: str,
) -> None:
    # Given/When
    with pytest.raises(InvalidTuningError) as caught:
        _ = TwistToMotorTuning(
            left_motor_sign=left_motor_sign,
            right_motor_sign=right_motor_sign,
        )

    # Then
    assert caught.value.field_name == field_name
