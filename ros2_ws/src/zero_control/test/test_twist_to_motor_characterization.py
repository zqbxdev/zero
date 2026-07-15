from math import isclose, pi

from zero_control import BodyTwist, TwistToMotorTuning
from zero_control import motor_rpm_to_twist, twist_to_motor_targets


def test_finite_twist_conversion_uses_shared_geometry() -> None:
    # Given
    tuning = TwistToMotorTuning(track_width_m=0.4, wheel_radius_m=0.05)

    # When
    targets = twist_to_motor_targets(0.3, 0.5, tuning)

    # Then
    rpm_per_mps = 60.0 / (2.0 * pi * tuning.wheel_radius_m)
    assert isclose(targets.left_target_rpm, 0.2 * rpm_per_mps)
    assert isclose(targets.right_target_rpm, 0.4 * rpm_per_mps)


def test_forward_conversion_saturates_each_motor_independently() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=50.0, track_width_m=0.4)

    # When
    targets = twist_to_motor_targets(0.4, 1.0, tuning)

    # Then
    assert isclose(targets.left_target_rpm, 38.19718634205488)
    assert targets.right_target_rpm == 50.0


def test_inverse_conversion_preserves_finite_overspeed_rpm() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=30.0, wheel_radius_m=0.05)

    # When
    twist = motor_rpm_to_twist(60.0, 60.0, tuning)

    # Then
    assert twist == BodyTwist(linear_x=2.0 * pi * tuning.wheel_radius_m, angular_z=0.0)
