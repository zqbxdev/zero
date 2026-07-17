from math import isclose, pi

from zero_control import BodyTwist, TwistToMotorTuning, motor_rpm_to_twist


def test_shared_inverse_maps_signed_forward_reverse_and_spin() -> None:
    # Given: the shared V1 simulation geometry
    tuning = TwistToMotorTuning(track_width_m=0.407, wheel_radius_m=0.05)

    # When: representative signed motor feedback is converted
    forward = motor_rpm_to_twist(60.0, 60.0, tuning)
    reverse = motor_rpm_to_twist(-60.0, -60.0, tuning)
    spin = motor_rpm_to_twist(-60.0, 60.0, tuning)

    # Then: the shared inverse preserves direction and differential yaw
    expected_speed = 2.0 * pi * tuning.wheel_radius_m
    assert isclose(forward.linear_x, expected_speed)
    assert forward.angular_z == 0.0
    assert isclose(reverse.linear_x, -expected_speed)
    assert reverse.angular_z == 0.0
    assert spin.linear_x == 0.0
    assert isclose(spin.angular_z, 2.0 * expected_speed / tuning.track_width_m)


def test_shared_inverse_removes_configured_motor_polarity() -> None:
    # Given: a simulation tuning with an inverted left motor
    tuning = TwistToMotorTuning(left_motor_sign=-1, right_motor_sign=1)

    # When: physical RPM corresponding to logical forward motion is converted
    twist = motor_rpm_to_twist(-60.0, 60.0, tuning)

    # Then: logical body motion is forward without yaw
    assert twist.linear_x > 0.0
    assert twist.angular_z == 0.0


def test_shared_inverse_preserves_finite_overspeed_for_callers_to_handle() -> None:
    # Given: a lower forward RPM contract than the supplied finite feedback
    tuning = TwistToMotorTuning(max_rpm=30.0, wheel_radius_m=0.05)

    # When: the shared inverse receives finite overspeed RPM
    twist = motor_rpm_to_twist(60.0, 60.0, tuning)

    # Then: it converts rather than hiding overspeed behind target clamping
    assert twist == BodyTwist(linear_x=2.0 * pi * tuning.wheel_radius_m, angular_z=0.0)
