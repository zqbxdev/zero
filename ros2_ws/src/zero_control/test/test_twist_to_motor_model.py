from math import inf, isclose, nan, pi

from zero_control import (
    BodyTwist,
    InvalidTuningError,
    MotorTargets,
    TwistToMotorModel,
    TwistToMotorTuning,
    motor_rpm_to_twist,
    twist_to_motor_targets,
)


def _rpm_for_velocity(velocity_mps: float, wheel_radius_m: float = 0.05) -> float:
    return velocity_mps / (2.0 * pi * wheel_radius_m) * 60.0


def test_straight_forward_and_reverse_use_wheel_circumference() -> None:
    # Given
    tuning = TwistToMotorTuning()

    # When
    forward = twist_to_motor_targets(0.5, 0.0, tuning)
    reverse = twist_to_motor_targets(-0.5, 0.0, tuning)

    # Then
    expected_rpm = _rpm_for_velocity(0.5)
    assert isclose(forward.left_target_rpm, expected_rpm)
    assert isclose(forward.right_target_rpm, expected_rpm)
    assert isclose(reverse.left_target_rpm, -expected_rpm)
    assert isclose(reverse.right_target_rpm, -expected_rpm)


def test_left_and_right_turns_preserve_wheel_sign_convention() -> None:
    # Given
    model = TwistToMotorModel()

    # When
    left_turn = model.convert(linear_x=0.0, angular_z=0.5)
    right_turn = model.convert(linear_x=0.0, angular_z=-0.5)

    # Then
    assert left_turn.left_target_rpm < 0.0 < left_turn.right_target_rpm
    assert right_turn.right_target_rpm < 0.0 < right_turn.left_target_rpm
    assert isclose(left_turn.left_target_rpm, -right_turn.left_target_rpm)
    assert isclose(left_turn.right_target_rpm, -right_turn.right_target_rpm)


def test_opposite_rpm_produces_zero_linear_and_positive_angular_twist() -> None:
    # Given
    tuning = TwistToMotorTuning()

    # When
    twist = motor_rpm_to_twist(-60.0, 60.0, tuning)

    # Then
    expected_angular = 4.0 * pi * tuning.wheel_radius_m / tuning.track_width_m
    assert twist.linear_x == 0.0
    assert isclose(twist.angular_z, expected_angular)


def test_zero_is_stable_in_both_directions() -> None:
    # Given
    tuning = TwistToMotorTuning()

    # When
    targets = twist_to_motor_targets(0.0, 0.0, tuning)
    twist = motor_rpm_to_twist(0.0, 0.0, tuning)

    # Then
    assert targets == MotorTargets(0.0, 0.0)
    assert twist == BodyTwist(0.0, 0.0)


def test_input_components_are_clamped_before_conversion() -> None:
    # Given
    tuning = TwistToMotorTuning(
        max_linear_mps=0.4,
        max_angular_radps=0.2,
        track_width_m=0.4,
    )

    # When
    targets = twist_to_motor_targets(5.0, -3.0, tuning)

    # Then
    assert isclose(targets.left_target_rpm, _rpm_for_velocity(0.44))
    assert isclose(targets.right_target_rpm, _rpm_for_velocity(0.36))


def test_final_wheel_rpm_is_saturated_independently() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=100.0, track_width_m=0.4)

    # When
    targets = twist_to_motor_targets(0.4, 1.0, tuning)

    # Then
    assert isclose(targets.left_target_rpm, _rpm_for_velocity(0.2))
    assert targets.right_target_rpm == 100.0


def test_non_finite_forward_inputs_fail_closed_to_zero() -> None:
    # Given
    invalid_inputs = ((nan, 0.0), (0.0, inf), (-inf, 0.0), (0.0, nan))

    # When
    targets = tuple(twist_to_motor_targets(linear, angular) for linear, angular in invalid_inputs)

    # Then
    assert all(target == MotorTargets(0.0, 0.0) for target in targets)


def test_non_finite_inverse_rpm_inputs_fail_closed_to_zero() -> None:
    # Given
    invalid_inputs = ((nan, 0.0), (0.0, inf), (-inf, 0.0), (0.0, nan))

    # When
    twists = tuple(motor_rpm_to_twist(left, right) for left, right in invalid_inputs)

    # Then
    assert all(twist == BodyTwist(0.0, 0.0) for twist in twists)


def test_inverse_preserves_finite_actual_rpm_above_forward_target_limit() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=30.0)

    # When
    twist = motor_rpm_to_twist(60.0, 60.0, tuning)

    # Then
    assert isclose(twist.linear_x, 2.0 * pi * tuning.wheel_radius_m)
    assert twist.angular_z == 0.0


def test_twist_to_rpm_to_twist_round_trip_when_unsaturated() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=300.0)
    original = BodyTwist(linear_x=0.3, angular_z=-0.4)

    # When
    targets = twist_to_motor_targets(original.linear_x, original.angular_z, tuning)
    restored = motor_rpm_to_twist(targets.left_target_rpm, targets.right_target_rpm, tuning)

    # Then
    assert isclose(restored.linear_x, original.linear_x)
    assert isclose(restored.angular_z, original.angular_z)


def test_rpm_to_twist_to_rpm_round_trip_when_unsaturated() -> None:
    # Given
    tuning = TwistToMotorTuning(max_rpm=300.0)
    original = MotorTargets(left_target_rpm=-20.0, right_target_rpm=20.0)

    # When
    twist = motor_rpm_to_twist(original.left_target_rpm, original.right_target_rpm, tuning)
    restored = twist_to_motor_targets(twist.linear_x, twist.angular_z, tuning)

    # Then
    assert isclose(restored.left_target_rpm, original.left_target_rpm)
    assert isclose(restored.right_target_rpm, original.right_target_rpm)


def test_default_tuning_uses_v1_simulation_contract() -> None:
    # Given
    tuning = TwistToMotorTuning()

    # When
    defaults = (
        tuning.max_linear_mps,
        tuning.max_angular_radps,
        tuning.max_rpm,
        tuning.track_width_m,
        tuning.wheel_radius_m,
    )

    # Then
    assert defaults == (1.0, 1.0, 300.0, 0.407, 0.05)


def test_non_positive_or_non_finite_tuning_is_rejected() -> None:
    # Given
    constructors = (
        lambda: TwistToMotorTuning(max_linear_mps=0.0),
        lambda: TwistToMotorTuning(max_angular_radps=-1.0),
        lambda: TwistToMotorTuning(max_rpm=nan),
        lambda: TwistToMotorTuning(track_width_m=inf),
        lambda: TwistToMotorTuning(wheel_radius_m=0.0),
        lambda: TwistToMotorTuning(max_linear_mps=True),
        lambda: TwistToMotorTuning(max_angular_radps=True),
        lambda: TwistToMotorTuning(max_rpm=True),
        lambda: TwistToMotorTuning(track_width_m=True),
        lambda: TwistToMotorTuning(wheel_radius_m=True),
    )

    # When
    errors: list[InvalidTuningError] = []
    for constructor in constructors:
        try:
            _ = constructor()
        except InvalidTuningError as error:
            errors.append(error)

    # Then
    assert tuple(error.field_name for error in errors) == (
        "max_linear_mps",
        "max_angular_radps",
        "max_rpm",
        "track_width_m",
        "wheel_radius_m",
        "max_linear_mps",
        "max_angular_radps",
        "max_rpm",
        "track_width_m",
        "wheel_radius_m",
    )
