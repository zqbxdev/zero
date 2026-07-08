from math import inf, nan

from zero_control.twist_to_motor_model import (
    InvalidTuningError,
    TwistToMotorModel,
    TwistToMotorTuning,
)


def test_forward_command_produces_equal_positive_rpm() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=0.5, angular_z=0.0)

    assert command.left_target_rpm == 500.0
    assert command.right_target_rpm == 500.0


def test_reverse_command_produces_equal_negative_rpm() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=-0.25, angular_z=0.0)

    assert command.left_target_rpm == -250.0
    assert command.right_target_rpm == -250.0


def test_positive_angular_command_turns_left_in_simple_sim_sign_convention() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=0.0, angular_z=0.5)

    assert command.left_target_rpm == -250.0
    assert command.right_target_rpm == 250.0


def test_negative_angular_command_turns_right_in_simple_sim_sign_convention() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=0.0, angular_z=-0.5)

    assert command.left_target_rpm == 250.0
    assert command.right_target_rpm == -250.0


def test_mixed_command_keeps_turning_difference() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=0.5, angular_z=0.4)

    assert command.left_target_rpm == 300.0
    assert command.right_target_rpm == 700.0


def test_zero_command_produces_zero_rpm() -> None:
    model = TwistToMotorModel()

    command = model.convert(linear_x=0.0, angular_z=0.0)

    assert command.left_target_rpm == 0.0
    assert command.right_target_rpm == 0.0


def test_linear_input_is_clamped_to_configured_limit() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=2.0, angular_z=0.0)

    assert command.left_target_rpm == 1000.0
    assert command.right_target_rpm == 1000.0


def test_angular_input_is_clamped_to_configured_limit() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_angular_radps=1.0, max_rpm=1000.0))

    command = model.convert(linear_x=0.0, angular_z=2.0)

    assert command.left_target_rpm == -500.0
    assert command.right_target_rpm == 500.0


def test_combined_output_is_clamped_to_max_rpm() -> None:
    model = TwistToMotorModel(TwistToMotorTuning(max_linear_mps=1.0, max_rpm=600.0))

    command = model.convert(linear_x=1.0, angular_z=1.0)

    assert command.left_target_rpm == 300.0
    assert command.right_target_rpm == 600.0


def test_nan_or_infinite_input_is_rejected_with_zero_rpm() -> None:
    model = TwistToMotorModel()

    nan_command = model.convert(linear_x=nan, angular_z=0.0)
    infinite_command = model.convert(linear_x=0.0, angular_z=inf)

    assert nan_command.left_target_rpm == 0.0
    assert nan_command.right_target_rpm == 0.0
    assert infinite_command.left_target_rpm == 0.0
    assert infinite_command.right_target_rpm == 0.0


def test_invalid_tuning_values_are_rejected() -> None:
    try:
        _ = TwistToMotorTuning(max_linear_mps=0.0)
    except InvalidTuningError as error:
        assert str(error) == "max_linear_mps must be positive and finite"
    else:
        raise AssertionError("expected invalid tuning to be rejected")
