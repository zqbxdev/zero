from zero_hardware.fake_motor_model import ControlMode, FakeMotorModel


def test_default_mode_is_stop() -> None:
    model = FakeMotorModel()

    feedback = model.feedback()

    assert model.mode is ControlMode.STOP
    assert feedback.left_enabled is False
    assert feedback.right_enabled is False
    assert feedback.left_target_rpm == 0.0
    assert feedback.right_target_rpm == 0.0


def test_manual_command_updates_target_and_actual_ramps_by_bounded_step() -> None:
    model = FakeMotorModel(max_rpm_step=25.0)
    _ = model.set_mode(ControlMode.MANUAL)

    model.command(120.0, -80.0, received_at_seconds=1.0)
    feedback = model.update(dt_seconds=0.05, now_seconds=1.05)

    assert feedback.left_target_rpm == 120.0
    assert feedback.right_target_rpm == -80.0
    assert feedback.left_actual_rpm == 25.0
    assert feedback.right_actual_rpm == -25.0
    assert feedback.left_enabled is True
    assert feedback.right_enabled is True


def test_default_model_ramps_slowly_enough_to_observe() -> None:
    model = FakeMotorModel()
    _ = model.set_mode(ControlMode.MANUAL)

    model.command(120.0, 120.0, received_at_seconds=1.0)
    first_feedback = model.update(dt_seconds=0.2, now_seconds=1.2)
    second_feedback = model.update(dt_seconds=0.2, now_seconds=1.4)

    assert first_feedback.left_actual_rpm == 10.0
    assert first_feedback.right_actual_rpm == 10.0
    assert second_feedback.left_actual_rpm == 20.0
    assert second_feedback.right_actual_rpm == 20.0


def test_auto_command_clamps_to_configured_command_limit() -> None:
    model = FakeMotorModel(max_command_rpm=100.0)
    _ = model.set_mode(ControlMode.AUTO)

    model.command(200.0, -250.0, received_at_seconds=1.0)
    feedback = model.feedback()

    assert feedback.left_target_rpm == 100.0
    assert feedback.right_target_rpm == -100.0
    assert feedback.left_pwm_duty == 100.0
    assert feedback.right_pwm_duty == -100.0


def test_stop_ignores_new_commands_and_ramps_stored_values_to_zero() -> None:
    model = FakeMotorModel(max_target_step=30.0, max_rpm_step=20.0)
    _ = model.set_mode(ControlMode.MANUAL)
    model.command(90.0, -70.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.05, now_seconds=1.05)
    _ = model.set_mode(ControlMode.STOP)

    model.command(200.0, 200.0, received_at_seconds=1.1)
    feedback = model.update(dt_seconds=0.05, now_seconds=1.1)

    assert feedback.left_target_rpm == 60.0
    assert feedback.right_target_rpm == -40.0
    assert feedback.left_actual_rpm == 40.0
    assert feedback.right_actual_rpm == -40.0
    assert feedback.left_enabled is False
    assert feedback.right_enabled is False
    assert feedback.left_pwm_duty == 0.0
    assert feedback.right_pwm_duty == 0.0


def test_invalid_control_mode_is_rejected_without_changing_mode() -> None:
    model = FakeMotorModel()
    _ = model.set_mode(ControlMode.AUTO)

    result = model.set_mode_value(99)

    assert result.accepted is False
    assert result.current_mode == int(ControlMode.AUTO)
    assert result.message == "Unsupported control mode: 99"
    assert model.mode is ControlMode.AUTO


def test_encoder_counts_follow_actual_rpm() -> None:
    model = FakeMotorModel(max_rpm_step=120.0, encoder_ticks_per_rev=60.0)
    _ = model.set_mode(ControlMode.MANUAL)
    model.command(60.0, -60.0, received_at_seconds=1.0)

    feedback = model.update(dt_seconds=1.0, now_seconds=1.0)

    assert feedback.left_encoder_count == 60
    assert feedback.right_encoder_count == -60


def test_supported_mode_change_reports_the_new_current_mode() -> None:
    # Given
    model = FakeMotorModel()

    # When
    manual_result = model.set_mode_value(int(ControlMode.MANUAL))
    auto_result = model.set_mode_value(int(ControlMode.AUTO))

    # Then
    assert manual_result.accepted is True
    assert manual_result.current_mode == int(ControlMode.MANUAL)
    assert manual_result.message == "Control mode set to MANUAL"
    assert auto_result.accepted is True
    assert auto_result.current_mode == int(ControlMode.AUTO)
    assert model.mode is ControlMode.AUTO


def test_target_and_actual_ramp_limits_remain_independent_in_stop() -> None:
    # Given
    model = FakeMotorModel(max_target_step=30.0, max_rpm_step=10.0)
    _ = model.set_mode(ControlMode.MANUAL)
    model.command(90.0, -90.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.1)
    _ = model.set_mode(ControlMode.STOP)

    # When
    feedback = model.update(dt_seconds=0.2, now_seconds=1.2)

    # Then
    assert feedback.left_target_rpm == 60.0
    assert feedback.right_target_rpm == -60.0
    assert feedback.left_actual_rpm == 20.0
    assert feedback.right_actual_rpm == -20.0


def test_stop_converges_to_zero_without_accepting_a_replacement_command() -> None:
    # Given
    model = FakeMotorModel(max_target_step=40.0, max_rpm_step=20.0)
    _ = model.set_mode(ControlMode.AUTO)
    model.command(80.0, 80.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.1)
    _ = model.set_mode(ControlMode.STOP)
    model.command(-200.0, -200.0, received_at_seconds=1.2)

    # When
    first = model.update(dt_seconds=0.2, now_seconds=1.2)
    second = model.update(dt_seconds=0.2, now_seconds=1.3)

    # Then
    assert first.left_target_rpm == 40.0
    assert first.left_actual_rpm == 40.0
    assert second.left_target_rpm == 0.0
    assert second.left_actual_rpm == 20.0


def test_default_encoder_progression_uses_2048_simulation_ticks_per_revolution() -> None:
    # Given
    model = FakeMotorModel(max_rpm_step=60.0)
    _ = model.set_mode(ControlMode.MANUAL)
    model.command(60.0, 60.0, received_at_seconds=1.0)

    # When
    feedback = model.update(dt_seconds=1.0, now_seconds=1.0)

    # Then
    assert feedback.left_encoder_count == 2048
    assert feedback.right_encoder_count == 2048


def test_finite_command_clamping_preserves_each_motor_direction() -> None:
    # Given
    model = FakeMotorModel(max_command_rpm=75.0)
    _ = model.set_mode(ControlMode.AUTO)

    # When
    model.command(80.0, -90.0, received_at_seconds=1.0)
    feedback = model.feedback()

    # Then
    assert feedback.left_target_rpm == 75.0
    assert feedback.right_target_rpm == -75.0
