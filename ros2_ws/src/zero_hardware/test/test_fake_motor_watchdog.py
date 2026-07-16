from math import inf, nan

from zero_hardware.fake_motor_model import ControlMode, FakeMotorModel, MotorFeedback


def _active_model(
    command_timeout_seconds: float = 0.5,
    max_rpm_step: float = 10.0,
    max_target_step: float = 20.0,
) -> FakeMotorModel:
    model = FakeMotorModel(
        command_timeout_seconds=command_timeout_seconds,
        max_rpm_step=max_rpm_step,
        max_target_step=max_target_step,
    )
    _ = model.set_mode(ControlMode.AUTO)
    return model


def test_command_is_fresh_at_exact_timeout_and_faults_immediately_after() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(80.0, -80.0, received_at_seconds=1.0)

    # When
    boundary = model.update(dt_seconds=0.2, now_seconds=1.5)
    timed_out = model.update(dt_seconds=0.2, now_seconds=1.500001)

    # Then
    assert boundary.fault is False
    assert boundary.left_target_rpm == 80.0
    assert timed_out.fault is True
    assert timed_out.fault_message == "Motor command timed out"
    assert timed_out.left_target_rpm == 0.0
    assert timed_out.right_target_rpm == 0.0


def test_non_finite_rpm_commands_fail_closed_with_exact_fault_message() -> None:
    # Given
    invalid_commands = (
        (nan, 20.0),
        (inf, 20.0),
        (-inf, 20.0),
        (20.0, nan),
        (20.0, inf),
        (20.0, -inf),
    )

    # When
    feedback: list[MotorFeedback] = []
    for index, (left_rpm, right_rpm) in enumerate(invalid_commands, start=1):
        model = _active_model()
        model.command(left_rpm, right_rpm, received_at_seconds=float(index))
        feedback.append(model.feedback())

    # Then
    assert all(item.fault is True for item in feedback)
    assert all(item.fault_message == "Motor command contains non-finite rpm" for item in feedback)
    assert all(item.left_target_rpm == 0.0 for item in feedback)
    assert all(item.right_target_rpm == 0.0 for item in feedback)


def test_non_finite_receive_or_update_time_fails_closed() -> None:
    # Given
    command_time_models: list[FakeMotorModel] = []
    for invalid_time in (nan, inf, -inf):
        model = _active_model()
        _ = model.update(dt_seconds=0.2, now_seconds=5.0)
        model.command(40.0, 40.0, received_at_seconds=invalid_time)
        command_time_models.append(model)
    update_time_model = _active_model()
    update_time_model.command(40.0, 40.0, received_at_seconds=5.0)

    # When
    update_feedback = update_time_model.update(dt_seconds=0.2, now_seconds=nan)

    # Then
    assert all(model.feedback().fault is True for model in command_time_models)
    assert all(
        model.feedback().fault_message == "Motor command timestamp is non-finite"
        for model in command_time_models
    )
    assert update_feedback.fault is True
    assert update_feedback.fault_message == "Motor update time is non-finite"
    assert update_feedback.left_target_rpm == 0.0


def test_non_finite_update_interval_fails_closed_without_encoder_progression() -> None:
    # Given
    feedback: list[MotorFeedback] = []
    for invalid_interval in (nan, inf, -inf):
        model = _active_model()
        model.command(40.0, 40.0, received_at_seconds=5.0)

        # When
        feedback.append(model.update(dt_seconds=invalid_interval, now_seconds=5.0))

    # Then
    assert all(item.fault is True for item in feedback)
    assert all(item.fault_message == "Motor update interval is non-finite" for item in feedback)
    assert all(item.left_target_rpm == 0.0 for item in feedback)
    assert all(item.left_encoder_count == 0 for item in feedback)


def test_timeout_zeroes_target_while_actual_rpm_ramps_deterministically() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5, max_rpm_step=20.0)
    model.command(100.0, 100.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.1)
    _ = model.update(dt_seconds=0.2, now_seconds=1.2)
    _ = model.update(dt_seconds=0.2, now_seconds=1.3)

    # When
    timed_out = model.update(dt_seconds=0.2, now_seconds=1.6)
    ramp_two = model.update(dt_seconds=0.2, now_seconds=1.7)
    ramp_three = model.update(dt_seconds=0.2, now_seconds=1.8)

    # Then
    assert timed_out.left_target_rpm == 0.0
    assert timed_out.left_actual_rpm == 40.0
    assert ramp_two.left_actual_rpm == 20.0
    assert ramp_three.left_actual_rpm == 0.0
    assert ramp_three.fault is True


def test_stop_rejects_even_a_later_command_while_fault_remains_latched() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(80.0, 80.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.6)
    _ = model.set_mode(ControlMode.STOP)

    # When
    model.command(50.0, 50.0, received_at_seconds=2.0)
    feedback = model.update(dt_seconds=0.2, now_seconds=2.0)

    # Then
    assert feedback.fault is True
    assert feedback.fault_message == "Motor command timed out"
    assert feedback.left_target_rpm == 0.0
    assert feedback.left_enabled is False


def test_stop_preserves_target_ramp_without_raising_a_watchdog_fault() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5, max_target_step=20.0)
    model.command(100.0, 100.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.1)
    _ = model.set_mode(ControlMode.STOP)

    # When
    feedback = model.update(dt_seconds=0.2, now_seconds=2.0)

    # Then
    assert feedback.fault is False
    assert feedback.fault_message == ""
    assert feedback.left_target_rpm == 80.0
    assert feedback.right_target_rpm == 80.0


def test_leaving_stop_invalidates_cached_target_until_a_new_command_arrives() -> None:
    # Given
    model = _active_model(max_rpm_step=10.0, max_target_step=20.0)
    model.command(100.0, 100.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.1)
    _ = model.set_mode(ControlMode.STOP)
    stopped = model.update(dt_seconds=0.2, now_seconds=1.2)

    # When
    _ = model.set_mode(ControlMode.AUTO)
    resumed = model.feedback()
    ramped = model.update(dt_seconds=0.2, now_seconds=1.3)

    # Then
    assert stopped.left_target_rpm == 80.0
    assert stopped.left_actual_rpm == 20.0
    assert resumed.left_target_rpm == 0.0
    assert resumed.left_pwm_duty == 0.0
    assert resumed.left_actual_rpm == 20.0
    assert ramped.left_target_rpm == 0.0
    assert ramped.left_pwm_duty == 0.0
    assert ramped.left_actual_rpm == 10.0


def test_non_finite_command_in_stop_reports_fault_without_applying_target() -> None:
    # Given
    invalid_commands = (
        (nan, 20.0),
        (inf, 20.0),
        (-inf, 20.0),
        (20.0, nan),
        (20.0, inf),
        (20.0, -inf),
    )

    # When
    feedback: list[MotorFeedback] = []
    for index, (left_rpm, right_rpm) in enumerate(invalid_commands, start=1):
        model = FakeMotorModel()
        model.command(left_rpm, right_rpm, received_at_seconds=float(index))
        feedback.append(model.feedback())

    # Then
    assert all(item.fault is True for item in feedback)
    assert all(item.fault_message == "Motor command contains non-finite rpm" for item in feedback)
    assert all(item.left_target_rpm == 0.0 for item in feedback)
    assert all(item.right_target_rpm == 0.0 for item in feedback)
    assert all(item.left_pwm_duty == 0.0 for item in feedback)
    assert all(item.right_pwm_duty == 0.0 for item in feedback)


def test_clock_rollback_fault_requires_time_to_advance_before_recovery() -> None:
    # Given
    model = _active_model()
    model.command(70.0, 70.0, received_at_seconds=10.0)
    _ = model.update(dt_seconds=0.2, now_seconds=10.1)

    # When
    rolled_back = model.update(dt_seconds=0.2, now_seconds=9.0)
    model.command(30.0, 30.0, received_at_seconds=10.1)
    at_fault_time = model.feedback()
    model.command(30.0, 30.0, received_at_seconds=10.2)
    recovered = model.feedback()

    # Then
    assert rolled_back.fault is True
    assert rolled_back.fault_message == "Motor clock moved backwards"
    assert rolled_back.left_target_rpm == 0.0
    assert at_fault_time.fault is True
    assert at_fault_time.left_target_rpm == 0.0
    assert recovered.fault is False
    assert recovered.fault_message == ""
    assert recovered.left_target_rpm == 30.0


def test_commands_at_or_before_timeout_fault_timestamp_are_rejected() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(80.0, 80.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=2.0)

    # When
    model.command(20.0, 20.0, received_at_seconds=1.9)
    before_fault = model.feedback()
    model.command(30.0, 30.0, received_at_seconds=2.0)
    at_fault = model.feedback()

    # Then
    assert before_fault.fault is True
    assert before_fault.left_target_rpm == 0.0
    assert at_fault.fault is True
    assert at_fault.left_target_rpm == 0.0


def test_later_nonzero_command_clears_fault_and_becomes_the_target() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(80.0, 80.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=2.0)

    # When
    model.command(45.0, -35.0, received_at_seconds=2.1)
    feedback = model.feedback()

    # Then
    assert feedback.fault is False
    assert feedback.fault_message == ""
    assert feedback.left_target_rpm == 45.0
    assert feedback.right_target_rpm == -35.0


def test_later_fresh_zero_command_is_valid_fault_recovery() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(80.0, 80.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=2.0)

    # When
    model.command(0.0, 0.0, received_at_seconds=2.1)
    feedback = model.feedback()

    # Then
    assert feedback.fault is False
    assert feedback.fault_message == ""
    assert feedback.left_target_rpm == 0.0
    assert feedback.right_target_rpm == 0.0


def test_repeated_timeout_recovery_and_rollback_never_replays_old_targets() -> None:
    # Given
    model = _active_model(command_timeout_seconds=0.5)
    model.command(90.0, 90.0, received_at_seconds=1.0)
    _ = model.update(dt_seconds=0.2, now_seconds=1.6)
    model.command(0.0, 0.0, received_at_seconds=1.7)
    _ = model.update(dt_seconds=0.2, now_seconds=1.8)
    model.command(60.0, 60.0, received_at_seconds=1.9)
    _ = model.update(dt_seconds=0.2, now_seconds=2.0)

    # When
    rollback = model.update(dt_seconds=0.2, now_seconds=1.5)
    model.command(90.0, 90.0, received_at_seconds=1.0)
    replay = model.feedback()
    model.command(25.0, 25.0, received_at_seconds=2.1)
    recovered = model.feedback()

    # Then
    assert rollback.fault_message == "Motor clock moved backwards"
    assert replay.fault is True
    assert replay.left_target_rpm == 0.0
    assert recovered.fault is False
    assert recovered.left_target_rpm == 25.0
