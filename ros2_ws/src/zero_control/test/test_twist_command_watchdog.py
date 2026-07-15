from math import inf, nan

import pytest

from zero_control import (
    BodyTwist,
    InvalidWatchdogTuningError,
    RosTimeNanoseconds,
    TwistCommandWatchdog,
    TwistCommandWatchdogTuning,
)


ZERO_TWIST = BodyTwist(linear_x=0.0, angular_z=0.0)


def _ros_time(seconds: int, nanoseconds: int = 0) -> RosTimeNanoseconds:
    return RosTimeNanoseconds(seconds * 1_000_000_000 + nanoseconds)


def test_missing_command_outputs_zero() -> None:
    # Given
    watchdog = TwistCommandWatchdog()

    # When
    command = watchdog.command_at(now_nanoseconds=_ros_time(1))

    # Then
    assert command == ZERO_TWIST


def test_fresh_command_is_returned() -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    accepted = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(10),
    )

    # When
    command = watchdog.command_at(now_nanoseconds=_ros_time(10, 250_000_000))

    # Then
    assert accepted
    assert command == BodyTwist(linear_x=0.4, angular_z=-0.2)


def test_command_at_exact_timeout_is_still_fresh() -> None:
    # Given
    watchdog = TwistCommandWatchdog(TwistCommandWatchdogTuning(command_timeout_seconds=0.5))
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(10),
    )

    # When
    command = watchdog.command_at(now_nanoseconds=_ros_time(10, 500_000_000))

    # Then
    assert command == BodyTwist(linear_x=0.4, angular_z=-0.2)


@pytest.mark.parametrize(
    "received_at_nanoseconds",
    (RosTimeNanoseconds(1), RosTimeNanoseconds(2_000_000_000_000_000_000)),
)
def test_timeout_boundary_is_exact_across_ros_time_epochs(
    received_at_nanoseconds: RosTimeNanoseconds,
) -> None:
    # Given
    tuning = TwistCommandWatchdogTuning(command_timeout_seconds=0.5)
    watchdog = TwistCommandWatchdog(tuning)
    expected = BodyTwist(linear_x=0.4, angular_z=-0.2)
    _ = watchdog.accept_command(
        expected,
        received_at_nanoseconds=received_at_nanoseconds,
    )

    # When
    at_timeout = watchdog.command_at(
        now_nanoseconds=RosTimeNanoseconds(
            received_at_nanoseconds + tuning.command_timeout_nanoseconds,
        ),
    )
    one_nanosecond_late = watchdog.command_at(
        now_nanoseconds=RosTimeNanoseconds(
            received_at_nanoseconds + tuning.command_timeout_nanoseconds + 1,
        ),
    )

    # Then
    assert at_timeout == expected
    assert one_nanosecond_late == ZERO_TWIST


def test_stale_command_stays_zero_until_a_new_command() -> None:
    # Given
    watchdog = TwistCommandWatchdog(TwistCommandWatchdogTuning(command_timeout_seconds=0.5))
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(10),
    )

    # When
    stale = watchdog.command_at(now_nanoseconds=_ros_time(10, 500_000_001))
    still_stale = watchdog.command_at(now_nanoseconds=_ros_time(10, 750_000_000))

    # Then
    assert stale == ZERO_TWIST
    assert still_stale == ZERO_TWIST


@pytest.mark.parametrize(
    ("linear_x", "angular_z"),
    ((nan, 0.0), (inf, 0.0), (-inf, 0.0), (0.0, nan), (0.0, inf), (0.0, -inf)),
)
def test_nonfinite_command_invalidates_the_previous_command(
    linear_x: float,
    angular_z: float,
) -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(1),
    )

    # When
    accepted = watchdog.accept_command(
        BodyTwist(linear_x=linear_x, angular_z=angular_z),
        received_at_nanoseconds=_ros_time(1, 100_000_000),
    )
    command = watchdog.command_at(now_nanoseconds=_ros_time(1, 200_000_000))
    still_invalid = watchdog.command_at(now_nanoseconds=_ros_time(1, 300_000_000))

    # Then
    assert not accepted
    assert command == ZERO_TWIST
    assert still_invalid == ZERO_TWIST


@pytest.mark.parametrize("received_at_nanoseconds", (nan, inf, -inf, 1.0, True))
def test_noninteger_timestamp_invalidates_the_previous_command(
    received_at_nanoseconds: RosTimeNanoseconds,
) -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(1),
    )

    # When
    accepted = watchdog.accept_command(
        BodyTwist(linear_x=0.1, angular_z=0.2),
        received_at_nanoseconds=received_at_nanoseconds,
    )
    command = watchdog.command_at(now_nanoseconds=_ros_time(1, 100_000_000))
    still_invalid = watchdog.command_at(now_nanoseconds=_ros_time(1, 200_000_000))

    # Then
    assert not accepted
    assert command == ZERO_TWIST
    assert still_invalid == ZERO_TWIST


def test_clock_rollback_invalidates_pre_pause_command() -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(10),
    )
    _ = watchdog.command_at(now_nanoseconds=_ros_time(10, 250_000_000))

    # When
    rolled_back = watchdog.command_at(now_nanoseconds=_ros_time(5))
    after_recovery = watchdog.command_at(now_nanoseconds=_ros_time(5, 100_000_000))

    # Then
    assert rolled_back == ZERO_TWIST
    assert after_recovery == ZERO_TWIST


def test_time_regressed_command_is_rejected_until_a_later_new_command() -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=-0.2),
        received_at_nanoseconds=_ros_time(10),
    )
    _ = watchdog.command_at(now_nanoseconds=_ros_time(10, 250_000_000))

    # When
    regressed_accepted = watchdog.accept_command(
        BodyTwist(linear_x=0.3, angular_z=0.1),
        received_at_nanoseconds=_ros_time(5),
    )
    after_regression = watchdog.command_at(now_nanoseconds=_ros_time(5))
    recovery_accepted = watchdog.accept_command(
        BodyTwist(linear_x=0.2, angular_z=0.0),
        received_at_nanoseconds=_ros_time(5, 100_000_000),
    )
    recovered = watchdog.command_at(now_nanoseconds=_ros_time(5, 100_000_000))

    # Then
    assert not regressed_accepted
    assert after_regression == ZERO_TWIST
    assert recovery_accepted
    assert recovered == BodyTwist(linear_x=0.2, angular_z=0.0)


def test_repeated_rollbacks_never_replay_an_older_command() -> None:
    # Given
    watchdog = TwistCommandWatchdog()
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.4, angular_z=0.0),
        received_at_nanoseconds=_ros_time(10),
    )
    first = watchdog.command_at(now_nanoseconds=_ros_time(10, 100_000_000))

    # When
    first_rollback = watchdog.command_at(now_nanoseconds=_ros_time(1))
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.2, angular_z=0.0),
        received_at_nanoseconds=_ros_time(1, 100_000_000),
    )
    second = watchdog.command_at(now_nanoseconds=_ros_time(1, 200_000_000))
    second_rollback = watchdog.command_at(now_nanoseconds=_ros_time(0, 100_000_000))
    before_second_recovery = watchdog.command_at(
        now_nanoseconds=_ros_time(0, 200_000_000),
    )
    _ = watchdog.accept_command(
        BodyTwist(linear_x=0.1, angular_z=0.0),
        received_at_nanoseconds=_ros_time(0, 300_000_000),
    )
    third = watchdog.command_at(now_nanoseconds=_ros_time(0, 300_000_000))

    # Then
    assert first == BodyTwist(linear_x=0.4, angular_z=0.0)
    assert first_rollback == ZERO_TWIST
    assert second == BodyTwist(linear_x=0.2, angular_z=0.0)
    assert second_rollback == ZERO_TWIST
    assert before_second_recovery == ZERO_TWIST
    assert third == BodyTwist(linear_x=0.1, angular_z=0.0)


@pytest.mark.parametrize(
    ("field_name", "command_timeout_seconds", "publish_rate_hz"),
    (
        ("command_timeout_seconds", 0.0, 20.0),
        ("command_timeout_seconds", nan, 20.0),
        ("command_timeout_seconds", True, 20.0),
        ("command_timeout_seconds", 0.0000000015, 20.0),
        ("publish_rate_hz", 0.5, 0.0),
        ("publish_rate_hz", 0.5, inf),
        ("publish_rate_hz", 0.5, True),
    ),
)
def test_watchdog_tuning_requires_positive_finite_values(
    field_name: str,
    command_timeout_seconds: float,
    publish_rate_hz: float,
) -> None:
    # Given/When
    with pytest.raises(InvalidWatchdogTuningError) as caught:
        _ = TwistCommandWatchdogTuning(
            command_timeout_seconds=command_timeout_seconds,
            publish_rate_hz=publish_rate_hz,
        )

    # Then
    assert caught.value.field_name == field_name


def test_timeout_is_converted_once_to_an_exact_nanosecond_threshold() -> None:
    # Given/When
    tuning = TwistCommandWatchdogTuning(command_timeout_seconds=0.5)

    # Then
    assert tuning.command_timeout_nanoseconds == 500_000_000
