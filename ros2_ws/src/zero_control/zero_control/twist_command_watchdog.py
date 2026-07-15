from __future__ import annotations

from dataclasses import dataclass, field
from decimal import Decimal
from math import isfinite
from typing import Final, NewType

from .twist_to_motor_model import BodyTwist


RosTimeNanoseconds = NewType("RosTimeNanoseconds", int)
DurationNanoseconds = NewType("DurationNanoseconds", int)


class InvalidWatchdogTuningError(ValueError):
    def __init__(self, field_name: str, detail: str = "must be positive and finite") -> None:
        self.field_name: str = field_name
        self.detail: str = detail
        super().__init__(f"{field_name} {detail}")


@dataclass(frozen=True, slots=True)
class TwistCommandWatchdogTuning:
    command_timeout_seconds: float = 0.5
    publish_rate_hz: float = 20.0
    command_timeout_nanoseconds: DurationNanoseconds = field(init=False)

    def __post_init__(self) -> None:
        _require_positive_finite("command_timeout_seconds", self.command_timeout_seconds)
        _require_positive_finite("publish_rate_hz", self.publish_rate_hz)
        object.__setattr__(
            self,
            "command_timeout_nanoseconds",
            _seconds_to_nanoseconds("command_timeout_seconds", self.command_timeout_seconds),
        )


@dataclass(frozen=True, slots=True)
class TimestampedTwistCommand:
    command: BodyTwist
    received_at_nanoseconds: RosTimeNanoseconds


ZERO_TWIST: Final = BodyTwist(linear_x=0.0, angular_z=0.0)


@dataclass(slots=True)  # noqa: MUTABLE_OK
class TwistCommandWatchdog:
    """Pure Python watchdog retaining command state between ROS callbacks.
    在 ROS 回调间保留命令状态的纯 Python 看门狗。
    """

    tuning: TwistCommandWatchdogTuning = field(default_factory=TwistCommandWatchdogTuning)
    _last_command: TimestampedTwistCommand | None = field(default=None, init=False)
    _last_observed_time_nanoseconds: RosTimeNanoseconds | None = field(default=None, init=False)

    def accept_command(
        self,
        command: BodyTwist,
        received_at_nanoseconds: RosTimeNanoseconds,
    ) -> bool:
        if not _is_exact_integer(received_at_nanoseconds):
            self._last_command = None
            return False
        if self._time_regressed(received_at_nanoseconds):
            self._last_command = None
            self._last_observed_time_nanoseconds = received_at_nanoseconds
            return False

        self._last_observed_time_nanoseconds = received_at_nanoseconds
        if not isfinite(command.linear_x) or not isfinite(command.angular_z):
            self._last_command = None
            return False

        self._last_command = TimestampedTwistCommand(
            command=command,
            received_at_nanoseconds=received_at_nanoseconds,
        )
        return True

    def command_at(self, now_nanoseconds: RosTimeNanoseconds) -> BodyTwist:
        if not _is_exact_integer(now_nanoseconds):
            self._last_command = None
            return ZERO_TWIST
        if self._time_regressed(now_nanoseconds):
            self._last_command = None
            self._last_observed_time_nanoseconds = now_nanoseconds
            return ZERO_TWIST

        self._last_observed_time_nanoseconds = now_nanoseconds
        sample = self._last_command
        if sample is None:
            return ZERO_TWIST
        if (
            now_nanoseconds - sample.received_at_nanoseconds
            > self.tuning.command_timeout_nanoseconds
        ):
            self._last_command = None
            return ZERO_TWIST
        return sample.command

    def _time_regressed(self, now_nanoseconds: RosTimeNanoseconds) -> bool:
        previous_time = self._last_observed_time_nanoseconds
        return previous_time is not None and now_nanoseconds < previous_time


def _require_positive_finite(field_name: str, value: float) -> None:
    if type(value) is bool or value <= 0.0 or not isfinite(value):
        raise InvalidWatchdogTuningError(field_name)


def _seconds_to_nanoseconds(field_name: str, value: float) -> DurationNanoseconds:
    nanoseconds = Decimal(str(value)) * Decimal(1_000_000_000)
    integral_nanoseconds = nanoseconds.to_integral_value()
    if nanoseconds != integral_nanoseconds:
        raise InvalidWatchdogTuningError(field_name, "must resolve to whole nanoseconds")
    return DurationNanoseconds(int(integral_nanoseconds))


def _is_exact_integer(value: RosTimeNanoseconds) -> bool:
    return type(value) is int
