from __future__ import annotations

from dataclasses import dataclass, field
from decimal import Decimal
from math import isfinite
from typing import Final, NewType

from .twist_to_motor_model import BodyTwist


# ROS clock timestamp [ns].
# ROS 时钟时间戳 [ns]。
RosTimeNanoseconds = NewType("RosTimeNanoseconds", int)
# Duration threshold [ns].
# 持续时间阈值 [ns]。
DurationNanoseconds = NewType("DurationNanoseconds", int)


class InvalidWatchdogTuningError(ValueError):
    def __init__(self, field_name: str, detail: str = "must be positive and finite") -> None:
        self.field_name: str = field_name
        self.detail: str = detail
        super().__init__(f"{field_name} {detail}")


@dataclass(frozen=True, slots=True)
class TwistCommandWatchdogTuning:
    # /cmd_vel freshness timeout [s].
    # /cmd_vel 新鲜度超时时间 [s]。
    command_timeout_seconds: float = 0.5
    # Raw motor command publish rate [Hz].
    # 原始电机命令发布频率 [Hz]。
    publish_rate_hz: float = 20.0
    # Exact integer timeout threshold [ns].
    # 精确整数超时阈值 [ns]。
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
    # Accepted body twist command.
    # 已接受的船体速度命令。
    command: BodyTwist
    # Local receive timestamp from the ROS clock [ns].
    # 来自 ROS 时钟的本地接收时间戳 [ns]。
    received_at_nanoseconds: RosTimeNanoseconds


ZERO_TWIST: Final = BodyTwist(linear_x=0.0, angular_z=0.0)


@dataclass(slots=True)  # noqa: MUTABLE_OK
class TwistCommandWatchdog:
    """
    Retain command state between ROS callbacks in a pure Python watchdog.

    在 ROS 回调间保留命令状态的纯 Python 看门狗。
    """

    # Watchdog timing configuration.
    # 看门狗时间配置。
    tuning: TwistCommandWatchdogTuning = field(default_factory=TwistCommandWatchdogTuning)
    # Last accepted command sample, cleared on invalid input or timeout.
    # 最近一次接受的命令样本；输入无效或超时时清空。
    _last_command: TimestampedTwistCommand | None = field(default=None, init=False)
    # Last observed ROS time used to reject clock rollback [ns].
    # 用于拒绝时钟回退的最近 ROS 时间 [ns]。
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
