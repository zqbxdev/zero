from __future__ import annotations

from dataclasses import dataclass, field
from decimal import Decimal
from math import isfinite
from typing import Final

from zero_control import (
    BodyTwist,
    DurationNanoseconds,
    RosTimeNanoseconds,
    TwistToMotorTuning,
    motor_rpm_to_twist,
)


class InvalidGazeboDriveAdapterTuningError(ValueError):
    def __init__(self, field_name: str, detail: str = "must be positive and finite") -> None:
        self.field_name: str = field_name
        self.detail: str = detail
        super().__init__(f"{field_name} {detail}")


@dataclass(frozen=True, slots=True)
class GazeboDriveAdapterTuning:
    # Shared differential-drive kinematics used by zero_control and Gazebo.
    # zero_control 与 Gazebo 共用的差速运动学参数。
    kinematics: TwistToMotorTuning = field(default_factory=TwistToMotorTuning)
    # Motor feedback timeout [s].
    # 电机反馈超时时间 [s]。
    motor_state_timeout_seconds: float = 0.5
    # Motor feedback timeout [ns], derived exactly from seconds.
    # 由秒精确换算得到的电机反馈超时时间 [ns]。
    motor_state_timeout_nanoseconds: DurationNanoseconds = field(init=False)

    def __post_init__(self) -> None:
        _require_positive_finite(
            "motor_state_timeout_seconds",
            self.motor_state_timeout_seconds,
        )
        object.__setattr__(
            self,
            "motor_state_timeout_nanoseconds",
            _seconds_to_nanoseconds(
                "motor_state_timeout_seconds",
                self.motor_state_timeout_seconds,
            ),
        )


@dataclass(frozen=True, slots=True)
class MotorStateInput:
    # MotorState source timestamp [ns].
    # MotorState 源时间戳 [ns]。
    stamp_nanoseconds: RosTimeNanoseconds
    # Left motor target output-shaft speed [rpm].
    # 左电机目标输出轴转速 [rpm]。
    left_target_rpm: float
    # Right motor target output-shaft speed [rpm].
    # 右电机目标输出轴转速 [rpm]。
    right_target_rpm: float
    # Left motor measured output-shaft speed [rpm].
    # 左电机实测输出轴转速 [rpm]。
    left_actual_rpm: float
    # Right motor measured output-shaft speed [rpm].
    # 右电机实测输出轴转速 [rpm]。
    right_actual_rpm: float
    # Left Hall encoder accumulated count [ticks].
    # 左霍尔编码器累计计数 [ticks]。
    left_encoder_count: int
    # Right Hall encoder accumulated count [ticks].
    # 右霍尔编码器累计计数 [ticks]。
    right_encoder_count: int
    # Left signed PWM duty cycle [%], range -100 to 100.
    # 左电机带符号 PWM 占空比 [%]，范围 -100 到 100。
    left_pwm_duty: float
    # Right signed PWM duty cycle [%], range -100 to 100.
    # 右电机带符号 PWM 占空比 [%]，范围 -100 到 100。
    right_pwm_duty: float
    # True when left motor output is enabled.
    # 左电机输出使能时为 true。
    left_enabled: bool
    # True when right motor output is enabled.
    # 右电机输出使能时为 true。
    right_enabled: bool
    # True when the motor controller reports a fault.
    # 电机控制器报告故障时为 true。
    fault: bool
    # Human-readable fault detail; empty when fault is false.
    # 可读故障信息；无故障时为空。
    fault_message: str


@dataclass(frozen=True, slots=True)
class TimestampedTwist:
    # Body twist accepted for Gazebo command publication.
    # 可发布给 Gazebo 命令的船体速度。
    twist: BodyTwist
    # Local receive timestamp [ns].
    # 本地接收时间戳 [ns]。
    received_at_nanoseconds: RosTimeNanoseconds


# Safe Gazebo command used when motor feedback cannot be trusted.
# 电机反馈不可信时使用的安全 Gazebo 命令。
ZERO_TWIST: Final = BodyTwist(linear_x=0.0, angular_z=0.0)


@dataclass(slots=True)  # noqa: MUTABLE_OK
class GazeboDriveAdapterModel:
    """Retain trusted motor-feedback twist across ROS callbacks.
    在 ROS 回调之间保留可信电机反馈速度。
    """

    tuning: GazeboDriveAdapterTuning = field(default_factory=GazeboDriveAdapterTuning)
    _last_twist: TimestampedTwist | None = field(default=None, init=False)
    _last_observed_time_nanoseconds: RosTimeNanoseconds | None = field(
        default=None,
        init=False,
    )

    def accept_motor_state(
        self,
        state: MotorStateInput,
        received_at_nanoseconds: RosTimeNanoseconds,
    ) -> bool:
        # Reject samples on invalid or regressed local time before deriving twist.
        # 在计算速度前拒绝本地时间无效或回退的样本。
        if not _is_exact_integer(received_at_nanoseconds):
            self._last_twist = None
            return False
        if self._time_regressed(received_at_nanoseconds):
            self._last_twist = None
            self._last_observed_time_nanoseconds = received_at_nanoseconds
            return False

        self._last_observed_time_nanoseconds = received_at_nanoseconds
        actual_rpm_is_valid = (
            isfinite(state.left_actual_rpm)
            and isfinite(state.right_actual_rpm)
            and abs(state.left_actual_rpm) <= self.tuning.kinematics.max_rpm
            and abs(state.right_actual_rpm) <= self.tuning.kinematics.max_rpm
        )
        if state.fault or not actual_rpm_is_valid:
            self._last_twist = None
            return False

        # Convert measured motor RPM back to the Gazebo body-frame command.
        # 将实测电机转速反算为 Gazebo 船体坐标系命令。
        inverse = motor_rpm_to_twist(
            state.left_actual_rpm,
            state.right_actual_rpm,
            self.tuning.kinematics,
        )
        self._last_twist = TimestampedTwist(
            twist=BodyTwist(
                linear_x=_clamp(
                    inverse.linear_x,
                    self.tuning.kinematics.max_linear_mps,
                ),
                angular_z=_clamp(
                    inverse.angular_z,
                    self.tuning.kinematics.max_angular_radps,
                ),
            ),
            received_at_nanoseconds=received_at_nanoseconds,
        )
        return True

    def twist_at(self, now_nanoseconds: RosTimeNanoseconds) -> BodyTwist:
        # The adapter fails closed to zero twist when feedback is stale or time is invalid.
        # 反馈超时或时间无效时，适配器闭锁为零速度。
        if not _is_exact_integer(now_nanoseconds):
            self._last_twist = None
            return ZERO_TWIST
        if self._time_regressed(now_nanoseconds):
            self._last_twist = None
            self._last_observed_time_nanoseconds = now_nanoseconds
            return ZERO_TWIST

        self._last_observed_time_nanoseconds = now_nanoseconds
        sample = self._last_twist
        if sample is None:
            return ZERO_TWIST
        if (
            now_nanoseconds - sample.received_at_nanoseconds
            > self.tuning.motor_state_timeout_nanoseconds
        ):
            self._last_twist = None
            return ZERO_TWIST
        return sample.twist

    def _time_regressed(self, now_nanoseconds: RosTimeNanoseconds) -> bool:
        previous_time = self._last_observed_time_nanoseconds
        return previous_time is not None and now_nanoseconds < previous_time


def _clamp(value: float, magnitude: float) -> float:
    return min(max(value, -magnitude), magnitude)


def _require_positive_finite(field_name: str, value: float) -> None:
    if type(value) is bool or value <= 0.0 or not isfinite(value):
        raise InvalidGazeboDriveAdapterTuningError(field_name)


def _seconds_to_nanoseconds(field_name: str, value: float) -> DurationNanoseconds:
    nanoseconds = Decimal(str(value)) * Decimal(1_000_000_000)
    integral_nanoseconds = nanoseconds.to_integral_value()
    if nanoseconds != integral_nanoseconds:
        raise InvalidGazeboDriveAdapterTuningError(
            field_name,
            "must resolve to whole nanoseconds",
        )
    return DurationNanoseconds(int(integral_nanoseconds))


def _is_exact_integer(value: RosTimeNanoseconds) -> bool:
    return type(value) is int
