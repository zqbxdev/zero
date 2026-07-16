from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TypeAlias

from .command_guard_types import ControlMode


RawNanoseconds: TypeAlias = int | float


class InvalidInitializerTuningError(ValueError):
    def __init__(self, field_name: str) -> None:
        self.field_name: str = field_name
        super().__init__(f"{field_name} must be a positive integer")


class InitializerState(Enum):
    WAIT_ESTOP_TRUE = "wait_estop_true"
    WAIT_AUTO_RESPONSE = "wait_auto_response"
    WAIT_HEALTHY_STATUS = "wait_healthy_status"
    WAIT_ESTOP_FALSE_LATCHED = "wait_estop_false_latched"
    WAIT_RELEASE_RESPONSE = "wait_release_response"
    WAIT_LATCH_RELEASE = "wait_latch_release"
    SUCCESS = "success"
    FAILED = "failed"


class TerminalResult(Enum):
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


@dataclass(frozen=True, slots=True)
class InitializerTuning:
    # USV and safety status timeout [ns].
    # 无人船状态和安全状态超时时间 [ns]。
    status_timeout_ns: int
    # Motor feedback timeout [ns].
    # 电机反馈超时时间 [ns]。
    motor_state_timeout_ns: int

    def __post_init__(self) -> None:
        _require_positive_integer("status_timeout_ns", self.status_timeout_ns)
        _require_positive_integer("motor_state_timeout_ns", self.motor_state_timeout_ns)


@dataclass(frozen=True, slots=True)
class UsvStatusObservation:
    # Local receive timestamp [ns].
    # 本地接收时间戳 [ns]。
    received_at_ns: RawNanoseconds
    # Current operating mode.
    # 当前运行模式。
    mode: ControlMode
    # True when the USV reports a fault.
    # 无人船报告故障时为 true。
    fault: bool
    # Numeric fault code; 0 means no fault.
    # 数值故障码，0 表示无故障。
    fault_code: int
    # Human-readable fault detail; empty when fault is false.
    # 可读故障信息；无故障时为空。
    fault_message: str


@dataclass(frozen=True, slots=True)
class MotorStateObservation:
    # Local receive timestamp [ns].
    # 本地接收时间戳 [ns]。
    received_at_ns: RawNanoseconds
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
class SafetyStatusObservation:
    # Local receive timestamp [ns].
    # 本地接收时间戳 [ns]。
    received_at_ns: RawNanoseconds
    # Guard-visible operating mode.
    # 安全门可见的运行模式。
    mode: ControlMode
    # True when the guard reports a safety fault.
    # 安全门报告安全故障时为 true。
    fault: bool
    # Numeric guard fault code; 0 means no fault.
    # 数值安全门故障码，0 表示无故障。
    fault_code: int
    # Human-readable safety fault detail; empty when fault is false.
    # 可读安全故障信息；无故障时为空。
    fault_message: str


@dataclass(frozen=True, slots=True)
class ModeRequestResult:
    # Local response timestamp [ns].
    # 本地响应时间戳 [ns]。
    received_at_ns: RawNanoseconds
    # True when the mode service accepted the request.
    # 模式服务接受请求时为 true。
    accepted: bool
    # Control mode reported after request handling.
    # 请求处理后报告的控制模式。
    current_mode: ControlMode
    # Human-readable result or rejection reason.
    # 可读结果或拒绝原因。
    message: str = ""


@dataclass(frozen=True, slots=True)
class ReleaseRequestResult:
    # Local response timestamp [ns].
    # 本地响应时间戳 [ns]。
    received_at_ns: RawNanoseconds
    # True when the release service accepted the request.
    # 释放服务接受请求时为 true。
    accepted: bool
    # Human-readable result or rejection reason.
    # 可读结果或拒绝原因。
    message: str = ""


def _require_positive_integer(field_name: str, value: int) -> None:
    if type(value) is not int or value <= 0:
        raise InvalidInitializerTuningError(field_name)
