from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
from math import isfinite


class InvalidSafetyTuningError(ValueError):
    def __init__(self, field_name: str, detail: str = "must be positive and finite") -> None:
        self.field_name: str = field_name
        self.detail: str = detail
        super().__init__(f"{field_name} {detail}")


class CommandSource(Enum):
    AUTO = "auto"
    MANUAL = "manual"
    DEBUG = "debug"


class ControlMode(IntEnum):
    UNKNOWN = 0
    STOP = 1
    MANUAL = 2
    AUTO = 3
    FAULT = 4


class EStopInputState(Enum):
    UNKNOWN = "unknown"
    INACTIVE = "inactive"
    ACTIVE = "active"


class SafetyFaultCode(IntEnum):
    NONE = 0
    MODE_UNKNOWN = 1
    COMMAND_TIMEOUT = 2
    E_STOP_INPUT_ACTIVE = 3
    E_STOP_ACTIVE = 3
    MODE_BLOCKED = 4
    INVALID_COMMAND = 5
    COMMAND_CLAMPED = 6
    E_STOP_UNKNOWN = 7
    E_STOP_LATCHED = 8
    USV_STATUS_UNKNOWN = 9
    USV_STATUS_STALE = 10
    USV_STATUS_FAULT = 11
    MOTOR_STATE_UNKNOWN = 12
    MOTOR_STATE_STALE = 13
    MOTOR_STATE_FAULT = 14
    MODE_FAULT = 15


@dataclass(frozen=True, slots=True)
class SafetyTuning:
    # Maximum absolute motor target speed allowed by the guard [rpm].
    # 安全门允许的最大电机目标转速绝对值 [rpm]。
    max_command_rpm: float = 300.0
    # Raw motor command timeout [s].
    # 原始电机命令超时时间 [s]。
    command_timeout_seconds: float = 0.5
    # USV status timeout [s].
    # 无人船状态超时时间 [s]。
    status_timeout_seconds: float = 0.5
    # Motor feedback timeout [s].
    # 电机反馈超时时间 [s]。
    motor_state_timeout_seconds: float = 0.5
    # Command source expected for the current guard instance.
    # 当前安全门实例期望的命令来源。
    input_source: CommandSource = CommandSource.AUTO

    def __post_init__(self) -> None:
        _require_positive_finite("max_command_rpm", self.max_command_rpm)
        _require_positive_finite("command_timeout_seconds", self.command_timeout_seconds)
        _require_positive_finite("status_timeout_seconds", self.status_timeout_seconds)
        _require_positive_finite("motor_state_timeout_seconds", self.motor_state_timeout_seconds)


@dataclass(frozen=True, slots=True)
class UsvStatusInput:
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
class MotorStateInput:
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
class GuardedCommand:
    # Left motor guarded output-shaft speed [rpm].
    # 左电机安全后输出轴转速 [rpm]。
    left_target_rpm: float
    # Right motor guarded output-shaft speed [rpm].
    # 右电机安全后输出轴转速 [rpm]。
    right_target_rpm: float


@dataclass(frozen=True, slots=True)
class SafetyStatus:
    # Guard-visible operating mode.
    # 安全门可见的运行模式。
    mode: ControlMode
    # True when the guard is blocking or reporting a safety fault.
    # 安全门阻断或报告安全故障时为 true。
    fault: bool
    # Numeric guard fault code; 0 means no fault.
    # 数值安全门故障码，0 表示无故障。
    fault_code: int
    # Human-readable safety fault detail; empty when fault is false.
    # 可读安全故障信息；无故障时为空。
    fault_message: str


@dataclass(frozen=True, slots=True)
class GuardDecision:
    # Motor command selected by the guard.
    # 安全门选定的电机命令。
    command: GuardedCommand
    # Safety status associated with the selected command.
    # 与所选命令关联的安全状态。
    status: SafetyStatus


@dataclass(frozen=True, slots=True)
class ReleaseResult:
    # True when the e-stop latch release was accepted.
    # 急停锁存释放被接受时为 true。
    accepted: bool
    # Human-readable release result or rejection reason.
    # 可读释放结果或拒绝原因。
    message: str


def parse_command_source(value: str) -> CommandSource:
    for source in CommandSource:
        if source.value == value:
            return source
    allowed = ", ".join(("auto", "debug", "manual"))
    raise InvalidSafetyTuningError("input_source", f"must be one of: {allowed}")


def parse_control_mode(value: int) -> ControlMode:
    try:
        return ControlMode(value)
    except ValueError:
        return ControlMode.UNKNOWN


def _require_positive_finite(field_name: str, value: float) -> None:
    if value <= 0.0 or not isfinite(value):
        raise InvalidSafetyTuningError(field_name)
