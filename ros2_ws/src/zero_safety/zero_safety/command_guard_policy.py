from __future__ import annotations

from dataclasses import dataclass
from typing import NoReturn

from .command_guard_types import (
    CommandSource,
    ControlMode,
    EStopInputState,
    GuardDecision,
    GuardedCommand,
    MotorStateInput,
    SafetyFaultCode,
    SafetyStatus,
    SafetyTuning,
    UsvStatusInput,
)


@dataclass(frozen=True, slots=True)
class CommandSample:
    # Left motor requested output-shaft speed [rpm].
    # 左电机请求输出轴转速 [rpm]。
    left_target_rpm: float
    # Right motor requested output-shaft speed [rpm].
    # 右电机请求输出轴转速 [rpm]。
    right_target_rpm: float
    # Local receive timestamp [s].
    # 本地接收时间戳 [s]。
    received_at_seconds: float
    # True when both requested speeds are finite.
    # 两侧请求转速均为有限值时为 true。
    valid: bool


@dataclass(frozen=True, slots=True)
class UsvStatusSample:
    # Latest high-level USV status sample.
    # 最新无人船高层状态样本。
    status: UsvStatusInput
    # Local receive timestamp [s].
    # 本地接收时间戳 [s]。
    received_at_seconds: float


@dataclass(frozen=True, slots=True)
class MotorStateSample:
    # Latest motor feedback sample.
    # 最新电机反馈样本。
    state: MotorStateInput
    # Local receive timestamp [s].
    # 本地接收时间戳 [s]。
    received_at_seconds: float


@dataclass(frozen=True, slots=True)
class GuardSnapshot:
    # Latest external e-stop input state.
    # 最新外部急停输入状态。
    e_stop_input: EStopInputState
    # True while the guard keeps a local e-stop latch active.
    # 安全门保持本地急停锁存时为 true。
    e_stop_latched: bool
    # Latest USV status used as an authoritative safety input.
    # 用作权威安全输入的最新无人船状态。
    usv_status: UsvStatusSample | None
    # Latest motor state used as an authoritative safety input.
    # 用作权威安全输入的最新电机状态。
    motor_state: MotorStateSample | None
    # Latest raw motor command waiting for guard evaluation.
    # 等待安全门评估的最新原始电机命令。
    command: CommandSample | None
    # True when the USV status sample is not invalidated by clock movement.
    # 无人船状态样本未因时钟变化失效时为 true。
    usv_clock_valid: bool
    # True when the motor state sample is not invalidated by clock movement.
    # 电机状态样本未因时钟变化失效时为 true。
    motor_clock_valid: bool
    # True when the raw command sample is not invalidated by clock movement.
    # 原始命令样本未因时钟变化失效时为 true。
    command_clock_valid: bool


def blocking_status(
    snapshot: GuardSnapshot,
    tuning: SafetyTuning,
    now_seconds: float,
) -> SafetyStatus | None:
    # Fault priority: e-stop input, latch, authoritative health, source, command.
    # 故障优先级：急停输入、锁存、权威健康、来源、命令。
    match snapshot.e_stop_input:
        case EStopInputState.UNKNOWN:
            return status(snapshot, SafetyFaultCode.E_STOP_UNKNOWN, "E-stop state is unknown")
        case EStopInputState.ACTIVE:
            return status(
                snapshot,
                SafetyFaultCode.E_STOP_INPUT_ACTIVE,
                "E-stop input is active",
            )
        case EStopInputState.INACTIVE:
            pass
        case unreachable:
            assert_never(unreachable)

    if snapshot.e_stop_latched:
        return status(snapshot, SafetyFaultCode.E_STOP_LATCHED, "E-stop latch is active")

    authoritative = authoritative_status_blocker(snapshot, tuning, now_seconds)
    if authoritative is not None:
        return authoritative

    usv_sample = snapshot.usv_status
    assert usv_sample is not None
    if not mode_allows_source(usv_sample.status.mode, tuning.input_source):
        return status(snapshot, SafetyFaultCode.MODE_BLOCKED, "Control mode blocks command source")

    command = snapshot.command
    if command is None:
        return status(snapshot, SafetyFaultCode.COMMAND_TIMEOUT, "No motor command received")
    if not command.valid:
        return status(
            snapshot,
            SafetyFaultCode.INVALID_COMMAND,
            "Motor command contains non-finite rpm",
        )
    if not snapshot.command_clock_valid or _is_stale(
        command.received_at_seconds,
        now_seconds,
        tuning.command_timeout_seconds,
    ):
        return status(snapshot, SafetyFaultCode.COMMAND_TIMEOUT, "Motor command timed out")
    return None


def authoritative_status_blocker(
    snapshot: GuardSnapshot,
    tuning: SafetyTuning,
    now_seconds: float,
) -> SafetyStatus | None:
    # Check authoritative status streams before applying command-source policy.
    # 先检查权威状态流，再应用命令来源策略。
    usv_sample = snapshot.usv_status
    if usv_sample is None:
        return status(snapshot, SafetyFaultCode.USV_STATUS_UNKNOWN, "USV status is unknown")
    if not snapshot.usv_clock_valid or _is_stale(
        usv_sample.received_at_seconds,
        now_seconds,
        tuning.status_timeout_seconds,
    ):
        return status(snapshot, SafetyFaultCode.USV_STATUS_STALE, "USV status is stale")

    motor_sample = snapshot.motor_state
    if motor_sample is None:
        return status(snapshot, SafetyFaultCode.MOTOR_STATE_UNKNOWN, "Motor state is unknown")
    if not snapshot.motor_clock_valid or _is_stale(
        motor_sample.received_at_seconds,
        now_seconds,
        tuning.motor_state_timeout_seconds,
    ):
        return status(snapshot, SafetyFaultCode.MOTOR_STATE_STALE, "Motor state is stale")

    match usv_sample.status.mode:
        case ControlMode.UNKNOWN:
            return status(snapshot, SafetyFaultCode.MODE_UNKNOWN, "Control mode is unknown")
        case ControlMode.FAULT:
            return status(snapshot, SafetyFaultCode.MODE_FAULT, "Control mode reports fault")
        case ControlMode.STOP | ControlMode.MANUAL | ControlMode.AUTO:
            pass
        case unreachable:
            assert_never(unreachable)

    if usv_sample.status.fault or usv_sample.status.fault_code != 0:
        return status(snapshot, SafetyFaultCode.USV_STATUS_FAULT, "USV status reports fault")
    if motor_sample.state.fault:
        return status(snapshot, SafetyFaultCode.MOTOR_STATE_FAULT, "Motor state reports fault")
    return None


def guarded_decision(
    snapshot: GuardSnapshot,
    tuning: SafetyTuning,
) -> GuardDecision:
    # Clamp an otherwise-safe command to the configured rpm envelope.
    # 将已通过其他安全检查的命令限幅到配置的转速范围内。
    sample = snapshot.command
    assert sample is not None
    command = GuardedCommand(
        left_target_rpm=_clamp(
            sample.left_target_rpm,
            -tuning.max_command_rpm,
            tuning.max_command_rpm,
        ),
        right_target_rpm=_clamp(
            sample.right_target_rpm,
            -tuning.max_command_rpm,
            tuning.max_command_rpm,
        ),
    )
    if _command_changed(sample, command):
        return GuardDecision(
            command=command,
            status=status(
                snapshot,
                SafetyFaultCode.COMMAND_CLAMPED,
                "Motor command clamped to safety rpm limit",
            ),
        )
    return GuardDecision(command=command, status=status(snapshot, SafetyFaultCode.NONE, ""))


def mode_allows_source(mode: ControlMode, source: CommandSource) -> bool:
    # AUTO accepts autonomous commands; MANUAL accepts operator/debug commands only.
    # AUTO 仅接受自主命令；MANUAL 仅接受人工或调试命令。
    match mode:
        case ControlMode.AUTO:
            return source is CommandSource.AUTO
        case ControlMode.MANUAL:
            return source in (CommandSource.MANUAL, CommandSource.DEBUG)
        case ControlMode.STOP | ControlMode.FAULT | ControlMode.UNKNOWN:
            return False
        case unreachable:
            assert_never(unreachable)


def status(
    snapshot: GuardSnapshot,
    fault_code: SafetyFaultCode,
    fault_message: str,
) -> SafetyStatus:
    mode = ControlMode.UNKNOWN if snapshot.usv_status is None else snapshot.usv_status.status.mode
    return SafetyStatus(
        mode=mode,
        fault=fault_code is not SafetyFaultCode.NONE,
        fault_code=int(fault_code),
        fault_message=fault_message,
    )


def _command_changed(sample: CommandSample, command: GuardedCommand) -> bool:
    return (
        sample.left_target_rpm != command.left_target_rpm
        or sample.right_target_rpm != command.right_target_rpm
    )


def _is_stale(received_at_seconds: float, now_seconds: float, timeout_seconds: float) -> bool:
    # Strictly greater keeps boundary-age samples valid for one final tick.
    # 严格大于使到达边界时间的样本在最后一个 tick 仍有效。
    return now_seconds - received_at_seconds > timeout_seconds


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled variant: {value!r}")
