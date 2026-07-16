from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import NoReturn

from .command_guard_policy import (
    CommandSample,
    GuardSnapshot,
    MotorStateSample,
    UsvStatusSample,
    authoritative_status_blocker,
    blocking_status,
    guarded_decision,
)
from .command_guard_types import (
    CommandSource,
    ControlMode,
    EStopInputState,
    GuardDecision,
    GuardedCommand,
    InvalidSafetyTuningError,
    MotorStateInput,
    ReleaseResult,
    SafetyFaultCode,
    SafetyStatus,
    SafetyTuning,
    UsvStatusInput,
    parse_command_source,
    parse_control_mode,
)

__all__ = (
    "CommandGuardModel",
    "CommandSource",
    "ControlMode",
    "GuardDecision",
    "GuardedCommand",
    "InvalidSafetyTuningError",
    "MotorStateInput",
    "ReleaseResult",
    "SafetyFaultCode",
    "SafetyStatus",
    "SafetyTuning",
    "UsvStatusInput",
    "parse_command_source",
    "parse_control_mode",
)


@dataclass(slots=True)  # noqa: MUTABLE_OK
class CommandGuardModel:
    """Store authoritative callback samples and the persistent e-stop latch.
    存储权威回调样本和持久急停锁存。
    """

    tuning: SafetyTuning = SafetyTuning()
    _e_stop_input: EStopInputState = field(default=EStopInputState.UNKNOWN, init=False)
    _e_stop_latched: bool = field(default=False, init=False)
    _usv_status: UsvStatusSample | None = field(default=None, init=False)
    _motor_state: MotorStateSample | None = field(default=None, init=False)
    _last_command: CommandSample | None = field(default=None, init=False)
    _usv_clock_valid: bool = field(default=False, init=False)
    _motor_clock_valid: bool = field(default=False, init=False)
    _command_clock_valid: bool = field(default=False, init=False)
    _last_clock_seconds: float | None = field(default=None, init=False)

    def set_e_stop(self, active: bool) -> None:
        # Active input sets a persistent latch; only release() clears the latch.
        # 急停输入有效会设置持久锁存；只有 release() 会清除该锁存。
        self._e_stop_input = EStopInputState.ACTIVE if active else EStopInputState.INACTIVE
        if active:
            self._e_stop_latched = True

    def accept_usv_status(
        self,
        status: UsvStatusInput,
        received_at_seconds: float,
    ) -> bool:
        if not self._prepare_receive(received_at_seconds):
            self._usv_status = None
            self._usv_clock_valid = False
            return False
        self._usv_status = UsvStatusSample(status, received_at_seconds)
        self._usv_clock_valid = True
        return True

    def accept_motor_state(
        self,
        state: MotorStateInput,
        received_at_seconds: float,
    ) -> bool:
        if not self._prepare_receive(received_at_seconds):
            self._motor_state = None
            self._motor_clock_valid = False
            return False
        self._motor_state = MotorStateSample(state, received_at_seconds)
        self._motor_clock_valid = True
        return True

    def accept_command(
        self,
        left_target_rpm: float,
        right_target_rpm: float,
        now_seconds: float,
    ) -> bool:
        if not self._prepare_receive(now_seconds):
            self._last_command = None
            self._command_clock_valid = False
            return False
        valid = isfinite(left_target_rpm) and isfinite(right_target_rpm)
        self._last_command = CommandSample(
            left_target_rpm=left_target_rpm,
            right_target_rpm=right_target_rpm,
            received_at_seconds=now_seconds,
            valid=valid,
        )
        self._command_clock_valid = True
        return True

    def decide(self, now_seconds: float) -> GuardDecision:
        self._observe_clock(now_seconds)
        snapshot = self._snapshot()
        blocker = blocking_status(snapshot, self.tuning, now_seconds)
        if blocker is not None:
            return GuardDecision(command=_zero_command(), status=blocker)
        return guarded_decision(snapshot, self.tuning)

    def release(self, now_seconds: float) -> ReleaseResult:
        """Try to clear the local e-stop latch after authoritative safety checks.
        在权威安全检查通过后尝试清除本地急停锁存。
        """

        self._observe_clock(now_seconds)
        match self._e_stop_input:
            case EStopInputState.UNKNOWN:
                return ReleaseResult(False, "Cannot release while e-stop state is unknown")
            case EStopInputState.ACTIVE:
                return ReleaseResult(False, "Cannot release while e-stop input is active")
            case EStopInputState.INACTIVE:
                pass
            case unreachable:
                assert_never(unreachable)

        snapshot = self._snapshot()
        blocker = authoritative_status_blocker(snapshot, self.tuning, now_seconds)
        if blocker is not None:
            return ReleaseResult(False, f"Cannot release: {blocker.fault_message}")

        sample = self._usv_status
        assert sample is not None
        match sample.status.mode:
            case ControlMode.AUTO:
                pass
            case ControlMode.STOP | ControlMode.MANUAL:
                return ReleaseResult(False, "Cannot release unless control mode is AUTO")
            case ControlMode.UNKNOWN | ControlMode.FAULT:
                return ReleaseResult(False, "Cannot release while control mode is unsafe")
            case unreachable:
                assert_never(unreachable)

        self._e_stop_latched = False
        # Drop any pre-release command so output waits for a fresh guarded command.
        # 丢弃释放前命令，使输出等待新的安全后命令。
        self._last_command = None
        self._command_clock_valid = False
        return ReleaseResult(True, "E-stop latch released")

    def _snapshot(self) -> GuardSnapshot:
        return GuardSnapshot(
            e_stop_input=self._e_stop_input,
            e_stop_latched=self._e_stop_latched,
            usv_status=self._usv_status,
            motor_state=self._motor_state,
            command=self._last_command,
            usv_clock_valid=self._usv_clock_valid,
            motor_clock_valid=self._motor_clock_valid,
            command_clock_valid=self._command_clock_valid,
        )

    def _prepare_receive(self, received_at_seconds: float) -> bool:
        if not isfinite(received_at_seconds):
            return False
        self._observe_clock(received_at_seconds)
        return True

    def _observe_clock(self, now_seconds: float) -> None:
        # Invalidate samples when local time is invalid or moves backwards.
        # 本地时间无效或回退时使样本失效。
        if not isfinite(now_seconds):
            self._invalidate_samples()
            return
        if self._last_clock_seconds is not None and now_seconds < self._last_clock_seconds:
            self._invalidate_samples()
        self._invalidate_future_samples(now_seconds)
        self._last_clock_seconds = now_seconds

    def _invalidate_future_samples(self, now_seconds: float) -> None:
        # Keep sample payloads but mark their clocks invalid until fresh data arrives.
        # 保留样本内容，但标记其时钟无效，直到收到新数据。
        if self._usv_status is not None and now_seconds < self._usv_status.received_at_seconds:
            self._usv_clock_valid = False
        if self._motor_state is not None and now_seconds < self._motor_state.received_at_seconds:
            self._motor_clock_valid = False
        if self._last_command is not None and now_seconds < self._last_command.received_at_seconds:
            self._command_clock_valid = False

    def _invalidate_samples(self) -> None:
        self._usv_clock_valid = False
        self._motor_clock_valid = False
        self._command_clock_valid = False


def _zero_command() -> GuardedCommand:
    return GuardedCommand(left_target_rpm=0.0, right_target_rpm=0.0)


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled variant: {value!r}")
