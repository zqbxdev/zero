from __future__ import annotations

from dataclasses import dataclass, field
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


class SafetyFaultCode(IntEnum):
    NONE = 0
    MODE_UNKNOWN = 1
    COMMAND_TIMEOUT = 2
    E_STOP_ACTIVE = 3
    MODE_BLOCKED = 4
    INVALID_COMMAND = 5
    COMMAND_CLAMPED = 6
    E_STOP_UNKNOWN = 7


def _require_positive_finite(field_name: str, value: float) -> None:
    if value <= 0.0 or not isfinite(value):
        raise InvalidSafetyTuningError(field_name)


@dataclass(frozen=True, slots=True)
class SafetyTuning:
    max_command_rpm: float = 300.0
    command_timeout_seconds: float = 0.5
    input_source: CommandSource = CommandSource.AUTO

    def __post_init__(self) -> None:
        _require_positive_finite("max_command_rpm", self.max_command_rpm)
        _require_positive_finite("command_timeout_seconds", self.command_timeout_seconds)


@dataclass(frozen=True, slots=True)
class GuardedCommand:
    left_target_rpm: float
    right_target_rpm: float


@dataclass(frozen=True, slots=True)
class SafetyStatus:
    mode: ControlMode
    fault: bool
    fault_code: int
    fault_message: str


@dataclass(frozen=True, slots=True)
class GuardDecision:
    command: GuardedCommand
    status: SafetyStatus


@dataclass(frozen=True, slots=True)
class ReleaseResult:
    accepted: bool
    message: str


@dataclass(frozen=True, slots=True)
class CommandSample:
    left_target_rpm: float
    right_target_rpm: float
    received_at_seconds: float
    valid: bool


# Store mutable command guard state updated by ROS callbacks.
# 存储由 ROS 回调更新的可变命令安全门状态。
@dataclass(slots=True)  # noqa: MUTABLE_OK
class CommandGuardModel:
    tuning: SafetyTuning = SafetyTuning()
    _mode: ControlMode = field(default=ControlMode.UNKNOWN, init=False)
    _e_stop_known: bool = field(default=False, init=False)
    _e_stop_active: bool = field(default=False, init=False)
    _e_stop_latched: bool = field(default=False, init=False)
    _last_command: CommandSample | None = field(default=None, init=False)

    def set_mode_value(self, mode_value: int) -> None:
        try:
            self._mode = ControlMode(mode_value)
        except ValueError:
            self._mode = ControlMode.UNKNOWN

    def set_e_stop(self, active: bool) -> None:
        self._e_stop_known = True
        self._e_stop_active = active
        if active:
            self._e_stop_latched = True

    def accept_command(
        self,
        left_target_rpm: float,
        right_target_rpm: float,
        now_seconds: float,
    ) -> None:
        valid = isfinite(left_target_rpm) and isfinite(right_target_rpm)
        self._last_command = CommandSample(
            left_target_rpm=left_target_rpm,
            right_target_rpm=right_target_rpm,
            received_at_seconds=now_seconds,
            valid=valid,
        )

    def decide(self, now_seconds: float) -> GuardDecision:
        blocking_status = self._blocking_status(now_seconds)
        if blocking_status is not None:
            return GuardDecision(command=_zero_command(), status=blocking_status)

        sample = self._last_command
        assert sample is not None

        left_target_rpm = _clamp(
            sample.left_target_rpm,
            -self.tuning.max_command_rpm,
            self.tuning.max_command_rpm,
        )
        right_target_rpm = _clamp(
            sample.right_target_rpm,
            -self.tuning.max_command_rpm,
            self.tuning.max_command_rpm,
        )
        command = GuardedCommand(
            left_target_rpm=left_target_rpm,
            right_target_rpm=right_target_rpm,
        )
        if _command_changed(sample, command):
            return GuardDecision(
                command=command,
                status=self._status(
                    SafetyFaultCode.COMMAND_CLAMPED,
                    "Motor command clamped to safety rpm limit",
                ),
            )
        return GuardDecision(command=command, status=self._status(SafetyFaultCode.NONE, ""))

    def release(self, now_seconds: float) -> ReleaseResult:
        if not self._e_stop_known:
            return ReleaseResult(False, "Cannot release while e-stop state is unknown")
        if self._e_stop_active:
            return ReleaseResult(False, "Cannot release while e-stop input is active")

        blocker = self._release_blocker(now_seconds)
        if blocker is not None:
            return ReleaseResult(False, blocker)

        self._e_stop_latched = False
        self._last_command = None
        return ReleaseResult(True, "E-stop latch released")

    def _blocking_status(self, now_seconds: float) -> SafetyStatus | None:
        if not self._e_stop_known:
            return self._status(SafetyFaultCode.E_STOP_UNKNOWN, "E-stop state is unknown")
        if self._e_stop_active or self._e_stop_latched:
            return self._status(SafetyFaultCode.E_STOP_ACTIVE, "E-stop is active or latched")

        sample = self._last_command
        if sample is None:
            return self._status(SafetyFaultCode.COMMAND_TIMEOUT, "No motor command received")
        if not sample.valid:
            return self._status(SafetyFaultCode.INVALID_COMMAND, "Motor command contains non-finite rpm")
        if self._command_is_stale(sample, now_seconds):
            return self._status(SafetyFaultCode.COMMAND_TIMEOUT, "Motor command timed out")
        if self._mode is ControlMode.UNKNOWN:
            return self._status(SafetyFaultCode.MODE_UNKNOWN, "Control mode is unknown")
        if not self._mode_allows_source():
            return self._status(SafetyFaultCode.MODE_BLOCKED, "Control mode blocks command source")
        return None

    def _release_blocker(self, now_seconds: float) -> str | None:
        sample = self._last_command
        if sample is None:
            return "Cannot release without a fresh valid command"
        if not sample.valid:
            return "Cannot release while current command is invalid"
        if self._command_is_stale(sample, now_seconds):
            return "Cannot release while current command is stale"
        if self._mode is ControlMode.UNKNOWN:
            return "Cannot release while control mode is unknown"
        if not self._mode_allows_source():
            return "Cannot release because current mode blocks command source"
        if not _command_is_zero(sample):
            return "Cannot release while current command is non-zero"
        return None

    def _command_is_stale(self, sample: CommandSample, now_seconds: float) -> bool:
        return now_seconds - sample.received_at_seconds > self.tuning.command_timeout_seconds

    def _mode_allows_source(self) -> bool:
        match self._mode:
            case ControlMode.AUTO:
                return self.tuning.input_source is CommandSource.AUTO
            case ControlMode.MANUAL:
                return self.tuning.input_source in (CommandSource.MANUAL, CommandSource.DEBUG)
            case ControlMode.STOP | ControlMode.FAULT | ControlMode.UNKNOWN:
                return False

    def _status(self, fault_code: SafetyFaultCode, fault_message: str) -> SafetyStatus:
        return SafetyStatus(
            mode=self._mode,
            fault=fault_code is not SafetyFaultCode.NONE,
            fault_code=int(fault_code),
            fault_message=fault_message,
        )


def parse_command_source(value: str) -> CommandSource:
    for source in CommandSource:
        if source.value == value:
            return source
    allowed = ", ".join(("auto", "debug", "manual"))
    raise InvalidSafetyTuningError("input_source", f"must be one of: {allowed}")


def _command_changed(sample: CommandSample, command: GuardedCommand) -> bool:
    return (
        sample.left_target_rpm != command.left_target_rpm
        or sample.right_target_rpm != command.right_target_rpm
    )


def _command_is_zero(sample: CommandSample) -> bool:
    return sample.left_target_rpm == 0.0 and sample.right_target_rpm == 0.0


def _zero_command() -> GuardedCommand:
    return GuardedCommand(left_target_rpm=0.0, right_target_rpm=0.0)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)
