from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from math import isfinite
from typing import Final, NoReturn


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled control mode: {value!r}")


class ControlMode(IntEnum):
    STOP = 1
    MANUAL = 2
    AUTO = 3


ACCEPTED_CONTROL_MODES: Final[frozenset[int]] = frozenset(
    mode.value for mode in ControlMode
)


@dataclass(frozen=True, slots=True)
class ModeChangeResult:
    # True when the requested control mode was accepted.
    # 请求的控制模式被接受时为 true。
    accepted: bool
    # Current control mode after request handling.
    # 请求处理后的当前控制模式。
    current_mode: int
    # Human-readable mode-change result or rejection reason.
    # 可读模式切换结果或拒绝原因。
    message: str


@dataclass(frozen=True, slots=True)
class MotorFeedback:
    # Left motor target output-shaft speed [rpm].
    # 左电机目标输出轴转速 [rpm]。
    left_target_rpm: float
    # Right motor target output-shaft speed [rpm].
    # 右电机目标输出轴转速 [rpm]。
    right_target_rpm: float
    # Left motor simulated output-shaft speed [rpm].
    # 左电机仿真输出轴转速 [rpm]。
    left_actual_rpm: float
    # Right motor simulated output-shaft speed [rpm].
    # 右电机仿真输出轴转速 [rpm]。
    right_actual_rpm: float
    # Left simulated Hall encoder accumulated count [ticks].
    # 左仿真霍尔编码器累计计数 [ticks]。
    left_encoder_count: int
    # Right simulated Hall encoder accumulated count [ticks].
    # 右仿真霍尔编码器累计计数 [ticks]。
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
    # True when the fake motor model reports a fault.
    # fake 电机模型报告故障时为 true。
    fault: bool
    # Human-readable fault detail; empty when fault is false.
    # 可读故障信息；无故障时为空。
    fault_message: str


@dataclass(slots=True)  # noqa: MUTABLE_OK
class FakeMotorModel:
    """Maintain mutable fake motor state.
    维护可变 fake 电机状态。

    Each timer tick advances simulated hardware state, so in-place updates are required.
    每个 timer tick 都会推进模拟硬件状态，因此需要原地更新。
    """

    # Current fake motor control mode.
    # 当前 fake 电机控制模式。
    mode: ControlMode = ControlMode.STOP
    # Left motor target output-shaft speed [rpm].
    # 左电机目标输出轴转速 [rpm]。
    left_target_rpm: float = 0.0
    # Right motor target output-shaft speed [rpm].
    # 右电机目标输出轴转速 [rpm]。
    right_target_rpm: float = 0.0
    # Left motor simulated output-shaft speed [rpm].
    # 左电机仿真输出轴转速 [rpm]。
    left_actual_rpm: float = 0.0
    # Right motor simulated output-shaft speed [rpm].
    # 右电机仿真输出轴转速 [rpm]。
    right_actual_rpm: float = 0.0
    # Left simulated Hall encoder accumulated count [ticks].
    # 左仿真霍尔编码器累计计数 [ticks]。
    left_encoder_count: int = 0
    # Right simulated Hall encoder accumulated count [ticks].
    # 右仿真霍尔编码器累计计数 [ticks]。
    right_encoder_count: int = 0
    # Maximum actual-speed change per update tick [rpm].
    # 每个 update tick 允许的最大实际转速变化量 [rpm]。
    max_rpm_step: float = 10.0
    # Maximum target-speed decay per STOP update tick [rpm].
    # STOP 模式下每个 update tick 允许的最大目标转速衰减量 [rpm]。
    max_target_step: float = 20.0
    # Maximum absolute accepted command speed [rpm].
    # 可接受命令转速的最大绝对值 [rpm]。
    max_command_rpm: float = 300.0
    # Command watchdog timeout [s].
    # 命令看门狗超时时间 [s]。
    command_timeout_seconds: float = 0.5
    # Simulation-only encoder scale; this is not real hardware calibration.
    # 仅用于仿真的编码器比例；这不是真实硬件标定参数。
    encoder_ticks_per_rev: float = 2048.0
    # True when the fake motor model is faulted.
    # fake 电机模型处于故障状态时为 true。
    fault: bool = field(default=False, init=False)
    # Human-readable fault detail; empty when fault is false.
    # 可读故障信息；无故障时为空。
    fault_message: str = field(default="", init=False)
    # Local timestamp of the last accepted command [s].
    # 最近一次接受命令的本地时间戳 [s]。
    _last_command_at_seconds: float | None = field(default=None, init=False)
    # Local timestamp associated with the active fault [s].
    # 当前故障关联的本地时间戳 [s]。
    _fault_at_seconds: float | None = field(default=None, init=False)
    # Latest local timestamp observed by command or update paths [s].
    # command 或 update 路径观察到的最新本地时间戳 [s]。
    _last_observed_at_seconds: float | None = field(default=None, init=False)

    def set_mode(self, mode: ControlMode) -> ModeChangeResult:
        previous_mode = self.mode
        match mode:
            case ControlMode.STOP:
                self._last_command_at_seconds = None
            case ControlMode.MANUAL | ControlMode.AUTO:
                if previous_mode is ControlMode.STOP:
                    self.left_target_rpm = 0.0
                    self.right_target_rpm = 0.0
            case unreachable:
                assert_never(unreachable)
        self.mode = mode
        return ModeChangeResult(
            accepted=True,
            current_mode=int(self.mode),
            message=f"Control mode set to {self.mode.name}",
        )

    def set_mode_value(self, mode_value: int) -> ModeChangeResult:
        if mode_value not in ACCEPTED_CONTROL_MODES:
            return ModeChangeResult(
                accepted=False,
                current_mode=int(self.mode),
                message=f"Unsupported control mode: {mode_value}",
            )
        return self.set_mode(ControlMode(mode_value))

    def command(
        self,
        left_target_rpm: float,
        right_target_rpm: float,
        received_at_seconds: float,
    ) -> None:
        if not isfinite(received_at_seconds):
            self._enter_fault(
                "Motor command timestamp is non-finite",
                self._fault_reference_time(),
            )
            return
        command_is_finite = isfinite(left_target_rpm) and isfinite(right_target_rpm)
        if self.mode is ControlMode.STOP:
            if not command_is_finite:
                self._enter_fault("Motor command contains non-finite rpm", received_at_seconds)
            # STOP ignores finite commands so stale input cannot override a stopped state.
            # STOP 会忽略有限命令，避免旧输入覆盖已停止状态。
            return
        if (
            self.fault
            and self._fault_at_seconds is not None
            and received_at_seconds <= self._fault_at_seconds
        ):
            return
        if (
            self._last_observed_at_seconds is not None
            and received_at_seconds < self._last_observed_at_seconds
        ):
            self._enter_fault("Motor clock moved backwards", self._last_observed_at_seconds)
            return

        self._last_observed_at_seconds = received_at_seconds
        if not command_is_finite:
            self._enter_fault("Motor command contains non-finite rpm", received_at_seconds)
            return

        self._clear_fault()
        self._last_command_at_seconds = received_at_seconds
        self.left_target_rpm = self._clamp(left_target_rpm, self.max_command_rpm)
        self.right_target_rpm = self._clamp(right_target_rpm, self.max_command_rpm)

    def update(self, dt_seconds: float, now_seconds: float) -> MotorFeedback:
        effective_dt_seconds = dt_seconds
        if not isfinite(dt_seconds):
            self._enter_fault("Motor update interval is non-finite", self._fault_reference_time())
            effective_dt_seconds = 0.0
        elif not isfinite(now_seconds):
            self._enter_fault("Motor update time is non-finite", self._fault_reference_time())
        elif (
            self._last_observed_at_seconds is not None
            and now_seconds < self._last_observed_at_seconds
        ):
            self._enter_fault("Motor clock moved backwards", self._last_observed_at_seconds)
        else:
            self._last_observed_at_seconds = now_seconds
            if self._command_timed_out(now_seconds):
                self._enter_fault("Motor command timed out", now_seconds)

        if self.mode is ControlMode.STOP:
            # STOP ramps targets down instead of clearing them instantly.
            # STOP 会让目标转速逐步回零，而不是瞬间清零。
            self.left_target_rpm = self._step_toward(
                self.left_target_rpm,
                0.0,
                self.max_target_step,
            )
            self.right_target_rpm = self._step_toward(
                self.right_target_rpm,
                0.0,
                self.max_target_step,
            )

        self.left_actual_rpm = self._step_toward(
            self.left_actual_rpm,
            self.left_target_rpm,
            self.max_rpm_step,
        )
        self.right_actual_rpm = self._step_toward(
            self.right_actual_rpm,
            self.right_target_rpm,
            self.max_rpm_step,
        )
        # Encoders follow actual RPM so command changes are visible only after ramping.
        # 编码器跟随实际转速，因此命令变化要经过爬坡后才体现在计数中。
        self.left_encoder_count += self._encoder_delta(self.left_actual_rpm, effective_dt_seconds)
        self.right_encoder_count += self._encoder_delta(
            self.right_actual_rpm,
            effective_dt_seconds,
        )
        return self.feedback()

    def feedback(self) -> MotorFeedback:
        enabled = self.mode is not ControlMode.STOP and not self.fault
        return MotorFeedback(
            left_target_rpm=self.left_target_rpm,
            right_target_rpm=self.right_target_rpm,
            left_actual_rpm=self.left_actual_rpm,
            right_actual_rpm=self.right_actual_rpm,
            left_encoder_count=self.left_encoder_count,
            right_encoder_count=self.right_encoder_count,
            left_pwm_duty=self._pwm_duty(self.left_target_rpm) if enabled else 0.0,
            right_pwm_duty=self._pwm_duty(self.right_target_rpm) if enabled else 0.0,
            left_enabled=enabled,
            right_enabled=enabled,
            fault=self.fault,
            fault_message=self.fault_message,
        )

    def _command_timed_out(self, now_seconds: float) -> bool:
        return (
            not self.fault
            and self.mode is not ControlMode.STOP
            and self._last_command_at_seconds is not None
            and now_seconds - self._last_command_at_seconds > self.command_timeout_seconds
        )

    def _enter_fault(self, message: str, fault_at_seconds: float) -> None:
        self.fault = True
        self.fault_message = message
        if self._fault_at_seconds is None or fault_at_seconds > self._fault_at_seconds:
            self._fault_at_seconds = fault_at_seconds
        self.left_target_rpm = 0.0
        self.right_target_rpm = 0.0

    def _clear_fault(self) -> None:
        self.fault = False
        self.fault_message = ""
        self._fault_at_seconds = None

    def _fault_reference_time(self) -> float:
        return self._last_observed_at_seconds or 0.0

    @staticmethod
    def _step_toward(current: float, target: float, limit: float) -> float:
        delta = target - current
        if abs(delta) <= limit:
            return target
        return current + limit if delta > 0.0 else current - limit

    @staticmethod
    def _clamp(value: float, absolute_limit: float) -> float:
        return max(-absolute_limit, min(absolute_limit, value))

    def _pwm_duty(self, target_rpm: float) -> float:
        return self._clamp((target_rpm / self.max_command_rpm) * 100.0, 100.0)

    def _encoder_delta(self, rpm: float, dt_seconds: float) -> int:
        revolutions = rpm * dt_seconds / 60.0
        return round(revolutions * self.encoder_ticks_per_rev)
