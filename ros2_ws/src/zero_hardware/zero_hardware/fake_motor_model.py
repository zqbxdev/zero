from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Final


class ControlMode(IntEnum):
    STOP = 1
    MANUAL = 2
    AUTO = 3


ACCEPTED_CONTROL_MODES: Final[frozenset[int]] = frozenset(
    mode.value for mode in ControlMode
)


@dataclass(frozen=True, slots=True)
class ModeChangeResult:
    accepted: bool
    current_mode: int
    message: str


@dataclass(frozen=True, slots=True)
class MotorFeedback:
    left_target_rpm: float
    right_target_rpm: float
    left_actual_rpm: float
    right_actual_rpm: float
    left_encoder_count: int
    right_encoder_count: int
    left_pwm_duty: float
    right_pwm_duty: float
    left_enabled: bool
    right_enabled: bool


@dataclass(slots=True)  # noqa: MUTABLE_OK
class FakeMotorModel:
    """可变 fake 电机状态机。 / Mutable fake motor state machine.

    每个 timer tick 都会推进模拟硬件状态，因此需要原地更新。
    Each timer tick advances simulated hardware state, so in-place updates are required.
    """

    mode: ControlMode = ControlMode.STOP
    left_target_rpm: float = 0.0
    right_target_rpm: float = 0.0
    left_actual_rpm: float = 0.0
    right_actual_rpm: float = 0.0
    left_encoder_count: int = 0
    right_encoder_count: int = 0
    max_rpm_step: float = 10.0
    max_target_step: float = 20.0
    max_command_rpm: float = 300.0
    encoder_ticks_per_rev: float = 2048.0

    def set_mode(self, mode: ControlMode) -> ModeChangeResult:
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

    def command(self, left_target_rpm: float, right_target_rpm: float) -> None:
        if self.mode is ControlMode.STOP:
            return
        self.left_target_rpm = self._clamp(left_target_rpm, self.max_command_rpm)
        self.right_target_rpm = self._clamp(right_target_rpm, self.max_command_rpm)

    def update(self, dt_seconds: float) -> MotorFeedback:
        if self.mode is ControlMode.STOP:
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
        self.left_encoder_count += self._encoder_delta(self.left_actual_rpm, dt_seconds)
        self.right_encoder_count += self._encoder_delta(self.right_actual_rpm, dt_seconds)
        return self.feedback()

    def feedback(self) -> MotorFeedback:
        enabled = self.mode is not ControlMode.STOP
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
        )

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
