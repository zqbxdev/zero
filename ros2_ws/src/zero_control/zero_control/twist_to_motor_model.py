from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


class InvalidTuningError(ValueError):
    def __init__(self, field_name: str) -> None:
        self.field_name: str = field_name
        super().__init__(f"{field_name} must be positive and finite")


@dataclass(frozen=True, slots=True)
class TwistToMotorTuning:
    max_linear_mps: float = 1.0
    max_angular_radps: float = 1.0
    max_rpm: float = 1000.0
    track_width_m: float = 1.0

    def __post_init__(self) -> None:
        _require_positive_finite("max_linear_mps", self.max_linear_mps)
        _require_positive_finite("max_angular_radps", self.max_angular_radps)
        _require_positive_finite("max_rpm", self.max_rpm)
        _require_positive_finite("track_width_m", self.track_width_m)


@dataclass(frozen=True, slots=True)
class MotorTargets:
    left_target_rpm: float
    right_target_rpm: float


class TwistToMotorModel:
    def __init__(self, tuning: TwistToMotorTuning | None = None) -> None:
        self._tuning: TwistToMotorTuning = tuning or TwistToMotorTuning()

    def convert(self, linear_x: float, angular_z: float) -> MotorTargets:
        if not isfinite(linear_x) or not isfinite(angular_z):
            return MotorTargets(left_target_rpm=0.0, right_target_rpm=0.0)

        linear_cmd = _clamp(linear_x, -self._tuning.max_linear_mps, self._tuning.max_linear_mps)
        angular_cmd = _clamp(
            angular_z,
            -self._tuning.max_angular_radps,
            self._tuning.max_angular_radps,
        )
        half_width = self._tuning.track_width_m * 0.5
        left_velocity = linear_cmd - angular_cmd * half_width
        right_velocity = linear_cmd + angular_cmd * half_width
        return MotorTargets(
            left_target_rpm=_velocity_to_rpm(left_velocity, self._tuning),
            right_target_rpm=_velocity_to_rpm(right_velocity, self._tuning),
        )


def _velocity_to_rpm(velocity_mps: float, tuning: TwistToMotorTuning) -> float:
    rpm = velocity_mps / tuning.max_linear_mps * tuning.max_rpm
    return _clamp(rpm, -tuning.max_rpm, tuning.max_rpm)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def _require_positive_finite(field_name: str, value: float) -> None:
    if value <= 0.0 or not isfinite(value):
        raise InvalidTuningError(field_name)
