from __future__ import annotations

from dataclasses import dataclass
from math import isfinite, pi


class InvalidTuningError(ValueError):
    def __init__(self, field_name: str, detail: str = "must be positive and finite") -> None:
        self.field_name: str = field_name
        self.detail: str = detail
        super().__init__(f"{field_name} {detail}")


@dataclass(frozen=True, slots=True)
class TwistToMotorTuning:
    """V1 simulation tuning for command limits and hull geometry.
    V1 仿真调参项，用于命令限幅和船体几何参数。
    """

    max_linear_mps: float = 1.0
    max_angular_radps: float = 1.0
    max_rpm: float = 300.0
    track_width_m: float = 0.407
    wheel_radius_m: float = 0.05
    left_motor_sign: int = 1
    right_motor_sign: int = 1

    def __post_init__(self) -> None:
        _require_positive_finite("max_linear_mps", self.max_linear_mps)
        _require_positive_finite("max_angular_radps", self.max_angular_radps)
        _require_positive_finite("max_rpm", self.max_rpm)
        _require_positive_finite("track_width_m", self.track_width_m)
        _require_positive_finite("wheel_radius_m", self.wheel_radius_m)
        _require_motor_sign("left_motor_sign", self.left_motor_sign)
        _require_motor_sign("right_motor_sign", self.right_motor_sign)


@dataclass(frozen=True, slots=True)
class MotorTargets:
    left_target_rpm: float
    right_target_rpm: float


@dataclass(frozen=True, slots=True)
class BodyTwist:
    linear_x: float
    angular_z: float


class TwistToMotorModel:
    def __init__(self, tuning: TwistToMotorTuning | None = None) -> None:
        self._tuning: TwistToMotorTuning = tuning or TwistToMotorTuning()

    def convert(self, linear_x: float, angular_z: float) -> MotorTargets:
        return twist_to_motor_targets(linear_x, angular_z, self._tuning)


def twist_to_motor_targets(
    linear_x: float,
    angular_z: float,
    tuning: TwistToMotorTuning | None = None,
) -> MotorTargets:
    selected_tuning = tuning or TwistToMotorTuning()
    if not isfinite(linear_x) or not isfinite(angular_z):
        return MotorTargets(left_target_rpm=0.0, right_target_rpm=0.0)

    linear_cmd = _clamp(linear_x, -selected_tuning.max_linear_mps, selected_tuning.max_linear_mps)
    angular_cmd = _clamp(
        angular_z,
        -selected_tuning.max_angular_radps,
        selected_tuning.max_angular_radps,
    )
    half_width = selected_tuning.track_width_m * 0.5
    left_velocity = linear_cmd - angular_cmd * half_width
    right_velocity = linear_cmd + angular_cmd * half_width
    left_logical_rpm = _clamp_rpm(
        _velocity_to_rpm(left_velocity, selected_tuning),
        selected_tuning,
    )
    right_logical_rpm = _clamp_rpm(
        _velocity_to_rpm(right_velocity, selected_tuning),
        selected_tuning,
    )
    return MotorTargets(
        left_target_rpm=_apply_motor_sign(left_logical_rpm, selected_tuning.left_motor_sign),
        right_target_rpm=_apply_motor_sign(right_logical_rpm, selected_tuning.right_motor_sign),
    )


def motor_rpm_to_twist(
    left_target_rpm: float,
    right_target_rpm: float,
    tuning: TwistToMotorTuning | None = None,
) -> BodyTwist:
    """Convert finite measured motor speed to body twist without target-limit clamping.
    将有限实测电机转速转换为船体速度，不做目标限幅。
    """

    selected_tuning = tuning or TwistToMotorTuning()
    if not isfinite(left_target_rpm) or not isfinite(right_target_rpm):
        return BodyTwist(linear_x=0.0, angular_z=0.0)

    left_logical_rpm = selected_tuning.left_motor_sign * left_target_rpm

    right_logical_rpm = selected_tuning.right_motor_sign * right_target_rpm
    left_velocity = _rpm_to_velocity(left_logical_rpm, selected_tuning)
    right_velocity = _rpm_to_velocity(right_logical_rpm, selected_tuning)
    return BodyTwist(
        linear_x=(left_velocity + right_velocity) * 0.5,
        angular_z=(right_velocity - left_velocity) / selected_tuning.track_width_m,
    )


def _velocity_to_rpm(velocity_mps: float, tuning: TwistToMotorTuning) -> float:
    return velocity_mps / (2.0 * pi * tuning.wheel_radius_m) * 60.0


def _rpm_to_velocity(rpm: float, tuning: TwistToMotorTuning) -> float:
    return rpm / 60.0 * (2.0 * pi * tuning.wheel_radius_m)


def _clamp_rpm(rpm: float, tuning: TwistToMotorTuning) -> float:
    return _clamp(rpm, -tuning.max_rpm, tuning.max_rpm)


def _apply_motor_sign(logical_rpm: float, motor_sign: int) -> float:
    if logical_rpm == 0.0:
        return 0.0
    return motor_sign * logical_rpm


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return min(max(value, minimum), maximum)


def _require_positive_finite(field_name: str, value: float) -> None:
    if type(value) is bool or value <= 0.0 or not isfinite(value):
        raise InvalidTuningError(field_name)


def _require_motor_sign(field_name: str, value: int) -> None:
    if type(value) is not int or value not in (-1, 1):
        raise InvalidTuningError(field_name, "must be the exact integer -1 or 1")
