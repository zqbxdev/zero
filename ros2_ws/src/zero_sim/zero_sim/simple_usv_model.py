from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin


@dataclass(frozen=True, slots=True)
class SimpleUsvTuning:
    rpm_to_linear_velocity: float = 0.001
    rpm_difference_to_angular_velocity: float = 0.002


@dataclass(frozen=True, slots=True)
class SimpleUsvPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True, slots=True)
class SimpleUsvTwist:
    linear_velocity: float
    angular_velocity: float


@dataclass(frozen=True, slots=True)
class SimpleUsvState:
    pose: SimpleUsvPose
    twist: SimpleUsvTwist

# Integrate a simple 2D USV pose from actual motor RPM.
# 使用实际电机转速积分更新简易二维无人船位姿。
@dataclass(slots=True)  # noqa: MUTABLE_OK
class SimpleUsvModel:

    pose: SimpleUsvPose = SimpleUsvPose()
    tuning: SimpleUsvTuning = SimpleUsvTuning()

    def integrate(
        self,
        left_actual_rpm: float,
        right_actual_rpm: float,
        dt_seconds: float,
    ) -> SimpleUsvState:
        linear_velocity = (
            (left_actual_rpm + right_actual_rpm)
            * 0.5
            * self.tuning.rpm_to_linear_velocity
        )
        angular_velocity = (
            (right_actual_rpm - left_actual_rpm)
            * self.tuning.rpm_difference_to_angular_velocity
        )
        next_pose = SimpleUsvPose(
            x=self.pose.x + linear_velocity * cos(self.pose.yaw) * dt_seconds,
            y=self.pose.y + linear_velocity * sin(self.pose.yaw) * dt_seconds,
            yaw=self.pose.yaw + angular_velocity * dt_seconds,
        )
        self.pose = next_pose
        return SimpleUsvState(
            pose=next_pose,
            twist=SimpleUsvTwist(
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
            ),
        )
