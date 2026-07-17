#!/usr/bin/env python3
from __future__ import annotations

from typing import Final

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from zero_control import RosTimeNanoseconds, TwistToMotorTuning
from zero_interfaces.msg import MotorState

from .gazebo_drive_adapter_model import (
    GazeboDriveAdapterModel,
    GazeboDriveAdapterTuning,
    MotorStateInput,
)


PUBLISH_RATE_HZ: Final = 20.0


class GazeboDriveAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_drive_adapter")
        # Match the Gazebo DiffDrive envelope to zero_control motor kinematics.
        # 将 Gazebo DiffDrive 运动范围与 zero_control 电机运动学保持一致。
        kinematics = TwistToMotorTuning(
            max_linear_mps=float(self.declare_parameter("max_linear_mps", 1.0).value),
            max_angular_radps=float(
                self.declare_parameter("max_angular_radps", 1.0).value,
            ),
            max_rpm=float(self.declare_parameter("max_rpm", 300.0).value),
            track_width_m=float(self.declare_parameter("track_width_m", 0.407).value),
            wheel_radius_m=float(self.declare_parameter("wheel_radius_m", 0.05).value),
            left_motor_sign=int(self.declare_parameter("left_motor_sign", 1).value),
            right_motor_sign=int(self.declare_parameter("right_motor_sign", 1).value),
        )
        # Pure adapter model converts safe motor feedback into Gazebo body twist.
        # 纯适配模型把安全电机反馈转换为 Gazebo 船体速度。
        self._model = GazeboDriveAdapterModel(
            GazeboDriveAdapterTuning(
                kinematics=kinematics,
                motor_state_timeout_seconds=float(
                    self.declare_parameter("motor_state_timeout_seconds", 0.5).value,
                ),
            ),
        )
        # ROS-side command bridged to Gazebo /model/zero_usv/cmd_vel.
        # ROS 侧命令会桥接到 Gazebo /model/zero_usv/cmd_vel。
        self._twist_publisher = self.create_publisher(
            Twist,
            "/zero/gazebo/cmd_vel",
            10,
        )
        # Consume protected motor feedback after zero_safety has applied gating.
        # 消费 zero_safety 安全门处理后的电机反馈。
        self.create_subscription(
            MotorState,
            "/zero/motor_state",
            self._on_motor_state,
            10,
        )
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_tick)

    def _on_motor_state(self, msg: MotorState) -> None:
        state = MotorStateInput(
            stamp_nanoseconds=RosTimeNanoseconds(
                int(msg.stamp.sec) * 1_000_000_000 + int(msg.stamp.nanosec),
            ),
            left_target_rpm=float(msg.left_target_rpm),
            right_target_rpm=float(msg.right_target_rpm),
            left_actual_rpm=float(msg.left_actual_rpm),
            right_actual_rpm=float(msg.right_actual_rpm),
            left_encoder_count=int(msg.left_encoder_count),
            right_encoder_count=int(msg.right_encoder_count),
            left_pwm_duty=float(msg.left_pwm_duty),
            right_pwm_duty=float(msg.right_pwm_duty),
            left_enabled=bool(msg.left_enabled),
            right_enabled=bool(msg.right_enabled),
            fault=bool(msg.fault),
            fault_message=str(msg.fault_message),
        )
        _ = self._model.accept_motor_state(
            state,
            received_at_nanoseconds=self._now_nanoseconds(),
        )

    def _publish_tick(self) -> None:
        # Publish zero twist when feedback is invalid, faulty, stale, or time-regressed.
        # 反馈无效、故障、超时或时间回退时发布零速度。
        body_twist = self._model.twist_at(now_nanoseconds=self._now_nanoseconds())
        message = Twist()
        message.linear.x = body_twist.linear_x
        message.angular.z = body_twist.angular_z
        self._twist_publisher.publish(message)

    def _now_nanoseconds(self) -> RosTimeNanoseconds:
        return RosTimeNanoseconds(self.get_clock().now().nanoseconds)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GazeboDriveAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
