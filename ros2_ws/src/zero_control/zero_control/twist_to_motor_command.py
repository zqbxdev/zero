from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from zero_interfaces.msg import MotorCommand

from .twist_command_watchdog import (
    RosTimeNanoseconds,
    TwistCommandWatchdog,
    TwistCommandWatchdogTuning,
)
from .twist_to_motor_model import BodyTwist, TwistToMotorModel, TwistToMotorTuning


class TwistToMotorCommand(Node):
    def __init__(self) -> None:
        super().__init__("twist_to_motor_command")
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 1.0)
        self.declare_parameter("max_rpm", 300.0)
        self.declare_parameter("track_width_m", 0.407)
        self.declare_parameter("wheel_radius_m", 0.05)
        self.declare_parameter("left_motor_sign", 1)
        self.declare_parameter("right_motor_sign", 1)
        self.declare_parameter("command_timeout_seconds", 0.5)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("output_topic", "/zero/motor_command_raw")

        tuning = TwistToMotorTuning(
            max_linear_mps=self.get_parameter(
                "max_linear_mps",
            ).get_parameter_value().double_value,
            max_angular_radps=self.get_parameter(
                "max_angular_radps",
            ).get_parameter_value().double_value,
            max_rpm=self.get_parameter("max_rpm").get_parameter_value().double_value,
            track_width_m=self.get_parameter(
                "track_width_m",
            ).get_parameter_value().double_value,
            wheel_radius_m=self.get_parameter(
                "wheel_radius_m",
            ).get_parameter_value().double_value,
            left_motor_sign=self.get_parameter(
                "left_motor_sign",
            ).get_parameter_value().integer_value,
            right_motor_sign=self.get_parameter(
                "right_motor_sign",
            ).get_parameter_value().integer_value,
        )
        watchdog_tuning = TwistCommandWatchdogTuning(
            command_timeout_seconds=self.get_parameter(
                "command_timeout_seconds",
            ).get_parameter_value().double_value,
            publish_rate_hz=self.get_parameter(
                "publish_rate_hz",
            ).get_parameter_value().double_value,
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self._model: TwistToMotorModel = TwistToMotorModel(tuning)
        self._watchdog: TwistCommandWatchdog = TwistCommandWatchdog(watchdog_tuning)
        self._motor_command_pub = self.create_publisher(MotorCommand, output_topic, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 10)
        self.create_timer(1.0 / watchdog_tuning.publish_rate_hz, self._publish_tick)

    def _on_twist(self, msg: Twist) -> None:
        accepted = self._watchdog.accept_command(
            BodyTwist(
                linear_x=float(msg.linear.x),
                angular_z=float(msg.angular.z),
            ),
            received_at_nanoseconds=RosTimeNanoseconds(
                self.get_clock().now().nanoseconds,
            ),
        )
        if not accepted:
            self.get_logger().warning("Rejected invalid or time-regressed /cmd_vel")

    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        command_twist = self._watchdog.command_at(
            RosTimeNanoseconds(now.nanoseconds),
        )
        targets = self._model.convert(command_twist.linear_x, command_twist.angular_z)
        command = MotorCommand()
        command.stamp = now.to_msg()
        command.left_target_rpm = targets.left_target_rpm
        command.right_target_rpm = targets.right_target_rpm
        self._motor_command_pub.publish(command)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TwistToMotorCommand()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
