from __future__ import annotations

from math import isfinite

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from zero_interfaces.msg import MotorCommand

from .twist_to_motor_model import TwistToMotorModel, TwistToMotorTuning


class TwistToMotorCommand(Node):
    def __init__(self) -> None:
        super().__init__("twist_to_motor_command")
        self.declare_parameter("max_linear_mps", 1.0)
        self.declare_parameter("max_angular_radps", 1.0)
        self.declare_parameter("max_rpm", 1000.0)
        self.declare_parameter("track_width_m", 1.0)
        self.declare_parameter("output_topic", "/zero/motor_command_raw")

        tuning = TwistToMotorTuning(
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_radps=float(self.get_parameter("max_angular_radps").value),
            max_rpm=float(self.get_parameter("max_rpm").value),
            track_width_m=float(self.get_parameter("track_width_m").value),
        )
        output_topic = str(self.get_parameter("output_topic").value)
        self._model: TwistToMotorModel = TwistToMotorModel(tuning)
        self._motor_command_pub = self.create_publisher(MotorCommand, output_topic, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 10)

    def _on_twist(self, msg: Twist) -> None:
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if not isfinite(linear_x) or not isfinite(angular_z):
            self.get_logger().warning("Rejected non-finite /cmd_vel; publishing zero motor command")
        targets = self._model.convert(linear_x, angular_z)
        command = MotorCommand()
        command.stamp = self.get_clock().now().to_msg()
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
