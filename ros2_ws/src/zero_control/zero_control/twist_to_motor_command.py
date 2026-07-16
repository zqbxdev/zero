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
        # Maximum commanded forward body velocity [m/s].
        # 最大前向船体速度命令 [m/s]。
        self.declare_parameter("max_linear_mps", 1.0)
        # Maximum commanded yaw rate [rad/s].
        # 最大艏向角速度命令 [rad/s]。
        self.declare_parameter("max_angular_radps", 1.0)
        # Maximum absolute motor target speed [rpm].
        # 最大电机目标转速绝对值 [rpm]。
        self.declare_parameter("max_rpm", 300.0)
        # Distance between left and right propulsion tracks [m].
        # 左右推进轨迹间距 [m]。
        self.declare_parameter("track_width_m", 0.407)
        # Effective propulsion wheel radius [m].
        # 等效推进轮半径 [m]。
        self.declare_parameter("wheel_radius_m", 0.05)
        # Left motor polarity; must be the exact integer -1 or 1.
        # 左电机极性；必须是精确整数 -1 或 1。
        self.declare_parameter("left_motor_sign", 1)
        # Right motor polarity; must be the exact integer -1 or 1.
        # 右电机极性；必须是精确整数 -1 或 1。
        self.declare_parameter("right_motor_sign", 1)
        # /cmd_vel freshness timeout [s].
        # /cmd_vel 新鲜度超时时间 [s]。
        self.declare_parameter("command_timeout_seconds", 0.5)
        # Raw motor command publish rate [Hz].
        # 原始电机命令发布频率 [Hz]。
        self.declare_parameter("publish_rate_hz", 20.0)
        # Raw motor command output topic before the safety guard.
        # 进入安全门前的原始电机命令输出 topic。
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
        # Publish raw targets; integrated flows must pass through zero_safety next.
        # 发布原始目标；集成流程必须继续经过 zero_safety。
        self._motor_command_pub = self.create_publisher(MotorCommand, output_topic, 10)
        # Cache /cmd_vel input in the watchdog instead of publishing directly.
        # 将 /cmd_vel 输入缓存到看门狗，而不是直接发布。
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 10)
        # Periodic output lets the watchdog emit zero when commands go stale.
        # 周期性输出使看门狗能在命令过期时发布零命令。
        self.create_timer(1.0 / watchdog_tuning.publish_rate_hz, self._publish_tick)

    def _on_twist(self, msg: Twist) -> None:
        accepted = self._watchdog.accept_command(
            BodyTwist(
                linear_x=float(msg.linear.x),
                angular_z=float(msg.angular.z),
            ),
            # Preserve exact integer ROS time for watchdog comparisons [ns].
            # 保留精确整数 ROS 时间供看门狗比较 [ns]。
            received_at_nanoseconds=RosTimeNanoseconds(
                self.get_clock().now().nanoseconds,
            ),
        )
        if not accepted:
            self.get_logger().warning("Rejected invalid or time-regressed /cmd_vel")

    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        command_twist = self._watchdog.command_at(
            # Preserve exact integer ROS time for timeout checks [ns].
            # 保留精确整数 ROS 时间用于超时检查 [ns]。
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
