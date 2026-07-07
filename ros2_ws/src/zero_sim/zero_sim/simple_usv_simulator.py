from __future__ import annotations

from math import cos, sin

from builtin_interfaces.msg import Time
import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from zero_interfaces.msg import MotorState

from zero_sim.simple_usv_model import SimpleUsvModel, SimpleUsvState, SimpleUsvTuning


class SimpleUsvSimulator(Node):
    def __init__(self) -> None:
        super().__init__("simple_usv_simulator")
        tuning = SimpleUsvTuning(
            rpm_to_linear_velocity=float(
                self.declare_parameter("rpm_to_linear_velocity", 0.001).value,
            ),
            rpm_difference_to_angular_velocity=float(
                self.declare_parameter("rpm_difference_to_angular_velocity", 0.002).value,
            ),
        )
        update_rate_hz = float(self.declare_parameter("update_rate_hz", 20.0).value)
        self._odom_frame_id = str(self.declare_parameter("odom_frame_id", "odom").value)
        self._base_frame_id = str(
            self.declare_parameter("base_frame_id", "zero_base_link").value,
        )
        self._model = SimpleUsvModel(tuning=tuning)
        self._left_actual_rpm = 0.0
        self._right_actual_rpm = 0.0
        self._period_seconds = 1.0 / update_rate_hz
        self._odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        # Consume actual motor feedback, not direct commands, so fake and real hardware share semantics.
        # 使用实际电机反馈而不是直接命令，使 fake 和真实硬件语义一致。
        self.create_subscription(
            MotorState,
            "/zero/motor_state",
            self._on_motor_state,
            10,
        )
        self.create_timer(self._period_seconds, self._publish_tick)

    def _on_motor_state(self, msg: MotorState) -> None:
        self._left_actual_rpm = float(msg.left_actual_rpm)
        self._right_actual_rpm = float(msg.right_actual_rpm)

    def _publish_tick(self) -> None:
        state = self._model.integrate(
            left_actual_rpm=self._left_actual_rpm,
            right_actual_rpm=self._right_actual_rpm,
            dt_seconds=self._period_seconds,
        )
        stamp = self.get_clock().now().to_msg()
        self._odom_pub.publish(self._odom_msg(state, stamp))
        self._tf_broadcaster.sendTransform(self._transform_msg(state, stamp))

    def _odom_msg(self, state: SimpleUsvState, stamp: Time) -> Odometry:
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x = state.pose.x
        msg.pose.pose.position.y = state.pose.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = _quaternion_from_yaw(state.pose.yaw)
        msg.twist.twist.linear.x = state.twist.linear_velocity
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = state.twist.angular_velocity
        return msg

    def _transform_msg(self, state: SimpleUsvState, stamp: Time) -> TransformStamped:
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.transform.translation.x = state.pose.x
        msg.transform.translation.y = state.pose.y
        msg.transform.translation.z = 0.0
        msg.transform.rotation = _quaternion_from_yaw(state.pose.yaw)
        return msg


def _quaternion_from_yaw(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(half_yaw)
    quaternion.w = cos(half_yaw)
    return quaternion


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimpleUsvSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
