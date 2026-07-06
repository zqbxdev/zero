from __future__ import annotations

import rclpy
from rclpy.node import Node

from zero_hardware.fake_motor_model import ControlMode, FakeMotorModel, MotorFeedback
from zero_interfaces.msg import BatteryState, MotorCommand, MotorState, UsvStatus
from zero_interfaces.srv import SetControlMode


class FakeMotorController(Node):
    def __init__(self) -> None:
        super().__init__("fake_motor_controller")
        self._model = FakeMotorModel()
        self._period_seconds = 0.2
        # Publish simulated hardware feedback on the canonical /zero topics.
        # 在约定的 /zero topic 上发布模拟硬件反馈。
        self._motor_state_pub = self.create_publisher(MotorState, "/zero/motor_state", 10)
        self._battery_state_pub = self.create_publisher(BatteryState, "/zero/battery_state", 10)
        self._status_pub = self.create_publisher(UsvStatus, "/zero/status", 10)
        # Accept motor commands and mode changes through the shared interface contract.
        # 通过共享接口契约接收电机命令和模式切换。
        self.create_subscription(MotorCommand, "/zero/motor_command", self._on_motor_command, 10)
        self.create_service(SetControlMode, "/zero/set_control_mode", self._on_set_control_mode)
        # Advance the fake hardware loop at a human-observable period.
        # 用便于人工观察的周期推进 fake hardware 闭环。
        self.create_timer(self._period_seconds, self._publish_tick)

    def _on_motor_command(self, msg: MotorCommand) -> None:
        self._model.command(float(msg.left_target_rpm), float(msg.right_target_rpm))

    def _on_set_control_mode(
        self,
        request: SetControlMode.Request,
        response: SetControlMode.Response,
    ) -> SetControlMode.Response:
        result = self._model.set_mode_value(int(request.mode))
        response.accepted = result.accepted
        response.current_mode = result.current_mode
        response.message = result.message
        return response

    def _publish_tick(self) -> None:
        feedback = self._model.update(self._period_seconds)
        self._motor_state_pub.publish(self._motor_state_msg(feedback))
        self._battery_state_pub.publish(self._battery_state_msg())
        self._status_pub.publish(self._status_msg())

    def _motor_state_msg(self, feedback: MotorFeedback) -> MotorState:
        msg = MotorState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.left_target_rpm = feedback.left_target_rpm
        msg.right_target_rpm = feedback.right_target_rpm
        msg.left_actual_rpm = feedback.left_actual_rpm
        msg.right_actual_rpm = feedback.right_actual_rpm
        msg.left_encoder_count = feedback.left_encoder_count
        msg.right_encoder_count = feedback.right_encoder_count
        msg.left_pwm_duty = feedback.left_pwm_duty
        msg.right_pwm_duty = feedback.right_pwm_duty
        msg.left_enabled = feedback.left_enabled
        msg.right_enabled = feedback.right_enabled
        msg.fault = False
        msg.fault_message = ""
        return msg

    def _battery_state_msg(self) -> BatteryState:
        msg = BatteryState()
        msg.stamp = self.get_clock().now().to_msg()
        msg.voltage_v = 24.0
        msg.current_a = 1.2 if self._model.mode is not ControlMode.STOP else 0.1
        msg.state_of_charge_percent = 95.0
        msg.low_voltage = False
        msg.fault = False
        msg.fault_message = ""
        return msg

    def _status_msg(self) -> UsvStatus:
        msg = UsvStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = int(self._model.mode)
        msg.fault = False
        msg.fault_code = 0
        msg.fault_message = ""
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FakeMotorController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
