from __future__ import annotations

from math import isfinite

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from zero_interfaces.msg import MotorCommand, UsvStatus

from .command_guard_model import (
    CommandGuardModel,
    GuardedCommand,
    InvalidSafetyTuningError,
    SafetyStatus,
    SafetyTuning,
    parse_command_source,
)


class CommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("command_guard")
        tuning = SafetyTuning(
            max_command_rpm=float(self.declare_parameter("max_command_rpm", 300.0).value),
            command_timeout_seconds=float(
                self.declare_parameter("command_timeout_seconds", 0.5).value,
            ),
            input_source=parse_command_source(
                str(self.declare_parameter("input_source", "auto").value),
            ),
        )
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        _require_positive_finite("publish_rate_hz", publish_rate_hz)

        self._model: CommandGuardModel = CommandGuardModel(tuning)
        self._motor_command_pub = self.create_publisher(MotorCommand, "/zero/motor_command", 10)
        self._safety_status_pub = self.create_publisher(UsvStatus, "/zero/safety_status", 10)
        self.create_subscription(MotorCommand, "/zero/motor_command_raw", self._on_raw_command, 10)
        self.create_subscription(UsvStatus, "/zero/status", self._on_status, 10)
        self.create_subscription(Bool, "/zero/e_stop", self._on_e_stop, 10)
        self.create_service(Trigger, "/zero/release_e_stop", self._on_release_e_stop)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)

    def _on_raw_command(self, msg: MotorCommand) -> None:
        self._model.accept_command(
            left_target_rpm=float(msg.left_target_rpm),
            right_target_rpm=float(msg.right_target_rpm),
            now_seconds=self._now_seconds(),
        )

    def _on_status(self, msg: UsvStatus) -> None:
        self._model.set_mode_value(int(msg.mode))

    def _on_e_stop(self, msg: Bool) -> None:
        self._model.set_e_stop(bool(msg.data))

    def _on_release_e_stop(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        result = self._model.release(self._now_seconds())
        response.success = result.accepted
        response.message = result.message
        return response

    def _publish_tick(self) -> None:
        decision = self._model.decide(self._now_seconds())
        self._motor_command_pub.publish(self._command_msg(decision.command))
        self._safety_status_pub.publish(self._status_msg(decision.status))

    def _command_msg(self, command: GuardedCommand) -> MotorCommand:
        msg = MotorCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.left_target_rpm = command.left_target_rpm
        msg.right_target_rpm = command.right_target_rpm
        return msg

    def _status_msg(self, status: SafetyStatus) -> UsvStatus:
        msg = UsvStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.mode = int(status.mode)
        msg.fault = status.fault
        msg.fault_code = status.fault_code
        msg.fault_message = status.fault_message
        return msg

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def _require_positive_finite(field_name: str, value: float) -> None:
    if value <= 0.0 or not isfinite(value):
        raise InvalidSafetyTuningError(field_name)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
