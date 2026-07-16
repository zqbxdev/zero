from __future__ import annotations

from math import isfinite

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from zero_interfaces.msg import MotorCommand, MotorState, UsvStatus

from .command_guard_model import (
    CommandGuardModel,
    GuardedCommand,
    InvalidSafetyTuningError,
    MotorStateInput,
    SafetyStatus,
    SafetyTuning,
    UsvStatusInput,
    parse_command_source,
    parse_control_mode,
)
from .safety_qos import E_STOP_QOS


class CommandGuard(Node):
    def __init__(self) -> None:
        super().__init__("command_guard")
        # Safety limits and watchdog windows for guarded motor commands.
        # 经安全门处理后的电机命令限幅和看门狗时间窗口。
        tuning = SafetyTuning(
            max_command_rpm=float(self.declare_parameter("max_rpm", 300.0).value),
            command_timeout_seconds=float(
                self.declare_parameter("command_timeout_seconds", 0.5).value),
            status_timeout_seconds=float(
                self.declare_parameter("status_timeout_seconds", 0.5).value),
            motor_state_timeout_seconds=float(
                self.declare_parameter("motor_state_timeout_seconds", 0.5).value),
            input_source=parse_command_source(
                str(self.declare_parameter("input_source", "auto").value)),
        )
        # Guarded command/status publish rate [Hz].
        # 经安全门处理后的命令和状态发布频率 [Hz]。
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        _require_positive_finite("publish_rate_hz", publish_rate_hz)

        self._model: CommandGuardModel = CommandGuardModel(tuning)
        # Protected motor command output consumed by hardware-facing nodes.
        # 面向硬件节点消费的安全后电机命令输出。
        self._motor_command_pub = self.create_publisher(MotorCommand, "/zero/motor_command", 10)
        # Safety status mirrors the guard decision for observers and launch helpers.
        # 安全状态向观察者和启动辅助节点反映安全门决策。
        self._safety_status_pub = self.create_publisher(UsvStatus, "/zero/safety_status", 10)
        # Raw motor commands must pass through this guard before hardware use.
        # 原始电机命令必须经过此安全门才能供硬件使用。
        self.create_subscription(MotorCommand, "/zero/motor_command_raw", self._on_raw_command, 10)
        self.create_subscription(UsvStatus, "/zero/status", self._on_status, 10)
        self.create_subscription(MotorState, "/zero/motor_state", self._on_motor_state, 10)
        # E-stop input uses latched QoS so late joiners observe the last command.
        # 急停输入使用锁存 QoS，便于后加入节点读取最新命令。
        self.create_subscription(Bool, "/zero/e_stop", self._on_e_stop, E_STOP_QOS)
        # Release requests clear only the internal latch after all safety checks pass.
        # 释放请求仅在所有安全检查通过后清除内部锁存。
        self.create_service(Trigger, "/zero/release_e_stop", self._on_release_e_stop)
        self.create_timer(1.0 / publish_rate_hz, self._publish_tick)

    def _on_raw_command(self, msg: MotorCommand) -> None:
        _ = self._model.accept_command(
            left_target_rpm=float(msg.left_target_rpm),
            right_target_rpm=float(msg.right_target_rpm),
            now_seconds=self._now_seconds(),
        )

    def _on_status(self, msg: UsvStatus) -> None:
        status = UsvStatusInput(
            mode=parse_control_mode(int(msg.mode)),
            fault=bool(msg.fault),
            fault_code=int(msg.fault_code),
            fault_message=str(msg.fault_message),
        )
        _ = self._model.accept_usv_status(status, self._now_seconds())

    def _on_motor_state(self, msg: MotorState) -> None:
        state = MotorStateInput(
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
        _ = self._model.accept_motor_state(state, self._now_seconds())

    def _on_e_stop(self, msg: Bool) -> None:
        self._model.set_e_stop(bool(msg.data))

    def _on_release_e_stop(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        # The service response mirrors the model decision; status publishes on ticks.
        # 服务响应映射模型决策；安全状态仍由周期 tick 发布。
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
