from __future__ import annotations

from math import isfinite
from typing import NoReturn

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from zero_interfaces.msg import MotorState, UsvStatus
from zero_interfaces.srv import SetControlMode

from .command_guard_model import parse_control_mode
from .safety_initializer_model import (
    InitializerTuning,
    ModeRequestResult,
    MotorStateObservation,
    ReleaseRequestResult,
    SafetyInitializerModel,
    SafetyStatusObservation,
    TerminalResult,
    UsvStatusObservation,
)
from .safety_initializer_types import InvalidInitializerTuningError
from .safety_qos import E_STOP_QOS, INITIALIZER_STATUS_QOS


class SimulationSafetyInitializer(Node):
    def __init__(self) -> None:
        super().__init__("simulation_safety_initializer")
        # Status timeout parameter converted from seconds to nanoseconds.
        # 状态超时参数由秒转换为纳秒。
        status_timeout_ns = _seconds_to_ns(
            "status_timeout_seconds",
            float(self.declare_parameter("status_timeout_seconds", 0.5).value),
        )
        # Motor feedback timeout parameter converted from seconds to nanoseconds.
        # 电机反馈超时参数由秒转换为纳秒。
        motor_state_timeout_ns = _seconds_to_ns(
            "motor_state_timeout_seconds",
            float(self.declare_parameter("motor_state_timeout_seconds", 0.5).value),
        )
        # Initializer publish and service-request tick rate [Hz].
        # 初始化器发布和服务请求 tick 频率 [Hz]。
        publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 20.0).value)
        if publish_rate_hz <= 0.0 or not isfinite(publish_rate_hz):
            raise InvalidInitializerTuningError("publish_rate_hz")

        self._model = SafetyInitializerModel.start(
            InitializerTuning(status_timeout_ns, motor_state_timeout_ns),
            self._now_ns(),
        )
        # Latched e-stop command published during simulation safety bringup.
        # 仿真安全启动期间发布的锁存急停命令。
        self._e_stop_pub = self.create_publisher(Bool, "/zero/e_stop", E_STOP_QOS)
        # Terminal initializer result published once for launch orchestration.
        # 面向启动编排发布一次的初始化终态结果。
        self._terminal_status_pub = self.create_publisher(
            String, "/zero/safety_initializer_status", INITIALIZER_STATUS_QOS,
        )
        self._terminal_published = False
        # Authoritative feedback streams required before e-stop release.
        # 急停释放前所需的权威反馈流。
        self.create_subscription(UsvStatus, "/zero/status", self._on_usv_status, 10)
        self.create_subscription(MotorState, "/zero/motor_state", self._on_motor_state, 10)
        self.create_subscription(UsvStatus, "/zero/safety_status", self._on_safety_status, 10)
        # Services used to enter AUTO mode and release the guard latch.
        # 用于进入 AUTO 模式并释放安全门锁存的服务。
        self._mode_client = self.create_client(SetControlMode, "/zero/set_control_mode")
        self._release_client = self.create_client(Trigger, "/zero/release_e_stop")
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _on_usv_status(self, msg: UsvStatus) -> None:
        self._model = self._model.observe_usv_status(
            UsvStatusObservation(
                received_at_ns=self._now_ns(),
                mode=parse_control_mode(int(msg.mode)),
                fault=bool(msg.fault),
                fault_code=int(msg.fault_code),
                fault_message=str(msg.fault_message),
            ),
        )

    def _on_motor_state(self, msg: MotorState) -> None:
        self._model = self._model.observe_motor_state(
            MotorStateObservation(
                received_at_ns=self._now_ns(),
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
            ),
        )

    def _on_safety_status(self, msg: UsvStatus) -> None:
        self._model = self._model.observe_safety_status(
            SafetyStatusObservation(
                received_at_ns=self._now_ns(),
                mode=parse_control_mode(int(msg.mode)),
                fault=bool(msg.fault),
                fault_code=int(msg.fault_code),
                fault_message=str(msg.fault_message),
            ),
        )

    def _tick(self) -> None:
        step = self._model.advance(self._now_ns())
        self._model = step.model
        if step.publish_e_stop is not None:
            msg = Bool()
            msg.data = step.publish_e_stop
            self._e_stop_pub.publish(msg)
        if step.request_auto:
            self._request_auto()
        if step.request_release:
            self._request_release()
        self._report_terminal(step.terminal)

    def _request_auto(self) -> None:
        # Missing service is retried by later timer ticks until the model times out.
        # 服务暂不可用会由后续定时 tick 重试，直到模型超时。
        if not self._mode_client.service_is_ready():
            return
        request = SetControlMode.Request()
        request.mode = SetControlMode.Request.MODE_AUTO
        future = self._mode_client.call_async(request)
        future.add_done_callback(self._on_mode_response)

    def _request_release(self) -> None:
        # Missing service is retried by later timer ticks until the model times out.
        # 服务暂不可用会由后续定时 tick 重试，直到模型超时。
        if not self._release_client.service_is_ready():
            return
        future = self._release_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_release_response)

    def _on_mode_response(self, future: Future) -> None:
        # Normalize async failures into model results so the state machine owns failure handling.
        # 将异步失败归一化为模型结果，由状态机统一处理失败。
        now_ns = self._now_ns()
        exception = future.exception()
        if exception is not None:
            result = ModeRequestResult(now_ns, False, parse_control_mode(0), str(exception))
        else:
            response = future.result()
            if response is None:
                result = ModeRequestResult(now_ns, False, parse_control_mode(0), "empty response")
            else:
                result = ModeRequestResult(
                    now_ns,
                    bool(response.accepted),
                    parse_control_mode(int(response.current_mode)),
                    str(response.message),
                )
        self._model = self._model.accept_mode_result(result)

    def _on_release_response(self, future: Future) -> None:
        # Normalize async failures into model results so the state machine owns failure handling.
        # 将异步失败归一化为模型结果，由状态机统一处理失败。
        now_ns = self._now_ns()
        exception = future.exception()
        if exception is not None:
            result = ReleaseRequestResult(now_ns, False, str(exception))
        else:
            response = future.result()
            if response is None:
                result = ReleaseRequestResult(now_ns, False, "empty response")
            else:
                result = ReleaseRequestResult(
                    now_ns,
                    bool(response.success),
                    str(response.message),
                )
        self._model = self._model.accept_release_result(result)

    def _report_terminal(self, terminal: TerminalResult | None) -> None:
        if terminal is None or self._terminal_published:
            return
        msg = String()
        msg.data = terminal.value
        self._terminal_status_pub.publish(msg)
        self._terminal_published = True
        match terminal:
            case TerminalResult.SUCCESS:
                self.get_logger().info("SAFETY_INITIALIZER SUCCESS")
            case TerminalResult.FAILED:
                self.get_logger().error(
                    f"SAFETY_INITIALIZER FAILED: {self._model.failure_reason}",
                )
            case unreachable:
                assert_never(unreachable)
        self._timer.cancel()
        rclpy.shutdown()

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds


def _seconds_to_ns(field_name: str, value: float) -> int:
    if value <= 0.0 or not isfinite(value):
        raise InvalidInitializerTuningError(field_name)
    return int(value * 1_000_000_000)


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled terminal result: {value!r}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimulationSafetyInitializer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
