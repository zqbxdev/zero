from __future__ import annotations

from dataclasses import dataclass, replace
from typing import NoReturn

from . import safety_initializer_policy as _initializer_policy
from .command_guard_types import ControlMode, SafetyFaultCode
from .safety_initializer_policy import (
    AuthoritativeSnapshot,
    PENDING_E_STOP_CODES as _PENDING_E_STOP_CODES,
    RELEASED_E_STOP_CODES as _RELEASED_E_STOP_CODES,
    authoritative_health,
    normalize_time,
    target_e_stop,
    terminal_result,
)
from .safety_initializer_types import (
    InitializerState,
    InitializerTuning,
    ModeRequestResult,
    MotorStateObservation,
    RawNanoseconds,
    ReleaseRequestResult,
    SafetyStatusObservation,
    TerminalResult,
    UsvStatusObservation,
)


PENDING_E_STOP_CODES, RELEASED_E_STOP_CODES = _PENDING_E_STOP_CODES, _RELEASED_E_STOP_CODES


@dataclass(frozen=True, slots=True)
class InitializerStep:
    # State model after applying one initializer tick.
    # 执行一次初始化 tick 后的状态模型。
    model: SafetyInitializerModel
    # E-stop value to publish, or None when no publish is needed.
    # 需要发布的急停值；无需发布时为 None。
    publish_e_stop: bool | None
    # True when the AUTO mode service should be requested.
    # 需要请求 AUTO 模式服务时为 true。
    request_auto: bool = False
    # True when the e-stop release service should be requested.
    # 需要请求急停释放服务时为 true。
    request_release: bool = False
    # Terminal result to report once the initializer finishes.
    # 初始化结束后需要报告的终态结果。
    terminal: TerminalResult | None = None


@dataclass(frozen=True, slots=True)
class SafetyInitializerModel:
    """Immutable state machine for simulation safety bringup.
    仿真安全启动使用的不可变状态机。
    """

    tuning: InitializerTuning
    state: InitializerState
    # Timestamp when the current state was entered [ns].
    # 进入当前状态的时间戳 [ns]。
    state_started_ns: int
    # Deadline for the current state [ns].
    # 当前状态的截止时间 [ns]。
    deadline_ns: int
    # Last accepted local timestamp [ns].
    # 最近一次接受的本地时间戳 [ns]。
    last_now_ns: int
    # Latest authoritative USV status sample.
    # 最新权威无人船状态样本。
    usv_status: UsvStatusObservation | None = None
    # Latest authoritative motor state sample.
    # 最新权威电机状态样本。
    motor_state: MotorStateObservation | None = None
    # Latest guard safety status sample.
    # 最新安全门状态样本。
    safety_status: SafetyStatusObservation | None = None
    # Latest AUTO mode request result.
    # 最新 AUTO 模式请求结果。
    mode_result: ModeRequestResult | None = None
    # Latest e-stop release request result.
    # 最新急停释放请求结果。
    release_result: ReleaseRequestResult | None = None
    # Human-readable failure reason; empty unless state is FAILED.
    # 可读失败原因；仅在 FAILED 状态下非空。
    failure_reason: str = ""
    # True after terminal status has been emitted once.
    # 终态状态已发送一次后为 true。
    terminal_reported: bool = False

    @classmethod
    def start(cls, tuning: InitializerTuning, now_ns: RawNanoseconds) -> SafetyInitializerModel:
        normalized = normalize_time(now_ns)
        if normalized is None:
            return cls(
                tuning,
                InitializerState.FAILED,
                0,
                0,
                0,
                failure_reason="invalid start time",
            )
        return cls(
            tuning=tuning,
            state=InitializerState.WAIT_ESTOP_TRUE,
            state_started_ns=normalized,
            deadline_ns=normalized + tuning.status_timeout_ns,
            last_now_ns=normalized,
        )

    def observe_usv_status(self, observation: UsvStatusObservation) -> SafetyInitializerModel:
        model, received_at_ns = self._record_time(observation.received_at_ns)
        if received_at_ns is None:
            return model
        return replace(model, usv_status=observation)

    def observe_motor_state(self, observation: MotorStateObservation) -> SafetyInitializerModel:
        model, received_at_ns = self._record_time(observation.received_at_ns)
        if received_at_ns is None:
            return model
        return replace(model, motor_state=observation)

    def observe_safety_status(
        self,
        observation: SafetyStatusObservation,
    ) -> SafetyInitializerModel:
        model, received_at_ns = self._record_time(observation.received_at_ns)
        if received_at_ns is None:
            return model
        return replace(model, safety_status=observation)

    def accept_mode_result(self, result: ModeRequestResult) -> SafetyInitializerModel:
        model, received_at_ns = self._record_time(result.received_at_ns)
        if received_at_ns is None:
            return model
        return replace(model, mode_result=result)

    def accept_release_result(self, result: ReleaseRequestResult) -> SafetyInitializerModel:
        model, received_at_ns = self._record_time(result.received_at_ns)
        if received_at_ns is None:
            return model
        return replace(model, release_result=result)

    def advance(self, now_ns: RawNanoseconds) -> InitializerStep:
        if self._is_terminal():
            return self._terminal_step()
        model, normalized = self._record_time(now_ns)
        if normalized is None:
            return model._terminal_step()
        if normalized > model.deadline_ns:
            return model._fail(f"timeout in {model.state.value}")._terminal_step()
        step = model._advance_at(normalized)
        if step.model._is_terminal():
            return step.model._terminal_step()
        return step

    def _advance_at(self, now_ns: int) -> InitializerStep:
        match self.state:
            case InitializerState.WAIT_ESTOP_TRUE:
                # Require fresh guard feedback proving the initializer-held e-stop is visible.
                # 要求新的安全门反馈证明初始化器保持的急停已可见。
                sample = self.safety_status
                if sample is not None and sample.received_at_ns <= self.state_started_ns:
                    sample = None
                if sample is not None and sample.fault_code == int(
                    SafetyFaultCode.E_STOP_INPUT_ACTIVE,
                ):
                    model = self._enter(
                        InitializerState.WAIT_AUTO_RESPONSE,
                        now_ns,
                        self.tuning.status_timeout_ns,
                    )
                    return model._step(request_auto=True)
                return self._step()
            case InitializerState.WAIT_AUTO_RESPONSE:
                # Same-tick service responses are valid after the request state is entered.
                # 进入请求状态后的同一 tick 服务响应有效。
                sample = self.mode_result
                if sample is None or sample.received_at_ns < self.state_started_ns:
                    return self._step()
                if not sample.accepted or sample.current_mode is not ControlMode.AUTO:
                    return self._fail("AUTO request rejected or mismatched")._step()
                timeout_ns = max(self.tuning.status_timeout_ns, self.tuning.motor_state_timeout_ns)
                return self._enter(
                    InitializerState.WAIT_HEALTHY_STATUS,
                    now_ns,
                    timeout_ns,
                )._step()
            case InitializerState.WAIT_HEALTHY_STATUS:
                # AUTO alone is not enough; status and motor feedback must both be fresh.
                # 仅进入 AUTO 不足够；状态和电机反馈都必须新鲜。
                ready, error = self._authoritative_health(now_ns, self.state_started_ns)
                if error is not None:
                    return self._fail(error)._step()
                if not ready:
                    return self._step()
                return self._enter(
                    InitializerState.WAIT_ESTOP_FALSE_LATCHED,
                    now_ns,
                    self.tuning.status_timeout_ns,
                )._step()
            case InitializerState.WAIT_ESTOP_FALSE_LATCHED:
                # Lower external e-stop while expecting the guard's internal latch to remain set.
                # 拉低外部急停，同时期望安全门内部锁存仍保持。
                failure = _initializer_policy.required_health_failure(self, now_ns)
                if failure is not None:
                    return self._fail(failure)._step()
                return _initializer_policy.advance_false_handshake(self, now_ns)
            case InitializerState.WAIT_RELEASE_RESPONSE:
                # Release acceptance is meaningful only while authoritative health stays valid.
                # 释放接受仅在权威健康持续有效时才有意义。
                failure = _initializer_policy.required_health_failure(self, now_ns)
                if failure is not None:
                    return self._fail(failure)._step()
                sample = self.release_result
                if sample is None or sample.received_at_ns < self.state_started_ns:
                    return self._step()
                if not sample.accepted:
                    return self._fail("e-stop release rejected")._step()
                return self._enter(
                    InitializerState.WAIT_LATCH_RELEASE,
                    now_ns,
                    self.tuning.status_timeout_ns,
                )._step()
            case InitializerState.WAIT_LATCH_RELEASE:
                # Wait until guard status confirms the latch is no longer the blocking fault.
                # 等待安全门状态确认锁存不再是阻断故障。
                failure = _initializer_policy.required_health_failure(self, now_ns)
                if failure is not None:
                    return self._fail(failure)._step()
                return _initializer_policy.advance_release_confirmation(self)
            case InitializerState.SUCCESS | InitializerState.FAILED:
                raise AssertionError("terminal state reached nonterminal transition")
            case unreachable:
                assert_never(unreachable)

    def _authoritative_health(
        self,
        now_ns: int,
        minimum_received_ns: int,
    ) -> tuple[bool, str | None]:
        return authoritative_health(
            AuthoritativeSnapshot(self.tuning, self.usv_status, self.motor_state),
            now_ns,
            minimum_received_ns,
        )

    def _enter(
        self,
        state: InitializerState,
        now_ns: int,
        timeout_ns: int,
    ) -> SafetyInitializerModel:
        return replace(
            self,
            state=state,
            state_started_ns=now_ns,
            deadline_ns=now_ns + timeout_ns,
            mode_result=None,
            release_result=None,
        )

    def _record_time(
        self,
        raw_now_ns: RawNanoseconds,
    ) -> tuple[SafetyInitializerModel, int | None]:
        if self._is_terminal():
            return self, None
        normalized = normalize_time(raw_now_ns)
        if normalized is None:
            return self._fail("time must be a non-negative integer nanosecond value"), None
        if normalized < self.last_now_ns:
            return self._fail("clock moved backwards"), None
        return replace(self, last_now_ns=normalized), normalized

    def _step(
        self,
        *,
        request_auto: bool = False,
        request_release: bool = False,
    ) -> InitializerStep:
        return InitializerStep(
            model=self,
            publish_e_stop=target_e_stop(self.state),
            request_auto=request_auto,
            request_release=request_release,
        )

    def _terminal_step(self) -> InitializerStep:
        if self.terminal_reported:
            return InitializerStep(self, None)
        model = replace(self, terminal_reported=True)
        return InitializerStep(
            model,
            target_e_stop(model.state),
            terminal=terminal_result(model.state),
        )

    def _fail(self, reason: str) -> SafetyInitializerModel:
        if self._is_terminal():
            return self
        return replace(self, state=InitializerState.FAILED, failure_reason=reason)

    def _succeed(self) -> SafetyInitializerModel:
        return replace(self, state=InitializerState.SUCCESS)

    def _is_terminal(self) -> bool:
        return self.state in (InitializerState.SUCCESS, InitializerState.FAILED)


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled initializer state: {value!r}")
