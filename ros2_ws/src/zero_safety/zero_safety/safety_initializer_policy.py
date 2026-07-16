from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Final, NoReturn, Protocol, TypeVar

from .command_guard_types import ControlMode, SafetyFaultCode
from .safety_initializer_types import (
    InitializerState,
    InitializerTuning,
    MotorStateObservation,
    RawNanoseconds,
    SafetyStatusObservation,
    TerminalResult,
    UsvStatusObservation,
)


# Safety status fault codes that mean the initializer is still holding e-stop.
# 表示初始化器仍在保持急停的安全状态故障码。
PENDING_E_STOP_CODES: Final = frozenset(
    {int(SafetyFaultCode.E_STOP_INPUT_ACTIVE), int(SafetyFaultCode.E_STOP_UNKNOWN)},
)
# Safety status fault codes accepted after e-stop release.
# 急停释放后可接受的安全状态故障码。
RELEASED_E_STOP_CODES: Final = frozenset(
    {
        int(SafetyFaultCode.NONE),
        int(SafetyFaultCode.COMMAND_TIMEOUT),
        int(SafetyFaultCode.COMMAND_CLAMPED),
    },
)


@dataclass(frozen=True, slots=True)
class AuthoritativeSnapshot:
    # Timeout configuration used for authoritative-health checks.
    # 权威健康检查使用的超时配置。
    tuning: InitializerTuning
    # Latest USV status sample.
    # 最新无人船状态样本。
    usv_status: UsvStatusObservation | None
    # Latest motor state sample.
    # 最新电机状态样本。
    motor_state: MotorStateObservation | None


StepT = TypeVar("StepT", covariant=True)


class _DecisionModel(Protocol[StepT]):
    @property
    def tuning(self) -> InitializerTuning: ...

    @property
    def state_started_ns(self) -> int: ...

    @property
    def safety_status(self) -> SafetyStatusObservation | None: ...

    def _step(self, *, request_release: bool = False) -> StepT: ...

    def _fail(self, reason: str) -> _DecisionModel[StepT]: ...

    def _enter(
        self,
        state: InitializerState,
        now_ns: int,
        timeout_ns: int,
    ) -> _DecisionModel[StepT]: ...

    def _succeed(self) -> _DecisionModel[StepT]: ...

    def _authoritative_health(
        self,
        now_ns: int,
        minimum_received_ns: int,
    ) -> tuple[bool, str | None]: ...


def authoritative_health(
    snapshot: AuthoritativeSnapshot,
    now_ns: int,
    minimum_received_ns: int,
) -> tuple[bool, str | None]:
    usv = snapshot.usv_status
    if usv is None or usv.received_at_ns <= minimum_received_ns:
        return False, None
    if now_ns - usv.received_at_ns > snapshot.tuning.status_timeout_ns:
        return False, "USV status is stale"
    match usv.mode:
        case ControlMode.AUTO:
            pass
        case ControlMode.UNKNOWN | ControlMode.STOP | ControlMode.MANUAL | ControlMode.FAULT:
            return False, "USV mode is not AUTO"
        case unreachable:
            assert_never(unreachable)
    if usv.fault or usv.fault_code != 0:
        return False, "USV status reports fault"

    motor = snapshot.motor_state
    if motor is None or motor.received_at_ns <= minimum_received_ns:
        return False, None
    if now_ns - motor.received_at_ns > snapshot.tuning.motor_state_timeout_ns:
        return False, "Motor state is stale"
    if motor.fault:
        return False, "Motor state reports fault"
    values = (
        motor.left_target_rpm,
        motor.right_target_rpm,
        motor.left_actual_rpm,
        motor.right_actual_rpm,
        motor.left_pwm_duty,
        motor.right_pwm_duty,
    )
    if not all(isfinite(value) for value in values):
        return False, "Motor state contains non-finite values"
    return True, None


def advance_false_handshake(model: _DecisionModel[StepT], now_ns: int) -> StepT:
    """Wait for false e-stop input to leave only the guard latch blocking output.
    等待急停输入为 false 后，仅剩安全门锁存阻断输出。
    """
    sample = _new_safety_status(model)
    if sample is None:
        return model._step()
    if sample.mode is not ControlMode.AUTO:
        return model._fail("safety status mode rolled back before release")._step()
    fault_code = sample.fault_code
    if fault_code == int(SafetyFaultCode.E_STOP_LATCHED):
        entered = model._enter(
            InitializerState.WAIT_RELEASE_RESPONSE,
            now_ns,
            model.tuning.status_timeout_ns,
        )
        return entered._step(request_release=True)
    if fault_code in PENDING_E_STOP_CODES:
        return model._step()
    return model._fail("e-stop latch cleared before release request")._step()


def advance_release_confirmation(model: _DecisionModel[StepT]) -> StepT:
    """Accept only post-release guard statuses that prove the latch cleared.
    仅接受能证明锁存已清除的释放后安全门状态。
    """
    sample = _new_safety_status(model)
    if sample is None:
        return model._step()
    if sample.mode is not ControlMode.AUTO:
        return model._fail("safety status mode rolled back after release")._step()
    if sample.fault:
        return model._fail("safety status reports fault after release")._step()
    fault_code = sample.fault_code
    if fault_code in RELEASED_E_STOP_CODES:
        return model._succeed()._step()
    if fault_code == int(SafetyFaultCode.E_STOP_LATCHED):
        return model._step()
    return model._fail("e-stop release confirmation mismatched")._step()


def required_health_failure(model: _DecisionModel[StepT], now_ns: int) -> str | None:
    """Return an authoritative-health failure, if any.
    返回权威健康检查故障（如有）。
    """
    ready, error = model._authoritative_health(now_ns, 0)
    if error is not None:
        return error
    return None if ready else "Authoritative status stream disappeared"


def _new_safety_status(model: _DecisionModel[StepT]) -> SafetyStatusObservation | None:
    sample = model.safety_status
    if sample is None or sample.received_at_ns <= model.state_started_ns:
        return None
    return sample


def normalize_time(raw_now_ns: RawNanoseconds) -> int | None:
    if type(raw_now_ns) is not int or raw_now_ns < 0:
        return None
    return raw_now_ns


def target_e_stop(state: InitializerState) -> bool:
    # Hold e-stop true during startup/failure; publish false through release phases.
    # 启动和失败阶段保持急停 true；释放阶段发布 false。
    match state:
        case (
            InitializerState.WAIT_ESTOP_TRUE
            | InitializerState.WAIT_AUTO_RESPONSE
            | InitializerState.WAIT_HEALTHY_STATUS
            | InitializerState.FAILED
        ):
            return True
        case (
            InitializerState.WAIT_ESTOP_FALSE_LATCHED
            | InitializerState.WAIT_RELEASE_RESPONSE
            | InitializerState.WAIT_LATCH_RELEASE
            | InitializerState.SUCCESS
        ):
            return False
        case unreachable:
            assert_never(unreachable)


def terminal_result(state: InitializerState) -> TerminalResult:
    match state:
        case InitializerState.SUCCESS:
            return TerminalResult.SUCCESS
        case InitializerState.FAILED:
            return TerminalResult.FAILED
        case (
            InitializerState.WAIT_ESTOP_TRUE
            | InitializerState.WAIT_AUTO_RESPONSE
            | InitializerState.WAIT_HEALTHY_STATUS
            | InitializerState.WAIT_ESTOP_FALSE_LATCHED
            | InitializerState.WAIT_RELEASE_RESPONSE
            | InitializerState.WAIT_LATCH_RELEASE
        ):
            raise AssertionError("nonterminal state requested terminal result")
        case unreachable:
            assert_never(unreachable)


def assert_never(value: NoReturn) -> NoReturn:
    raise AssertionError(f"Unhandled control mode: {value!r}")
