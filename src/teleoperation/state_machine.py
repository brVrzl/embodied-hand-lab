from __future__ import annotations

from dataclasses import dataclass

from .contracts import ControllerState, SafetyAction


_TRANSITIONS: dict[ControllerState, frozenset[ControllerState]] = {
    ControllerState.DISCONNECTED: frozenset({ControllerState.CONNECTING, ControllerState.SHUTDOWN}),
    ControllerState.CONNECTING: frozenset({ControllerState.CONNECTED, ControllerState.FAULT, ControllerState.CONTROLLED_STOP}),
    ControllerState.CONNECTED: frozenset({ControllerState.ARMED, ControllerState.CONTROLLED_STOP, ControllerState.FAULT}),
    ControllerState.ARMED: frozenset({ControllerState.EDG_READY, ControllerState.CONTROLLED_STOP, ControllerState.FAULT}),
    ControllerState.EDG_READY: frozenset({ControllerState.HOLDING, ControllerState.CONTROLLED_STOP, ControllerState.FAULT}),
    ControllerState.HOLDING: frozenset({ControllerState.RUNNING, ControllerState.CONTROLLED_STOP, ControllerState.FAULT}),
    ControllerState.RUNNING: frozenset({ControllerState.HOLDING, ControllerState.CONTROLLED_STOP, ControllerState.FAULT}),
    ControllerState.CONTROLLED_STOP: frozenset({ControllerState.DISCONNECTED, ControllerState.FAULT, ControllerState.SHUTDOWN}),
    ControllerState.FAULT: frozenset({ControllerState.CONTROLLED_STOP, ControllerState.DISCONNECTED, ControllerState.SHUTDOWN}),
    ControllerState.SHUTDOWN: frozenset(),
}


@dataclass(frozen=True, slots=True)
class Transition:
    previous: ControllerState
    current: ControllerState
    monotonic_ns: int
    reason: str


class LifecycleMachine:
    """Deterministic lifecycle model shared by supervision and tests.

    Hardware cleanup is implemented by the native worker.  This model prevents
    the Python supervisor from requesting impossible or motion-first states.
    """

    def __init__(self) -> None:
        self._state = ControllerState.DISCONNECTED
        self._history: list[Transition] = []

    @property
    def state(self) -> ControllerState:
        return self._state

    @property
    def history(self) -> tuple[Transition, ...]:
        return tuple(self._history)

    def transition(self, destination: ControllerState, *, monotonic_ns: int, reason: str) -> Transition:
        if destination not in _TRANSITIONS[self._state]:
            raise ValueError(f"invalid lifecycle transition {self._state.value} -> {destination.value}")
        if monotonic_ns < 0 or not reason.strip():
            raise ValueError("transition requires a non-negative timestamp and reason")
        item = Transition(self._state, destination, monotonic_ns, reason)
        self._state = destination
        self._history.append(item)
        return item


@dataclass(frozen=True, slots=True)
class StaleThresholds:
    warning_age_ns: int = 40_000_000
    hold_age_ns: int = 100_000_000
    controlled_stop_age_ns: int = 500_000_000
    fatal_communication_timeout_ns: int = 2_000_000_000

    def __post_init__(self) -> None:
        values = (
            self.warning_age_ns,
            self.hold_age_ns,
            self.controlled_stop_age_ns,
            self.fatal_communication_timeout_ns,
        )
        if values[0] < 0 or not all(a < b for a, b in zip(values, values[1:])):
            raise ValueError("stale thresholds must be non-negative and strictly increasing")


def stale_action(age_ns: int | None, thresholds: StaleThresholds) -> SafetyAction:
    """Return the conservative action for the newest target.

    ``None`` means no target has ever arrived.  Warnings are observable health
    information and do not themselves alter the command action.
    """

    if age_ns is None:
        return SafetyAction.HOLD
    if age_ns < 0:
        return SafetyAction.ABORT
    if age_ns >= thresholds.fatal_communication_timeout_ns:
        return SafetyAction.ABORT
    if age_ns >= thresholds.controlled_stop_age_ns:
        return SafetyAction.CONTROLLED_STOP
    if age_ns >= thresholds.hold_age_ns:
        return SafetyAction.HOLD
    return SafetyAction.ALLOW
