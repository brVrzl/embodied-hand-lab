from __future__ import annotations

import pytest

from teleoperation.contracts import ControllerState, SafetyAction
from teleoperation.safety import validate_zero_motion_target
from teleoperation.state_machine import LifecycleMachine, StaleThresholds, stale_action


def test_valid_startup_and_cleanup_transitions() -> None:
    machine = LifecycleMachine()
    for number, state in enumerate((ControllerState.CONNECTING, ControllerState.CONNECTED,
                                    ControllerState.ARMED, ControllerState.EDG_READY,
                                    ControllerState.HOLDING, ControllerState.CONTROLLED_STOP,
                                    ControllerState.SHUTDOWN), 1):
        machine.transition(state, monotonic_ns=number, reason="test")
    assert machine.state is ControllerState.SHUTDOWN


def test_startup_cannot_jump_to_running() -> None:
    with pytest.raises(ValueError, match="invalid lifecycle transition"):
        LifecycleMachine().transition(ControllerState.RUNNING, monotonic_ns=1, reason="unsafe")


def test_stale_actions_are_conservative_and_distinct() -> None:
    thresholds = StaleThresholds(10, 20, 30, 40)
    assert stale_action(None, thresholds) is SafetyAction.HOLD
    assert stale_action(19, thresholds) is SafetyAction.ALLOW
    assert stale_action(20, thresholds) is SafetyAction.HOLD
    assert stale_action(30, thresholds) is SafetyAction.CONTROLLED_STOP
    assert stale_action(40, thresholds) is SafetyAction.ABORT
    assert stale_action(-1, thresholds) is SafetyAction.ABORT


def test_zero_motion_validation() -> None:
    validate_zero_motion_target((0.0,) * 6, (0.00001,) * 6)
    with pytest.raises(ValueError, match="initial command delta"):
        validate_zero_motion_target((0.0,) * 6, (0.001,) * 6)
