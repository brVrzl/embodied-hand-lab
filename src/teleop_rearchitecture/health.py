"""Offline classification boundary between liveness and feasibility."""

from __future__ import annotations

from .contracts import CommandState, StopReason


HARD_STOP_REASONS = frozenset(StopReason)


def output_must_terminate(state: CommandState, reason: StopReason | None = None) -> bool:
    """Hard faults terminate output; a feasible-target rejection does not."""

    if reason in HARD_STOP_REASONS:
        return True
    return state in (CommandState.DISENGAGED, CommandState.HARD_STOP)
