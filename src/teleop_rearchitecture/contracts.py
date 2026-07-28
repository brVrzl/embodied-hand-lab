"""Small, robot-independent command contracts used only by offline prototypes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class CommandState(str, Enum):
    """Liveness state; feasibility is deliberately represented separately."""

    ACTIVE = "ACTIVE"
    HOLD_REJECTED = "HOLD_REJECTED"
    DISENGAGED = "DISENGAGED"
    HARD_STOP = "HARD_STOP"


class StopReason(str, Enum):
    """Reasons a future adapter must treat as output termination."""

    CLUTCH_RELEASE = "CLUTCH_RELEASE"
    STALE_INPUT = "STALE_INPUT"
    CONTROLLER_ALARM = "CONTROLLER_ALARM"
    SDK_FAILURE = "SDK_FAILURE"
    TIMING_FAULT = "TIMING_FAULT"


@dataclass(frozen=True, slots=True)
class JointCommand:
    """An immutable post-IK command in canonical J1..J6 SI radians.

    ``state`` is a liveness heartbeat, not a request for a hardware adapter to
    invent a target.  ``HOLD_REJECTED`` therefore has no joint replacement.
    The future JAKA adapter must consume only commands emitted by a shaper and
    must not perform IK, mapping, or filtering.
    """

    sequence: int
    generated_ns: int
    joint_position_rad: tuple[float, float, float, float, float, float] | None
    state: CommandState
    reason: str

    def __post_init__(self) -> None:
        if self.sequence < 0 or self.generated_ns < 0 or not self.reason:
            raise ValueError("command sequence, timestamp, and reason are required")
        values = self.joint_position_rad
        if self.state is CommandState.ACTIVE:
            if values is None or len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError("ACTIVE command requires six finite joint radians")
        elif values is not None:
            raise ValueError("non-ACTIVE command must not smuggle a replacement target")


class LatestCommandMailbox:
    """Bounded latest-wins mailbox; it cannot become a command backlog."""

    def __init__(self) -> None:
        self._latest: JointCommand | None = None
        self.replaced = 0

    def publish(self, command: JointCommand) -> None:
        if self._latest is not None:
            if command.sequence <= self._latest.sequence:
                raise ValueError("command sequence must increase")
            self.replaced += 1
        self._latest = command

    def take_latest(self) -> JointCommand | None:
        command, self._latest = self._latest, None
        return command

    @property
    def depth(self) -> int:
        return int(self._latest is not None)
