from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

from ..contracts import RobotState


@dataclass(slots=True)
class FakeJakaBackend:
    """Lifecycle/error stub.  It does not validate physical robot motion."""

    joint_position_rad: tuple[float, ...] = (0.0,) * 6
    fail_operations: set[str] = field(default_factory=set)
    connected: bool = False
    edg_active: bool = False
    commands: list[tuple[int, tuple[float, ...]]] = field(default_factory=list)
    cleanup_events: list[str] = field(default_factory=list)
    _owner_thread: int | None = None
    _state_sequence: int = 0

    def _check(self, operation: str) -> None:
        owner = threading.get_ident()
        if self._owner_thread is None:
            self._owner_thread = owner
        elif owner != self._owner_thread:
            raise RuntimeError("JAKA backend accessed by a non-owner thread")
        if operation in self.fail_operations:
            raise RuntimeError(f"injected JAKA failure: {operation}")

    def connect(self) -> None:
        self._check("connect")
        if self.connected:
            raise RuntimeError("backend is already connected")
        self.connected = True

    def read_state(self) -> RobotState:
        self._check("read_state")
        if not self.connected:
            raise RuntimeError("backend is disconnected")
        self._state_sequence += 1
        return RobotState(self._state_sequence, time.monotonic_ns(), tuple(self.joint_position_rad),
                          (0.0,) * 6, None, True, True, self.edg_active)

    def enter_edg(self) -> None:
        self._check("enter_edg")
        if not self.connected or self.edg_active:
            raise RuntimeError("invalid EDG entry")
        self.edg_active = True

    def command_joints(self, sequence: int, joint_position_rad: tuple[float, ...]) -> None:
        self._check("command_joints")
        if not self.edg_active or len(joint_position_rad) != 6:
            raise RuntimeError("joint command requires active EDG and six joints")
        values = tuple(float(value) for value in joint_position_rad)
        self.joint_position_rad = values
        self.commands.append((sequence, values))

    def leave_edg(self) -> None:
        self._check("leave_edg")
        if self.edg_active:
            self.edg_active = False
            self.cleanup_events.append("leave_edg")

    def disconnect(self) -> None:
        self._check("disconnect")
        self.leave_edg()
        if self.connected:
            self.connected = False
            self.cleanup_events.append("disconnect")

    def __enter__(self) -> "FakeJakaBackend":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.disconnect()
