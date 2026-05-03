from __future__ import annotations

from embodiment_core.logger import get_logger
from embodiment_core.types import HandState

from .interfaces import HandBackend, HandCommand


class MockRH56Backend(HandBackend):
    def __init__(self, finger_count: int = 6) -> None:
        self._finger_count = finger_count
        self._connected = False
        self._state = HandState(
            mode="idle",
            finger_positions=[0.0] * finger_count,
            finger_currents=[0.0] * finger_count,
            contact_flags=[False] * finger_count,
            force_estimate=[0.0] * finger_count,
        )
        self._logger = get_logger("MockRH56Backend")

    def connect(self) -> bool:
        self._connected = True
        self._logger.info("Connected to mock RH56 backend.")
        return True

    def execute(self, command: HandCommand) -> bool:
        if not self._connected:
            raise RuntimeError("RH56 backend is not connected.")
        mode = command.command.lower()
        self._state.mode = mode
        if mode == "open":
            self._state.finger_positions = [0.0] * self._finger_count
        elif mode == "close":
            self._state.finger_positions = [1.0] * self._finger_count
        elif mode == "pinch":
            self._state.finger_positions = [1.0, 1.0] + [0.2] * (self._finger_count - 2)
        elif mode == "preset_grasp":
            self._state.finger_positions = [command.strength] * self._finger_count
        else:
            raise ValueError(f"Unsupported hand command: {command.command}")
        self._state.force_estimate = [round(value * command.strength, 3) for value in self._state.finger_positions]
        self._state.contact_flags = [value > 0.8 for value in self._state.finger_positions]
        self._logger.info("Mock RH56 execute: %s", command)
        return True

    def read_state(self) -> HandState:
        if not self._connected:
            raise RuntimeError("RH56 backend is not connected.")
        return HandState(
            mode=self._state.mode,
            finger_positions=list(self._state.finger_positions),
            finger_currents=list(self._state.finger_currents),
            contact_flags=list(self._state.contact_flags),
            force_estimate=list(self._state.force_estimate),
        )

    def stop(self) -> None:
        self._state.mode = "stopped"
        self._logger.warning("Mock RH56 stop called.")

