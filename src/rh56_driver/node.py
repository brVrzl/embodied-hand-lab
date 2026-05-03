from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml
from embodiment_core.logger import get_logger
from embodiment_core.types import HandState

from .interfaces import HandBackend, HandCommand
from .jaka_tool_backend import RH56JakaToolBackend
from .mock_backend import MockRH56Backend
from .serial_backend import RH56Ros2ServiceBackend, RH56SerialBackend


class RH56Driver:
    def __init__(self, config: dict, backend: HandBackend | None = None) -> None:
        self.config = config
        self.mode = config.get("mode", "mock")
        self.backend = backend or self._build_backend()
        self.logger = get_logger("RH56Driver")

    @classmethod
    def from_yaml(cls, path: str | Path) -> "RH56Driver":
        return cls(load_yaml(path))

    def _build_backend(self) -> HandBackend:
        backend_type = self.config.get("backend_type", self.mode)
        if self.mode == "mock" or backend_type == "mock":
            return MockRH56Backend(finger_count=self.config.get("finger_count", 6))
        if self.mode == "real" and backend_type == "serial_protocol":
            return RH56SerialBackend(self.config)
        if self.mode == "real" and backend_type == "jaka_tool_rs485":
            return RH56JakaToolBackend(self.config)
        if self.mode == "real" and backend_type == "ros2_services":
            return RH56Ros2ServiceBackend(self.config)
        raise NotImplementedError(f"Unsupported RH56 backend_type={backend_type!r}")

    def connect(self) -> bool:
        return self.backend.connect()

    def open(self) -> bool:
        return self.backend.execute(HandCommand(command="open"))

    def close(self, strength: float | None = None) -> bool:
        return self.backend.execute(
            HandCommand(command="close", strength=strength or self.config.get("close_strength", 0.5))
        )

    def pinch(self, strength: float | None = None) -> bool:
        return self.backend.execute(
            HandCommand(command="pinch", strength=strength or self.config.get("pinch_strength", 0.4))
        )

    def preset_grasp(self, preset_name: str, strength: float | None = None) -> bool:
        return self.backend.execute(
            HandCommand(
                command="preset_grasp",
                preset_name=preset_name,
                strength=strength or 0.5,
            )
        )

    def read_state(self) -> HandState:
        return self.backend.read_state()

    def stop(self) -> None:
        self.backend.stop()
