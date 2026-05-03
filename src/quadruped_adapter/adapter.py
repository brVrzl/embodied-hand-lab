from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml
from embodiment_core.types import QuadrupedState

from .interfaces import QuadrupedBackend
from .mock_backend import MockQuadrupedBackend


class QuadrupedAdapter:
    def __init__(self, config: dict, backend: QuadrupedBackend | None = None) -> None:
        self.config = config
        self.mode = config.get("mode", "mock")
        self.backend = backend or self._build_backend()

    @classmethod
    def from_yaml(cls, path: str | Path) -> "QuadrupedAdapter":
        return cls(load_yaml(path))

    def _build_backend(self) -> QuadrupedBackend:
        if self.mode == "mock":
            return MockQuadrupedBackend()
        raise NotImplementedError(
            "此处为待替换适配点: DeepRobotics real backend must be implemented for the selected model."
        )

    def connect(self) -> bool:
        return self.backend.connect()

    def get_robot_state(self) -> QuadrupedState:
        return self.backend.get_robot_state()

    def get_odom(self) -> dict:
        return self.backend.get_odom()

    def teleop(self, cmd_vel: dict[str, float]) -> bool:
        return self.backend.teleop(cmd_vel)

    def stand(self) -> bool:
        return self.backend.stand()

    def sit(self) -> bool:
        return self.backend.sit()

    def estop(self) -> None:
        self.backend.estop()

    def start_recording_hint(self) -> None:
        self.backend.start_recording_hint()

