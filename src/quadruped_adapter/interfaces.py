from __future__ import annotations

from abc import ABC, abstractmethod

from embodiment_core.types import QuadrupedState


class QuadrupedBackend(ABC):
    @abstractmethod
    def connect(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_robot_state(self) -> QuadrupedState:
        raise NotImplementedError

    @abstractmethod
    def get_odom(self) -> dict:
        raise NotImplementedError

    @abstractmethod
    def teleop(self, cmd_vel: dict[str, float]) -> bool:
        raise NotImplementedError

    @abstractmethod
    def stand(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def sit(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def estop(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def start_recording_hint(self) -> None:
        raise NotImplementedError

