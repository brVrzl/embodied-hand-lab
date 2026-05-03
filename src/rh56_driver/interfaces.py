from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from embodiment_core.types import HandState


@dataclass(slots=True)
class HandCommand:
    command: str
    strength: float = 0.4
    preset_name: str = ""


class HandBackend(ABC):
    @abstractmethod
    def connect(self) -> bool:
        raise NotImplementedError

    @abstractmethod
    def execute(self, command: HandCommand) -> bool:
        raise NotImplementedError

    @abstractmethod
    def read_state(self) -> HandState:
        raise NotImplementedError

    @abstractmethod
    def stop(self) -> None:
        raise NotImplementedError

