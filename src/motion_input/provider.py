"""Common live/replay provider lifecycle contract."""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Iterator

from .errors import ProviderStateError
from .model import DeviceDescriptor, MotionInputSample


class ProviderState(str, Enum):
    CLOSED = "closed"
    OPEN = "open"
    EXHAUSTED = "exhausted"
    FAILED = "failed"


class MotionInputProvider(ABC):
    """Blocking pull interface shared by live devices and replay.

    ``None`` means no sample arrived before the timeout. Exhaustion is exposed
    through ``state`` for finite replay providers. Implementations must be safe
    to close more than once; no provider is allowed to issue robot commands.
    """

    def __init__(self) -> None:
        self._state = ProviderState.CLOSED

    @property
    def state(self) -> ProviderState:
        return self._state

    @property
    @abstractmethod
    def descriptor(self) -> DeviceDescriptor:
        raise NotImplementedError

    def open(self) -> None:
        if self._state is not ProviderState.CLOSED:
            raise ProviderStateError(f"cannot open provider in state {self._state.value}")
        self._open()
        self._state = ProviderState.OPEN

    def close(self) -> None:
        if self._state is ProviderState.CLOSED:
            return
        try:
            self._close()
        finally:
            self._state = ProviderState.CLOSED

    def read(self, timeout_s: float | None = None) -> MotionInputSample | None:
        if self._state is not ProviderState.OPEN:
            raise ProviderStateError(f"cannot read provider in state {self._state.value}")
        if timeout_s is not None and timeout_s < 0:
            raise ValueError("timeout_s must be non-negative")
        return self._read(timeout_s)

    def __enter__(self) -> "MotionInputProvider":
        self.open()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def iter_samples(self, timeout_s: float | None = None) -> Iterator[MotionInputSample]:
        while self._state is ProviderState.OPEN:
            sample = self.read(timeout_s)
            if sample is not None:
                yield sample
            elif self._state is ProviderState.EXHAUSTED:
                return

    @abstractmethod
    def _open(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _close(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def _read(self, timeout_s: float | None) -> MotionInputSample | None:
        raise NotImplementedError
