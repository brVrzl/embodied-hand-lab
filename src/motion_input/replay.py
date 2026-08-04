"""Replay recordings through the exact same provider contract as live input."""

from __future__ import annotations

from enum import Enum
import time
from typing import Callable, Iterator

from .model import DeviceDescriptor, MotionInputSample
from .provider import MotionInputProvider, ProviderState
from .recording import MotionRecordingReader


class ReplayMode(str, Enum):
    AS_RECORDED = "as_recorded"
    FIXED_RATE = "fixed_rate"
    IMMEDIATE = "immediate"


class ReplayProvider(MotionInputProvider):
    def __init__(
        self,
        path: str,
        *,
        mode: ReplayMode = ReplayMode.AS_RECORDED,
        speed: float = 1.0,
        fixed_rate_hz: float | None = None,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        super().__init__()
        if speed <= 0:
            raise ValueError("replay speed must be positive")
        if mode is ReplayMode.FIXED_RATE and (fixed_rate_hz is None or fixed_rate_hz <= 0):
            raise ValueError("fixed_rate_hz must be positive in fixed-rate mode")
        self.path = path
        self.mode = mode
        self.speed = speed
        self.fixed_rate_hz = fixed_rate_hz
        self._monotonic_ns = monotonic_ns
        self._sleep = sleep
        self._descriptor = MotionRecordingReader(path).read_header().device
        self._iterator: Iterator[MotionInputSample] | None = None
        self._pending: MotionInputSample | None = None
        self._previous_sample: MotionInputSample | None = None
        self._last_emit_ns: int | None = None

    @property
    def descriptor(self) -> DeviceDescriptor:
        return self._descriptor

    def _open(self) -> None:
        self._iterator = MotionRecordingReader(self.path).samples()
        self._pending = None
        self._previous_sample = None
        self._last_emit_ns = None

    def _close(self) -> None:
        if self._iterator is not None and hasattr(self._iterator, "close"):
            self._iterator.close()  # type: ignore[union-attr]
        self._iterator = None
        self._pending = None

    def _read(self, timeout_s: float | None) -> MotionInputSample | None:
        assert self._iterator is not None
        if self._pending is None:
            try:
                self._pending = next(self._iterator)
            except StopIteration:
                self._state = ProviderState.EXHAUSTED
                return None

        delay_s = self._delay_before(self._pending)
        if timeout_s is not None and delay_s > timeout_s:
            if timeout_s:
                self._sleep(timeout_s)
            return None
        if delay_s > 0:
            self._sleep(delay_s)
        sample = self._pending
        self._pending = None
        self._previous_sample = sample
        self._last_emit_ns = self._monotonic_ns()
        return sample

    def _delay_before(self, sample: MotionInputSample) -> float:
        if self._previous_sample is None or self.mode is ReplayMode.IMMEDIATE:
            return 0.0
        if self.mode is ReplayMode.FIXED_RATE:
            assert self.fixed_rate_hz is not None
            target_interval_ns = int(1_000_000_000 / (self.fixed_rate_hz * self.speed))
        else:
            target_interval_ns = _sample_interval_ns(self._previous_sample, sample)
            target_interval_ns = max(0, int(target_interval_ns / self.speed))
        if self._last_emit_ns is None:
            return target_interval_ns / 1_000_000_000
        elapsed_ns = self._monotonic_ns() - self._last_emit_ns
        return max(0, target_interval_ns - elapsed_ns) / 1_000_000_000


def _sample_interval_ns(previous: MotionInputSample, current: MotionInputSample) -> int:
    for old, new in (
        (previous.capture_timestamp, current.capture_timestamp),
        (previous.receive_timestamp, current.receive_timestamp),
    ):
        if old.comparable_to(new):
            return new.nanoseconds - old.nanoseconds
    return 0
