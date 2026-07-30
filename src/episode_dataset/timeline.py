from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Generic, TypeVar


T = TypeVar("T")


class TimestampRegression(ValueError):
    """A source crossed backwards in its declared monotonic clock domain."""


@dataclass(frozen=True, slots=True)
class SourceSelection(Generic[T]):
    value: T | None
    source_timestamp_ns: int | None
    signed_offset_ns: int | None
    valid: bool
    stale: bool
    reason: str | None = None


class CausalTimeline(Generic[T]):
    """Bounded latest-at-or-before selection for one monotonic source.

    Future samples are never selected. A sample older than ``max_age_ns`` is
    returned for diagnostics but is marked stale and invalid.
    """

    def __init__(self, *, max_age_ns: int, capacity: int = 4096) -> None:
        if max_age_ns < 0:
            raise ValueError("max_age_ns must be non-negative")
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.max_age_ns = int(max_age_ns)
        self._items: deque[tuple[int, T]] = deque(maxlen=capacity)

    def append(self, timestamp_ns: int, value: T) -> None:
        timestamp_ns = int(timestamp_ns)
        if self._items and timestamp_ns < self._items[-1][0]:
            raise TimestampRegression(
                f"source timestamp regressed from {self._items[-1][0]} to {timestamp_ns}"
            )
        self._items.append((timestamp_ns, value))

    def latest_at_or_before(self, canonical_timestamp_ns: int) -> SourceSelection[T]:
        canonical_timestamp_ns = int(canonical_timestamp_ns)
        selected: tuple[int, T] | None = None
        for timestamp_ns, value in reversed(self._items):
            if timestamp_ns <= canonical_timestamp_ns:
                selected = (timestamp_ns, value)
                break
        if selected is None:
            return SourceSelection(None, None, None, False, False, "no_causal_sample")
        timestamp_ns, value = selected
        offset_ns = timestamp_ns - canonical_timestamp_ns
        stale = -offset_ns > self.max_age_ns
        return SourceSelection(
            value,
            timestamp_ns,
            offset_ns,
            not stale,
            stale,
            "source_sample_stale" if stale else None,
        )


class CanonicalClock:
    """30 Hz episode clock that never emits a catch-up burst."""

    def __init__(self, fps: int = 30) -> None:
        if fps <= 0:
            raise ValueError("fps must be positive")
        self.fps = int(fps)
        self.period_ns = round(1_000_000_000 / self.fps)
        self.start_ns: int | None = None
        self.next_ns: int | None = None
        self.frame_index = 0

    def start(self, timestamp_ns: int) -> tuple[int, int]:
        if self.start_ns is not None:
            raise RuntimeError("canonical clock already started")
        self.start_ns = int(timestamp_ns)
        self.next_ns = self.start_ns + self.period_ns
        self.frame_index = 1
        return 0, self.start_ns

    def due(self, now_ns: int) -> tuple[int, int] | None:
        if self.next_ns is None:
            raise RuntimeError("canonical clock has not started")
        if now_ns < self.next_ns:
            return None
        timestamp_ns = self.next_ns
        index = self.frame_index
        self.frame_index += 1
        self.next_ns += self.period_ns
        if now_ns >= self.next_ns:
            # Drop missed canonical slots. Never replay expired samples in a
            # burst and never move the episode time origin.
            missed = (now_ns - self.next_ns) // self.period_ns + 1
            self.next_ns += int(missed) * self.period_ns
        return index, timestamp_ns
