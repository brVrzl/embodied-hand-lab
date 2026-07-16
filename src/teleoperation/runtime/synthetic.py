from __future__ import annotations

import enum
import math
import random
import time
from dataclasses import dataclass

from ..contracts import Pose3D, PoseTarget, TimestampSet


class SyntheticPattern(str, enum.Enum):
    FIXED = "fixed"
    STEP = "step"
    RAMP = "ramp"
    SINE_TRANSLATION = "sine_translation"
    SINE_ROTATION = "sine_rotation"


@dataclass(slots=True)
class FaultSchedule:
    dropout_start_s: float | None = None
    dropout_duration_s: float = 0.0
    timestamp_jitter_ns: int = 0
    duplicate_every: int = 0
    reorder_every: int = 0
    burst_every: int = 0
    burst_count: int = 3
    slowdown_after_s: float | None = None
    slowdown_factor: float = 1.0


class SyntheticPoseSource:
    """Device-neutral test source.  Its output is never a hardware enable."""

    def __init__(self, pattern: SyntheticPattern, *, seed: int = 0, amplitude: float = 0.01,
                 frequency_hz: float = 0.5, faults: FaultSchedule | None = None) -> None:
        self.pattern = pattern
        self.amplitude = amplitude
        self.frequency_hz = frequency_hz
        self.faults = faults or FaultSchedule()
        self._random = random.Random(seed)
        self._sequence = 0

    def should_drop(self, elapsed_s: float) -> bool:
        start = self.faults.dropout_start_s
        return start is not None and start <= elapsed_s < start + self.faults.dropout_duration_s

    def period_scale(self, elapsed_s: float) -> float:
        start = self.faults.slowdown_after_s
        return self.faults.slowdown_factor if start is not None and elapsed_s >= start else 1.0

    def samples(self, elapsed_s: float) -> tuple[PoseTarget, ...]:
        count = self.faults.burst_count if self.faults.burst_every and self._sequence % self.faults.burst_every == 0 else 1
        targets = []
        for _ in range(count):
            sequence = self._sequence
            self._sequence += 1
            if self.faults.duplicate_every and sequence and sequence % self.faults.duplicate_every == 0:
                sequence -= 1
            elif self.faults.reorder_every and sequence > 1 and sequence % self.faults.reorder_every == 0:
                sequence -= 2
            targets.append(self._make(elapsed_s, sequence))
        return tuple(targets)

    def _make(self, elapsed_s: float, sequence: int) -> PoseTarget:
        phase = 2.0 * math.pi * self.frequency_hz * elapsed_s
        x = 0.0
        angle = 0.0
        if self.pattern == SyntheticPattern.STEP:
            x = self.amplitude if elapsed_s >= 1.0 else 0.0
        elif self.pattern == SyntheticPattern.RAMP:
            x = self.amplitude * min(elapsed_s, 1.0)
        elif self.pattern == SyntheticPattern.SINE_TRANSLATION:
            x = self.amplitude * math.sin(phase)
        elif self.pattern == SyntheticPattern.SINE_ROTATION:
            angle = self.amplitude * math.sin(phase)
        receive = time.monotonic_ns()
        source_capture = receive
        if self.faults.timestamp_jitter_ns:
            source_capture = max(0, source_capture + self._random.randint(-self.faults.timestamp_jitter_ns, self.faults.timestamp_jitter_ns))
        pose = Pose3D((x, 0.0, 0.0), (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0)))
        return PoseTarget("synthetic", sequence, "robot_base", pose,
                          TimestampSet(receive, source_capture_ns=source_capture, processing_ns=receive))
