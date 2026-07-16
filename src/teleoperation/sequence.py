from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class SequenceDisposition(str, Enum):
    FIRST = "first"
    NEW = "new"
    DUPLICATE = "duplicate"
    REORDERED = "reordered"


@dataclass(slots=True)
class SequenceTracker:
    last_sequence: int | None = None

    def observe(self, sequence: int) -> SequenceDisposition:
        if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
            raise ValueError("sequence must be a non-negative integer")
        if self.last_sequence is None:
            self.last_sequence = sequence
            return SequenceDisposition.FIRST
        if sequence == self.last_sequence:
            return SequenceDisposition.DUPLICATE
        if sequence < self.last_sequence:
            return SequenceDisposition.REORDERED
        self.last_sequence = sequence
        return SequenceDisposition.NEW

    def reset(self) -> None:
        self.last_sequence = None
