from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from ..contracts import ArmPoseSample, OperatorActionSample, RunGateSample


@dataclass(frozen=True, slots=True)
class AdapterSnapshot:
    pose: ArmPoseSample | None
    run_gate: RunGateSample
    connected: bool
    generation: int
    reason: str = ""
    operator_action: OperatorActionSample | None = None


class PoseInput(Protocol):
    def latest(self, *, now_ns: int, after_generation: int = -1) -> AdapterSnapshot | None: ...

    def close(self) -> None: ...
