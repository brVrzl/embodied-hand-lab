from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Sequence

import numpy as np

from rh56_driver.hand_schema import CANONICAL_HAND_ORDER


@dataclass(frozen=True, slots=True)
class HardwareConstraintResult:
    feasible: bool
    thumb_index_blocking_risk: float
    reasons: list[str]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        result["canonical_hand_order"] = list(CANONICAL_HAND_ORDER)
        return result


def evaluate_rh56_hardware_constraints(
    hand_command: Sequence[float],
    *,
    max_thumb_index_blocking_risk: float = 0.70,
) -> HardwareConstraintResult:
    command = np.asarray(hand_command, dtype=np.float64).reshape(-1)
    if command.size != len(CANONICAL_HAND_ORDER):
        raise ValueError(f"Expected {len(CANONICAL_HAND_ORDER)} RH56 command values, got {command.size}.")
    if not np.isfinite(command).all():
        raise ValueError("RH56 command contains NaN or infinite values.")
    command = np.clip(command, 0.0, 1.0)
    by_name = dict(zip(CANONICAL_HAND_ORDER, command.tolist(), strict=True))

    index = float(by_name["index"])
    thumb_close = float(by_name["thumb_close"])
    thumb_lateral = float(by_name["thumb_lateral"])
    risk = estimate_thumb_index_blocking_risk(
        index=index,
        thumb_close=thumb_close,
        thumb_lateral=thumb_lateral,
    )
    reasons: list[str] = []
    if risk >= max_thumb_index_blocking_risk:
        reasons.append("thumb_index_blocking_risk")
    if index > 0.78 and thumb_close > 0.72:
        reasons.append("deep_index_thumb_closure")
    if thumb_lateral > 0.85 and thumb_close > 0.60:
        reasons.append("high_thumb_opposition_with_bend")

    return HardwareConstraintResult(
        feasible=not reasons,
        thumb_index_blocking_risk=risk,
        reasons=reasons,
    )


def estimate_thumb_index_blocking_risk(
    *,
    index: float,
    thumb_close: float,
    thumb_lateral: float,
) -> float:
    index_term = _ramp(index, low=0.48, high=0.78)
    thumb_close_term = _ramp(thumb_close, low=0.50, high=0.75)
    opposition_term = _ramp(thumb_lateral, low=0.58, high=0.88)
    direct_collision_term = _ramp(index + thumb_close, low=1.22, high=1.58)
    risk = 0.45 * index_term * thumb_close_term + 0.45 * index_term * thumb_close_term * opposition_term
    risk += 0.10 * direct_collision_term
    return float(np.clip(risk, 0.0, 1.0))


def _ramp(value: float, *, low: float, high: float) -> float:
    if high <= low:
        raise ValueError("high must be greater than low.")
    return float(np.clip((float(value) - low) / (high - low), 0.0, 1.0))
