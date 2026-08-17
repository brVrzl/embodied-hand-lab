"""Absolute-time ACT action-chunk execution.

The policy predicts actions for future control ticks.  This module keeps that
time relationship explicit instead of treating a new query as permission to
discard all horizons after a small prefix.  It is intentionally independent
of cameras, robot adapters, and model frameworks so it can be exercised by
offline replay tests.

The weighting follows LeRobot's ``ACTTemporalEnsembler``: candidates are
ordered from oldest query to newest query and receive
``exp(-coefficient * ordinal)``.  Therefore a positive coefficient gives the
oldest overlapping prediction the largest weight.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class TemporalPrediction:
    query_id: int
    query_timestamp_ns: int
    query_command_tick: int
    chunk: np.ndarray


@dataclass(frozen=True)
class TemporalContributor:
    query_id: int
    query_timestamp_ns: int
    query_command_tick: int
    source_horizon: int
    target_command_tick: int
    predicted_action: np.ndarray
    ensemble_weight: float

    def as_dict(self) -> dict[str, Any]:
        return {
            "query_id": self.query_id,
            "query_timestamp_ns": self.query_timestamp_ns,
            "query_command_tick": self.query_command_tick,
            "source_horizon": self.source_horizon,
            "target_command_tick": self.target_command_tick,
            "predicted_action": self.predicted_action.tolist(),
            "ensemble_weight": self.ensemble_weight,
        }


@dataclass(frozen=True)
class TemporalSelection:
    command_tick: int
    action: np.ndarray | None
    contributors: tuple[TemporalContributor, ...]
    fallback_used: bool
    fallback_action_age_ticks: int | None

    @property
    def num_contributors(self) -> int:
        return len(self.contributors)

    def as_dict(self) -> dict[str, Any]:
        ages = [self.command_tick - c.query_command_tick for c in self.contributors]
        return {
            "command_tick": self.command_tick,
            "fallback_used": self.fallback_used,
            "fallback_action_age_ticks": self.fallback_action_age_ticks,
            "num_contributors": len(self.contributors),
            "oldest_contributing_prediction_age_ticks": max(ages) if ages else None,
            "newest_contributing_prediction_age_ticks": min(ages) if ages else None,
            "contributors": [c.as_dict() for c in self.contributors],
        }


class AbsoluteTimeTemporalEnsembler:
    """Bounded absolute command-tick temporal ensemble.

    ``add_prediction`` maps every chunk horizon ``h`` to
    ``query_command_tick + h``.  ``select(t)`` only considers predictions
    whose target tick is exactly ``t``; no future prediction can leak into a
    command.  Expired and capacity-bounded entries are counted rather than
    silently hidden.
    """

    def __init__(
        self,
        *,
        action_dim: int,
        chunk_size: int,
        coefficient: float = 0.01,
        max_source_horizon: int | None = None,
        max_prediction_age_ticks: int | None = None,
        capacity: int = 4096,
    ) -> None:
        if action_dim < 1 or chunk_size < 1:
            raise ValueError("action_dim and chunk_size must be positive")
        if not math.isfinite(coefficient):
            raise ValueError("temporal ensemble coefficient must be finite")
        if max_source_horizon is not None and not 0 <= max_source_horizon < chunk_size:
            raise ValueError("max_source_horizon must be within the chunk")
        if max_prediction_age_ticks is not None and max_prediction_age_ticks < 0:
            raise ValueError("max_prediction_age_ticks must be non-negative")
        if capacity < 1:
            raise ValueError("temporal ensemble capacity must be positive")
        self.action_dim = int(action_dim)
        self.chunk_size = int(chunk_size)
        self.coefficient = float(coefficient)
        self.max_source_horizon = chunk_size - 1 if max_source_horizon is None else int(max_source_horizon)
        self.max_prediction_age_ticks = max_prediction_age_ticks
        self.capacity = int(capacity)
        self._by_tick: dict[int, list[tuple[TemporalPrediction, int]]] = {}
        self._entry_count = 0
        self._last_action: np.ndarray | None = None
        self._last_action_tick: int | None = None
        self._stats = {
            "predictions_added": 0,
            "prediction_points_added": 0,
            "prediction_points_capacity_dropped": 0,
            "prediction_points_expired": 0,
            "prediction_points_age_expired": 0,
            "selected_ticks": 0,
            "fallback_ticks": 0,
        }

    def reset(self) -> None:
        self._by_tick.clear()
        self._entry_count = 0
        self._last_action = None
        self._last_action_tick = None

    def add_prediction(
        self,
        *,
        query_id: int,
        query_timestamp_ns: int,
        query_command_tick: int,
        chunk: np.ndarray,
    ) -> int:
        array = np.asarray(chunk, dtype=np.float64)
        expected = (self.chunk_size, self.action_dim)
        if array.shape != expected or not np.isfinite(array).all():
            raise ValueError(f"prediction chunk must be finite {expected}, got {array.shape}")
        prediction = TemporalPrediction(
            int(query_id), int(query_timestamp_ns), int(query_command_tick), array.copy()
        )
        added = 0
        for horizon in range(self.max_source_horizon + 1):
            target_tick = prediction.query_command_tick + horizon
            self._by_tick.setdefault(target_tick, []).append((prediction, horizon))
            self._entry_count += 1
            added += 1
        self._stats["predictions_added"] += 1
        self._stats["prediction_points_added"] += added
        self._trim_capacity()
        return added

    def _trim_capacity(self) -> None:
        while self._entry_count > self.capacity:
            oldest_tick = min(self._by_tick)
            entries = self._by_tick[oldest_tick]
            entries.pop(0)
            self._entry_count -= 1
            self._stats["prediction_points_capacity_dropped"] += 1
            if not entries:
                del self._by_tick[oldest_tick]

    def _expire_before(self, command_tick: int) -> None:
        expired_ticks = [tick for tick in self._by_tick if tick < command_tick]
        for tick in expired_ticks:
            self._stats["prediction_points_expired"] += len(self._by_tick[tick])
            self._entry_count -= len(self._by_tick[tick])
            del self._by_tick[tick]

    def select(self, *, command_tick: int, command_timestamp_ns: int = 0) -> TemporalSelection:
        del command_timestamp_ns  # reserved for a future wall/monotonic age gate
        tick = int(command_tick)
        self._expire_before(tick)
        raw_candidates = self._by_tick.pop(tick, [])
        self._entry_count -= len(raw_candidates)
        candidates: list[tuple[TemporalPrediction, int]] = []
        for prediction, horizon in raw_candidates:
            age = tick - prediction.query_command_tick
            if self.max_prediction_age_ticks is not None and age > self.max_prediction_age_ticks:
                self._stats["prediction_points_age_expired"] += 1
            else:
                candidates.append((prediction, horizon))
        if candidates:
            # Match LeRobot/original ACT: the oldest query is index zero and
            # receives the largest weight for positive coefficient.
            candidates.sort(key=lambda item: (item[0].query_command_tick, item[0].query_id))
            unnormalized = np.exp(
                -self.coefficient * np.arange(len(candidates), dtype=np.float64)
            )
            weights = unnormalized / np.sum(unnormalized)
            contributors = tuple(
                TemporalContributor(
                    query_id=prediction.query_id,
                    query_timestamp_ns=prediction.query_timestamp_ns,
                    query_command_tick=prediction.query_command_tick,
                    source_horizon=horizon,
                    target_command_tick=tick,
                    predicted_action=prediction.chunk[horizon].copy(),
                    ensemble_weight=float(weight),
                )
                for (prediction, horizon), weight in zip(candidates, weights, strict=True)
            )
            action = np.average(
                np.stack([contributor.predicted_action for contributor in contributors]),
                axis=0,
                weights=weights,
            )
            self._last_action = np.asarray(action, dtype=np.float64)
            self._last_action_tick = tick
            self._stats["selected_ticks"] += 1
            return TemporalSelection(tick, self._last_action.copy(), contributors, False, None)
        self._stats["fallback_ticks"] += 1
        age = None if self._last_action_tick is None else tick - self._last_action_tick
        action = None if self._last_action is None else self._last_action.copy()
        return TemporalSelection(tick, action, (), True, age)

    def stats(self) -> dict[str, int | float]:
        return {**self._stats, "buffered_prediction_points": self._entry_count}
