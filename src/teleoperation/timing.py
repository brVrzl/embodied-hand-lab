from __future__ import annotations

import math
import statistics
from collections.abc import Sequence

from .contracts import TimingStatistics


def _percentile(sorted_values: Sequence[float], percentile: float) -> float:
    if not sorted_values:
        raise ValueError("cannot calculate percentile of empty values")
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    rank = (len(sorted_values) - 1) * percentile
    lower = math.floor(rank)
    upper = math.ceil(rank)
    if lower == upper:
        return float(sorted_values[lower])
    weight = rank - lower
    return float(sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight)


def timing_statistics(
    name: str,
    values: Sequence[int | float],
    *,
    unit: str = "ns",
    requested_period_ns: int | None = None,
    deadline_ns: int | None = None,
) -> TimingStatistics:
    if not values:
        raise ValueError("timing values may not be empty")
    samples = [float(value) for value in values]
    if not all(math.isfinite(value) and value >= 0.0 for value in samples):
        raise ValueError("timing values must be finite and non-negative")
    ordered = sorted(samples)
    deadline = requested_period_ns if deadline_ns is None else deadline_ns
    misses = 0
    consecutive = 0
    max_consecutive = 0
    if deadline is not None:
        for value in samples:
            if value > deadline:
                misses += 1
                consecutive += 1
                max_consecutive = max(max_consecutive, consecutive)
            else:
                consecutive = 0
    return TimingStatistics(
        name=name,
        unit=unit,
        count=len(samples),
        requested_period_ns=requested_period_ns,
        mean=statistics.fmean(samples),
        median=statistics.median(samples),
        stddev=statistics.pstdev(samples),
        minimum=ordered[0],
        maximum=ordered[-1],
        p95=_percentile(ordered, 0.95),
        p99=_percentile(ordered, 0.99),
        p999=_percentile(ordered, 0.999) if len(samples) >= 1_000 else None,
        missed_deadlines=misses,
        max_consecutive_missed_deadlines=max_consecutive,
    )
