"""Causal fixed-width histories of RH56 native actuator-load feedback.

The physical-bottle master is sampled at a canonical row rate (currently
30 Hz), while ``FORCE_ACT`` is polled independently (currently about 10 Hz).
Several canonical rows therefore carry the same zero-order-held source sample.
This module first recovers episode-local *unique source updates* and only then
constructs histories.  Repeated canonical rows never become extra history
elements.

``FORCE_ACT`` remains six signed raw actuator/push-rod load-register counts.
The normalized feature below is an actuator-load *magnitude share*; it is not a
fingertip-force distribution, a tactile map, or contact localization.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


RH56_CHANNEL_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
FORCE_DIM = len(RH56_CHANNEL_ORDER)
SUPPORTED_FEATURE_GROUPS = (
    "raw_load",
    "load_delta",
    "source_age_s",
    "valid",
    "baseline_corrected_load",
    "actuator_load_magnitude_share",
)


def _timestamps(value: np.ndarray, label: str, *, strictly_increasing: bool) -> np.ndarray:
    array = np.asarray(value, dtype=np.int64)
    if array.ndim != 1 or not len(array):
        raise ValueError(f"{label} must be a non-empty timestamp vector")
    difference = np.diff(array)
    if (strictly_increasing and np.any(difference <= 0)) or (
        not strictly_increasing and np.any(difference < 0)
    ):
        qualifier = "strictly increasing" if strictly_increasing else "nondecreasing"
        raise ValueError(f"{label} must be {qualifier}")
    return array


def _force_matrix(value: np.ndarray, count: int, label: str) -> np.ndarray:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (count, FORCE_DIM):
        raise ValueError(f"{label} must have shape [{count},{FORCE_DIM}], got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{label} contains a non-finite value")
    return array


@dataclass(frozen=True, slots=True)
class UniqueForceSources:
    """Episode-local source updates recovered from zero-order-held rows."""

    timestamp_ns: np.ndarray
    load: np.ndarray
    valid: np.ndarray
    first_canonical_row: np.ndarray


@dataclass(frozen=True, slots=True)
class CausalForceHistory:
    """Oldest-to-newest fixed-width force histories for canonical query rows."""

    query_timestamp_ns: np.ndarray
    source_timestamp_ns: np.ndarray
    source_age_s: np.ndarray
    present: np.ndarray
    valid: np.ndarray
    raw_load: np.ndarray
    load_delta: np.ndarray
    baseline_corrected_load: np.ndarray
    actuator_load_magnitude_share: np.ndarray

    @property
    def history_updates(self) -> int:
        return int(self.raw_load.shape[1])


def unique_force_sources(
    canonical_timestamp_ns: np.ndarray,
    force_timestamp_ns: np.ndarray,
    force_valid: np.ndarray,
    force: np.ndarray,
) -> UniqueForceSources:
    """Recover unique source updates and verify zero-order-hold consistency.

    Inputs are one logical episode only.  Every row sharing a force source
    timestamp must carry exactly the same raw values and validity bit; otherwise
    the dataset cannot truthfully be interpreted as a cached source sample.
    """

    canonical = _timestamps(
        canonical_timestamp_ns, "canonical_timestamp_ns", strictly_increasing=True
    )
    source = _timestamps(
        force_timestamp_ns, "force_timestamp_ns", strictly_increasing=False
    )
    if len(source) != len(canonical):
        raise ValueError("canonical and force timestamp row counts differ")
    valid = np.asarray(force_valid, dtype=np.bool_)
    if valid.shape != canonical.shape:
        raise ValueError("force_valid must match the canonical row count")
    values = _force_matrix(force, len(canonical), "force")
    if np.any(source > canonical):
        raise ValueError("a FORCE_ACT source timestamp is later than its canonical row")

    repeated = source[1:] == source[:-1]
    if np.any(valid[1:][repeated] != valid[:-1][repeated]):
        raise ValueError("cached rows disagree on FORCE_ACT validity")
    if np.any(values[1:][repeated] != values[:-1][repeated]):
        raise ValueError("cached rows disagree on FORCE_ACT values")

    first = np.r_[True, source[1:] != source[:-1]]
    rows = np.flatnonzero(first)
    unique_timestamp = source[rows]
    if np.any(np.diff(unique_timestamp) <= 0):
        raise ValueError("unique FORCE_ACT source timestamps are not strictly increasing")
    return UniqueForceSources(
        timestamp_ns=unique_timestamp,
        load=values[rows],
        valid=valid[rows],
        first_canonical_row=rows,
    )


def build_causal_force_history(
    sources: UniqueForceSources,
    query_timestamp_ns: np.ndarray,
    *,
    history_updates: int,
    baseline_load: np.ndarray | None = None,
) -> CausalForceHistory:
    """Build histories from actual source updates without future leakage.

    ``history_updates`` counts unique source updates, not canonical dataset
    rows.  Prefix positions without enough same-episode history are zero-padded
    and marked ``present=False``.  Invalid source samples are retained in the
    timestamp audit but zeroed in numeric features and marked ``valid=False``.
    """

    if history_updates < 1:
        raise ValueError("history_updates must be positive")
    source_time = _timestamps(
        sources.timestamp_ns, "sources.timestamp_ns", strictly_increasing=True
    )
    source_load = _force_matrix(sources.load, len(source_time), "sources.load")
    source_valid = np.asarray(sources.valid, dtype=np.bool_)
    if source_valid.shape != source_time.shape:
        raise ValueError("sources.valid must match the source count")
    query = _timestamps(
        query_timestamp_ns, "query_timestamp_ns", strictly_increasing=True
    )
    baseline = (
        np.zeros(FORCE_DIM, dtype=np.float64)
        if baseline_load is None
        else np.asarray(baseline_load, dtype=np.float64)
    )
    if baseline.shape != (FORCE_DIM,) or not np.isfinite(baseline).all():
        raise ValueError(f"baseline_load must be a finite [{FORCE_DIM}] vector")

    latest = np.searchsorted(source_time, query, side="right") - 1
    offsets = np.arange(history_updates - 1, -1, -1, dtype=np.int64)
    indices = latest[:, None] - offsets[None, :]
    present = indices >= 0
    safe_indices = np.maximum(indices, 0)
    history_timestamp = np.where(present, source_time[safe_indices], -1)
    valid = present & source_valid[safe_indices]
    if np.any(history_timestamp[present] > np.broadcast_to(query[:, None], present.shape)[present]):
        raise AssertionError("future FORCE_ACT source entered a causal history")

    raw = np.zeros((len(query), history_updates, FORCE_DIM), dtype=np.float64)
    raw[valid] = source_load[safe_indices[valid]]
    centered = np.zeros_like(raw)
    centered[valid] = raw[valid] - baseline

    delta = np.zeros_like(raw)
    adjacent_valid = valid[:, 1:] & valid[:, :-1]
    pair_delta = raw[:, 1:] - raw[:, :-1]
    delta[:, 1:][adjacent_valid] = pair_delta[adjacent_valid]

    magnitude = np.abs(centered)
    magnitude_sum = magnitude.sum(axis=2, keepdims=True)
    share = np.divide(
        magnitude,
        magnitude_sum,
        out=np.zeros_like(magnitude),
        where=magnitude_sum > 0,
    )
    share[~valid] = 0.0

    age = np.zeros((len(query), history_updates), dtype=np.float64)
    query_grid = np.broadcast_to(query[:, None], age.shape)
    age[present] = (query_grid[present] - history_timestamp[present]) / 1e9
    if np.any(age[present] < 0):
        raise AssertionError("negative source age entered a causal history")

    return CausalForceHistory(
        query_timestamp_ns=query,
        source_timestamp_ns=history_timestamp,
        source_age_s=age,
        present=present,
        valid=valid,
        raw_load=raw,
        load_delta=delta,
        baseline_corrected_load=centered,
        actuator_load_magnitude_share=share,
    )


def flatten_force_history(
    history: CausalForceHistory,
    feature_groups: Sequence[str],
) -> tuple[np.ndarray, tuple[str, ...]]:
    """Flatten selected oldest-to-newest feature groups for a derived view."""

    groups = tuple(str(group) for group in feature_groups)
    if not groups or len(set(groups)) != len(groups):
        raise ValueError("feature_groups must be a non-empty sequence without duplicates")
    unsupported = sorted(set(groups) - set(SUPPORTED_FEATURE_GROUPS))
    if unsupported:
        raise ValueError(f"unsupported force-history feature groups: {unsupported}")

    matrices: list[np.ndarray] = []
    names: list[str] = []
    for group in groups:
        value = np.asarray(getattr(history, group))
        if value.ndim == 2:
            value = value[:, :, None]
            suffixes = (group,)
        else:
            suffixes = tuple(f"{group}_{channel}" for channel in RH56_CHANNEL_ORDER)
        matrices.append(value.reshape(len(value), -1).astype(np.float32))
        for lag in range(history.history_updates - 1, -1, -1):
            names.extend(f"force_history_t_minus_{lag}_{suffix}" for suffix in suffixes)
    return np.concatenate(matrices, axis=1), tuple(names)
