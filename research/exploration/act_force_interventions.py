"""Dependency-light primitives for offline ACT ``FORCE_ACT`` interventions.

The functions in this module operate only on already-recorded arrays.  They do
not import the robot, teleoperation, camera, or actuator stacks.  ``FORCE_ACT``
values remain raw RH56 actuator/push-rod load counts; no contact location or SI
force meaning is assigned to them here.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from typing import Mapping, Sequence

import numpy as np


FORCE_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
FORCE_FEATURE_ORDER = tuple(f"rh56_force_{name}" for name in FORCE_ORDER)
FORCE_DIM = len(FORCE_ORDER)
HAND_Q_DIM = 6
ACTION_DIM = 12


def require_force_order(names: Sequence[str]) -> None:
    """Reject an ambiguous or reordered six-channel force declaration."""

    received = tuple(str(name) for name in names)
    if received not in (FORCE_ORDER, FORCE_FEATURE_ORDER):
        raise ValueError(
            "RH56 force order must be exactly "
            f"{','.join(FORCE_ORDER)}; got {','.join(received)}"
        )


def _matrix(value: np.ndarray, width: int, label: str) -> np.ndarray:
    array = np.asarray(value)
    if array.ndim != 2 or array.shape[1] != width:
        raise ValueError(f"{label} must have shape [N,{width}], got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError(f"{label} contains a non-finite value")
    return array


def _vector(value: np.ndarray, length: int, label: str) -> np.ndarray:
    array = np.asarray(value)
    if array.shape != (length,):
        raise ValueError(f"{label} must have shape [{length}], got {array.shape}")
    return array


@dataclass(frozen=True, slots=True)
class EpisodeSignals:
    """Causal recorded signals for one reviewed logical demonstration."""

    key: str
    source_episode: int
    session: str
    canonical_timestamp_ns: np.ndarray
    force_timestamp_ns: np.ndarray
    hand_q: np.ndarray
    force: np.ndarray
    action: np.ndarray

    def __post_init__(self) -> None:
        count = len(self.canonical_timestamp_ns)
        if not self.key:
            raise ValueError("episode key must be non-empty")
        if not self.session:
            raise ValueError(f"{self.key}: collection session must be non-empty")
        if count == 0:
            raise ValueError(f"{self.key}: episode must contain at least one row")
        canonical = _vector(
            self.canonical_timestamp_ns, count, f"{self.key}.canonical_timestamp_ns"
        ).astype(np.int64, copy=False)
        force_time = _vector(
            self.force_timestamp_ns, count, f"{self.key}.force_timestamp_ns"
        ).astype(np.int64, copy=False)
        hand_q = _matrix(self.hand_q, HAND_Q_DIM, f"{self.key}.hand_q")
        force = _matrix(self.force, FORCE_DIM, f"{self.key}.force")
        action = _matrix(self.action, ACTION_DIM, f"{self.key}.action")
        if len(hand_q) != count or len(force) != count or len(action) != count:
            raise ValueError(f"{self.key}: row counts do not agree")
        if np.any(np.diff(canonical) <= 0):
            raise ValueError(
                f"{self.key}: canonical timestamps must be unique and strictly increasing"
            )
        if np.any(np.diff(force_time) < 0):
            raise ValueError(f"{self.key}: force timestamps regress")
        if np.any(force_time > canonical):
            raise ValueError(f"{self.key}: a FORCE_ACT source is later than its observation")
        repeated = force_time[1:] == force_time[:-1]
        if np.any(force[1:][repeated] != force[:-1][repeated]):
            raise ValueError(f"{self.key}: cached FORCE_ACT rows disagree")


@dataclass(frozen=True, slots=True)
class QConditionedForceModel:
    """Small ridge model of raw actuator load conditioned on measured hand q."""

    coefficients: np.ndarray
    ridge: float

    def predict(self, hand_q: np.ndarray) -> np.ndarray:
        q = _matrix(np.asarray(hand_q, dtype=np.float64), HAND_Q_DIM, "hand_q")
        design = np.concatenate(
            (np.ones((len(q), 1), dtype=np.float64), q, q * q), axis=1
        )
        prediction = design @ self.coefficients
        if prediction.shape != (len(q), FORCE_DIM) or not np.isfinite(prediction).all():
            raise ValueError("q-conditioned force prediction is invalid")
        return prediction.astype(np.float32)


def fit_q_conditioned_force(
    hand_q: np.ndarray, force: np.ndarray, *, ridge: float = 1e-3
) -> QConditionedForceModel:
    """Fit an interpretable q/q-squared ridge baseline without sklearn."""

    q = _matrix(np.asarray(hand_q, dtype=np.float64), HAND_Q_DIM, "hand_q")
    target = _matrix(np.asarray(force, dtype=np.float64), FORCE_DIM, "force")
    if len(q) != len(target) or len(q) < 2:
        raise ValueError("q-conditioned fit requires at least two paired rows")
    if not np.isfinite(ridge) or ridge < 0:
        raise ValueError("ridge must be finite and non-negative")
    design = np.concatenate(
        (np.ones((len(q), 1), dtype=np.float64), q, q * q), axis=1
    )
    penalty = np.eye(design.shape[1], dtype=np.float64) * float(ridge)
    penalty[0, 0] = 0.0
    coefficients = np.linalg.solve(design.T @ design + penalty, design.T @ target)
    return QConditionedForceModel(coefficients=coefficients, ridge=float(ridge))


def _unique_force_rows(force_timestamp_ns: np.ndarray) -> np.ndarray:
    timestamps = np.asarray(force_timestamp_ns, dtype=np.int64)
    return np.flatnonzero(np.r_[True, timestamps[1:] != timestamps[:-1]])


def causal_time_shift(
    force: np.ndarray,
    force_timestamp_ns: np.ndarray,
    canonical_timestamp_ns: np.ndarray,
    *,
    lag_s: float,
    fill: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Shift force backward in time using actual episode-local source samples.

    Returned source timestamps are ``-1`` for fill rows.  Every non-negative
    source timestamp is from the same episode and is no later than the target
    observation timestamp minus ``lag_s``.  Repeated 30 Hz cached rows never
    become independent lag elements.
    """

    values = _matrix(np.asarray(force), FORCE_DIM, "force")
    count = len(values)
    force_time = _vector(force_timestamp_ns, count, "force_timestamp_ns").astype(
        np.int64, copy=False
    )
    canonical = _vector(
        canonical_timestamp_ns, count, "canonical_timestamp_ns"
    ).astype(np.int64, copy=False)
    fill_value = np.asarray(fill, dtype=values.dtype)
    if fill_value.shape != (FORCE_DIM,) or not np.isfinite(fill_value).all():
        raise ValueError(f"fill must be a finite [{FORCE_DIM}] vector")
    if not np.isfinite(lag_s) or lag_s <= 0:
        raise ValueError("lag_s must be finite and positive")
    if np.any(np.diff(canonical) <= 0) or np.any(np.diff(force_time) < 0):
        raise ValueError("lag inputs have invalid timestamp ordering")
    if np.any(force_time > canonical):
        raise ValueError("lag input contains a future FORCE_ACT source")
    repeated = force_time[1:] == force_time[:-1]
    if np.any(values[1:][repeated] != values[:-1][repeated]):
        raise ValueError("cached FORCE_ACT rows disagree")

    unique_rows = _unique_force_rows(force_time)
    unique_time = force_time[unique_rows]
    unique_force = values[unique_rows]
    lag_ns = int(round(float(lag_s) * 1e9))
    cutoff = canonical - lag_ns
    selected = np.searchsorted(unique_time, cutoff, side="right") - 1
    output = np.broadcast_to(fill_value, values.shape).copy()
    source_time = np.full(count, -1, dtype=np.int64)
    available = selected >= 0
    output[available] = unique_force[selected[available]]
    source_time[available] = unique_time[selected[available]]
    if np.any(source_time[available] > cutoff[available]):
        raise ValueError("causal lag selected a future FORCE_ACT source")
    return output, source_time


def _stable_seed(text: str, seed: int) -> int:
    digest = hashlib.sha256(f"{seed}:{text}".encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "little", signed=False)


def channel_derangement(*, seed: int) -> np.ndarray:
    """Return a deterministic six-channel permutation with no fixed channel."""

    rng = np.random.default_rng(int(seed))
    identity = np.arange(FORCE_DIM)
    for _ in range(100):
        candidate = rng.permutation(FORCE_DIM)
        if np.all(candidate != identity):
            return candidate
    raise RuntimeError("could not construct a channel derangement")


def dropout_mask(
    canonical_timestamp_ns: np.ndarray,
    *,
    episode_key: str,
    interval_s: float,
    period_s: float,
    seed: int,
) -> np.ndarray:
    """Construct deterministic contiguous missing-force intervals in time."""

    timestamp = np.asarray(canonical_timestamp_ns, dtype=np.int64)
    if timestamp.ndim != 1 or not len(timestamp) or np.any(np.diff(timestamp) <= 0):
        raise ValueError("dropout timestamps must be non-empty and strictly increasing")
    if (
        not np.isfinite(interval_s)
        or not np.isfinite(period_s)
        or interval_s <= 0
        or period_s <= interval_s
    ):
        raise ValueError("dropout requires finite 0 < interval_s < period_s")
    interval_ns = int(round(float(interval_s) * 1e9))
    period_ns = int(round(float(period_s) * 1e9))
    rng = np.random.default_rng(_stable_seed(episode_key, seed))
    offset_ns = int(rng.integers(0, period_ns))
    elapsed = timestamp - timestamp[0]
    active = elapsed >= offset_ns
    return active & (((elapsed - offset_ns) % period_ns) < interval_ns)


def session_derangement(
    episodes: Sequence[EpisodeSignals], *, seed: int
) -> dict[str, str]:
    """Pair episodes one-to-one only inside a physical collection session.

    Targets with no valid same-session donor from a different source episode
    are omitted.  No fallback crosses a session or reuses the target's source
    episode.
    """

    if len({episode.key for episode in episodes}) != len(episodes):
        raise ValueError("episode keys must be unique")
    groups: dict[str, list[EpisodeSignals]] = {}
    for episode in episodes:
        groups.setdefault(episode.session, []).append(episode)
    result: dict[str, str] = {}
    for session, members in sorted(groups.items()):
        ordered = sorted(members, key=lambda item: item.key)
        if len(ordered) < 2:
            continue
        rng = np.random.default_rng(_stable_seed(session, seed))
        target_order = list(rng.permutation(len(ordered)))
        candidate_order: dict[int, list[int]] = {}
        for target_index in target_order:
            candidates = [
                donor_index
                for donor_index, donor in enumerate(ordered)
                if donor.key != ordered[target_index].key
                and donor.source_episode != ordered[target_index].source_episode
            ]
            rng.shuffle(candidates)
            candidate_order[target_index] = candidates

        assigned_donor_to_target: dict[int, int] = {}

        def assign(target_index: int, seen: set[int]) -> bool:
            for donor_index in candidate_order[target_index]:
                if donor_index in seen:
                    continue
                seen.add(donor_index)
                previous = assigned_donor_to_target.get(donor_index)
                if previous is None or assign(previous, seen):
                    assigned_donor_to_target[donor_index] = target_index
                    return True
            return False

        feasible = all(assign(target_index, set()) for target_index in target_order)
        if not feasible or len(assigned_donor_to_target) != len(ordered):
            continue
        for donor_index, target_index in assigned_donor_to_target.items():
            result[ordered[target_index].key] = ordered[donor_index].key
    return result


def progress_resample(donor_values: np.ndarray, target_count: int) -> np.ndarray:
    """Nearest-neighbour resampling by within-demonstration relative progress."""

    donor = _matrix(np.asarray(donor_values), FORCE_DIM, "donor force")
    if target_count <= 0:
        raise ValueError("target_count must be positive")
    if target_count == 1:
        indices = np.asarray([0], dtype=np.int64)
    else:
        indices = np.rint(
            np.linspace(0, len(donor) - 1, num=target_count, dtype=np.float64)
        ).astype(np.int64)
    return donor[indices].copy()


def action_chunk(
    episode_actions: np.ndarray, *, start: int, horizon: int
) -> tuple[np.ndarray, np.ndarray]:
    """Build a repeat-last action chunk without crossing an episode boundary."""

    actions = _matrix(np.asarray(episode_actions), ACTION_DIM, "episode actions")
    if start < 0 or start >= len(actions):
        raise ValueError("action chunk start is outside the episode")
    if horizon <= 0:
        raise ValueError("action chunk horizon must be positive")
    stop = min(len(actions), start + horizon)
    valid_count = stop - start
    chunk = np.empty((horizon, ACTION_DIM), dtype=actions.dtype)
    chunk[:valid_count] = actions[start:stop]
    chunk[valid_count:] = actions[stop - 1]
    is_pad = np.arange(horizon) >= valid_count
    return chunk, is_pad


def masked_action_metrics(
    prediction: np.ndarray, target: np.ndarray, valid: np.ndarray
) -> dict[str, object]:
    """Compute native-action errors while excluding repeat-last padding."""

    predicted = np.asarray(prediction, dtype=np.float64)
    truth = np.asarray(target, dtype=np.float64)
    mask = np.asarray(valid, dtype=bool)
    if predicted.shape != truth.shape or predicted.ndim != 3:
        raise ValueError("prediction and target must share shape [N,H,A]")
    if predicted.shape[2] != ACTION_DIM or mask.shape != predicted.shape[:2]:
        raise ValueError("invalid action or validity-mask shape")
    if not np.isfinite(predicted).all() or not np.isfinite(truth).all():
        raise ValueError("action arrays contain a non-finite value")
    count = int(mask.sum())
    if count == 0:
        raise ValueError("masked metric has no valid action targets")
    error = predicted - truth
    selected = error[mask]
    absolute = np.abs(selected)
    squared = selected * selected
    return {
        "valid_targets": count,
        "mae": float(absolute.mean()),
        "mse": float(squared.mean()),
        "jaka_mae": float(absolute[:, :6].mean()),
        "rh56_mae": float(absolute[:, 6:].mean()),
        "per_dimension_mae": absolute.mean(axis=0).tolist(),
    }


def masked_prediction_sensitivity(
    prediction: np.ndarray, factual: np.ndarray, valid: np.ndarray
) -> dict[str, object]:
    """Measure action change from factual force on the same observations."""

    result = masked_action_metrics(prediction, factual, valid)
    return {
        "valid_predictions": result["valid_targets"],
        "mean_absolute_change": result["mae"],
        "mean_squared_change": result["mse"],
        "jaka_mean_absolute_change": result["jaka_mae"],
        "rh56_mean_absolute_change": result["rh56_mae"],
        "per_dimension_mean_absolute_change": result["per_dimension_mae"],
    }


@dataclass(frozen=True, slots=True)
class InterventionSet:
    """Raw, pre-preprocessor force arrays and their audit metadata."""

    values: Mapping[str, Mapping[str, np.ndarray]]
    eligible: Mapping[str, frozenset[str]]
    changed_rows: Mapping[str, Mapping[str, np.ndarray]]
    metadata: Mapping[str, object]


def build_interventions(
    episodes: Sequence[EpisodeSignals],
    *,
    training_episode_keys: Sequence[str],
    lag_s: float,
    dropout_interval_s: float,
    dropout_period_s: float,
    q_ridge: float,
    seed: int,
) -> InterventionSet:
    """Build the complete controlled intervention family from recorded arrays."""

    by_key = {episode.key: episode for episode in episodes}
    if len(by_key) != len(episodes):
        raise ValueError("episode keys must be unique")
    missing = set(training_episode_keys) - set(by_key)
    if missing:
        raise ValueError(f"training episodes are missing: {sorted(missing)}")
    train = [by_key[key] for key in training_episode_keys]
    if not train:
        raise ValueError("at least one training episode is required")
    train_force = np.concatenate(
        [episode.force[_unique_force_rows(episode.force_timestamp_ns)] for episode in train]
    ).astype(np.float64, copy=False)
    train_q = np.concatenate(
        [episode.hand_q[_unique_force_rows(episode.force_timestamp_ns)] for episode in train]
    ).astype(np.float64, copy=False)
    train_mean = train_force.mean(axis=0).astype(np.float32)
    q_model = fit_q_conditioned_force(train_q, train_force, ridge=q_ridge)
    permutation = channel_derangement(seed=seed)
    donor_map = session_derangement(episodes, seed=seed)
    names = (
        "factual",
        "train_mean",
        "raw_zero",
        "causal_time_shift",
        "session_shuffle",
        "q_conditioned",
        "channel_permutation",
        "dropout",
    )
    values: dict[str, dict[str, np.ndarray]] = {name: {} for name in names}
    changed: dict[str, dict[str, np.ndarray]] = {name: {} for name in names}
    eligible: dict[str, set[str]] = {name: set() for name in names}
    lag_source_timestamps: dict[str, list[int]] = {}
    dropout_rows: dict[str, list[int]] = {}
    for episode in episodes:
        factual = np.asarray(episode.force, dtype=np.float32)
        episode_values: dict[str, np.ndarray] = {
            "factual": factual.copy(),
            "train_mean": np.broadcast_to(train_mean, factual.shape).copy(),
            "raw_zero": np.zeros_like(factual),
            "q_conditioned": q_model.predict(episode.hand_q),
            "channel_permutation": factual[:, permutation].copy(),
        }
        lagged, lag_times = causal_time_shift(
            factual,
            episode.force_timestamp_ns,
            episode.canonical_timestamp_ns,
            lag_s=lag_s,
            fill=train_mean,
        )
        episode_values["causal_time_shift"] = lagged.astype(np.float32, copy=False)
        lag_source_timestamps[episode.key] = lag_times.tolist()
        dropout = dropout_mask(
            episode.canonical_timestamp_ns,
            episode_key=episode.key,
            interval_s=dropout_interval_s,
            period_s=dropout_period_s,
            seed=seed,
        )
        dropped = factual.copy()
        dropped[dropout] = train_mean
        episode_values["dropout"] = dropped
        dropout_rows[episode.key] = np.flatnonzero(dropout).tolist()
        donor_key = donor_map.get(episode.key)
        if donor_key is not None:
            episode_values["session_shuffle"] = progress_resample(
                by_key[donor_key].force, len(factual)
            ).astype(np.float32, copy=False)
        for name, intervention in episode_values.items():
            values[name][episode.key] = intervention
            changed[name][episode.key] = np.any(intervention != factual, axis=1)
            eligible[name].add(episode.key)
    metadata: dict[str, object] = {
        "force_order": list(FORCE_ORDER),
        "force_semantics": "raw RH56 FORCE_ACT actuator/push-rod load counts",
        "intervention_location": "raw observation, before checkpoint preprocessing",
        "train_force_mean": train_mean.tolist(),
        "q_conditioned_features": [
            "intercept",
            *[f"q_{name}" for name in FORCE_ORDER],
            *[f"q_{name}_squared" for name in FORCE_ORDER],
        ],
        "q_conditioned_ridge": float(q_ridge),
        "training_force_weighting": "one row per unique FORCE_ACT source update",
        "channel_permutation_output_from_input": {
            FORCE_ORDER[output]: FORCE_ORDER[int(source)]
            for output, source in enumerate(permutation)
        },
        "causal_lag_s": float(lag_s),
        "causal_lag_source_timestamp_ns": lag_source_timestamps,
        "dropout_interval_s": float(dropout_interval_s),
        "dropout_period_s": float(dropout_period_s),
        "dropout_row_indices": dropout_rows,
        "session_shuffle_donor": donor_map,
        "session_shuffle_alignment": "nearest row at equal within-episode relative progress",
        "session_shuffle_ineligible": sorted(set(by_key) - set(donor_map)),
        "seed": int(seed),
    }
    return InterventionSet(
        values=values,
        eligible={name: frozenset(keys) for name, keys in eligible.items()},
        changed_rows=changed,
        metadata=metadata,
    )
