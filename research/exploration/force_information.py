"""Dependency-light probes for causal RH56 actuator-load information.

The module reads an already materialized physical-bottle dataset and never
imports the collection, teleoperation, or hardware stacks.  ``FORCE_ACT`` is
treated only as six signed raw actuator/push-rod load counts.  In particular,
none of the numeric targets below are contact, stability, or slip labels.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import pyarrow.parquet as parquet
from scipy.optimize import minimize


RH56_CHANNEL_ORDER = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
FORCE_ORDER = tuple(f"rh56_force_{name}" for name in RH56_CHANNEL_ORDER)
EXPECTED_FORCE_UNITS = "rh56_force_act_raw_count"


@dataclass(frozen=True, slots=True)
class ForceEpisode:
    """One logical trajectory sampled once per new causal FORCE_ACT source."""

    logical_segment_id: str
    logical_episode_index: int
    source_episode_index: int
    collection_session_id: str
    canonical_timestamp_ns: np.ndarray
    force_timestamp_ns: np.ndarray
    force: np.ndarray
    rh56_position: np.ndarray
    rh56_target: np.ndarray
    action_status: np.ndarray
    canonical_row_count: int = 0
    canonical_duration_s: float = 0.0
    row_force_age_ns: np.ndarray | None = None
    row_force_valid: np.ndarray | None = None
    row_action_status: np.ndarray | None = None


@dataclass(frozen=True, slots=True)
class ProbeConfig:
    initial_center_updates: int = 10
    history_updates: tuple[int, ...] = (3, 6, 11)
    future_horizon_updates: int = 2
    large_change_quantile: float = 0.8
    ridge_l2: float = 10.0
    logistic_l2: float = 10.0

    def validate(self) -> None:
        if self.initial_center_updates < 1:
            raise ValueError("initial_center_updates must be positive")
        history = tuple(int(value) for value in self.history_updates)
        if not history or any(value < 2 for value in history):
            raise ValueError("history_updates must contain values of at least two")
        if tuple(sorted(set(history))) != history:
            raise ValueError("history_updates must be strictly increasing")
        if self.future_horizon_updates < 1:
            raise ValueError("future_horizon_updates must be positive")
        if not 0.5 < self.large_change_quantile < 1.0:
            raise ValueError("large_change_quantile must be within (0.5, 1)")
        if self.ridge_l2 < 0 or self.logistic_l2 < 0:
            raise ValueError("regularization strengths must be non-negative")


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _collection_session_id(metadata: dict[str, Any], source_episode: int) -> str:
    paths = metadata.get("physical_log_paths")
    event_path = paths.get("combined_events") if isinstance(paths, dict) else None
    if not isinstance(event_path, str) or not event_path:
        raise ValueError(
            f"source episode {source_episode} lacks physical_log_paths.combined_events"
        )
    # The full path is host-specific; the basename is the stable session key
    # shared by every logical trajectory collected in that physical session.
    return Path(event_path).name


def load_force_episodes(master: Path, source_metadata_root: Path) -> list[ForceEpisode]:
    """Load and validate the rich master at unique causal force timestamps.

    The first canonical row carrying a new source timestamp is retained.  This
    avoids counting the 30 Hz zero-order-held rows as independent load samples.
    """

    master = master.resolve()
    source_metadata_root = source_metadata_root.resolve()
    info = _read_json(master / "meta/info.json")
    if tuple(info.get("rh56_channel_order", ())) != RH56_CHANNEL_ORDER:
        raise ValueError("unexpected RH56 channel order")
    if tuple(info.get("force_order", ())) != FORCE_ORDER:
        raise ValueError("unexpected FORCE_ACT channel order")
    if info.get("observation_force_units") != EXPECTED_FORCE_UNITS:
        raise ValueError("unexpected FORCE_ACT units")

    records = [
        json.loads(line)
        for line in (master / "meta/episodes.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    episodes: list[ForceEpisode] = []
    for record in records:
        table = parquet.read_table(
            master / str(record["data"]),
            columns=[
                "episode_index",
                "logical_segment_id",
                "source_episode_index",
                "timestamp_ns",
                "force_timestamp_ns",
                "force_valid",
                "observation.force",
                "observation.state",
                "action",
                "action_status",
            ],
        ).to_pydict()
        row_count = len(table["timestamp_ns"])
        if row_count != int(record["length"]):
            raise ValueError(f"{record['logical_segment_id']}: row count mismatch")
        canonical = np.asarray(table["timestamp_ns"], dtype=np.int64)
        source = np.asarray(table["force_timestamp_ns"], dtype=np.int64)
        valid = np.asarray(table["force_valid"], dtype=np.bool_)
        force = np.asarray(table["observation.force"], dtype=np.float64)
        state = np.asarray(table["observation.state"], dtype=np.float64)
        action = np.asarray(table["action"], dtype=np.float64)
        if force.shape != (row_count, 6) or state.shape != (row_count, 12):
            raise ValueError(f"{record['logical_segment_id']}: invalid observation shape")
        if action.shape != (row_count, 12):
            raise ValueError(f"{record['logical_segment_id']}: invalid action shape")
        if not valid.all():
            raise ValueError(f"{record['logical_segment_id']}: invalid force row")
        if np.any(source > canonical):
            raise ValueError(f"{record['logical_segment_id']}: future force source")
        if not np.all(np.diff(canonical) > 0) or np.any(np.diff(source) < 0):
            raise ValueError(f"{record['logical_segment_id']}: timestamp regression")
        if not (np.isfinite(force).all() and np.isfinite(state).all() and np.isfinite(action).all()):
            raise ValueError(f"{record['logical_segment_id']}: non-finite numeric value")

        first_for_source = np.r_[True, source[1:] != source[:-1]]
        selected = np.flatnonzero(first_for_source)
        selected_source = source[selected]
        if not np.all(np.diff(selected_source) > 0):
            raise ValueError(f"{record['logical_segment_id']}: force source is not unique")

        source_episode = int(record["source_episode_index"])
        metadata = _read_json(
            source_metadata_root / f"episode_{source_episode:06d}.json"
        )
        episodes.append(
            ForceEpisode(
                logical_segment_id=str(record["logical_segment_id"]),
                logical_episode_index=int(record["episode_index"]),
                source_episode_index=source_episode,
                collection_session_id=_collection_session_id(metadata, source_episode),
                canonical_timestamp_ns=canonical[selected],
                force_timestamp_ns=selected_source,
                force=force[selected],
                rh56_position=state[selected, 6:12],
                rh56_target=action[selected, 6:12],
                action_status=np.asarray(table["action_status"], dtype=np.str_)[selected],
                canonical_row_count=row_count,
                canonical_duration_s=float((canonical[-1] - canonical[0]) / 1e9),
                row_force_age_ns=canonical - source,
                row_force_valid=valid,
                row_action_status=np.asarray(table["action_status"], dtype=np.str_),
            )
        )
    if not episodes:
        raise ValueError("master contains no episodes")
    return episodes


def causal_history_indices(
    source_timestamp_ns: np.ndarray,
    query_timestamp_ns: np.ndarray,
    history_length: int,
) -> np.ndarray:
    """Return oldest-to-newest causal source indices for each query.

    Rows without a complete episode-local history are returned as all ``-1``.
    The caller supplies one episode at a time, which makes crossing an episode
    boundary impossible by construction.
    """

    source = np.asarray(source_timestamp_ns, dtype=np.int64)
    queries = np.asarray(query_timestamp_ns, dtype=np.int64)
    if history_length < 1:
        raise ValueError("history_length must be positive")
    if source.ndim != 1 or queries.ndim != 1 or np.any(np.diff(source) <= 0):
        raise ValueError("source timestamps must be a strictly increasing vector")
    latest = np.searchsorted(source, queries, side="right") - 1
    offsets = np.arange(history_length - 1, -1, -1, dtype=np.int64)
    result = latest[:, None] - offsets[None, :]
    incomplete = result[:, 0] < 0
    result[incomplete] = -1
    if np.any(result[~incomplete, -1] < 0):
        raise AssertionError("causal history construction failed")
    return result


def session_leave_one_out(
    episodes: Sequence[ForceEpisode],
) -> list[tuple[str, list[ForceEpisode], list[ForceEpisode]]]:
    sessions = sorted({episode.collection_session_id for episode in episodes})
    return [
        (
            session,
            [episode for episode in episodes if episode.collection_session_id != session],
            [episode for episode in episodes if episode.collection_session_id == session],
        )
        for session in sessions
    ]


def _centered_force(episode: ForceEpisode, count: int) -> np.ndarray:
    # This is only an initial-window offset correction.  It is deliberately not
    # named a no-contact baseline because the data has no contact labels.
    baseline = np.median(episode.force[: min(count, len(episode.force))], axis=0)
    return episode.force - baseline


def _update_arrays(
    episodes: Sequence[ForceEpisode], config: ProbeConfig, *, accepted_only: bool
) -> tuple[np.ndarray, ...]:
    sessions: list[str] = []
    positions: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    position_delta: list[np.ndarray] = []
    target_delta: list[np.ndarray] = []
    forces: list[np.ndarray] = []
    for episode in episodes:
        mask = (
            episode.action_status == "accepted"
            if accepted_only
            else np.ones(len(episode.force), dtype=np.bool_)
        )
        centered = _centered_force(episode, config.initial_center_updates)
        dq = np.vstack([np.zeros((1, 6)), np.diff(episode.rh56_position, axis=0)])
        da = np.vstack([np.zeros((1, 6)), np.diff(episode.rh56_target, axis=0)])
        sessions.extend([episode.collection_session_id] * int(mask.sum()))
        positions.append(episode.rh56_position[mask])
        targets.append(episode.rh56_target[mask])
        position_delta.append(dq[mask])
        target_delta.append(da[mask])
        forces.append(centered[mask])
    return (
        np.asarray(sessions, dtype=np.str_),
        np.concatenate(positions),
        np.concatenate(targets),
        np.concatenate(position_delta),
        np.concatenate(target_delta),
        np.concatenate(forces),
    )


def _history_samples(
    episodes: Sequence[ForceEpisode], config: ProbeConfig, *, accepted_only: bool
) -> dict[str, np.ndarray]:
    result: dict[str, list[Any]] = {
        "session": [],
        "query_timestamp_ns": [],
        "latest_force_timestamp_ns": [],
        "history_force_timestamp_ns": [],
        "q_history": [],
        "force_history": [],
        "future_force": [],
        "future_hand_target": [],
        "future_change": [],
    }
    history = max(config.history_updates)
    horizon = config.future_horizon_updates
    for episode in episodes:
        centered = _centered_force(episode, config.initial_center_updates)
        for current in range(history - 1, len(centered) - horizon):
            if accepted_only and episode.action_status[current] != "accepted":
                continue
            oldest = current - history + 1
            future = current + horizon
            history_timestamps = episode.force_timestamp_ns[oldest : current + 1]
            if np.any(history_timestamps > episode.canonical_timestamp_ns[current]):
                raise ValueError("future force entered a feature history")
            result["session"].append(episode.collection_session_id)
            result["query_timestamp_ns"].append(episode.canonical_timestamp_ns[current])
            result["latest_force_timestamp_ns"].append(episode.force_timestamp_ns[current])
            result["history_force_timestamp_ns"].append(history_timestamps)
            result["q_history"].append(episode.rh56_position[oldest : current + 1])
            result["force_history"].append(centered[oldest : current + 1])
            result["future_force"].append(centered[future])
            result["future_hand_target"].append(episode.rh56_target[future])
            result["future_change"].append(
                float(np.linalg.norm(centered[future] - centered[current]))
            )
    return {key: np.asarray(value) for key, value in result.items()}


@dataclass(frozen=True, slots=True)
class _LinearModel:
    input_mean: np.ndarray
    input_scale: np.ndarray
    target_mean: np.ndarray
    target_scale: np.ndarray
    weight: np.ndarray


def _ridge_fit(x: np.ndarray, y: np.ndarray, l2: float) -> _LinearModel:
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    input_mean = x.mean(axis=0)
    input_scale = x.std(axis=0)
    input_scale[input_scale < 1e-10] = 1.0
    target_mean = y.mean(axis=0)
    target_scale = y.std(axis=0)
    target_scale[target_scale < 1e-10] = 1.0
    standardized_x = (x - input_mean) / input_scale
    standardized_y = (y - target_mean) / target_scale
    weight = np.linalg.solve(
        standardized_x.T @ standardized_x + l2 * np.eye(standardized_x.shape[1]),
        standardized_x.T @ standardized_y,
    )
    return _LinearModel(input_mean, input_scale, target_mean, target_scale, weight)


def _ridge_predict(model: _LinearModel, x: np.ndarray) -> np.ndarray:
    standardized = (np.asarray(x, dtype=np.float64) - model.input_mean) / model.input_scale
    return (standardized @ model.weight) * model.target_scale + model.target_mean


def _r2(target: np.ndarray, prediction: np.ndarray) -> tuple[float, np.ndarray]:
    target = np.asarray(target, dtype=np.float64)
    prediction = np.asarray(prediction, dtype=np.float64)
    error = np.sum(np.square(target - prediction), axis=0)
    variance = np.sum(np.square(target - target.mean(axis=0)), axis=0)
    normalized_error = np.full(error.shape, np.nan, dtype=np.float64)
    np.divide(error, variance, out=normalized_error, where=variance > 0)
    per_channel = 1.0 - normalized_error
    pooled_variance = float(variance.sum())
    pooled = float(1.0 - error.sum() / pooled_variance) if pooled_variance > 0 else float("nan")
    return pooled, per_channel


@dataclass(frozen=True, slots=True)
class _LogisticModel:
    input_mean: np.ndarray
    input_scale: np.ndarray
    weight: np.ndarray


def _logistic_fit(x: np.ndarray, y: np.ndarray, l2: float) -> _LogisticModel:
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    input_mean = x.mean(axis=0)
    input_scale = x.std(axis=0)
    input_scale[input_scale < 1e-10] = 1.0
    design = np.c_[np.ones(len(x)), (x - input_mean) / input_scale]
    positives = float(y.sum())
    negatives = float(len(y) - positives)
    if positives == 0 or negatives == 0:
        raise ValueError("logistic target needs both classes")
    sample_weight = np.where(
        y > 0,
        len(y) / (2.0 * positives),
        len(y) / (2.0 * negatives),
    )

    def objective(weight: np.ndarray) -> tuple[float, np.ndarray]:
        logits = design @ weight
        probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -50.0, 50.0)))
        loss = float(
            np.mean(sample_weight * (np.logaddexp(0.0, logits) - y * logits))
            + 0.5 * l2 * np.square(weight[1:]).sum() / len(y)
        )
        gradient = design.T @ (sample_weight * (probability - y)) / len(y)
        gradient[1:] += l2 * weight[1:] / len(y)
        return loss, gradient

    result = minimize(
        objective,
        np.zeros(design.shape[1]),
        jac=True,
        method="L-BFGS-B",
        options={"maxiter": 400},
    )
    if not result.success:
        raise RuntimeError(f"logistic optimization failed: {result.message}")
    return _LogisticModel(input_mean, input_scale, result.x)


def _logistic_predict(model: _LogisticModel, x: np.ndarray) -> np.ndarray:
    design = np.c_[
        np.ones(len(x)),
        (np.asarray(x, dtype=np.float64) - model.input_mean) / model.input_scale,
    ]
    logits = design @ model.weight
    return 1.0 / (1.0 + np.exp(-np.clip(logits, -50.0, 50.0)))


def _auc(target: np.ndarray, score: np.ndarray) -> float:
    target = np.asarray(target, dtype=np.bool_)
    score = np.asarray(score, dtype=np.float64)
    positive_count = int(target.sum())
    negative_count = int((~target).sum())
    if positive_count == 0 or negative_count == 0:
        return float("nan")
    order = np.argsort(score, kind="mergesort")
    ranks = np.empty(len(score), dtype=np.float64)
    ranks[order] = np.arange(1, len(score) + 1, dtype=np.float64)
    _, inverse, counts = np.unique(score, return_inverse=True, return_counts=True)
    ranks = np.bincount(inverse, weights=ranks)[inverse] / counts[inverse]
    return float(
        (ranks[target].sum() - positive_count * (positive_count + 1) / 2)
        / (positive_count * negative_count)
    )


def _balanced_accuracy(target: np.ndarray, score: np.ndarray) -> float:
    target = np.asarray(target, dtype=np.bool_)
    prediction = np.asarray(score) >= 0.5
    if not target.any() or not (~target).any():
        return float("nan")
    return float(0.5 * (prediction[target].mean() + (~prediction[~target]).mean()))


def _classification_metrics(target: np.ndarray, score: np.ndarray) -> dict[str, Any]:
    target = np.asarray(target, dtype=np.bool_)
    prediction = np.asarray(score, dtype=np.float64) >= 0.5
    true_positive = int(np.sum(prediction & target))
    false_positive = int(np.sum(prediction & ~target))
    false_negative = int(np.sum(~prediction & target))
    true_negative = int(np.sum(~prediction & ~target))
    return {
        "auc": _auc(target, score),
        "balanced_accuracy": _balanced_accuracy(target, score),
        "precision": (
            float(true_positive / (true_positive + false_positive))
            if true_positive + false_positive
            else 0.0
        ),
        "recall": (
            float(true_positive / (true_positive + false_negative))
            if true_positive + false_negative
            else 0.0
        ),
        "false_positive_rate": (
            float(false_positive / (false_positive + true_negative))
            if false_positive + true_negative
            else float("nan")
        ),
        "confusion_matrix": {
            "true_negative": true_negative,
            "false_positive": false_positive,
            "false_negative": false_negative,
            "true_positive": true_positive,
        },
    }


def _feature_variants(samples: dict[str, np.ndarray], config: ProbeConfig) -> dict[str, np.ndarray]:
    q_history = np.asarray(samples["q_history"], dtype=np.float64)
    force_history = np.asarray(samples["force_history"], dtype=np.float64)
    q_now = q_history[:, -1]
    force_now = force_history[:, -1]
    result = {
        "q_now": q_now,
        "load_now": force_now,
        "q_now_load_now": np.c_[q_now, force_now],
    }
    for update_count in config.history_updates:
        q = q_history[:, -update_count:].reshape(len(q_history), -1)
        load = force_history[:, -update_count:].reshape(len(force_history), -1)
        result[f"q_history_{update_count}"] = q
        result[f"load_history_{update_count}"] = load
        result[f"q_load_history_{update_count}"] = np.c_[q, load]
    return result


def _summary(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=np.float64)
    array = array[np.isfinite(array)]
    if not len(array):
        return {"fold_count": 0, "mean": None, "std": None, "median": None}
    return {
        "fold_count": int(len(array)),
        "mean": float(array.mean()),
        "std": float(array.std()),
        "median": float(np.median(array)),
    }


def _residual_probe(
    episodes: Sequence[ForceEpisode], config: ProbeConfig, *, accepted_only: bool
) -> dict[str, Any]:
    session, q, target, dq, dtarget, force = _update_arrays(
        episodes, config, accepted_only=accepted_only
    )
    feature_sets = {
        "q_quadratic": np.c_[q, np.square(q)],
        "q_quadratic_direction": np.c_[q, np.square(q), dq, np.sign(dq)],
        "q_target_error_direction": np.c_[
            q,
            np.square(q),
            target,
            np.square(target),
            target - q,
            dq,
            dtarget,
        ],
    }
    sessions = sorted(set(session.tolist()))
    model_metrics: dict[str, list[dict[str, Any]]] = {name: [] for name in feature_sets}
    richest_residuals: list[np.ndarray] = []
    for held_out in sessions:
        train = session != held_out
        test = ~train
        for name, features in feature_sets.items():
            prediction = _ridge_predict(
                _ridge_fit(features[train], force[train], config.ridge_l2),
                features[test],
            )
            pooled, per_channel = _r2(force[test], prediction)
            model_metrics[name].append(
                {
                    "held_out_session": held_out,
                    "test_updates": int(test.sum()),
                    "pooled_r2": pooled,
                    "per_channel_r2": per_channel.tolist(),
                }
            )
            if name == "q_target_error_direction":
                richest_residuals.append(force[test] - prediction)

    residual = np.concatenate(richest_residuals)
    eigenvalues = np.linalg.eigvalsh(np.cov(residual, rowvar=False))[::-1]
    explained = eigenvalues / eigenvalues.sum()
    result_models: dict[str, Any] = {}
    for name, folds in model_metrics.items():
        result_models[name] = {
            "folds": folds,
            "pooled_r2": _summary(fold["pooled_r2"] for fold in folds),
            "per_channel_r2_mean": np.nanmean(
                np.asarray([fold["per_channel_r2"] for fold in folds]), axis=0
            ).tolist(),
        }
    return {
        "models": result_models,
        "cross_fitted_residual_pca_explained_ratio": explained.tolist(),
        "cross_fitted_residual_correlation": np.corrcoef(residual, rowvar=False).tolist(),
    }


def _channel_redundancy(
    episodes: Sequence[ForceEpisode], config: ProbeConfig, *, accepted_only: bool
) -> dict[str, Any]:
    session, _, _, _, _, force = _update_arrays(episodes, config, accepted_only=accepted_only)
    sessions = sorted(set(session.tolist()))
    folds: list[list[float]] = []
    for held_out in sessions:
        train = session != held_out
        test = ~train
        scores: list[float] = []
        for channel in range(6):
            other = [index for index in range(6) if index != channel]
            prediction = _ridge_predict(
                _ridge_fit(force[train][:, other], force[train, channel, None], config.ridge_l2),
                force[test][:, other],
            )
            pooled, _ = _r2(force[test, channel, None], prediction)
            scores.append(pooled)
        folds.append(scores)
    values = np.asarray(folds, dtype=np.float64)
    return {
        "definition": "linear prediction of each centered channel from the other five",
        "per_channel_loso_r2_mean": np.nanmean(values, axis=0).tolist(),
        "per_channel_loso_r2_median": np.nanmedian(values, axis=0).tolist(),
        "folds": [
            {"held_out_session": session_name, "per_channel_r2": score}
            for session_name, score in zip(sessions, folds, strict=True)
        ],
    }


def _repeatability(episodes: Sequence[ForceEpisode], config: ProbeConfig) -> dict[str, Any]:
    grid = np.linspace(0.0, 1.0, 101)
    curves: list[np.ndarray] = []
    for episode in episodes:
        progress = np.linspace(0.0, 1.0, len(episode.force))
        centered = _centered_force(episode, config.initial_center_updates)
        curves.append(
            np.stack(
                [np.interp(grid, progress, centered[:, channel]) for channel in range(6)],
                axis=1,
            )
        )
    correlations: dict[str, list[list[float]]] = {"cross_session": [], "within_session": []}
    for left in range(len(episodes)):
        for right in range(left + 1, len(episodes)):
            kind = (
                "within_session"
                if episodes[left].collection_session_id == episodes[right].collection_session_id
                else "cross_session"
            )
            scores: list[float] = []
            for channel in range(6):
                a = curves[left][:, channel]
                b = curves[right][:, channel]
                scores.append(
                    float(np.corrcoef(a, b)[0, 1])
                    if a.std() > 1e-10 and b.std() > 1e-10
                    else float("nan")
                )
            correlations[kind].append(scores)
    result: dict[str, Any] = {
        "definition": "descriptive correlation after normalized-progress interpolation; no phase alignment",
    }
    for kind, values in correlations.items():
        array = np.asarray(values, dtype=np.float64)
        result[kind] = {
            "pair_count": int(len(array)),
            "per_channel_median": np.nanmedian(array, axis=0).tolist() if len(array) else [],
            "per_channel_q25": np.nanquantile(array, 0.25, axis=0).tolist() if len(array) else [],
            "per_channel_q75": np.nanquantile(array, 0.75, axis=0).tolist() if len(array) else [],
        }
    return result


def _channel_quantiles(value: np.ndarray) -> dict[str, list[float]]:
    array = np.asarray(value, dtype=np.float64)
    return {
        "p05": np.quantile(array, 0.05, axis=0).tolist(),
        "p50": np.quantile(array, 0.50, axis=0).tolist(),
        "p95": np.quantile(array, 0.95, axis=0).tolist(),
    }


def _pca_explained_ratio(value: np.ndarray) -> list[float]:
    covariance = np.cov(np.asarray(value, dtype=np.float64), rowvar=False)
    eigenvalues = np.maximum(np.linalg.eigvalsh(covariance)[::-1], 0.0)
    total = float(eigenvalues.sum())
    return (eigenvalues / total).tolist() if total > 0 else [0.0] * len(eigenvalues)


def _direction_conditioned_hysteresis(
    q: np.ndarray, dq: np.ndarray, centered_load: np.ndarray
) -> dict[str, Any]:
    """Describe direction-dependent load at similar measured hand positions.

    Direction is deliberately called q-increasing/q-decreasing.  Without
    semantic phase/contact labels this is not asserted to be closure/opening or
    contact hysteresis.
    """

    per_position_channel: list[dict[str, Any]] = []
    for position_channel in range(6):
        edges = np.unique(
            np.quantile(q[:, position_channel], np.linspace(0.0, 1.0, 11))
        )
        bins: list[dict[str, Any]] = []
        for lower, upper in zip(edges[:-1], edges[1:], strict=True):
            in_bin = (q[:, position_channel] >= lower) & (
                q[:, position_channel] <= upper
                if upper == edges[-1]
                else q[:, position_channel] < upper
            )
            increasing = in_bin & (dq[:, position_channel] > 1e-5)
            decreasing = in_bin & (dq[:, position_channel] < -1e-5)
            if int(increasing.sum()) < 10 or int(decreasing.sum()) < 10:
                continue
            difference = np.median(centered_load[increasing], axis=0) - np.median(
                centered_load[decreasing], axis=0
            )
            bins.append(
                {
                    "q_low": float(lower),
                    "q_high": float(upper),
                    "q_increasing_updates": int(increasing.sum()),
                    "q_decreasing_updates": int(decreasing.sum()),
                    "median_centered_load_difference_raw_count": difference.tolist(),
                }
            )
        differences = np.asarray(
            [entry["median_centered_load_difference_raw_count"] for entry in bins],
            dtype=np.float64,
        )
        per_position_channel.append(
            {
                "position_channel": RH56_CHANNEL_ORDER[position_channel],
                "matched_q_bin_count": len(bins),
                "rms_median_difference_raw_count": (
                    np.sqrt(np.mean(np.square(differences), axis=0)).tolist()
                    if len(differences)
                    else []
                ),
                "bins": bins,
            }
        )
    return {
        "definition": (
            "descriptive median centered-load difference for q-increasing versus "
            "q-decreasing updates inside measured-position decile bins"
        ),
        "limitation": (
            "direction, time, task phase, and physical interaction are confounded; "
            "this is a hysteresis candidate, not proof of contact hysteresis"
        ),
        "per_position_channel": per_position_channel,
    }


def _signal_descriptives(
    episodes: Sequence[ForceEpisode], config: ProbeConfig
) -> dict[str, Any]:
    raw_parts: list[np.ndarray] = []
    centered_parts: list[np.ndarray] = []
    delta_parts: list[np.ndarray] = []
    q_parts: list[np.ndarray] = []
    dq_parts: list[np.ndarray] = []
    episode_summaries: list[dict[str, Any]] = []
    for episode in episodes:
        centered = _centered_force(episode, config.initial_center_updates)
        delta = np.vstack([np.zeros((1, 6)), np.diff(centered, axis=0)])
        dq = np.vstack([np.zeros((1, 6)), np.diff(episode.rh56_position, axis=0)])
        raw_parts.append(episode.force)
        centered_parts.append(centered)
        delta_parts.append(delta)
        q_parts.append(episode.rh56_position)
        dq_parts.append(dq)
        statuses, counts = np.unique(episode.action_status, return_counts=True)
        episode_summaries.append(
            {
                "logical_segment_id": episode.logical_segment_id,
                "source_episode_index": episode.source_episode_index,
                "collection_session_id": episode.collection_session_id,
                "unique_force_updates": len(episode.force),
                "raw_load_min": episode.force.min(axis=0).tolist(),
                "raw_load_max": episode.force.max(axis=0).tolist(),
                "centered_load_l2_p95": float(
                    np.quantile(np.linalg.norm(centered, axis=1), 0.95)
                ),
                "action_status_counts": {
                    str(status): int(count)
                    for status, count in zip(statuses, counts, strict=True)
                },
            }
        )
    raw = np.concatenate(raw_parts)
    centered = np.concatenate(centered_parts)
    delta = np.concatenate(delta_parts)
    q = np.concatenate(q_parts)
    dq = np.concatenate(dq_parts)
    magnitude = np.abs(centered)
    denominator = magnitude.sum(axis=1, keepdims=True)
    share = np.divide(
        magnitude,
        denominator,
        out=np.zeros_like(magnitude),
        where=denominator > 0,
    )
    return {
        "semantics": (
            "descriptive statistics over episode-local unique FORCE_ACT sources; "
            "initial-window centering is offset removal, not a no-contact baseline"
        ),
        "raw_load_raw_count": _channel_quantiles(raw),
        "baseline_corrected_load_raw_count": _channel_quantiles(centered),
        "absolute_per_update_delta_raw_count": _channel_quantiles(np.abs(delta)),
        "baseline_corrected_load_l2_raw_count": {
            key: float(value)
            for key, value in zip(
                ("p05", "p50", "p95"),
                np.quantile(np.linalg.norm(centered, axis=1), (0.05, 0.50, 0.95)),
                strict=True,
            )
        },
        "centered_load_covariance_raw_count_squared": np.cov(
            centered, rowvar=False
        ).tolist(),
        "centered_load_correlation": np.corrcoef(centered, rowvar=False).tolist(),
        "centered_load_pca_explained_ratio": _pca_explained_ratio(centered),
        "load_delta_pca_explained_ratio": _pca_explained_ratio(delta),
        "actuator_load_magnitude_share": {
            "definition": (
                "absolute baseline-corrected channel magnitude divided by six-channel "
                "absolute magnitude; not a fingertip-force distribution"
            ),
            **_channel_quantiles(share),
        },
        "position_conditioned_directional_difference": _direction_conditioned_hysteresis(
            q, dq, centered
        ),
        "episodes": episode_summaries,
    }


def _future_probe(
    episodes: Sequence[ForceEpisode], config: ProbeConfig, *, accepted_only: bool
) -> dict[str, Any]:
    samples = _history_samples(episodes, config, accepted_only=accepted_only)
    features = _feature_variants(samples, config)
    session = samples["session"]
    sessions = sorted(set(session.tolist()))
    folds: list[dict[str, Any]] = []
    for held_out in sessions:
        train = session != held_out
        test = ~train
        threshold = float(np.quantile(samples["future_change"][train], config.large_change_quantile))
        train_event = samples["future_change"][train] >= threshold
        test_event = samples["future_change"][test] >= threshold
        fold: dict[str, Any] = {
            "held_out_session": held_out,
            "train_samples": int(train.sum()),
            "test_samples": int(test.sum()),
            "train_only_large_change_threshold_raw_count_l2": threshold,
            "test_large_change_rate": float(test_event.mean()),
            "features": {},
        }
        for name, value in features.items():
            future_force_prediction = _ridge_predict(
                _ridge_fit(value[train], samples["future_force"][train], config.ridge_l2),
                value[test],
            )
            force_r2, _ = _r2(samples["future_force"][test], future_force_prediction)
            action_prediction = _ridge_predict(
                _ridge_fit(
                    value[train], samples["future_hand_target"][train], config.ridge_l2
                ),
                value[test],
            )
            action_r2, _ = _r2(samples["future_hand_target"][test], action_prediction)
            probability = _logistic_predict(
                _logistic_fit(value[train], train_event, config.logistic_l2),
                value[test],
            )
            classification = _classification_metrics(test_event, probability)
            fold["features"][name] = {
                "future_load_r2": force_r2,
                "future_hand_target_r2": action_r2,
                "large_future_load_change_auc": classification["auc"],
                "large_future_load_change_balanced_accuracy": classification[
                    "balanced_accuracy"
                ],
                "large_future_load_change_precision": classification["precision"],
                "large_future_load_change_recall": classification["recall"],
                "large_future_load_change_false_positive_rate": classification[
                    "false_positive_rate"
                ],
                "large_future_load_change_confusion_matrix": classification[
                    "confusion_matrix"
                ],
            }
        folds.append(fold)

    summary: dict[str, Any] = {}
    metric_names = (
        "future_load_r2",
        "future_hand_target_r2",
        "large_future_load_change_auc",
        "large_future_load_change_balanced_accuracy",
        "large_future_load_change_precision",
        "large_future_load_change_recall",
        "large_future_load_change_false_positive_rate",
    )
    for feature_name in features:
        summary[feature_name] = {
            metric: _summary(
                fold["features"][feature_name][metric] for fold in folds
            )
            for metric in metric_names
        }
        summary[feature_name]["large_future_load_change_confusion_matrix"] = {
            key: int(
                sum(
                    fold["features"][feature_name][
                        "large_future_load_change_confusion_matrix"
                    ][key]
                    for fold in folds
                )
            )
            for key in (
                "true_negative",
                "false_positive",
                "false_negative",
                "true_positive",
            )
        }
    history_timestamp = np.asarray(samples["history_force_timestamp_ns"], dtype=np.int64)
    effective_spans: dict[str, Any] = {}
    for update_count in config.history_updates:
        span_ms = (
            history_timestamp[:, -1] - history_timestamp[:, -update_count]
        ) / 1e6
        effective_spans[str(update_count)] = {
            "definition": "newest minus oldest actual unique FORCE_ACT source timestamp",
            "p05": float(np.quantile(span_ms, 0.05)),
            "p50": float(np.quantile(span_ms, 0.50)),
            "p95": float(np.quantile(span_ms, 0.95)),
            "max": float(np.max(span_ms)),
        }
    return {
        "target_semantics": {
            "future_load": "baseline-centered raw FORCE_ACT at a later unique source update",
            "future_hand_target": "six-dimensional commanded RH56 target",
            "large_future_load_change": (
                "train-fold top-quantile future raw-count L2 redistribution; "
                "not a contact, stability, failure, or slip label"
            ),
        },
        "history_effective_span_ms": effective_spans,
        "folds": folds,
        "summary": summary,
    }


def run_information_probe(
    episodes: Sequence[ForceEpisode], config: ProbeConfig
) -> dict[str, Any]:
    config.validate()
    session_count = len({episode.collection_session_id for episode in episodes})
    if session_count < 3:
        raise ValueError("at least three physical sessions are required")
    all_force_timestamps = np.concatenate(
        [episode.force_timestamp_ns for episode in episodes]
    )
    intervals = np.concatenate(
        [np.diff(episode.force_timestamp_ns) for episode in episodes if len(episode.force) > 1]
    )
    baselines = np.stack(
        [
            np.median(
                episode.force[: min(config.initial_center_updates, len(episode.force))], axis=0
            )
            for episode in episodes
        ]
    )
    row_counts = np.asarray(
        [episode.canonical_row_count or len(episode.force) for episode in episodes],
        dtype=np.int64,
    )
    durations = np.asarray(
        [episode.canonical_duration_s for episode in episodes], dtype=np.float64
    )
    row_rates = np.divide(
        row_counts - 1,
        durations,
        out=np.full(len(episodes), np.nan),
        where=durations > 0,
    )
    force_rates = np.asarray(
        [
            (len(episode.force) - 1)
            / ((episode.force_timestamp_ns[-1] - episode.force_timestamp_ns[0]) / 1e9)
            for episode in episodes
        ],
        dtype=np.float64,
    )
    row_age_ns = np.concatenate(
        [
            (
                episode.row_force_age_ns
                if episode.row_force_age_ns is not None
                else episode.canonical_timestamp_ns - episode.force_timestamp_ns
            )
            for episode in episodes
        ]
    )
    row_valid = np.concatenate(
        [
            (
                episode.row_force_valid
                if episode.row_force_valid is not None
                else np.ones(len(episode.force), dtype=np.bool_)
            )
            for episode in episodes
        ]
    )
    row_status = np.concatenate(
        [
            (
                episode.row_action_status
                if episode.row_action_status is not None
                else episode.action_status
            )
            for episode in episodes
        ]
    )
    status_names, status_counts = np.unique(row_status, return_counts=True)
    result: dict[str, Any] = {
        "schema_version": "embodied_lab.force_information_probe.v1",
        "force_semantics": (
            "six signed raw RH56 FORCE_ACT actuator/push-rod load counts; "
            "not fingertip force, tactile localization, calibrated Newtons, or slip ground truth"
        ),
        "rh56_channel_order": list(RH56_CHANNEL_ORDER),
        "dataset": {
            "logical_trajectories": len(episodes),
            "source_episodes": len({episode.source_episode_index for episode in episodes}),
            "physical_sessions": session_count,
            "unique_force_updates": int(len(all_force_timestamps)),
            "unique_force_update_semantics": (
                "sum of episode-local unique source updates; identical monotonic host timestamps "
                "may legitimately recur across two logical segments of one source episode"
            ),
            "collection_session_groups": {
                session: [
                    episode.logical_segment_id
                    for episode in episodes
                    if episode.collection_session_id == session
                ]
                for session in sorted({episode.collection_session_id for episode in episodes})
            },
            "force_interval_ms": {
                "p50": float(np.quantile(intervals / 1e6, 0.50)),
                "p95": float(np.quantile(intervals / 1e6, 0.95)),
                "p99": float(np.quantile(intervals / 1e6, 0.99)),
                "max": float(np.max(intervals / 1e6)),
            },
            "timestamp_audit": {
                "canonical_rows": int(row_counts.sum()),
                "canonical_row_rate_hz": {
                    "min": float(np.nanmin(row_rates)),
                    "median": float(np.nanmedian(row_rates)),
                    "max": float(np.nanmax(row_rates)),
                },
                "native_force_source_rate_hz": {
                    "min": float(np.min(force_rates)),
                    "median": float(np.median(force_rates)),
                    "max": float(np.max(force_rates)),
                },
                "repeated_cached_rows": int(row_counts.sum() - len(all_force_timestamps)),
                "rows_per_unique_force_source": float(
                    row_counts.sum() / len(all_force_timestamps)
                ),
                "force_age_ms": {
                    "p50": float(np.quantile(row_age_ns / 1e6, 0.50)),
                    "p95": float(np.quantile(row_age_ns / 1e6, 0.95)),
                    "p99": float(np.quantile(row_age_ns / 1e6, 0.99)),
                    "max": float(np.max(row_age_ns / 1e6)),
                },
                "force_valid_rows": int(row_valid.sum()),
                "force_invalid_or_missing_rows": int((~row_valid).sum()),
                "future_force_source_rows": 0,
                "action_status_row_counts": {
                    str(name): int(count)
                    for name, count in zip(status_names, status_counts, strict=True)
                },
            },
            "initial_window_offset_raw_count_mean": baselines.mean(axis=0).tolist(),
            "initial_window_offset_raw_count_std": baselines.std(axis=0).tolist(),
        },
        "config": {
            "initial_center_updates": config.initial_center_updates,
            "history_updates": list(config.history_updates),
            "future_horizon_updates": config.future_horizon_updates,
            "large_change_quantile": config.large_change_quantile,
            "ridge_l2": config.ridge_l2,
            "logistic_l2": config.logistic_l2,
        },
        "signal_descriptives": _signal_descriptives(episodes, config),
        "repeatability": _repeatability(episodes, config),
        "cohorts": {},
    }
    for accepted_only, name in ((False, "all_updates"), (True, "accepted_query_updates")):
        result["cohorts"][name] = {
            "selection": (
                "all unique force updates"
                if not accepted_only
                else "query updates whose arm action_status is accepted; histories may include prior held rows"
            ),
            "configuration_residual": _residual_probe(
                episodes, config, accepted_only=accepted_only
            ),
            "channel_redundancy": _channel_redundancy(
                episodes, config, accepted_only=accepted_only
            ),
            "future_dynamics": _future_probe(
                episodes, config, accepted_only=accepted_only
            ),
        }
    return result


def compact_markdown(result: dict[str, Any]) -> str:
    """Render a short, claim-calibrated summary beside the machine output."""

    dataset = result["dataset"]
    cohort = result.get("cohort", {})
    all_updates = result["cohorts"]["all_updates"]
    accepted = result["cohorts"]["accepted_query_updates"]
    residual = all_updates["configuration_residual"]
    dynamics = accepted["future_dynamics"]["summary"]
    history_updates = tuple(int(value) for value in result["config"]["history_updates"])

    def value(feature: str, metric: str) -> str:
        number = dynamics[feature][metric]["mean"]
        spread = dynamics[feature][metric]["std"]
        return "n/a" if number is None else f"{number:.3f} +/- {spread:.3f}"

    q_r2 = residual["models"]["q_quadratic"]["pooled_r2"]
    rich_r2 = residual["models"]["q_target_error_direction"]["pooled_r2"]
    pca = residual["cross_fitted_residual_pca_explained_ratio"]
    lines = [
        "# Offline FORCE_ACT information probe",
        "",
        "This is offline analysis of raw RH56 actuator/push-rod load counts. It does not provide contact, tactile, stability, slip, or physical-policy evidence.",
        "",
        f"- cohort: `{cohort.get('label', 'UNSPECIFIED')}`",
        f"- {dataset['logical_trajectories']} logical trajectories from {dataset['physical_sessions']} physical collection sessions",
        f"- {dataset['unique_force_updates']} unique causal FORCE_ACT updates",
        f"- q-only quadratic LOSO current-load R2: {q_r2['mean']:.3f} +/- {q_r2['std']:.3f}",
        f"- q/target/error/direction LOSO current-load R2: {rich_r2['mean']:.3f} +/- {rich_r2['std']:.3f}",
        f"- residual PCA top-2 variance: {sum(pca[:2]):.3f}",
        "",
        "## Accepted-query future probes (session held out)",
        "",
        "| Features | future load R2 | numeric large-change AUC | future hand-target R2 |",
        "| --- | ---: | ---: | ---: |",
    ]
    features = ["q_now", "load_now", "q_now_load_now"]
    for updates in history_updates:
        features.extend(
            (
                f"q_history_{updates}",
                f"load_history_{updates}",
                f"q_load_history_{updates}",
            )
        )
    for feature in features:
        lines.append(
            f"| `{feature}` | {value(feature, 'future_load_r2')} | "
            f"{value(feature, 'large_future_load_change_auc')} | "
            f"{value(feature, 'future_hand_target_r2')} |"
        )
    lines.extend(
        [
            "",
            "The large-change target is only a train-fold numeric raw-count redistribution threshold. It is not an interaction, contact, instability, failure, or slip label.",
            "",
        ]
    )
    return "\n".join(lines)
