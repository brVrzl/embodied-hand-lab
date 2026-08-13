#!/usr/bin/env python3
"""Quantify bottle-task curation, task trimming, and grasp transitions offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import yaml


ACTION_EPSILON = 1e-4
MIN_CLOSURE_MAGNITUDE = 0.10
CHUNK_SIZE = 16


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _read_master(master: Path) -> dict[str, list[dict[str, Any]]]:
    import pyarrow.parquet as parquet

    result: dict[str, list[dict[str, Any]]] = {}
    for line in (master / "meta/episodes.jsonl").read_text(encoding="utf-8").splitlines():
        record = json.loads(line)
        segment = str(record.get("logical_segment_id", record["episode_index"]))
        result[segment] = parquet.read_table(master / record["data"]).to_pylist()
    return result


def _action_change(actions: np.ndarray) -> np.ndarray:
    if len(actions) < 2:
        return np.empty(0, dtype=np.float64)
    return np.linalg.norm(np.diff(actions, axis=0), axis=1)


def _distribution(values: Sequence[float] | np.ndarray) -> dict[str, float | int] | None:
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return None
    return {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def _first_sustained(values: np.ndarray, threshold: float) -> int | None:
    above = values >= threshold
    for index in range(len(above)):
        if above[index] and np.count_nonzero(above[index : index + 5]) >= min(4, len(above) - index):
            return index
    return None


def transition_summary(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Describe first grasp closure using a per-trajectory action-magnitude heuristic."""

    actions = np.asarray([row["action"] for row in rows], dtype=np.float64)
    if actions.ndim != 2 or actions.shape[1] != 12 or not np.isfinite(actions).all():
        raise ValueError("rows must contain finite 12-D actions")
    closure_actions = actions[:, 6:11]
    search_end = max(1, int(np.ceil(len(actions) * 0.60)))
    baseline_window = min(15, search_end)
    openness_score = np.convolve(
        np.sum(closure_actions[:search_end], axis=1),
        np.ones(baseline_window) / baseline_window,
        mode="valid",
    )
    baseline_start = int(np.argmin(openness_score))
    baseline_end = baseline_start + baseline_window
    baseline = np.median(closure_actions[baseline_start:baseline_end], axis=0)
    positive_delta = np.maximum(actions[:, 6:11] - baseline, 0.0)
    closure = np.linalg.norm(positive_delta, axis=1)
    peak = float(np.max(closure))
    threshold = max(MIN_CLOSURE_MAGNITUDE, 0.25 * peak)
    relative_onset = _first_sustained(closure[baseline_end:], threshold)
    onset = None if relative_onset is None else baseline_end + relative_onset
    transition_queries = 0
    if onset is not None:
        for index in range(len(rows)):
            horizons = np.arange(index, min(index + CHUNK_SIZE, len(rows)))
            if len(horizons) > 2 and np.all(horizons[:2] < onset) and np.any(horizons[2:] >= onset):
                transition_queries += 1
    changes = _action_change(actions)
    peak_index = int(np.argmax(closure))
    return {
        "rows": len(rows),
        "duration_s": (
            (int(rows[-1]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9
            if len(rows) > 1 else 0.0
        ),
        "heuristic": (
            "First sustained RH56 closure-vector L2 magnitude above max(0.10, "
            "25% of that trajectory's peak), after the lowest-closure 15-row "
            "baseline window in the first 60% of the trajectory."
        ),
        "open_baseline_source_frame_range": [
            int(rows[baseline_start].get("source_frame_index", baseline_start)),
            int(rows[baseline_end - 1].get("source_frame_index", baseline_end - 1)),
        ],
        "open_baseline_rh56": baseline.tolist(),
        "grasp_onset_local_frame": onset,
        "grasp_onset_source_frame": (
            None if onset is None else int(rows[onset].get("source_frame_index", onset))
        ),
        "grasp_onset_s": (
            None if onset is None else (int(rows[onset]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9
        ),
        "approach_rows": 0 if onset is None else onset,
        "transition_containing_chunks": transition_queries,
        "post_grasp_rows": 0 if onset is None else len(rows) - onset,
        "closure_threshold": threshold,
        "peak_closure_magnitude": peak,
        "peak_closure_delta_by_channel": positive_delta[peak_index].tolist(),
        "stationary_action_fraction": (
            None if not len(changes) else float(np.mean(changes <= ACTION_EPSILON))
        ),
        "action_change_magnitude": _distribution(changes),
        "rh56_action_change_magnitude": _distribution(_action_change(actions[:, 6:])),
    }


def _aggregate(name: str, episodes: Mapping[str, Sequence[Mapping[str, Any]]]) -> dict[str, Any]:
    per_episode = {key: transition_summary(rows) for key, rows in episodes.items()}
    actions = np.concatenate([
        np.asarray([row["action"] for row in rows], dtype=np.float64)
        for rows in episodes.values()
    ])
    changes = np.concatenate([
        _action_change(np.asarray([row["action"] for row in rows], dtype=np.float64))
        for rows in episodes.values()
    ])
    total = sum(value["rows"] for value in per_episode.values())
    return {
        "name": name,
        "episode_count": len(per_episode),
        "rows": total,
        "duration_s": sum(value["duration_s"] for value in per_episode.values()),
        "approach_rows": sum(value["approach_rows"] for value in per_episode.values()),
        "transition_containing_chunks": sum(value["transition_containing_chunks"] for value in per_episode.values()),
        "post_grasp_rows": sum(value["post_grasp_rows"] for value in per_episode.values()),
        "stationary_action_fraction": float(np.mean(changes <= ACTION_EPSILON)),
        "action_change_magnitude": _distribution(changes),
        "rh56_action_range": (np.max(actions[:, 6:], axis=0) - np.min(actions[:, 6:], axis=0)).tolist(),
        "per_episode": per_episode,
    }


def _raw_segment_rows(raw_root: Path, segment: Mapping[str, Any], *, pretrim: bool) -> list[dict[str, Any]]:
    source = int(segment["source_episode"])
    rows = _read_jsonl(raw_root / "data/chunk-000" / f"episode_{source:06d}.jsonl")
    if pretrim:
        start = int(segment.get("pretrim_start_frame", 0))
        end = int(segment.get("pretrim_end_frame", len(rows) - 1))
    else:
        start = int(segment.get("start_frame", 0))
        end = int(segment.get("end_frame", len(rows) - 1))
    selected = [dict(row) for row in rows[start : end + 1]]
    for row in selected:
        row.setdefault("source_frame_index", int(row["frame_index"]))
    return selected


def _trim_report(raw_root: Path, segments: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    reports: dict[str, Any] = {}
    removed_changes: list[float] = []
    before_total = 0
    after_total = 0
    for segment in segments:
        source = int(segment["source_episode"])
        rows = _read_jsonl(raw_root / "data/chunk-000" / f"episode_{source:06d}.jsonl")
        pre_start = int(segment.get("pretrim_start_frame", 0))
        pre_end = int(segment.get("pretrim_end_frame", len(rows) - 1))
        start = int(segment.get("start_frame", 0))
        end = int(segment.get("end_frame", pre_end))
        prefix = list(range(pre_start, start))
        suffix = list(range(end + 1, pre_end + 1))
        removed = prefix + suffix
        actions = np.asarray([row["action"] for row in rows], dtype=np.float64)
        changes = np.r_[0.0, _action_change(actions)]
        removed_changes.extend(changes[removed].tolist())
        before = pre_end - pre_start + 1
        after = end - start + 1
        before_total += before
        after_total += after
        reports[str(segment["id"])] = {
            "source_episode": source,
            "pretrim_frame_range": [pre_start, pre_end],
            "task_frame_range": [start, end],
            "task_timestamp_range_ns": [int(rows[start]["timestamp_ns"]), int(rows[end]["timestamp_ns"])],
            "pre_task_rows_removed": len(prefix),
            "post_task_rows_removed": len(suffix),
            "pre_task_duration_s": len(prefix) / 30.0,
            "post_task_tail_duration_s": len(suffix) / 30.0,
            "removed_action_change_magnitude": _distribution(changes[removed]),
            "removed_stationary_action_fraction": (
                None if not removed else float(np.mean(changes[removed] <= ACTION_EPSILON))
            ),
        }
    return {
        "rows_before": before_total,
        "rows_after": after_total,
        "rows_removed": before_total - after_total,
        "percent_removed": 100.0 * (before_total - after_total) / before_total,
        "post_task_tail_rows": sum(value["post_task_rows_removed"] for value in reports.values()),
        "post_task_tail_fraction_before_trim": sum(value["post_task_rows_removed"] for value in reports.values()) / before_total,
        "removed_action_change_magnitude": _distribution(removed_changes),
        "removed_stationary_action_fraction": float(np.mean(np.asarray(removed_changes) <= ACTION_EPSILON)),
        "per_episode": reports,
    }


def _interval_action_report(
    raw_root: Path,
    intervals: Sequence[tuple[int, int, int]],
    *,
    mixed_rows: int,
) -> dict[str, Any]:
    """Describe reset rows known to have leaked into the historical mixed view."""

    reports: list[dict[str, Any]] = []
    all_changes: list[float] = []
    for source, start, end in intervals:
        rows = _read_jsonl(raw_root / "data/chunk-000" / f"episode_{source:06d}.jsonl")
        actions = np.asarray([row["action"] for row in rows], dtype=np.float64)
        changes = np.r_[0.0, _action_change(actions)][start : end + 1]
        all_changes.extend(changes.tolist())
        reports.append({
            "source_episode": source,
            "source_frame_range": [start, end],
            "rows": end - start + 1,
            "duration_s": (end - start + 1) / 30.0,
            "action_change_magnitude": _distribution(changes),
            "stationary_action_fraction": float(np.mean(changes <= ACTION_EPSILON)),
        })
    return {
        "rows": len(all_changes),
        "fraction_of_old_mixed_rows": len(all_changes) / mixed_rows,
        "reason": (
            "source 99 frames 966-1169 and source 102 frames 878-1004 were "
            "retained by the old malformed first segments"
        ),
        "scope": (
            "lower bound; excluded non-nominal sources were not assigned "
            "unsupported manual completion labels"
        ),
        "action_change_magnitude": _distribution(all_changes),
        "stationary_action_fraction": float(
            np.mean(np.asarray(all_changes) <= ACTION_EPSILON)
        ),
        "intervals": reports,
    }


def analyze(args: argparse.Namespace) -> None:
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    raw_root = (args.config.parent / config["source_root"]).resolve()
    included = [value for value in config["segments"] if value.get("include")]
    before = {str(value["id"]): _raw_segment_rows(raw_root, value, pretrim=True) for value in included}
    trimmed = _read_master(args.nominal_master.resolve())
    mixed = _read_master(args.mixed_master.resolve())
    mixed_rows = sum(len(rows) for rows in mixed.values())
    result = {
        "schema_version": "embodied_lab.physical_bottle_curation_analysis.v1",
        "action_stationary_epsilon_l2": ACTION_EPSILON,
        "human_audit": config["audit_provenance"],
        "views": {
            "old_mixed_materialized": _aggregate("old_mixed_materialized", mixed),
            "nominal16_before_task_trim": _aggregate("nominal16_before_task_trim", before),
            "nominal16_task_trimmed": _aggregate("nominal16_task_trimmed", trimmed),
        },
        "task_trimming": _trim_report(raw_root, included),
        "old_mixed_known_reset_tail_lower_bound": _interval_action_report(
            raw_root,
            ((99, 966, 1169), (102, 878, 1004)),
            mixed_rows=mixed_rows,
        ),
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "curation_transition_analysis.json").write_text(
        json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "output": str(args.output / "curation_transition_analysis.json"),
        "views": {
            name: {
                key: value[key]
                for key in ("episode_count", "rows", "duration_s", "approach_rows", "transition_containing_chunks", "post_grasp_rows", "stationary_action_fraction")
            }
            for name, value in result["views"].items()
        },
        "task_trimming": {key: result["task_trimming"][key] for key in ("rows_before", "rows_after", "rows_removed", "percent_removed", "post_task_tail_fraction_before_trim")},
    }, indent=2))


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mixed-master", type=Path, required=True)
    parser.add_argument("--nominal-master", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser


if __name__ == "__main__":
    analyze(make_parser().parse_args())
