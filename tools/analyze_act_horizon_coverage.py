#!/usr/bin/env python3
"""Audit ACT transition supervision and padding for several action horizons."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import pyarrow.parquet as parquet


MIN_CLOSURE_MAGNITUDE = 0.10


def _distribution(values: Sequence[float]) -> dict[str, float] | None:
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return None
    return {
        "mean": float(np.mean(array)),
        "std": float(np.std(array)),
        "min": float(np.min(array)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "max": float(np.max(array)),
    }


def _first_sustained(values: np.ndarray, threshold: float) -> int | None:
    above = values >= threshold
    for index in range(len(above)):
        window = above[index : index + 5]
        if above[index] and np.count_nonzero(window) >= min(4, len(window)):
            return index
    return None


def _transition_info(rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    actions = np.asarray([row["action"] for row in rows], dtype=np.float64)
    closure_actions = actions[:, 6:11]
    search_end = max(1, int(np.ceil(len(actions) * 0.60)))
    baseline_window = min(15, search_end)
    openness = np.convolve(
        np.sum(closure_actions[:search_end], axis=1),
        np.ones(baseline_window) / baseline_window,
        mode="valid",
    )
    baseline_start = int(np.argmin(openness))
    baseline_end = baseline_start + baseline_window
    baseline = np.median(closure_actions[baseline_start:baseline_end], axis=0)
    positive_delta = np.maximum(closure_actions - baseline, 0.0)
    closure = np.linalg.norm(positive_delta, axis=1)
    peak_index = int(np.argmax(closure))
    threshold = max(MIN_CLOSURE_MAGNITUDE, 0.25 * float(closure[peak_index]))
    relative_onset = _first_sustained(closure[baseline_end:], threshold)
    onset = None if relative_onset is None else baseline_end + relative_onset

    grip = np.asarray([bool(row.get("hand_grip", False)) for row in rows])
    releases = np.flatnonzero(grip[:-1] & ~grip[1:]) + 1
    if onset is not None:
        releases = releases[releases > onset]
    release = None if not len(releases) else int(releases[-1])
    return {
        "rows": len(rows),
        "duration_s": (
            (int(rows[-1]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9
            if len(rows) > 1 else 0.0
        ),
        "grasp_onset_local_frame": onset,
        "grasp_onset_source_frame": (
            None if onset is None else int(rows[onset]["source_frame_index"])
        ),
        "grasp_onset_s": (
            None if onset is None else
            (int(rows[onset]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9
        ),
        "release_onset_local_frame": release,
        "release_onset_source_frame": (
            None if release is None else int(rows[release]["source_frame_index"])
        ),
        "release_onset_s": (
            None if release is None else
            (int(rows[release]["timestamp_ns"]) - int(rows[0]["timestamp_ns"])) / 1e9
        ),
        "open_baseline_rh56": baseline.tolist(),
        "closure_threshold": threshold,
        "peak_closure_magnitude": float(closure[peak_index]),
        "peak_closure_delta_by_channel": positive_delta[peak_index].tolist(),
        "approach_rows": 0 if onset is None else onset,
        "post_grasp_rows": 0 if onset is None else len(rows) - onset,
        "stationary_action_fraction": (
            None if len(actions) < 2 else
            float(np.mean(np.linalg.norm(np.diff(actions, axis=0), axis=1) <= 1e-4))
        ),
        "rh56_action_change": _distribution(
            np.linalg.norm(np.diff(actions[:, 6:], axis=0), axis=1).tolist()
        ),
    }


def _horizon_counts(lengths: Sequence[int], events: Sequence[int | None], horizon: int) -> dict[str, Any]:
    future = 0
    after_first_two = 0
    total_rows = int(sum(lengths))
    padded_actions = 0
    padded_rows = 0
    for length, event in zip(lengths, events, strict=True):
        if event is not None:
            future += min(horizon - 1, event)
            after_first_two += min(max(horizon - 2, 0), max(event - 1, 0))
        for index in range(length):
            valid = min(horizon, length - index)
            padded_actions += horizon - valid
            padded_rows += int(valid < horizon)
    return {
        "future_event_queries": future,
        "future_event_query_fraction_of_all_rows": future / total_rows,
        "event_after_first_two_actions_queries": after_first_two,
        "rows_with_boundary_padding": padded_rows,
        "boundary_padded_action_slots": padded_actions,
        "boundary_padded_action_fraction": padded_actions / (total_rows * horizon),
    }


def analyze(master: Path, split_config: Path, horizons: Sequence[int]) -> dict[str, Any]:
    split = json.loads(
        (master.parent / "manifests/splits.json").read_text(encoding="utf-8")
    )["splits"]
    records = [
        json.loads(line)
        for line in (master / "meta/episodes.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    config_text = split_config.read_text(encoding="utf-8")
    import yaml

    config = yaml.safe_load(config_text)
    group_for = {
        segment: group["id"]
        for group in config["split_groups"]
        for segment in group["segments"]
    }
    split_for_index = {
        int(index): name for name, indices in split.items() for index in indices
    }
    episodes: dict[str, Any] = {}
    for record in records:
        rows = parquet.read_table(master / record["data"]).to_pylist()
        segment = str(record["logical_segment_id"])
        info = _transition_info(rows)
        info.update({
            "source_episode": int(record["source_episode_index"]),
            "source_frame_range": [
                int(rows[0]["source_frame_index"]), int(rows[-1]["source_frame_index"])
            ],
            "source_timestamp_range_ns": [
                int(rows[0]["timestamp_ns"]), int(rows[-1]["timestamp_ns"])
            ],
            "split": split_for_index[int(record["episode_index"])],
            "session_group": group_for[segment],
        })
        episodes[segment] = info
    lengths = [value["rows"] for value in episodes.values()]
    grasp = [value["grasp_onset_local_frame"] for value in episodes.values()]
    release = [value["release_onset_local_frame"] for value in episodes.values()]
    horizon_report = {}
    for horizon in horizons:
        if horizon < 1:
            raise ValueError("horizons must be positive")
        horizon_report[str(horizon)] = {
            "grasp": _horizon_counts(lengths, grasp, horizon),
            "release": _horizon_counts(lengths, release, horizon),
        }
    total_rows = sum(lengths)
    return {
        "schema_version": "embodied_lab.act_horizon_coverage.v1",
        "master": str(master.resolve()),
        "trajectory_count": len(episodes),
        "rows": total_rows,
        "duration_s": float(sum(value["duration_s"] for value in episodes.values())),
        "split": {
            name: {
                "trajectories": len(indices),
                "rows": int(sum(records[index]["length"] for index in indices)),
                "sessions": sorted({episodes[records[index]["logical_segment_id"]]["session_group"] for index in indices}),
            }
            for name, indices in split.items()
        },
        "transition_definition": (
            "Grasp is the first sustained RH56 action closure above max(0.10, 25% of "
            "trajectory peak) relative to its lowest-closure 15-row baseline. Release is "
            "the final persisted hand_grip true-to-false edge after grasp. Both are "
            "objective proxies, not semantic contact labels."
        ),
        "approach_rows": int(sum(value["approach_rows"] for value in episodes.values())),
        "post_grasp_rows": int(sum(value["post_grasp_rows"] for value in episodes.values())),
        "grasp_onset_timing_s": _distribution([
            value["grasp_onset_s"] for value in episodes.values()
            if value["grasp_onset_s"] is not None
        ]),
        "release_onset_timing_s": _distribution([
            value["release_onset_s"] for value in episodes.values()
            if value["release_onset_s"] is not None
        ]),
        "horizons": horizon_report,
        "episodes": episodes,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--master", type=Path, required=True)
    parser.add_argument("--dataset-config", type=Path, required=True)
    parser.add_argument("--horizons", type=int, nargs="+", default=(16, 32, 60))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.master, args.dataset_config, args.horizons)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("trajectory_count", "rows", "duration_s", "split", "horizons")}, indent=2))


if __name__ == "__main__":
    main()
