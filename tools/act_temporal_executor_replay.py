#!/usr/bin/env python3
"""Replay saved ACT chunks through command-disabled executor semantics.

This tool never opens a hardware or command socket.  It uses query completion
and command timestamps from a saved rollout to preserve causal availability,
then compares legacy consume-K with absolute-time temporal ensembling.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Any

import numpy as np

from embodiment_core.act_temporal_executor import AbsoluteTimeTemporalEnsembler


def _jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _available_queries(queries: list[dict[str, Any]], command_times: np.ndarray) -> dict[int, int]:
    available: dict[int, int] = {}
    for row in queries:
        sequence = int(row["query_sequence"])
        completion = int(row["query_end_ns"])
        available[sequence] = int(np.searchsorted(command_times, completion, side="left"))
    return available


def _consume_k(
    chunks: np.ndarray,
    query_available_tick: dict[int, int],
    command_count: int,
    consume_actions: int,
) -> dict[str, Any]:
    horizon, action_dim = chunks.shape[1:]
    if not 1 <= consume_actions <= horizon:
        raise ValueError(f"consume_actions must be within [1,{horizon}]")
    available_by_tick: dict[int, list[int]] = {}
    for sequence, tick in query_available_tick.items():
        available_by_tick.setdefault(tick, []).append(sequence)
    latest = 0
    active = 0
    index = consume_actions
    actions = np.zeros((command_count, action_dim), dtype=np.float64)
    selected_h = np.full(command_count, -1, dtype=np.int64)
    selected_q = np.zeros(command_count, dtype=np.int64)
    fallback = np.ones(command_count, dtype=bool)
    for tick in range(command_count):
        for sequence in available_by_tick.get(tick, ()):
            latest = max(latest, sequence)
        if latest and (active == 0 or index >= consume_actions):
            active = latest
            index = 0
        if active:
            h = min(index, consume_actions - 1)
            actions[tick] = chunks[active - 1, h]
            selected_h[tick] = h
            selected_q[tick] = active
            fallback[tick] = False
            index += 1
    return {
        "actions": actions,
        "selected_horizon": selected_h,
        "selected_query": selected_q,
        "contributors": [
            [{
                "query_id": int(q),
                "query_command_tick": int(query_available_tick.get(int(q), 0)),
                "source_horizon": int(h),
                "ensemble_weight": 1.0,
            }]
            if q > 0 else []
            for q, h in zip(selected_q, selected_h, strict=True)
        ],
        "fallback": fallback,
        "buffer_stats": {
            "predictions_added": len(query_available_tick),
            "prediction_points_added": int(len(query_available_tick) * horizon),
            "prediction_points_selected": int(np.count_nonzero(~fallback)),
            "prediction_points_discarded": int(len(query_available_tick) * horizon - np.count_nonzero(~fallback)),
        },
    }


def _temporal_ensemble(
    chunks: np.ndarray,
    queries: list[dict[str, Any]],
    command_times: np.ndarray,
    *,
    coefficient: float,
    max_prediction_age_ticks: int | None = None,
) -> dict[str, Any]:
    horizon, action_dim = chunks.shape[1:]
    available = _available_queries(queries, command_times)
    by_tick: dict[int, list[int]] = {}
    for sequence, tick in available.items():
        by_tick.setdefault(tick, []).append(sequence)
    ensemble = AbsoluteTimeTemporalEnsembler(
        action_dim=action_dim,
        chunk_size=horizon,
        coefficient=coefficient,
        max_prediction_age_ticks=max_prediction_age_ticks,
        capacity=max(4096, horizon * max(1, len(queries))),
    )
    actions = np.zeros((len(command_times), action_dim), dtype=np.float64)
    selected_h = np.full(len(command_times), -1, dtype=np.int64)
    selected_q = np.zeros(len(command_times), dtype=np.int64)
    fallback = np.ones(len(command_times), dtype=bool)
    contributors: list[list[dict[str, Any]]] = []
    for tick in range(len(command_times)):
        for sequence in by_tick.get(tick, ()):
            query = queries[sequence - 1]
            ensemble.add_prediction(
                query_id=sequence,
                query_timestamp_ns=int(query["query_end_ns"]),
                query_command_tick=tick,
                chunk=chunks[sequence - 1],
            )
        selection = ensemble.select(command_tick=tick, command_timestamp_ns=int(command_times[tick]))
        contributors.append([item.as_dict() for item in selection.contributors])
        if selection.action is not None:
            actions[tick] = selection.action
        if selection.contributors:
            newest = selection.contributors[-1]
            selected_h[tick] = newest.source_horizon
            selected_q[tick] = newest.query_id
            fallback[tick] = False
    stats = ensemble.stats()
    stats["prediction_points_selected"] = int(sum(len(row) for row in contributors))
    stats["prediction_points_discarded"] = int(
        len(queries) * horizon - stats["prediction_points_selected"]
    )
    return {
        "actions": actions,
        "selected_horizon": selected_h,
        "selected_query": selected_q,
        "contributors": contributors,
        "fallback": fallback,
        "buffer_stats": stats,
    }


def _metrics(result: dict[str, Any], command_times: np.ndarray, *, threshold: float = 0.1) -> dict[str, Any]:
    actions = result["actions"]
    closure = np.min(actions[:, 6:11], axis=1) >= threshold
    onset = np.flatnonzero(closure)
    selected_h = result["selected_horizon"]
    valid_h = selected_h[selected_h >= 0]
    contributors = result["contributors"]
    contributor_ages = [
        tick - int(item["query_command_tick"])
        for tick, row in enumerate(contributors)
        for item in row
    ]
    steps = np.diff(actions, axis=0) if len(actions) > 1 else np.empty((0, actions.shape[1]))
    return {
        "coordinated_closure_threshold": threshold,
        "coordinated_closure_fraction": float(np.mean(closure)),
        "first_coordinated_closure_command_tick": None if not len(onset) else int(onset[0]),
        "first_coordinated_closure_time_s": None if not len(onset) else float((command_times[onset[0]] - command_times[0]) / 1e9),
        "selected_source_horizon": {
            "count": int(len(valid_h)),
            "min": None if not len(valid_h) else int(np.min(valid_h)),
            "p50": None if not len(valid_h) else float(np.quantile(valid_h, 0.5)),
            "p95": None if not len(valid_h) else float(np.quantile(valid_h, 0.95)),
            "max": None if not len(valid_h) else int(np.max(valid_h)),
        },
        "contributing_prediction_age_ticks": (
            None if not contributor_ages else {
                "p50": float(np.quantile(contributor_ages, 0.5)),
                "p95": float(np.quantile(contributor_ages, 0.95)),
                "max": int(np.max(contributor_ages)),
            }
        ),
        "fallback_fraction": float(np.mean(result["fallback"])),
        "mean_abs_command_step": float(np.mean(np.abs(steps))) if len(steps) else 0.0,
        "p95_abs_command_step": float(np.quantile(np.abs(steps), 0.95)) if len(steps) else 0.0,
        "max_abs_command_step": float(np.max(np.abs(steps))) if len(steps) else 0.0,
        "query_counts": {str(key): int(value) for key, value in Counter(result["selected_query"]).items()},
        "buffer_stats": result["buffer_stats"],
    }


def replay_rollout(root: Path) -> dict[str, Any]:
    archive = np.load(root / "act_predictions.npz")
    chunks = np.asarray(archive["chunks"], dtype=np.float64)
    if chunks.ndim != 3 or not np.isfinite(chunks).all():
        raise ValueError(f"expected finite [N,H,A] chunks, got {chunks.shape}")
    queries = _jsonl(root / "queries.jsonl")
    commands = _jsonl(root / "commands.jsonl")
    command_times = np.asarray([int(row["command_start_ns"]) for row in commands], dtype=np.int64)
    available = _available_queries(queries, command_times)
    outputs: dict[str, Any] = {}
    for k in (2, 8, 16):
        result = _consume_k(chunks, available, len(commands), k)
        outputs[f"consume_k{k}"] = _metrics(result, command_times)
    result = _temporal_ensemble(chunks, queries, command_times, coefficient=0.01)
    outputs["temporal_ensemble_m0.01"] = _metrics(result, command_times)
    return {
        "schema_version": "embodied_lab.act_temporal_executor_replay.v1",
        "rollout": str(root.resolve()),
        "prediction_shape": list(chunks.shape),
        "command_count": len(commands),
        "query_count": len(queries),
        "query_available_tick_range": (
            None if not available else [min(available.values()), max(available.values())]
        ),
        "modes": outputs,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rollout", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = replay_rollout(args.rollout.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
