#!/usr/bin/env python3
"""Analyze saved physical ACT chunks and teacher-forced bottle replays offline."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
from typing import Any

import numpy as np


JAKA_NAMES = tuple(f"JAKA_J{index}" for index in range(1, 7))
RH56_NAMES = (
    "index",
    "middle",
    "ring",
    "pinky",
    "thumb_close",
    "thumb_lateral",
)
PLOTTED_HORIZONS = (0, 1, 2, 4, 8, 15)
HEURISTIC_CLOSURE_THRESHOLD = 0.1


def _json_lines(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{path}:{number}: malformed JSON") from exc
        if not isinstance(value, dict):
            raise ValueError(f"{path}:{number}: expected a JSON object")
        rows.append(value)
    return rows


def _distribution(values: np.ndarray | list[float]) -> dict[str, float | int] | None:
    array = np.asarray(values, dtype=np.float64)
    if not array.size:
        return None
    return {
        "count": int(array.size),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "p99": float(np.quantile(array, 0.99)),
        "max": float(np.max(array)),
    }


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def _rate_hz(timestamps_ns: list[int]) -> float | None:
    if len(timestamps_ns) < 2 or timestamps_ns[-1] <= timestamps_ns[0]:
        return None
    return (len(timestamps_ns) - 1) * 1e9 / (timestamps_ns[-1] - timestamps_ns[0])


def _source_age(
    rows: list[dict[str, Any]], source_key: str
) -> dict[str, float | int] | None:
    values = [
        (int(row["observation_ready_ns"]) - int(row[source_key])) / 1e6
        for row in rows
        if row.get(source_key) is not None
    ]
    result = _distribution(values)
    if result is not None:
        result["min"] = float(np.min(values))
        result["future_source_count"] = int(np.count_nonzero(np.asarray(values) < 0.0))
    return result


def _projection_summary(summary: dict[str, Any]) -> dict[str, Any]:
    events = summary.get("rh56_command_projection", {}).get("events", [])
    corrections: dict[str, list[float]] = defaultdict(list)
    for event in events:
        corrections[str(event["channel"])].append(abs(float(event["delta"])))
    return {
        "event_count": len(events),
        "per_channel": {
            channel: _distribution(values) for channel, values in corrections.items()
        },
    }


def _rh56_domain_summary(
    actions: np.ndarray,
    *,
    valid: np.ndarray | None = None,
    legal_min: float = 0.0,
    legal_max: float = 1.0,
) -> dict[str, Any]:
    array = np.asarray(actions, dtype=np.float64)
    if array.shape[-1] != 12 or not np.isfinite(array).all():
        raise ValueError("actions must be finite with a final dimension of 12")
    rh56 = array[..., 6:]
    if valid is not None:
        mask = np.asarray(valid, dtype=bool)
        if mask.shape != rh56.shape[:-1]:
            raise ValueError("valid mask does not match action rows/horizons")
        rh56 = rh56[mask]
    else:
        rh56 = rh56.reshape(-1, 6)
    if not rh56.size:
        raise ValueError("no RH56 predictions selected")
    per_channel: dict[str, Any] = {}
    for index, name in enumerate(RH56_NAMES):
        values = rh56[:, index]
        below = values < legal_min
        above = values > legal_max
        per_channel[name] = {
            "minimum": float(np.min(values)),
            "maximum": float(np.max(values)),
            "fraction_below_legal": float(np.mean(below)),
            "fraction_above_legal": float(np.mean(above)),
            "below_correction": _distribution(legal_min - values[below]),
            "above_correction": _distribution(values[above] - legal_max),
        }
    return {
        "legal_range": [legal_min, legal_max],
        "sample_count_per_channel": int(len(rh56)),
        "per_channel": per_channel,
    }


def _chunk_diagnostic(chunks: np.ndarray) -> dict[str, Any]:
    closing = chunks[:, :, 6:11]
    coordinated = np.min(closing, axis=2)
    early = np.max(coordinated[:, :2], axis=1)
    later = np.max(coordinated[:, 2:], axis=1)
    discarded = (early < HEURISTIC_CLOSURE_THRESHOLD) & (
        later >= HEURISTIC_CLOSURE_THRESHOLD
    )
    later_present = later >= HEURISTIC_CLOSURE_THRESHOLD
    if not np.any(discarded):
        answer = "NO"
    elif float(np.mean(discarded)) >= 0.05:
        answer = "YES"
    else:
        answer = "INCONCLUSIVE"
    horizon_delta = np.abs(chunks - chunks[:, :1, :])
    return {
        "heuristic_only": (
            "A grasp-like row is defined only for this diagnostic as all first five "
            "RH56 closure channels >= 0.1; this is not a semantic task label."
        ),
        "later_grasp_discarded_answer": answer,
        "queries": int(chunks.shape[0]),
        "queries_with_grasp_like_later_horizon": int(np.sum(later_present)),
        "queries_open_at_0_1_but_grasp_like_at_2_15": int(np.sum(discarded)),
        "fraction_open_at_0_1_but_grasp_like_at_2_15": float(np.mean(discarded)),
        "later_minus_early_coordinated_closure": _distribution(later - early),
        "mean_abs_difference_from_horizon_0": {
            "jaka": np.mean(horizon_delta[:, :, :6], axis=(0, 2)).tolist(),
            "rh56": np.mean(horizon_delta[:, :, 6:], axis=(0, 2)).tolist(),
        },
        "rh56_mean_by_horizon": np.mean(chunks[:, :, 6:], axis=0).tolist(),
        "rh56_min_by_horizon": np.min(chunks[:, :, 6:], axis=0).tolist(),
        "rh56_max_by_horizon": np.max(chunks[:, :, 6:], axis=0).tolist(),
    }


def _jaka_status_consumer_version(
    queries: list[dict[str, Any]], commands: list[dict[str, Any]]
) -> dict[str, Any]:
    query_status = {
        sequence: int(row["jaka_observation_ns"])
        for sequence, row in enumerate(queries, start=1)
    }
    compared = 0
    matched = 0
    for row in commands:
        sequence = int(row.get("policy_query_sequence", 0))
        if sequence <= 0 or sequence not in query_status:
            continue
        command_status = row.get("jaka_status", {}).get("observation_ns")
        if command_status is None:
            continue
        compared += 1
        matched += int(int(command_status) == query_status[sequence])
    ratio = None if compared == 0 else matched / compared
    if ratio == 1.0:
        classification = "post_fix_query_attached_status"
    elif ratio is not None and ratio < 0.1:
        classification = "pre_fix_independent_command_socket_consumer"
    else:
        classification = "inconclusive"
    return {
        "classification": classification,
        "commands_compared": compared,
        "timestamp_matches": matched,
        "match_ratio": ratio,
    }


def _simulate_consumption(chunks: np.ndarray, consume_actions: int) -> dict[str, Any]:
    """Simulate a 15 Hz latest-query source consumed by a 30 Hz command loop."""

    command_count = chunks.shape[0] * 2
    active_query = -1
    active_index = consume_actions
    commands: list[np.ndarray] = []
    adopted_queries: list[int] = []
    adopted_command_slots: list[int] = []
    indices: list[int] = []
    for command_slot in range(command_count):
        latest_query = min(command_slot // 2, chunks.shape[0] - 1)
        if active_query < 0 or (
            active_index >= consume_actions and latest_query > active_query
        ):
            active_query = latest_query
            active_index = 0
            adopted_queries.append(active_query)
            adopted_command_slots.append(command_slot)
        index = min(active_index, consume_actions - 1)
        commands.append(chunks[active_query, index])
        indices.append(index)
        active_index += 1
    command_array = np.stack(commands)
    boundaries = np.asarray(adopted_command_slots[1:], dtype=np.int64)
    if boundaries.size:
        jumps = np.abs(command_array[boundaries] - command_array[boundaries - 1])
    else:
        jumps = np.empty((0, 12), dtype=np.float32)
    maximum_nominal_age_ms = (consume_actions - 1) * 1000.0 / 30.0
    return {
        "consume_actions": consume_actions,
        "queries_available": int(chunks.shape[0]),
        "queries_adopted": len(adopted_queries),
        "queries_superseded_before_adoption": int(chunks.shape[0] - len(adopted_queries)),
        "maximum_chunk_index_used": int(max(indices)),
        "nominal_open_loop_ms": consume_actions * 1000.0 / 30.0,
        "maximum_nominal_prediction_age_ms": maximum_nominal_age_ms,
        "within_250ms_policy_age_gate": maximum_nominal_age_ms <= 250.0,
        "requery_boundary_abs_jump": {
            "jaka": _distribution(jumps[:, :6].reshape(-1)),
            "rh56": _distribution(jumps[:, 6:].reshape(-1)),
        },
        "rh56_command_min": np.min(command_array[:, 6:], axis=0).tolist(),
        "rh56_command_max": np.max(command_array[:, 6:], axis=0).tolist(),
    }


def _plot_rollout(
    output: Path,
    chunks: np.ndarray,
    queries: list[dict[str, Any]],
    commands: list[dict[str, Any]],
) -> list[str]:
    import matplotlib.pyplot as plt

    usable_queries = min(len(queries), chunks.shape[0])
    origin_ns = int(queries[0]["query_start_ns"])
    query_time = np.asarray(
        [(int(row["query_start_ns"]) - origin_ns) / 1e9 for row in queries[:usable_queries]]
    )
    paths: list[str] = []
    for label, start, names in (("jaka", 0, JAKA_NAMES), ("rh56", 6, RH56_NAMES)):
        figure, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
        for channel, axis in enumerate(axes.flat):
            for horizon in PLOTTED_HORIZONS:
                axis.plot(
                    query_time,
                    chunks[:usable_queries, horizon, start + channel],
                    linewidth=1.6 if horizon < 2 else 0.9,
                    alpha=0.95 if horizon < 2 else 0.7,
                    label=f"h={horizon}",
                )
            axis.set_title(names[channel])
            axis.grid(alpha=0.25)
        axes.flat[0].legend(ncol=3, fontsize=8)
        axes[-1, 0].set_xlabel("physical query time (s)")
        axes[-1, 1].set_xlabel("physical query time (s)")
        figure.suptitle(
            f"{label.upper()} predicted native targets; h=0,1 were configured for execution"
        )
        figure.tight_layout()
        path = output / f"predicted_{label}_horizons.png"
        figure.savefig(path, dpi=140)
        plt.close(figure)
        paths.append(str(path))

    figure, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    command_time = np.asarray(
        [(int(row["command_start_ns"]) - origin_ns) / 1e9 for row in commands]
    )
    measured_available = all(
        row.get("rh56_position") is not None for row in commands
    ) and bool(commands)
    for channel, axis in enumerate(axes.flat):
        for horizon in PLOTTED_HORIZONS:
            axis.plot(
                query_time,
                chunks[:usable_queries, horizon, 6 + channel],
                linewidth=1.5 if horizon < 2 else 0.8,
                alpha=0.9 if horizon < 2 else 0.55,
                label=f"pred h={horizon}",
            )
        projected = [float(row["projected_command"][6 + channel]) for row in commands]
        axis.plot(command_time, projected, color="black", linewidth=1.0, label="executed")
        if measured_available:
            measured = [float(row["rh56_position"][channel]) for row in commands]
            axis.plot(command_time, measured, color="tab:green", linewidth=1.0, label="measured")
        axis.vlines(query_time, *axis.get_ylim(), color="gray", alpha=0.015, linewidth=0.4)
        axis.set_title(RH56_NAMES[channel])
        axis.grid(alpha=0.25)
    axes.flat[0].legend(ncol=3, fontsize=7)
    axes[-1, 0].set_xlabel("physical time (s)")
    axes[-1, 1].set_xlabel("physical time (s)")
    suffix = ", and measured position" if measured_available else ""
    figure.suptitle(f"RH56 prediction horizons and executed target{suffix}")
    figure.tight_layout()
    path = output / "rh56_prediction_execution_measurement.png"
    figure.savefig(path, dpi=140)
    plt.close(figure)
    paths.append(str(path))
    return paths


def analyze_rollout(args: argparse.Namespace) -> None:
    root = args.rollout.resolve()
    required = (
        root / "act_predictions.npz",
        root / "queries.jsonl",
        root / "commands.jsonl",
        root / "rollout_summary.json",
    )
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"missing rollout artifacts: {missing}")
    archive = np.load(required[0])
    if "chunks" not in archive:
        raise ValueError("act_predictions.npz does not contain chunks")
    chunks = np.asarray(archive["chunks"], dtype=np.float64)
    if chunks.ndim != 3 or chunks.shape[1:] != (16, 12) or not np.isfinite(chunks).all():
        raise ValueError(f"expected finite [N,16,12] chunks, got {chunks.shape}")
    queries = _json_lines(required[1])
    commands = _json_lines(required[2])
    summary = json.loads(required[3].read_text(encoding="utf-8"))
    query_times = [int(row["query_start_ns"]) for row in queries]
    command_times = [int(row["command_start_ns"]) for row in commands]
    query_jaka = [int(row["jaka_observation_ns"]) for row in queries]
    raw_actions = np.asarray(
        [row["raw_policy_action"] for row in commands if row.get("raw_policy_action") is not None],
        dtype=np.float64,
    )
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "schema_version": "embodied_lab.act_bottle_rollout_analysis.v1",
        "rollout": str(root),
        "checkpoint": summary.get("checkpoint"),
        "rows": {"queries": len(queries), "commands": len(commands), "chunks": len(chunks)},
        "rates_hz": {
            "query": _rate_hz(query_times),
            "command": _rate_hz(command_times),
        },
        "interval_ms": {
            "query": _distribution(np.diff(query_times) / 1e6),
            "command": _distribution(np.diff(command_times) / 1e6),
        },
        "jaka_observation": {
            "unique_timestamps": len(set(query_jaka)),
            "repeated_timestamps": len(query_jaka) - len(set(query_jaka)),
            "age_ms": _source_age(queries, "jaka_observation_ns"),
            "consumer_version_evidence": _jaka_status_consumer_version(
                queries, commands
            ),
        },
        "source_age_ms": {
            key: _source_age(queries, source)
            for key, source in (
                ("workspace", "workspace_host_ns"),
                ("wrist", "wrist_host_ns"),
                ("rh56_angle", "rh56_angle_ns"),
                ("rh56_force", "rh56_force_ns"),
            )
        },
        "camera": summary.get("camera"),
        "rh56_raw_prediction": {
            "min": np.min(raw_actions[:, 6:], axis=0).tolist(),
            "max": np.max(raw_actions[:, 6:], axis=0).tolist(),
            "executed_action_domain": _rh56_domain_summary(raw_actions),
            "all_chunk_horizons_domain": _rh56_domain_summary(chunks),
        },
        "rh56_projection": _projection_summary(summary),
        "native_metrics": (
            json.loads((root / "native_metrics.json").read_text(encoding="utf-8"))
            if (root / "native_metrics.json").is_file()
            else None
        ),
        "chunk_diagnostic": _chunk_diagnostic(chunks),
        "consume_actions_ablation": {
            str(value): _simulate_consumption(chunks, value) for value in (2, 4, 8, 16)
        },
        "measured_rh56_position": (
            "available"
            if commands and all(row.get("rh56_position") is not None for row in commands)
            else "unavailable in this saved rollout; not reconstructed"
        ),
        "plots": _plot_rollout(output, chunks, queries, commands),
    }
    _write_json(output / "rollout_analysis.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


def _teacher_forced_episode(
    predictions: np.ndarray,
    ground_truth: np.ndarray,
    valid: np.ndarray,
    frames: np.ndarray,
) -> dict[str, Any]:
    gt_now = ground_truth[:, 0]
    pred_now = predictions[:, 0]
    gt_closing = gt_now[:, 6:11]
    pred_closing = pred_now[:, 6:11]
    recorded_closed = np.min(gt_closing, axis=1) >= HEURISTIC_CLOSURE_THRESHOLD
    immediate_predicted_closed = np.min(pred_closing, axis=1) >= HEURISTIC_CLOSURE_THRESHOLD
    future_gt = np.min(ground_truth[:, :, 6:11], axis=2)
    transition_queries = (
        (np.max(future_gt[:, :2], axis=1) < HEURISTIC_CLOSURE_THRESHOLD)
        & (np.max(future_gt[:, 2:], axis=1) >= HEURISTIC_CLOSURE_THRESHOLD)
        & np.any(valid[:, 2:], axis=1)
    )
    pred_coord = np.min(predictions[:, :, 6:11], axis=2)
    transition_predicted_later = (
        (np.max(pred_coord[:, :2], axis=1) < HEURISTIC_CLOSURE_THRESHOLD)
        & (np.max(pred_coord[:, 2:], axis=1) >= HEURISTIC_CLOSURE_THRESHOLD)
    )
    errors = np.abs(predictions - ground_truth)
    first_closed_frame = int(frames[np.flatnonzero(recorded_closed)[0]]) if np.any(recorded_closed) else None
    return {
        "rows": int(len(frames)),
        "first_recorded_closure_frame": first_closed_frame,
        "recorded_closure_rows": int(np.sum(recorded_closed)),
        "immediate_predicted_closure_ratio_on_recorded_closure_rows": (
            None
            if not np.any(recorded_closed)
            else float(np.mean(immediate_predicted_closed[recorded_closed]))
        ),
        "prediction_mean_on_recorded_closure_rows": (
            None
            if not np.any(recorded_closed)
            else np.mean(pred_closing[recorded_closed], axis=0).tolist()
        ),
        "ground_truth_transition_in_horizons_2_15_queries": int(np.sum(transition_queries)),
        "predicted_transition_in_horizons_2_15_on_those_queries": int(
            np.sum(transition_predicted_later & transition_queries)
        ),
        "first_action_mae": {
            "jaka": float(np.mean(errors[:, 0, :6])),
            "rh56": float(np.mean(errors[:, 0, 6:])),
        },
        "all_valid_horizon_mae": {
            "jaka": float(np.mean(errors[:, :, :6][valid])),
            "rh56": float(np.mean(errors[:, :, 6:][valid])),
        },
        "temporal_chunk_dynamic_range": {
            "jaka_mean_peak_to_peak": float(np.mean(np.ptp(predictions[:, :, :6], axis=1))),
            "rh56_mean_peak_to_peak": float(np.mean(np.ptp(predictions[:, :, 6:], axis=1))),
        },
    }


def _plot_teacher_forced(
    output: Path,
    source_episode: int,
    predictions: np.ndarray,
    ground_truth: np.ndarray,
    frames: np.ndarray,
) -> list[str]:
    import matplotlib.pyplot as plt

    gt_closed = np.min(ground_truth[:, 0, 6:11], axis=1) >= HEURISTIC_CLOSURE_THRESHOLD
    transition = int(frames[np.flatnonzero(gt_closed)[0]]) if np.any(gt_closed) else int(frames[0])
    time_s = (frames - transition) / 30.0
    paths: list[str] = []
    for label, start, names in (("jaka", 0, JAKA_NAMES), ("rh56", 6, RH56_NAMES)):
        figure, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
        for channel, axis in enumerate(axes.flat):
            axis.plot(
                time_s,
                ground_truth[:, 0, start + channel],
                color="black",
                linewidth=1.7,
                label="recorded action",
            )
            for horizon in PLOTTED_HORIZONS:
                axis.plot(
                    time_s,
                    predictions[:, horizon, start + channel],
                    linewidth=1.2 if horizon < 2 else 0.7,
                    alpha=0.9 if horizon < 2 else 0.55,
                    label=f"pred h={horizon}",
                )
            axis.axvline(0.0, color="tab:red", linestyle="--", linewidth=0.9)
            axis.set_title(names[channel])
            axis.grid(alpha=0.25)
        axes.flat[0].legend(ncol=3, fontsize=7)
        axes[-1, 0].set_xlabel("seconds from heuristic recorded-closure onset")
        axes[-1, 1].set_xlabel("seconds from heuristic recorded-closure onset")
        figure.suptitle(
            f"Held-out source episode {source_episode}: teacher-forced {label.upper()} targets"
        )
        figure.tight_layout()
        path = output / f"teacher_forced_ep{source_episode}_{label}.png"
        figure.savefig(path, dpi=140)
        plt.close(figure)
        paths.append(str(path))
    return paths


def analyze_teacher_forced(args: argparse.Namespace) -> None:
    archive = np.load(args.arrays.resolve())
    required = ("predictions", "ground_truth", "valid", "source_episode", "source_frame")
    missing = [key for key in required if key not in archive]
    if missing:
        raise ValueError(f"teacher-forced archive is missing {missing}")
    predictions = np.asarray(archive["predictions"], dtype=np.float64)
    ground_truth = np.asarray(archive["ground_truth"], dtype=np.float64)
    valid = np.asarray(archive["valid"], dtype=bool)
    episodes = np.asarray(archive["source_episode"], dtype=np.int64).reshape(-1)
    frames = np.asarray(archive["source_frame"], dtype=np.int64).reshape(-1)
    if predictions.shape != ground_truth.shape or predictions.shape[1:] != (16, 12):
        raise ValueError("teacher-forced arrays must have matching [N,16,12] shapes")
    if valid.shape != predictions.shape[:2] or len(episodes) != len(predictions):
        raise ValueError("teacher-forced provenance shapes do not match predictions")
    if not np.isfinite(predictions).all() or not np.isfinite(ground_truth).all():
        raise ValueError("teacher-forced arrays contain non-finite values")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    per_episode: dict[str, Any] = {}
    plots: list[str] = []
    for source_episode in sorted(np.unique(episodes).tolist()):
        selected = episodes == source_episode
        per_episode[str(source_episode)] = _teacher_forced_episode(
            predictions[selected], ground_truth[selected], valid[selected], frames[selected]
        )
        plots.extend(
            _plot_teacher_forced(
                output,
                int(source_episode),
                predictions[selected],
                ground_truth[selected],
                frames[selected],
            )
        )
    closure_ratios = [
        value["immediate_predicted_closure_ratio_on_recorded_closure_rows"]
        for value in per_episode.values()
        if value["immediate_predicted_closure_ratio_on_recorded_closure_rows"] is not None
    ]
    transition_total = sum(
        value["ground_truth_transition_in_horizons_2_15_queries"]
        for value in per_episode.values()
    )
    transition_predicted = sum(
        value["predicted_transition_in_horizons_2_15_on_those_queries"]
        for value in per_episode.values()
    )
    report = {
        "schema_version": "embodied_lab.act_bottle_teacher_forced_analysis.v1",
        "arrays": str(args.arrays.resolve()),
        "heuristic_only": (
            "Recorded closure is defined as all first five RH56 action channels >= 0.1; "
            "the dataset does not contain semantic phase labels."
        ),
        "episodes": sorted(np.unique(episodes).tolist()),
        "per_episode": per_episode,
        "answers": {
            "immediate_closure_on_recorded_grasp_observations": (
                "YES" if closure_ratios and min(closure_ratios) >= 0.95 else "INCONCLUSIVE"
            ),
            "anticipatory_closure_for_recorded_transition_within_chunk": (
                "NO" if transition_total and transition_predicted == 0 else "INCONCLUSIVE"
            ),
            "transition_queries": transition_total,
            "transitions_predicted_later": transition_predicted,
        },
        "rh56_prediction_domain": _rh56_domain_summary(
            predictions, valid=valid
        ),
        "plots": plots,
    }
    _write_json(output / "teacher_forced_analysis.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    rollout = subparsers.add_parser("rollout", help="analyze one saved physical rollout")
    rollout.add_argument("--rollout", type=Path, required=True)
    rollout.add_argument("--output", type=Path, required=True)
    rollout.set_defaults(func=analyze_rollout)
    teacher = subparsers.add_parser(
        "teacher-forced", help="analyze a saved held-out checkpoint replay"
    )
    teacher.add_argument("--arrays", type=Path, required=True)
    teacher.add_argument("--output", type=Path, required=True)
    teacher.set_defaults(func=analyze_teacher_forced)
    return parser


def main() -> int:
    args = make_parser().parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
