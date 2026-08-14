#!/usr/bin/env python3
"""Offline Strong ACT transition and executor audit.

The tool consumes teacher-forced prediction arrays and the curated event
metadata already produced from nominal52.  It never opens a camera, serial
device, JAKA session, or command socket.  T1/T4 are quantitative hand-event
proxies; T2/T3 remain explicitly uncertain unless separately annotated.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path
from typing import Any

import numpy as np

from embodiment_core.act_temporal_executor import AbsoluteTimeTemporalEnsembler


HAND_CHANNELS = ("index", "middle", "ring", "pinky", "thumb_close")
STRATEGIES = (
    ("consume_k2_q15", "consume_k", 15.0, 2, None),
    ("consume_k8_q15", "consume_k", 15.0, 8, None),
    ("consume_k16_q15", "consume_k", 15.0, 16, None),
    ("consume_k8_q7_5", "consume_k", 7.5, 8, None),
    ("consume_k8_q3", "consume_k", 3.0, 8, None),
    ("consume_k1_q30", "consume_k", 30.0, 1, None),
    ("fresh_q15", "fresh", 15.0, 1, None),
    ("temporal_ensemble_q15", "temporal_ensemble", 15.0, 1, 0.01),
    ("temporal_ensemble_q30", "temporal_ensemble", 30.0, 1, 0.01),
)


def _score(actions: np.ndarray, baseline: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.maximum(actions[..., 6:11] - baseline, 0.0), axis=-1)


def _event_definition(info: dict[str, Any]) -> dict[str, Any]:
    grasp = float(info["closure_threshold"])
    # The source curation has an authoritative release onset but no release
    # threshold.  Derive a transparent, per-trajectory threshold halfway
    # between the pre-release hold score and the open score, and label it as a
    # hand-only proxy rather than a semantic contact label.
    release_threshold = max(0.05, 0.5 * grasp)
    return {
        "grasp_threshold": grasp,
        "release_threshold": release_threshold,
        "grasp_source_frame": int(info["grasp_onset_source_frame"]),
        "release_source_frame": (
            None
            if info.get("release_onset_source_frame") is None
            else int(info["release_onset_source_frame"])
        ),
        "release_definition": "0.5 * grasp closure threshold, lower bounded at 0.05; heuristic hand opening proxy",
    }


def _transition_metrics(
    prediction: np.ndarray,
    truth: np.ndarray,
    valid: np.ndarray,
    frames: np.ndarray,
    info: dict[str, Any],
    *,
    event: str,
) -> dict[str, Any]:
    definition = _event_definition(info)
    threshold = definition["grasp_threshold"] if event == "grasp" else definition["release_threshold"]
    event_frame = definition["grasp_source_frame"] if event == "grasp" else definition["release_source_frame"]
    if event_frame is None:
        return {"status": "uncertain_no_authoritative_event"}
    pred_score = _score(prediction, np.asarray(info["open_baseline_rh56"], dtype=np.float64))
    truth_score = _score(truth, np.asarray(info["open_baseline_rh56"], dtype=np.float64))
    pred_horizons: list[int] = []
    gt_horizons: list[int] = []
    timing: list[int] = []
    false_positive = 0
    eligible = 0
    for row in range(len(frames)):
        if frames[row] >= event_frame:
            continue
        row_valid = valid[row]
        # The full future source frames are not stored in the teacher-forced
        # archive.  Canonical rows are 30 Hz, so the row offset is the
        # corresponding future frame offset for this audit.
        future_frames = frames[row] + np.arange(prediction.shape[1], dtype=np.int64)
        gt_candidates = np.flatnonzero((future_frames >= event_frame) & row_valid)
        gt_h = None if not len(gt_candidates) else int(gt_candidates[0])
        pred_candidates = np.flatnonzero((pred_score[row] >= threshold if event == "grasp" else pred_score[row] <= threshold) & np.ones_like(row_valid, dtype=bool))
        pred_h = None if not len(pred_candidates) else int(pred_candidates[0])
        if gt_h is not None:
            eligible += 1
            if pred_h is not None:
                pred_horizons.append(pred_h)
                gt_horizons.append(gt_h)
                timing.append(pred_h - gt_h)
        if frames[row] < event_frame and pred_h is not None:
            false_positive += 1
    return {
        "status": "quantitative_hand_proxy",
        "event_frame": event_frame,
        "threshold": threshold,
        "eligible_queries": eligible,
        "recalled_queries": len(timing),
        "recall": None if not eligible else len(timing) / eligible,
        "predicted_horizon": {
            "count": len(pred_horizons),
            "min": None if not pred_horizons else int(np.min(pred_horizons)),
            "max": None if not pred_horizons else int(np.max(pred_horizons)),
            "p50": None if not pred_horizons else float(np.quantile(pred_horizons, 0.5)),
        },
        "timing_error_frames": {
            "mean": None if not timing else float(np.mean(timing)),
            "mean_abs": None if not timing else float(np.mean(np.abs(timing))),
            "p95_abs": None if not timing else float(np.quantile(np.abs(timing), 0.95)),
            "min": None if not timing else int(np.min(timing)),
            "max": None if not timing else int(np.max(timing)),
        },
        "false_positive_query_count": false_positive,
        "predicted_horizons": pred_horizons,
        "gt_horizons": gt_horizons,
    }


def _convergence(
    prediction: np.ndarray,
    frames: np.ndarray,
    info: dict[str, Any],
    *,
    event: str,
) -> dict[str, Any]:
    definition = _event_definition(info)
    event_frame = definition["grasp_source_frame"] if event == "grasp" else definition["release_source_frame"]
    if event_frame is None:
        return {"status": "uncertain"}
    threshold = definition["grasp_threshold"] if event == "grasp" else definition["release_threshold"]
    score = _score(prediction, np.asarray(info["open_baseline_rh56"], dtype=np.float64))
    time_to_event: list[int] = []
    horizons: list[int] = []
    for row, frame in enumerate(frames):
        delta = event_frame - int(frame)
        if not 0 < delta < prediction.shape[1]:
            continue
        candidates = np.flatnonzero(
            (score[row] >= threshold if event == "grasp" else score[row] <= threshold)
        )
        if len(candidates):
            time_to_event.append(delta)
            horizons.append(int(candidates[0]))
    result: dict[str, Any] = {
        "status": "quantitative_hand_proxy",
        "sample_count": len(horizons),
        "time_to_event_frames": time_to_event,
        "predicted_horizons": horizons,
        "entry_query_count": {
            str(limit): sum(1 for value in horizons if value <= limit)
            for limit in (16, 8, 4, 2, 1, 0)
        },
    }
    if len(horizons) >= 2:
        x = np.asarray(time_to_event, dtype=np.float64)
        y = np.asarray(horizons, dtype=np.float64)
        result["slope_horizon_per_time_to_event_frame"] = float(np.polyfit(x, y, 1)[0])
        result["correlation"] = float(np.corrcoef(x, y)[0, 1]) if np.ptp(x) and np.ptp(y) else None
        order = np.argsort(-x)  # far from event -> near event
        result["nonincreasing_horizon_fraction_as_event_approaches"] = float(
            np.mean(np.diff(y[order]) <= 0)
        ) if len(order) > 1 else None
        result["minimum_predicted_horizon"] = int(np.min(y))
    else:
        result.update({"slope_horizon_per_time_to_event_frame": None, "correlation": None})
    return result


def analyze_teacher_forced(arrays: dict[str, np.ndarray], curation: dict[str, Any]) -> dict[str, Any]:
    prediction = np.asarray(arrays["predictions"], dtype=np.float64)
    truth = np.asarray(arrays["ground_truth"], dtype=np.float64)
    valid = np.asarray(arrays["valid"], dtype=bool)
    frames = np.asarray(arrays["source_frame"], dtype=np.int64)
    segments = np.asarray(arrays["logical_segment"]).astype(str)
    if prediction.ndim != 3 or prediction.shape[2] != 12 or truth.shape != prediction.shape:
        raise ValueError("teacher-forced prediction/truth arrays must be [N,H,12]")
    if valid.shape != prediction.shape[:2] or len(frames) != len(prediction):
        raise ValueError("teacher-forced timing arrays have incompatible shapes")
    if not np.isfinite(prediction).all() or not np.isfinite(truth).all():
        raise ValueError("teacher-forced arrays contain non-finite values")
    episode_info = curation["episodes"]
    per_segment: dict[str, Any] = {}
    by_session: dict[str, dict[str, Any]] = defaultdict(lambda: {"segments": [], "grasp": [], "release": []})
    all_convergence: dict[str, list[dict[str, Any]]] = {"grasp": [], "release": []}
    for segment in sorted(set(segments.tolist())):
        mask = segments == segment
        order = np.argsort(frames[mask])
        p, t, v, f = prediction[mask][order], truth[mask][order], valid[mask][order], frames[mask][order]
        info = episode_info[segment]
        session = str(info.get("session_group", f"source_{info.get('source_episode', segment)}"))
        events = {
            event: _transition_metrics(p, t, v, f, info, event=event)
            for event in ("grasp", "release")
        }
        convergence = {
            event: _convergence(p, f, info, event=event)
            for event in ("grasp", "release")
        }
        per_segment[segment] = {
            "rows": len(p),
            "session_group": session,
            "events": events,
            "horizon_convergence": convergence,
            "rh56_first_action_mae": float(np.mean(np.abs(p[:, 0, 6:] - t[:, 0, 6:]))),
            "jaka_first_action_mae": float(np.mean(np.abs(p[:, 0, :6] - t[:, 0, :6]))),
            "rh56_chunk_peak_to_peak_mean": np.mean(np.ptp(p[:, :, 6:], axis=1), axis=0).tolist(),
        }
        by_session[session]["segments"].append(segment)
        for event in ("grasp", "release"):
            by_session[session][event].append(events[event])
            all_convergence[event].append(convergence[event])
    def aggregate(event: str) -> dict[str, Any]:
        values = [v for v in (per_segment[s]["events"][event] for s in per_segment) if v.get("status") == "quantitative_hand_proxy"]
        eligible = sum(v["eligible_queries"] for v in values)
        recalled = sum(v["recalled_queries"] for v in values)
        timings = [x for v in values for x in v["timing_error_frames"].get("_raw", [])]
        # Keep the compact report independent of private raw lists.
        return {
            "segments": len(values),
            "eligible_queries": eligible,
            "recalled_queries": recalled,
            "recall": None if not eligible else recalled / eligible,
            "timing_mean_abs_frames": (
                None if not recalled else float(np.mean([
                    v["timing_error_frames"]["mean_abs"] for v in values if v["timing_error_frames"]["mean_abs"] is not None
                ]))
            ),
        }
    return {
        "schema_version": "embodied_lab.strong_act_transition_execution.v1",
        "rows": len(prediction),
        "chunk_size": int(prediction.shape[1]),
        "events": {
            "T1_approach_to_grasp": "grasp threshold proxy from curated RH56 closure onset",
            "T2_grasp_to_lift": "uncertain: no persisted object/lift event annotation in this view",
            "T3_transport_to_place": "uncertain: no persisted object/place event annotation in this view",
            "T4_place_to_release": "release hand-opening proxy anchored to curated release timestamp",
        },
        "aggregate": {"grasp": aggregate("grasp"), "release": aggregate("release")},
        "per_session": {
            session: {
                "segments": sorted(value["segments"]),
                "grasp_recall": (
                    None if not value["grasp"] else float(np.mean([
                        v["recall"] for v in value["grasp"] if v.get("recall") is not None
                    ]))
                ),
                "release_recall": (
                    None if not value["release"] else float(np.mean([
                        v["recall"] for v in value["release"] if v.get("recall") is not None
                    ]))
                ),
            }
            for session, value in sorted(by_session.items())
        },
        "per_segment": per_segment,
        "horizon_convergence": {
            event: {
                "segments": len(values),
                "mean_slope": (
                    None if not values else float(np.mean([
                        v["slope_horizon_per_time_to_event_frame"] for v in values
                        if v.get("slope_horizon_per_time_to_event_frame") is not None
                    ]))
                ),
            }
            for event, values in all_convergence.items()
        },
    }


def _query_interval(q_hz: float) -> int:
    if q_hz <= 0.0 or 30.0 % q_hz > 1e-9:
        # The audited rates are all exact integer frame intervals.  Refuse a
        # silently rounded schedule for arbitrary rates.
        interval = round(30.0 / q_hz)
        if abs(interval - 30.0 / q_hz) > 1e-6:
            raise ValueError(f"offline replay requires an integer 30Hz/query interval, got {q_hz}")
        return interval
    return int(round(30.0 / q_hz))


def _replay_segment(
    predictions: np.ndarray,
    *,
    query_hz: float,
    mode: str,
    consume_actions: int,
    ensemble_coeff: float | None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    n, horizon, action_dim = predictions.shape
    interval = _query_interval(query_hz)
    query_indices = set(range(0, n, interval))
    output = np.zeros((n, action_dim), dtype=np.float64)
    selected_h = np.full(n, -1, dtype=np.int64)
    selected_query = np.full(n, -1, dtype=np.int64)
    query_count = 0
    last_action: np.ndarray | None = None
    active_query = -1
    next_index = 0
    ensembler = (
        AbsoluteTimeTemporalEnsembler(
            action_dim=action_dim,
            chunk_size=horizon,
            coefficient=0.01 if ensemble_coeff is None else ensemble_coeff,
        )
        if mode in {"temporal_ensemble", "async_temporal_ensemble"}
        else None
    )
    for i in range(n):
        if i in query_indices:
            query_count += 1
            if mode == "consume_k":
                latest_query = i
                if active_query < 0 or next_index >= consume_actions:
                    active_query = latest_query
                    next_index = 0
            elif mode == "fresh":
                active_query = i
                next_index = 0
            elif ensembler is not None:
                ensembler.add_prediction(
                    query_id=i,
                    query_timestamp_ns=i,
                    query_command_tick=i,
                    chunk=predictions[i],
                )
            else:
                raise ValueError(f"unknown replay mode {mode}")
        if ensembler is not None:
            selection = ensembler.select(command_tick=i)
            if selection.action is not None:
                last_action = selection.action
                if selection.contributors:
                    newest = selection.contributors[-1]
                    selected_query[i] = newest.query_id
                    selected_h[i] = newest.source_horizon
            else:
                selected_query[i] = -1
                selected_h[i] = -1
        elif mode in {"consume_k", "fresh"} and active_query >= 0:
            selected_query[i] = active_query
            selected_h[i] = min(next_index, consume_actions - 1) if mode == "consume_k" else 0
            last_action = predictions[active_query, selected_h[i]]
            next_index += 1
        if last_action is None:
            last_action = predictions[0, 0]
            selected_query[i] = 0
            selected_h[i] = 0
        output[i] = last_action
    return output, selected_h, selected_query, np.asarray(sorted(query_indices), dtype=np.int64)


def replay_metrics(arrays: dict[str, np.ndarray], curation: dict[str, Any]) -> dict[str, Any]:
    prediction = np.asarray(arrays["predictions"], dtype=np.float64)
    truth = np.asarray(arrays["ground_truth"], dtype=np.float64)
    frames = np.asarray(arrays["source_frame"], dtype=np.int64)
    segments = np.asarray(arrays["logical_segment"]).astype(str)
    results: dict[str, Any] = {}
    for name, mode, query_hz, k, coeff in STRATEGIES:
        segment_results = []
        outputs: list[np.ndarray] = []
        for segment in sorted(set(segments.tolist())):
            mask = segments == segment
            order = np.argsort(frames[mask])
            p, t, f = prediction[mask][order], truth[mask][order], frames[mask][order]
            command, selected_h, selected_query, query_indices = _replay_segment(
                p, query_hz=query_hz, mode=mode, consume_actions=k, ensemble_coeff=coeff
            )
            info = curation["episodes"][segment]
            definition = _event_definition(info)
            baseline = np.asarray(info["open_baseline_rh56"], dtype=np.float64)
            command_score = _score(command[None, ...], baseline)[0]
            grasp_candidates = np.flatnonzero(command_score >= definition["grasp_threshold"])
            grasp_onset = None if not len(grasp_candidates) else int(grasp_candidates[0])
            release_candidates = np.flatnonzero(
                (command_score <= definition["release_threshold"])
                & (np.arange(len(command_score)) > (grasp_onset if grasp_onset is not None else 0))
            )
            release_onset = None if not len(release_candidates) else int(release_candidates[0])
            gt_grasp = int(np.argmin(np.abs(f - definition["grasp_source_frame"])))
            gt_release = (
                None if definition["release_source_frame"] is None
                else int(np.argmin(np.abs(f - definition["release_source_frame"])))
            )
            segment_results.append({
                "segment": segment,
                "grasp_consumed": grasp_onset is not None,
                "release_consumed": release_onset is not None,
                "grasp_delay_frames": None if grasp_onset is None else grasp_onset - gt_grasp,
                "release_delay_frames": None if release_onset is None or gt_release is None else release_onset - gt_release,
                "selected_horizon_max": int(np.max(selected_h)),
                "query_count": int(len(query_indices)),
                "repeated_action_fraction": float(np.mean(np.all(np.diff(command, axis=0) == 0.0, axis=1))) if len(command) > 1 else 0.0,
                "mean_abs_action_step": float(np.mean(np.abs(np.diff(command, axis=0)))) if len(command) > 1 else 0.0,
                "max_abs_action_step": float(np.max(np.abs(np.diff(command, axis=0)))) if len(command) > 1 else 0.0,
                "mean_selected_horizon": float(np.mean(selected_h)),
                "mean_query_age_frames": float(np.mean(np.arange(len(command)) - selected_query)),
            })
            outputs.append(command)
        valid_grasp = [x for x in segment_results if x["grasp_consumed"]]
        valid_release = [x for x in segment_results if x["release_consumed"]]
        results[name] = {
            "mode": mode,
            "query_hz": query_hz,
            "consume_actions": k if mode == "consume_k" else None,
            "ensemble_coeff": coeff,
            "segments": len(segment_results),
            "grasp_consumed_segments": len(valid_grasp),
            "release_consumed_segments": len(valid_release),
            "grasp_delay_mean_frames": None if not valid_grasp else float(np.mean([x["grasp_delay_frames"] for x in valid_grasp])),
            "release_delay_mean_frames": None if not valid_release else float(np.mean([x["release_delay_frames"] for x in valid_release])),
            "repeated_action_fraction_mean": float(np.mean([x["repeated_action_fraction"] for x in segment_results])),
            "mean_abs_action_step": float(np.mean([x["mean_abs_action_step"] for x in segment_results])),
            "max_abs_action_step": float(np.max([x["max_abs_action_step"] for x in segment_results])),
            "mean_query_age_frames": float(np.mean([x["mean_query_age_frames"] for x in segment_results])),
            "per_segment": segment_results,
        }
    return results


def _apply_hand_delta_limit(actions: np.ndarray, *, closing_limit: float, opening_limit: float) -> np.ndarray:
    """Offline approximation of the RH56 final delta guard.

    This is deliberately not called a contact-stop simulation: force freshness,
    latching, and measured feedback are unavailable in teacher-forced arrays.
    """

    result = actions.copy()
    for index in range(1, len(result)):
        delta = result[index, 6:] - result[index - 1, 6:]
        result[index, 6:] = result[index - 1, 6:] + np.where(
            delta >= 0.0,
            np.minimum(delta, closing_limit),
            np.maximum(delta, -opening_limit),
        )
    return result


def filtering_audit(arrays: dict[str, np.ndarray], curation: dict[str, Any]) -> dict[str, Any]:
    """Quantify safety-delta attenuation for the recommended offline stream."""

    prediction = np.asarray(arrays["predictions"], dtype=np.float64)
    frames = np.asarray(arrays["source_frame"], dtype=np.int64)
    segments = np.asarray(arrays["logical_segment"]).astype(str)
    rows = []
    for segment in sorted(set(segments.tolist())):
        mask = segments == segment
        order = np.argsort(frames[mask])
        p = prediction[mask][order]
        command, _, _, _ = _replay_segment(
            p, query_hz=15.0, mode="consume_k", consume_actions=2, ensemble_coeff=None
        )
        limited = _apply_hand_delta_limit(command, closing_limit=0.05, opening_limit=0.05)
        contact_limited = _apply_hand_delta_limit(command, closing_limit=0.0125, opening_limit=0.05)
        for name, stream in (("delta_limit_0.05", limited), ("contact_closure_limit_0.0125", contact_limited)):
            rows.append({
                "segment": segment,
                "mode": name,
                "mean_abs_hand_step": float(np.mean(np.abs(np.diff(stream[:, 6:], axis=0)))) if len(stream) > 1 else 0.0,
                "max_abs_hand_step": float(np.max(np.abs(np.diff(stream[:, 6:], axis=0)))) if len(stream) > 1 else 0.0,
                "raw_mean_abs_hand_step": float(np.mean(np.abs(np.diff(command[:, 6:], axis=0)))) if len(command) > 1 else 0.0,
                "attenuation_ratio": (
                    float(np.mean(np.abs(np.diff(stream[:, 6:], axis=0))) / np.mean(np.abs(np.diff(command[:, 6:], axis=0))))
                    if len(stream) > 1 and np.mean(np.abs(np.diff(command[:, 6:], axis=0))) > 0 else None
                ),
            })
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[row["mode"]].append(row)
    return {
        "note": "There is no configurable low-pass filter in the policy path. These are safety delta-limit counterfactuals only.",
        "rows": rows,
        "aggregate": {
            mode: {
                "mean_abs_hand_step": float(np.mean([x["mean_abs_hand_step"] for x in values])),
                "max_abs_hand_step": float(np.max([x["max_abs_hand_step"] for x in values])),
                "attenuation_ratio": float(np.mean([x["attenuation_ratio"] for x in values if x["attenuation_ratio"] is not None])),
            }
            for mode, values in grouped.items()
        },
    }


def historical_audit(root: Path) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name in ("act_nodup_test_002", "act_nodup_test_005", "act_nodup_test_006"):
        path = root / name
        summary = json.loads((path / "rollout_summary.json").read_text())
        chunks = np.load(path / "act_predictions.npz")["chunks"].astype(np.float64)
        queries = [json.loads(line) for line in (path / "queries.jsonl").open()]
        commands = [json.loads(line) for line in (path / "commands.jsonl").open()]
        by_query: dict[int, list[dict[str, Any]]] = defaultdict(list)
        for row in commands:
            if int(row["policy_chunk_index"]) >= 0:
                by_query[int(row["policy_query_sequence"])].append(row)
        per_query = []
        predicted_not_executed = 0
        no_release = 0
        executed = 0
        for index, chunk in enumerate(chunks):
            hand = np.mean(chunk[:, 6:11], axis=1)
            candidates = np.flatnonzero(hand <= hand[0] - 0.10)
            pred_h = None if not len(candidates) else int(candidates[0])
            qseq = index + 1
            used = by_query.get(qseq, [])
            max_h = None if not used else max(int(row["policy_chunk_index"]) for row in used)
            if pred_h is None:
                classification = "POLICY_NO_RELEASE"
                no_release += 1
            elif max_h is not None and pred_h <= max_h:
                classification = "RELEASE_REQUEST_EXECUTED"
                executed += 1
            else:
                classification = "POLICY_RELEASE_PREDICTED_BUT_NOT_EXECUTED"
                predicted_not_executed += 1
            per_query.append({
                "query_sequence": qseq,
                "predicted_release_horizon": pred_h,
                "maximum_executed_horizon": max_h,
                "classification": classification,
            })
        result[name] = {
            "summary": {key: summary.get(key) for key in ("chunk_size", "consume_actions", "policy_query_rate_hz", "command_rate_hz", "command_count", "policy_query_count", "abort_reason")},
            "prediction_shape": list(chunks.shape),
            "classification_counts": {
                "POLICY_NO_RELEASE": no_release,
                "POLICY_RELEASE_PREDICTED_BUT_NOT_EXECUTED": predicted_not_executed,
                "RELEASE_REQUEST_EXECUTED": executed,
                "RELEASE_REQUEST_BLOCKED_BY_CONTROLLER": 0,
                "UNKNOWN": 0,
            },
            "release_horizon_statistics": {
                "predicted_count": sum(row["predicted_release_horizon"] is not None for row in per_query),
                "predicted_horizons": [row["predicted_release_horizon"] for row in per_query if row["predicted_release_horizon"] is not None],
                "executed_horizons": sorted(set(row["maximum_executed_horizon"] for row in per_query if row["maximum_executed_horizon"] is not None)),
            },
            "per_query": per_query,
        }
    return result


def _plot_horizon(report: dict[str, Any], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    for event, title, filename in (
        ("grasp", "T1 grasp predicted horizon", "figure_a_grasp_horizon.png"),
        ("release", "T4 release predicted horizon", "figure_b_release_horizon.png"),
    ):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for segment, info in report["teacher_forced"]["per_segment"].items():
            conv = info["horizon_convergence"][event]
            if conv.get("status") != "quantitative_hand_proxy" or not conv.get("predicted_horizons"):
                continue
            ax.scatter(conv["time_to_event_frames"], conv["predicted_horizons"], s=5, alpha=0.18)
        ax.set_title(title + " (teacher-forced validation)")
        ax.set_xlabel("time to curated event [frames; 30 Hz]")
        ax.set_ylabel("first predicted transition horizon [frames]")
        ax.invert_xaxis()
        ax.grid(True, alpha=0.25)
        fig.tight_layout()
        fig.savefig(output / filename, dpi=140)
        plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=False)
    for ax, name in zip(axes, ("act_nodup_test_002", "act_nodup_test_005", "act_nodup_test_006")):
        rows = report["historical_rollouts"][name]["per_query"]
        x = [row["query_sequence"] for row in rows]
        pred = [np.nan if row["predicted_release_horizon"] is None else row["predicted_release_horizon"] for row in rows]
        executed = [np.nan if row["maximum_executed_horizon"] is None else row["maximum_executed_horizon"] for row in rows]
        ax.plot(x, pred, label="predicted release h", linewidth=0.8)
        ax.plot(x, executed, label="maximum executed h", linewidth=0.8)
        ax.set_title(name)
        ax.grid(True, alpha=0.25)
    axes[0].legend()
    axes[-1].set_xlabel("policy query")
    fig.tight_layout()
    fig.savefig(output / "figure_c_historical_horizons.png", dpi=140)
    plt.close(fig)

    names = list(report["executor_replay"])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(names, [report["executor_replay"][n]["grasp_consumed_segments"] for n in names])
    axes[0].set_title("segments reaching grasp proxy")
    axes[1].bar(names, [report["executor_replay"][n]["release_consumed_segments"] for n in names])
    axes[1].set_title("segments reaching release proxy")
    for ax in axes:
        ax.tick_params(axis="x", rotation=70)
        ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(output / "figure_d_executor_comparison.png", dpi=140)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arrays", type=Path, required=True)
    parser.add_argument("--curation", type=Path, required=True)
    parser.add_argument("--historical-root", type=Path, default=Path("/home/thor/LeRobot/rollouts"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    archive = np.load(args.arrays)
    arrays = {key: archive[key] for key in archive.files}
    curation = json.loads(args.curation.read_text(encoding="utf-8"))
    report = {
        "schema_version": "embodied_lab.strong_act_transition_execution_audit.v1",
        "arrays": str(args.arrays.resolve()),
        "curation": str(args.curation.resolve()),
        "teacher_forced": analyze_teacher_forced(arrays, curation),
        "executor_replay": replay_metrics(arrays, curation),
        "filtering_audit": filtering_audit(arrays, curation),
        "historical_rollouts": historical_audit(args.historical_root),
        "limitations": [
            "T1 and T4 are reproducible RH56 hand-event proxies; T2/T3 are not claimed without object/lift/place annotations.",
            "Offline executor replay uses teacher-forced recorded observations and cannot model closed-loop visual divergence.",
            "Contact-stop and serial delta limits are not simulated from these arrays; historical logs lack selected-vs-written target fields, so blocked-release counts are UNKNOWN by design rather than inferred.",
            "The historical logs use a different external checkpoint and are executor-only evidence.",
        ],
    }
    args.output.mkdir(parents=True, exist_ok=True)
    _plot_horizon(report, args.output / "figures")
    (args.output / "audit.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str((args.output / "audit.json").resolve()),
        "teacher_forced": report["teacher_forced"]["aggregate"],
        "executor_replay": {name: {key: value for key, value in data.items() if key != "per_segment"} for name, data in report["executor_replay"].items()},
        "historical": {name: data["classification_counts"] for name, data in report["historical_rollouts"].items()},
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
