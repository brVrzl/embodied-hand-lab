#!/usr/bin/env python3
"""Evaluate an ACT teacher-forced replay against curated grasp transitions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


def checkpoint_summary(arrays: dict[str, np.ndarray], curation: dict[str, Any]) -> dict[str, Any]:
    required = {"predictions", "ground_truth", "state", "valid", "source_frame", "logical_segment"}
    missing = sorted(required - set(arrays))
    if missing:
        raise ValueError(f"teacher-forced arrays missing {missing}")
    prediction = np.asarray(arrays["predictions"], dtype=np.float64)
    truth = np.asarray(arrays["ground_truth"], dtype=np.float64)
    state = np.asarray(arrays["state"], dtype=np.float64)
    valid = np.asarray(arrays["valid"], dtype=bool)
    frames = np.asarray(arrays["source_frame"], dtype=np.int64).reshape(-1)
    segments = np.asarray(arrays["logical_segment"]).astype(str).reshape(-1)
    if prediction.shape != truth.shape or prediction.shape[1:] != (16, 12):
        raise ValueError("predictions and ground truth must be [N,16,12]")
    if state.shape != (len(prediction), 12) or valid.shape != prediction.shape[:2]:
        raise ValueError("state/valid shape mismatch")
    if not np.isfinite(prediction).all() or not np.isfinite(truth).all() or not np.isfinite(state).all():
        raise ValueError("teacher-forced arrays must be finite")
    audit = curation["views"]["nominal16_task_trimmed"]["per_episode"]
    phase_error: dict[str, list[np.ndarray]] = {key: [] for key in ("approach", "transition", "post_grasp")}
    transition_total = 0
    transition_predicted = 0
    timing_error: list[int] = []
    per_segment: dict[str, Any] = {}
    for segment in sorted(set(segments.tolist())):
        if segment not in audit:
            raise ValueError(f"curation report has no segment {segment}")
        selected = segments == segment
        info = audit[segment]
        onset = int(info["grasp_onset_source_frame"])
        baseline = np.asarray(info["open_baseline_rh56"], dtype=np.float64)
        threshold = float(info["closure_threshold"])
        segment_truth = truth[selected]
        segment_prediction = prediction[selected]
        segment_valid = valid[selected]
        gt_closure = np.linalg.norm(np.maximum(segment_truth[:, :, 6:11] - baseline, 0.0), axis=2)
        pred_closure = np.linalg.norm(np.maximum(segment_prediction[:, :, 6:11] - baseline, 0.0), axis=2)
        transitions = (
            np.all(gt_closure[:, :2] < threshold, axis=1)
            & np.any((gt_closure[:, 2:] >= threshold) & segment_valid[:, 2:], axis=1)
        )
        predicted = (
            np.all(pred_closure[:, :2] < threshold, axis=1)
            & np.any(pred_closure[:, 2:] >= threshold, axis=1)
        )
        transition_total += int(np.sum(transitions))
        transition_predicted += int(np.sum(transitions & predicted))
        for row in np.flatnonzero(transitions & predicted):
            gt_horizon = int(np.flatnonzero((gt_closure[row] >= threshold) & segment_valid[row])[0])
            pred_horizon = int(np.flatnonzero(pred_closure[row] >= threshold)[0])
            timing_error.append(pred_horizon - gt_horizon)
        segment_frames = frames[selected]
        errors = np.abs(segment_prediction - segment_truth)
        for phase, mask in (
            ("approach", segment_frames < onset - 15),
            ("transition", (segment_frames >= onset - 15) & (segment_frames < onset)),
            ("post_grasp", segment_frames >= onset),
        ):
            if np.any(mask):
                phase_error[phase].append(errors[mask, 0])
        per_segment[segment] = {
            "rows": int(np.sum(selected)),
            "transition_queries": int(np.sum(transitions)),
            "transition_recalled": int(np.sum(transitions & predicted)),
            "rh56_predicted_chunk_peak_to_peak_mean": float(np.mean(np.ptp(segment_prediction[:, :, 6:], axis=1))),
        }
    valid_3d = valid[:, :, None]
    copy = np.broadcast_to(state[:, None, :], truth.shape)
    copy_error = np.abs(copy - truth)
    policy_error = np.abs(prediction - truth)
    phase_report: dict[str, Any] = {}
    for phase, values in phase_error.items():
        if values:
            array = np.concatenate(values)
            phase_report[phase] = {
                "rows": len(array),
                "first_action_mae_jaka": float(np.mean(array[:, :6])),
                "first_action_mae_rh56": float(np.mean(array[:, 6:])),
            }
    return {
        "schema_version": "embodied_lab.act_nominal_transition_replay.v1",
        "rows": len(prediction),
        "logical_segments": sorted(set(segments.tolist())),
        "transition": {
            "queries": transition_total,
            "recalled": transition_predicted,
            "recall": None if not transition_total else transition_predicted / transition_total,
            "predicted_minus_recorded_horizon": (
                None if not timing_error else {
                    "mean": float(np.mean(timing_error)),
                    "p50": float(np.quantile(timing_error, 0.5)),
                    "min": int(np.min(timing_error)),
                    "max": int(np.max(timing_error)),
                }
            ),
        },
        "phase_conditioned_first_action_error": phase_report,
        "all_valid_horizon_mae": {
            "policy_jaka": float(np.mean(policy_error[:, :, :6][np.broadcast_to(valid_3d, policy_error.shape)[:, :, :6]])),
            "policy_rh56": float(np.mean(policy_error[:, :, 6:][np.broadcast_to(valid_3d, policy_error.shape)[:, :, 6:]])),
            "copy_state_jaka": float(np.mean(copy_error[:, :, :6][np.broadcast_to(valid_3d, copy_error.shape)[:, :, :6]])),
            "copy_state_rh56": float(np.mean(copy_error[:, :, 6:][np.broadcast_to(valid_3d, copy_error.shape)[:, :, 6:]])),
        },
        "current_state_persistence": {
            "first_action_prediction_to_state_rh56_mae": float(np.mean(np.abs(prediction[:, 0, 6:] - state[:, 6:]))),
            "predicted_rh56_chunk_peak_to_peak_mean": float(np.mean(np.ptp(prediction[:, :, 6:], axis=1))),
            "copy_state_transition_recall": 0.0,
        },
        "per_segment": per_segment,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arrays", type=Path, required=True)
    parser.add_argument("--curation", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", required=True)
    args = parser.parse_args()
    archive = np.load(args.arrays)
    arrays = {key: archive[key] for key in archive.files}
    curation = json.loads(args.curation.read_text(encoding="utf-8"))
    report = checkpoint_summary(arrays, curation)
    report["label"] = args.label
    report["arrays"] = str(args.arrays.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    target = args.output / "transition_analysis.json"
    target.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
