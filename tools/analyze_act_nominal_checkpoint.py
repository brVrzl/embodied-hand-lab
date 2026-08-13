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
    if prediction.shape != truth.shape or prediction.ndim != 3 or prediction.shape[2] != 12:
        raise ValueError("predictions and ground truth must be [N,H,12]")
    horizon = prediction.shape[1]
    if state.shape != (len(prediction), 12) or valid.shape != prediction.shape[:2]:
        raise ValueError("state/valid shape mismatch")
    if not np.isfinite(prediction).all() or not np.isfinite(truth).all() or not np.isfinite(state).all():
        raise ValueError("teacher-forced arrays must be finite")
    audit = (
        curation["episodes"]
        if "episodes" in curation
        else curation["views"]["nominal16_task_trimmed"]["per_episode"]
    )
    phase_error: dict[str, list[np.ndarray]] = {
        key: []
        for key in ("approach", "grasp_transition", "post_grasp", "release_transition", "post_release")
    }
    transition_total = 0
    transition_predicted = 0
    timing_error: list[int] = []
    copy_transition_total = 0
    per_session: dict[str, dict[str, int]] = {}
    session_timing: dict[str, list[int]] = {}
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
        predicted = np.any(pred_closure >= threshold, axis=1)
        copy_hand = state[selected, 6:11]
        copy_closure = np.linalg.norm(np.maximum(copy_hand - baseline, 0.0), axis=1)
        copy_predicted = copy_closure >= threshold
        transition_total += int(np.sum(transitions))
        transition_predicted += int(np.sum(transitions & predicted))
        copy_transition_total += int(np.sum(transitions & copy_predicted))
        segment_timing: list[int] = []
        for row in np.flatnonzero(transitions & predicted):
            gt_horizon = int(np.flatnonzero((gt_closure[row] >= threshold) & segment_valid[row])[0])
            pred_horizon = int(np.flatnonzero(pred_closure[row] >= threshold)[0])
            delta = pred_horizon - gt_horizon
            timing_error.append(delta)
            segment_timing.append(delta)
        segment_frames = frames[selected]
        errors = np.abs(segment_prediction - segment_truth)
        release = info.get("release_onset_source_frame")
        phase_masks = [
            ("approach", segment_frames < onset - (horizon - 1)),
            ("grasp_transition", (segment_frames >= onset - (horizon - 1)) & (segment_frames < onset)),
        ]
        if release is None:
            phase_masks.append(("post_grasp", segment_frames >= onset))
        else:
            release = int(release)
            phase_masks.extend([
                ("post_grasp", (segment_frames >= onset) & (segment_frames < release - (horizon - 1))),
                ("release_transition", (segment_frames >= release - (horizon - 1)) & (segment_frames < release)),
                ("post_release", segment_frames >= release),
            ])
        for phase, mask in phase_masks:
            if np.any(mask):
                phase_error[phase].append(errors[mask, 0])
        session = str(info.get("session_group", f"source_{info.get('source_episode', segment)}"))
        session_result = per_session.setdefault(session, {"queries": 0, "recalled": 0})
        session_result["queries"] += int(np.sum(transitions))
        session_result["recalled"] += int(np.sum(transitions & predicted))
        session_timing.setdefault(session, []).extend(segment_timing)
        per_segment[segment] = {
            "rows": int(np.sum(selected)),
            "transition_queries": int(np.sum(transitions)),
            "transition_recalled": int(np.sum(transitions & predicted)),
            "transition_timing_error_mean_abs": (
                None if not segment_timing else float(np.mean(np.abs(segment_timing)))
            ),
            "rh56_predicted_chunk_peak_to_peak_mean": float(np.mean(np.ptp(segment_prediction[:, :, 6:], axis=1))),
            "rh56_predicted_chunk_peak_to_peak_by_channel": np.mean(
                np.ptp(segment_prediction[:, :, 6:], axis=1), axis=0
            ).tolist(),
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
    validation_loss = None
    if "validation_loss" in arrays:
        validation_loss = {
            "aggregate": float(np.mean(arrays["validation_loss"])),
            "l1": float(np.mean(arrays["validation_l1_loss"])),
            "kld": (
                None if not arrays.get("validation_kld_loss", np.empty(0)).size
                else float(np.mean(arrays["validation_kld_loss"]))
            ),
        }
    return {
        "schema_version": "embodied_lab.act_nominal_transition_replay.v2",
        "rows": len(prediction),
        "chunk_size": horizon,
        "logical_segments": sorted(set(segments.tolist())),
        "validation_loss": validation_loss,
        "transition": {
            "queries": transition_total,
            "recalled": transition_predicted,
            "recall": None if not transition_total else transition_predicted / transition_total,
            "predicted_minus_recorded_horizon": (
                None if not timing_error else {
                    "mean": float(np.mean(timing_error)),
                    "mean_abs": float(np.mean(np.abs(timing_error))),
                    "p50": float(np.quantile(timing_error, 0.5)),
                    "p95_abs": float(np.quantile(np.abs(timing_error), 0.95)),
                    "min": int(np.min(timing_error)),
                    "max": int(np.max(timing_error)),
                }
            ),
        },
        "transition_by_session": {
            session: {
                **value,
                "recall": None if not value["queries"] else value["recalled"] / value["queries"],
                "timing_error_mean_abs": (
                    None if not session_timing[session]
                    else float(np.mean(np.abs(session_timing[session])))
                ),
            }
            for session, value in sorted(per_session.items())
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
            "predicted_rh56_chunk_peak_to_peak_by_channel": np.mean(
                np.ptp(prediction[:, :, 6:], axis=1), axis=0
            ).tolist(),
            "copy_state_transition_recalled": copy_transition_total,
            "copy_state_transition_recall": (
                None if not transition_total else copy_transition_total / transition_total
            ),
        },
        "prediction_domain": {
            "per_channel_min": np.min(prediction, axis=(0, 1)).tolist(),
            "per_channel_max": np.max(prediction, axis=(0, 1)).tolist(),
            "rh56_below_legal_zero_fraction": np.mean(prediction[:, :, 6:] < 0.0, axis=(0, 1)).tolist(),
            "rh56_above_legal_one_fraction": np.mean(prediction[:, :, 6:] > 1.0, axis=(0, 1)).tolist(),
            "rh56_max_below_zero_magnitude": np.max(np.maximum(-prediction[:, :, 6:], 0.0), axis=(0, 1)).tolist(),
            "rh56_max_above_one_magnitude": np.max(np.maximum(prediction[:, :, 6:] - 1.0, 0.0), axis=(0, 1)).tolist(),
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
