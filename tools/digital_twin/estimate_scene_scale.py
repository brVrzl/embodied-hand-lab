#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json, write_yaml


def _fit_reference_scales(references: list[dict], outlier_sigma: float) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, float]:
    values, weights = [], []
    for index, reference in enumerate(references):
        reconstruction = float(reference["reconstruction_distance"])
        known = float(reference["known_distance_m"])
        uncertainty = float(reference.get("uncertainty_m", 0.0) or 0.0)
        if reconstruction <= 0 or known <= 0 or uncertainty < 0:
            raise ValueError(f"Reference {index} distances must be positive and uncertainty non-negative.")
        values.append(known / reconstruction)
        weights.append(1.0 / max(uncertainty / reconstruction, known * 1e-6 / reconstruction) ** 2)
    values_array = np.asarray(values)
    weights_array = np.asarray(weights)
    center = float(np.median(values_array))
    mad = float(np.median(np.abs(values_array - center)))
    robust_sigma = max(1.4826 * mad, abs(center) * 1e-9)
    inliers = np.abs(values_array - center) <= outlier_sigma * robust_sigma if len(values_array) >= 3 else np.ones(len(values_array), bool)
    if not np.any(inliers):
        raise ValueError("All scale references were rejected.")
    scale = float(np.average(values_array[inliers], weights=weights_array[inliers]))
    return scale, inliers, values_array, weights_array, robust_sigma


def estimate_scale(references: list[dict], *, outlier_sigma: float = 3.5, board_agreement_threshold: float = 0.02) -> dict:
    if not references:
        raise ValueError("At least one distance reference is required.")
    board_references = [reference for reference in references if str(reference.get("group", "")).upper() in {"A3", "A4"}]
    fit_references = board_references or references
    scale, fit_inliers, fit_values, fit_weights, robust_sigma = _fit_reference_scales(fit_references, outlier_sigma)
    fit_identity = {id(reference): bool(fit_inliers[index]) for index, reference in enumerate(fit_references)}
    output_references = []
    for index, reference in enumerate(references):
        individual_scale = float(reference["known_distance_m"]) / float(reference["reconstruction_distance"])
        predicted = scale * float(reference["reconstruction_distance"])
        residual = predicted - float(reference["known_distance_m"])
        accepted = fit_identity.get(id(reference), False)
        output_references.append({
            "name": reference.get("name", f"reference_{index}"),
            "source": reference.get("source"),
            "known_distance_m": float(reference["known_distance_m"]),
            "reconstruction_distance": float(reference["reconstruction_distance"]),
            "uncertainty_m": float(reference.get("uncertainty_m", 0.0) or 0.0),
            "group": reference.get("group", "ungrouped"),
            "physical_instance": reference.get("physical_instance"),
            "observation_id": reference.get("observation_id"),
            "individual_scale": individual_scale,
            "residual_m": residual,
            "accepted": accepted,
            "accepted_for_final_scale": accepted,
            "used_as_primary_source": id(reference) in {id(item) for item in fit_references},
        })
    accepted_residuals = [item["residual_m"] for item in output_references if item["accepted_for_final_scale"]]
    grouped = {}
    for group in sorted({str(reference.get("group", "ungrouped")) for reference in references}):
        subset = [reference for reference in references if str(reference.get("group", "ungrouped")) == group]
        group_scale, group_inliers, group_values, group_weights, group_sigma = _fit_reference_scales(subset, outlier_sigma)
        grouped[group] = {
            "estimated_scale_m_per_reconstruction_unit": group_scale,
            "observation_count": len(subset),
            "accepted_count": int(group_inliers.sum()),
            "individual_scales": group_values.tolist(),
            "robust_scale_sigma": group_sigma,
        }
    a3 = grouped.get("A3")
    a4 = grouped.get("A4")
    agreement = None
    warnings = []
    if a3 and a4:
        first, second = a3["estimated_scale_m_per_reconstruction_unit"], a4["estimated_scale_m_per_reconstruction_unit"]
        relative = abs(first - second) / max((abs(first) + abs(second)) / 2, 1e-12)
        agreement = {"relative_difference": relative, "threshold": board_agreement_threshold, "status": "agree" if relative <= board_agreement_threshold else "material_disagreement"}
        if relative > board_agreement_threshold:
            warnings.append("A3 and A4 scale estimates materially disagree; do not average until board identity/configuration, print scale, distortion, rolling shutter, bending, and reconstruction stability are diagnosed.")
    instance_estimates = {}
    for instance in sorted({str(reference.get("physical_instance")) for reference in references if reference.get("physical_instance")}):
        subset = [reference for reference in references if str(reference.get("physical_instance")) == instance]
        instance_scale, instance_inliers, instance_values, _instance_weights, instance_sigma = _fit_reference_scales(subset, outlier_sigma)
        instance_estimates[instance] = {
            "estimated_scale_m_per_reconstruction_unit": instance_scale,
            "observation_count": len(subset),
            "accepted_count": int(instance_inliers.sum()),
            "individual_scales": instance_values.tolist(),
            "robust_scale_sigma": instance_sigma,
        }
    leave_one_out = []
    if len(fit_references) >= 3:
        for omitted in range(len(fit_references)):
            subset = [reference for index, reference in enumerate(fit_references) if index != omitted]
            leave_one_out.append(_fit_reference_scales(subset, outlier_sigma)[0])
    uncertainty = float(np.sqrt(np.average((fit_values[fit_inliers] - scale) ** 2, weights=fit_weights[fit_inliers]))) if fit_inliers.sum() > 1 else None
    weak = len(references) < 2 or max(item["known_distance_m"] for item in output_references) < 0.1
    if weak:
        warnings.append("Scale is based on fewer than two independent references or only short baselines.")
    acceptance = (
        "rejected_material_A3_A4_disagreement" if agreement and agreement["status"] != "agree"
        else "provisional_weak_requires_independent_baseline" if weak
        else "candidate_accepted_pending_registration_validation"
    )
    return {
        "estimated_scale_m_per_reconstruction_unit": scale,
        "final_selected_scale": scale,
        "primary_source_policy": "A3_and_A4_only" if board_references else "all_available_references_no_board_group_present",
        "reference_count": len(references),
        "accepted_count": int(fit_inliers.sum()),
        "rms_residual_m": float(np.sqrt(np.mean(np.square(accepted_residuals)))),
        "max_abs_residual_m": float(np.max(np.abs(accepted_residuals))),
        "robust_scale_sigma": robust_sigma,
        "scale_uncertainty_indicator": uncertainty,
        "group_estimates": grouped,
        "physical_instance_estimates": instance_estimates,
        "A3_A4_agreement": agreement,
        "frame_subset_robustness": {
            "method": "leave_one_reference_out",
            "estimates": leave_one_out,
            "range": [min(leave_one_out), max(leave_one_out)] if leave_one_out else None,
        },
        "confidence_indicator": "weak" if weak else "multiple_reference_fit",
        "metric_acceptance_status": acceptance,
        "warnings": warnings,
        "references": output_references,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Robustly estimate metric reconstruction scale from known distances.")
    parser.add_argument("--input", type=Path, required=True, help="JSON/YAML with a `references` list.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--outlier-sigma", type=float, default=3.5)
    parser.add_argument("--board-agreement-threshold", type=float, default=0.02)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        data = load_structured(args.input)
        result = estimate_scale(data.get("references", []), outlier_sigma=args.outlier_sigma, board_agreement_threshold=args.board_agreement_threshold)
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        (write_yaml if args.output.suffix.lower() in {".yaml", ".yml"} else write_json)(args.output, result)
        print(f"Scale estimate written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
