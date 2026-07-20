#!/usr/bin/env python3
"""Compare per-board, base-CAD, and rail-hypothesis metric scale sources."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json


def main() -> None:
    parser = argparse.ArgumentParser(description="Assess independent metric-scale sources without treating approximate table dimensions as primary.")
    parser.add_argument("--charuco-scale", type=Path, required=True)
    parser.add_argument("--base-fit", type=Path, required=True)
    parser.add_argument("--cross-registration", type=Path, required=True)
    parser.add_argument("--manufacturer", type=Path, default=Path("digital_twin/configs/jaka_mini_base_geometry.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        charuco, base, cross, manufacturer = (
            load_structured(args.charuco_scale), load_structured(args.base_fit),
            load_structured(args.cross_registration), load_structured(args.manufacturer),
        )
        instances = charuco["physical_instance_estimates"]
        r02_per_r01 = float(cross["scale_Rb_per_Ra"])
        fitted_diameter_r01 = 2 * float(base["base_outer_circle"]["radius_R01_units"])
        base_scale_r02 = float(manufacturer["fixed_base"]["outer_diameter_m"]["value"]) / fitted_diameter_r01 / r02_per_r01
        fitted_spacing_r01 = float(base["parallel_rails"]["centerline_spacing_m"]) / float(base["scale_used_m_per_R01_unit"])
        diagonal_candidate = float(manufacturer["installed_orientation"]["hypotheses"][0]["candidate_rail_centerline_spacing_m"])
        rail_scale_r02 = diagonal_candidate / fitted_spacing_r01 / r02_per_r01
        source_scales = {
            name: float(item["estimated_scale_m_per_reconstruction_unit"]) for name, item in instances.items()
        }
        source_scales["base_outer_diameter_124mm"] = base_scale_r02
        robust_primary_names = ["Board_A3_1", "Board_A4_1", "Board_A4_2", "base_outer_diameter_124mm"]
        primary = np.asarray([source_scales[name] for name in robust_primary_names], float)
        selected = float(np.median(primary))
        sigma = float(1.4826 * np.median(np.abs(primary - selected)))
        max_pairwise = float((primary.max() - primary.min()) / np.mean(primary))
        leave_one_out = [float(np.median(np.delete(primary, index))) for index in range(len(primary))]
        result = {
            "schema_version": 1,
            "units": "meter_per_R02_unit",
            "sources": {
                "Board_A3_1": {"scale": source_scales["Board_A3_1"], "status": "primary_confirmed_board_geometry_indirect_sparse_association"},
                "Board_A4_1": {"scale": source_scales["Board_A4_1"], "status": "primary_confirmed_board_geometry_spatial_instance_cluster"},
                "Board_A4_2": {"scale": source_scales["Board_A4_2"], "status": "primary_confirmed_board_geometry_spatial_instance_cluster"},
                "base_outer_diameter_124mm": {"scale": base_scale_r02, "status": "primary_CAD_dimension_but_ROI_circle_fit_is_radius_gated"},
                "mounting_PCD_110mm": {"scale": None, "status": "MISSING_only_two_bolt_centers_visible"},
                "aluminium_profile_width_50mm": {"scale": None, "status": "MISSING_sparse_groove_points_do_not_define_both_profile_edges"},
                "rail_spacing_110_over_sqrt2": {"scale": rail_scale_r02, "status": "diagnostic_only_orientation_candidate_not_independent_manufacturer_spacing"},
            },
            "charuco_aggregate_candidate": float(charuco["final_selected_scale"]),
            "selected_final_provisional_scale": selected,
            "source_level_robust_sigma": sigma,
            "relative_uncertainty_indicator": sigma / selected,
            "max_primary_source_span_fraction": max_pairwise,
            "leave_one_source_out_medians": leave_one_out,
            "agreement_target_fraction": 0.02,
            "acceptance_status": "PROVISIONAL_primary_sources_exceed_2_percent_span" if max_pairwise > 0.02 else "PASS_candidate_pending_registration",
            "warnings": [
                "The two identical-pattern A4 physical instances disagree materially; do not treat repeated frame observations as independent baselines.",
                "The base-circle fit was restricted to a manufacturer-informed radius interval and is not a blind scale recovery.",
                "The rail-derived scale is excluded from the robust primary median until the 45-degree bolt-pattern orientation is visually verified.",
                "Table dimensions remain secondary validation checks and are not used in this scale estimate.",
            ],
        }
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        write_json(args.output, result)
        if args.report:
            lines = ["# Metric scale assessment", "", f"Selected provisional scale: **{selected:.9f} m/R02-unit**.", f"Source-level robust sigma: **{sigma:.6f} m/R02-unit ({100*sigma/selected:.2f}%)**.", f"Status: **{result['acceptance_status']}**.", "", "| Source | scale (m/R02-unit) | status |", "|---|---:|---|"]
            for name, item in result["sources"].items():
                value = "MISSING" if item["scale"] is None else f"{item['scale']:.9f}"
                lines.append(f"| {name} | {value} | {item['status']} |")
            lines += ["", *[f"- WARN: {warning}" for warning in result["warnings"]]]
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"Metric scale assessment written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
