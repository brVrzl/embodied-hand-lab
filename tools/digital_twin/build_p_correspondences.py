#!/usr/bin/env python3
"""Build auditable P-frame correspondences from the fitted fixed-base primitives."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_yaml
from digital_twin.registration.transforms import apply_similarity


def main() -> None:
    parser = argparse.ArgumentParser(description="Create provisional P correspondences in R02 from reconstruction-01 base fits and T_R02_R01.")
    parser.add_argument("--base-fit", type=Path, required=True)
    parser.add_argument("--cross-registration", type=Path, required=True, help="Accepted provisional T_R02_R01 JSON.")
    parser.add_argument("--manufacturer", type=Path, default=Path("digital_twin/configs/jaka_mini_base_geometry.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        fit, cross, manufacturer = load_structured(args.base_fit), load_structured(args.cross_registration), load_structured(args.manufacturer)
        if cross.get("transform") != "T_R02_R01" or not cross.get("charuco_texture_masked"):
            raise ValueError("Cross-registration must be the ChArUco-masked T_R02_R01 result.")
        p = fit["P_R01"]
        origin = np.asarray(p["origin_R01"], float)
        axes = {
            "x": np.asarray(p["x_axis_R01"], float),
            "y": np.asarray(p["y_axis_R01"], float),
            "z": np.asarray(p["z_axis_R01"], float),
        }
        for name, value in axes.items():
            value /= np.linalg.norm(value)
            axes[name] = value
        fitted_radius = float(fit["base_outer_circle"]["radius_R01_units"])
        nominal_radius = float(manufacturer["fixed_base"]["outer_diameter_m"]["value"]) / 2
        scale, rotation, translation = (
            float(cross["scale_Rb_per_Ra"]), np.asarray(cross["rotation_matrix"], float), np.asarray(cross["translation_Rb_units"], float)
        )

        definitions = [
            ("P_origin_from_outer_circle_center", origin, [0.0, 0.0, 0.0], 0.005,
             "center of fitted 124 mm fixed-base outer circle, lifted to the approximate mounting plane"),
            ("base_outer_plus_x", origin + fitted_radius * axes["x"], [nominal_radius, 0.0, 0.0], 0.004,
             "+x cardinal point of fitted fixed-base outer circle"),
            ("base_outer_minus_x", origin - fitted_radius * axes["x"], [-nominal_radius, 0.0, 0.0], 0.004,
             "-x cardinal point of fitted fixed-base outer circle"),
            ("base_outer_plus_y", origin + fitted_radius * axes["y"], [0.0, nominal_radius, 0.0], 0.004,
             "+y cardinal point of fitted fixed-base outer circle"),
            ("base_outer_minus_y", origin - fitted_radius * axes["y"], [0.0, -nominal_radius, 0.0], 0.004,
             "-y cardinal point of fitted fixed-base outer circle"),
            ("tabletop_below_P", origin - (0.050 / float(fit["scale_used_m_per_R01_unit"])) * axes["z"], [0.0, 0.0, -0.050], 0.004,
             "tabletop plane directly below fitted base center; 50 mm height is user-approximate"),
        ]
        rows = []
        for name, point_r01, target, uncertainty, description in definitions:
            point_r02 = apply_similarity(np.asarray([point_r01]), scale, rotation, translation)[0]
            rows.append({
                "name": name,
                "reconstruction_xyz": point_r02.tolist(),
                "target_xyz_m": target,
                "source_reconstruction": "02 via provisional masked T_R02_R01",
                "physical_landmark_description": description,
                "acquisition_method": "derived_constraint_from_ROI_sparse_primitive_fit",
                "selection_method": "automatic_constrained_fit",
                "uncertainty_m": uncertainty,
                "manual_or_automatic": "automatic_with_manual_ROI",
                "screenshot_reference": "artifacts/digital_twin/calibration/base_fit_01_top.png",
                "accepted": True,
                "status": "provisional",
                "reviewer_note": "Not an independently surveyed point; retain common-mode center/orientation uncertainty.",
            })
        result = {
            "schema_version": 1,
            "source_frame": "R02",
            "target_frame": "P",
            "status": "provisional_correspondences_not_independent_survey_control",
            "correspondences": rows,
            "rejected_candidates": [
                {
                    "name": "mounting_PCD_bolt_centers",
                    "status": "rejected_insufficient_visibility",
                    "reason": "Only two bolt heads were safely annotated; a four-point 110 mm PCD fit would be fabricated.",
                },
                {
                    "name": "rail_centerline_points",
                    "status": "withheld_from_registration",
                    "reason": "75.22 mm sparse fit supports the 77.782 mm hypothesis but is not an independent surveyed P coordinate.",
                },
            ],
            "limitations": [
                "The five circle correspondences share the same fitted center and are statistically correlated.",
                "The tabletop point uses the approximate 50 mm mounting-plane height.",
                "This file supports a provisional engineering registration only; it does not satisfy final surveyed T_P_R calibration.",
            ],
        }
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        write_yaml(args.output, result)
        print(f"P correspondences written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
