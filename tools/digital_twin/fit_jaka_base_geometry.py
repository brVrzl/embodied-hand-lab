#!/usr/bin/env python3
"""Fit table, rails, transverse member, and JAKA fixed-base circle from a local COLMAP model."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.calibration.base_geometry import fit_circle_ransac, fit_line_pca, fit_plane_ransac, plane_basis, project_to_plane_coordinates
from digital_twin.io import load_structured, write_json


def data_lines(path: Path, preserve_empty: bool = False) -> list[str]:
    result = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("#"):
            continue
        if line or preserve_empty:
            result.append(line)
    return result


def load_model(model: Path) -> tuple[dict[int, dict], dict[str, tuple[np.ndarray, np.ndarray]]]:
    points = {}
    for line in data_lines(model / "points3D.txt"):
        fields = line.split()
        points[int(fields[0])] = {"xyz": np.asarray(fields[1:4], float), "rgb": np.asarray(fields[4:7], np.uint8)}
    lines = data_lines(model / "images.txt", preserve_empty=True)
    observations = {}
    for offset in range(0, len(lines), 2):
        pose, row = lines[offset].split(), lines[offset + 1].split()
        xy = np.asarray([[float(row[i]), float(row[i + 1])] for i in range(0, len(row), 3)], float)
        ids = np.asarray([int(row[i + 2]) for i in range(0, len(row), 3)], np.int64)
        observations[pose[9]] = (xy, ids)
    return points, observations


def ids_in_annotations(entries: list[dict], observations: dict[str, tuple[np.ndarray, np.ndarray]]) -> set[int]:
    result: set[int] = set()
    for entry in entries:
        xy, point_ids = observations[entry["image"]]
        polygon = np.asarray(entry["polygon_px"], np.float32)
        inside = np.asarray([cv2.pointPolygonTest(polygon, tuple(map(float, point)), False) >= 0 for point in xy])
        result.update(map(int, point_ids[inside & (point_ids >= 0)]))
    return result


def robust_xyz(points: dict[int, dict], ids: set[int], *, low_saturation: bool = False, bright: bool = False) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    retained = [value for point_id, value in points.items() if point_id in ids]
    xyz = np.asarray([item["xyz"] for item in retained], float)
    rgb = np.asarray([item["rgb"] for item in retained], np.uint8)
    if not len(xyz):
        return xyz, rgb, np.empty(0, bool)
    keep = np.ones(len(xyz), bool)
    if low_saturation:
        keep &= np.ptp(rgb.astype(float), axis=1) < 55
    if bright:
        keep &= rgb.mean(axis=1) > 105
    return xyz, rgb, keep


def axis_point(origin: np.ndarray, ex: np.ndarray, ey: np.ndarray, coordinates: np.ndarray) -> np.ndarray:
    return origin + coordinates[0] * ex + coordinates[1] * ey


def write_ply(path: Path, groups: list[tuple[np.ndarray, tuple[int, int, int]]]) -> None:
    rows = [(*point, *color) for values, color in groups for point in values]
    header = ["ply", "format ascii 1.0", f"element vertex {len(rows)}", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "end_header"]
    path.write_text("\n".join(header + [" ".join(map(str, row)) for row in rows]) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Fit fixed JAKA base and mounting primitives in local reconstruction 01.")
    parser.add_argument("--model", type=Path, required=True, help="COLMAP text model directory.")
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--manufacturer", type=Path, default=Path("digital_twin/configs/jaka_mini_base_geometry.yaml"))
    parser.add_argument("--scale-m-per-unit", type=float, required=True, help="Provisional scale used only for thresholds and metric residual reporting.")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if args.scale_m_per_unit <= 0:
            raise ValueError("--scale-m-per-unit must be positive.")
        points, observations = load_model(args.model)
        annotations, manufacturer = load_structured(args.annotations), load_structured(args.manufacturer)
        all_xyz = np.asarray([item["xyz"] for item in points.values()], float)
        threshold_units = 0.0025 / args.scale_m_per_unit
        table = fit_plane_ransac(all_xyz, threshold_units, iterations=5000, seed=args.seed)
        normal = table.normal.copy()
        table_origin = all_xyz[table.inliers].mean(axis=0)

        base_ids = ids_in_annotations(annotations["base_outer_surface"], observations)
        base_xyz, base_rgb, base_color_keep = robust_xyz(points, base_ids, low_saturation=True, bright=True)
        signed = (base_xyz - table_origin) @ normal
        if np.median(signed[base_color_keep]) < 0:
            normal *= -1
            signed *= -1
        height_keep = (signed >= 0.025 / args.scale_m_per_unit) & (signed <= 0.16 / args.scale_m_per_unit)
        base_keep = base_color_keep & height_keep
        ex, ey = plane_basis(normal)
        base_xy = project_to_plane_coordinates(base_xyz[base_keep], table_origin, ex, ey)
        radius_nominal_units = 0.062 / args.scale_m_per_unit
        circle = fit_circle_ransac(
            base_xy, 0.0035 / args.scale_m_per_unit,
            (0.95 * radius_nominal_units, 1.08 * radius_nominal_units), iterations=30000, seed=args.seed,
        )
        base_center_table = axis_point(table_origin, ex, ey, circle.center)

        rail_results = {}
        rail_groups = []
        for rail_name, entries in annotations["parallel_rails"].items():
            ids = ids_in_annotations(entries, observations)
            xyz, rgb, color_keep = robust_xyz(points, ids, low_saturation=True, bright=True)
            heights = (xyz - table_origin) @ normal
            keep = color_keep & (heights >= 0.025 / args.scale_m_per_unit) & (heights <= 0.085 / args.scale_m_per_unit)
            coordinates = project_to_plane_coordinates(xyz[keep], table_origin, ex, ey)
            center, direction, lateral = fit_line_pca(coordinates)
            rail_results[rail_name] = {"center": center, "direction": direction, "lateral": lateral, "xyz": xyz[keep]}
            rail_groups.append(xyz[keep])
        first, second = rail_results["rail_1"], rail_results["rail_2"]
        if first["direction"] @ second["direction"] < 0:
            second["direction"] *= -1
        rail_direction = first["direction"] + second["direction"]
        rail_direction /= np.linalg.norm(rail_direction)
        rail_normal = np.asarray([-rail_direction[1], rail_direction[0]])
        rail_spacing_units = abs((second["center"] - first["center"]) @ rail_normal)

        transverse_ids = ids_in_annotations(annotations["front_transverse_member"], observations)
        transverse_xyz, _, transverse_keep = robust_xyz(points, transverse_ids, low_saturation=True, bright=True)
        transverse_heights = (transverse_xyz - table_origin) @ normal
        transverse_keep &= (transverse_heights >= 0.025 / args.scale_m_per_unit) & (transverse_heights <= 0.085 / args.scale_m_per_unit)
        transverse_xy = project_to_plane_coordinates(transverse_xyz[transverse_keep], table_origin, ex, ey)
        transverse_center = np.median(transverse_xy, axis=0)
        if rail_direction @ (transverse_center - circle.center) < 0:
            rail_direction *= -1
            rail_normal *= -1

        p_x = rail_direction[0] * ex + rail_direction[1] * ey
        p_y = np.cross(normal, p_x)
        p_y /= np.linalg.norm(p_y)
        p_origin = base_center_table + normal * (0.050 / args.scale_m_per_unit)
        fitted_diameter_m = 2 * circle.radius * args.scale_m_per_unit
        nominal_diameter = float(manufacturer["fixed_base"]["outer_diameter_m"]["value"])
        spacing_m = rail_spacing_units * args.scale_m_per_unit
        hypotheses = manufacturer["installed_orientation"]["hypotheses"]
        hypothesis_residuals = {
            item["name"]: abs(spacing_m - float(item["candidate_rail_centerline_spacing_m"])) for item in hypotheses
        }
        selected_hypothesis = min(hypothesis_residuals, key=hypothesis_residuals.get)
        result = {
            "schema_version": 1,
            "source_reconstruction": "01",
            "scale_used_m_per_R01_unit": args.scale_m_per_unit,
            "scale_status": "provisional_from_masked_T_R02_R01_and_02_charuco_scale",
            "table_plane_R01": {
                "normal": normal.tolist(), "offset": float(-normal @ table_origin),
                "inlier_count": int(table.inliers.sum()),
                "rms_m": float(np.sqrt(np.mean(table.residuals[table.inliers] ** 2)) * args.scale_m_per_unit),
            },
            "base_outer_circle": {
                "center_on_table_plane_R01": base_center_table.tolist(),
                "radius_R01_units": circle.radius,
                "fitted_diameter_m": fitted_diameter_m,
                "manufacturer_diameter_m": nominal_diameter,
                "diameter_residual_m": fitted_diameter_m - nominal_diameter,
                "inlier_count": int(circle.inliers.sum()),
                "candidate_point_count": len(base_xy),
                "rms_radial_m": float(np.sqrt(np.mean(circle.residuals[circle.inliers] ** 2)) * args.scale_m_per_unit),
                "status": "provisional_geometric_aid_not_mounting_PCD",
            },
            "mounting_hole_pattern": {
                "visible_manual_bolt_centers": annotations["mounting_bolts"]["manually_visible_centers_px"],
                "fitted_PCD_m": None,
                "status": "MISSING_four_bolt_centers_not_visible",
            },
            "parallel_rails": {
                "direction_R01": p_x.tolist(),
                "centerline_spacing_m": spacing_m,
                "hypothesis_residuals_m": hypothesis_residuals,
                "selected_interpretation": selected_hypothesis,
                "status": "provisional_ROI_sparse_fit_requires_metric_cross_section_check",
            },
            "P_R01": {
                "origin_R01": p_origin.tolist(),
                "origin_definition": "fitted outer-circle center lifted 50 mm from table plane; mounting PCD center preferred but unavailable",
                "x_axis_R01": p_x.tolist(), "y_axis_R01": p_y.tolist(), "z_axis_R01": normal.tolist(),
                "mounting_plane_height_source": "user_approximate_measurement_0.050m",
                "status": "provisional_not_sufficient_for_final_T_P_R",
            },
            "limitations": [
                "The sparse outer-circle fit is seeded by manual image ROIs and a provisional metric threshold.",
                "Only two bolt heads were safely annotated; mounting PCD and hole-pattern rotation are not fitted.",
                "Rail width/profile residual is not accepted from groove-dominated sparse points.",
                "P origin uses the circle center and approximate 50 mm mounting-plane height, not a surveyed mounting datum.",
            ],
        }
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        args.output_dir.mkdir(parents=True, exist_ok=True)
        write_json(args.output_dir / "base_fit_01.json", result)
        write_ply(args.output_dir / "base_fit_01.ply", [
            (all_xyz[table.inliers], (180, 140, 80)),
            (base_xyz[base_keep][circle.inliers], (255, 255, 255)),
            (rail_groups[0], (50, 180, 255)), (rail_groups[1], (0, 100, 255)),
            (transverse_xyz[transverse_keep], (255, 180, 0)),
        ])
        top_groups = [
            (project_to_plane_coordinates(base_xyz[base_keep], table_origin, ex, ey), "base candidates", "0.7"),
            (project_to_plane_coordinates(rail_groups[0], table_origin, ex, ey), "rail 1", "tab:blue"),
            (project_to_plane_coordinates(rail_groups[1], table_origin, ex, ey), "rail 2", "tab:cyan"),
            (transverse_xy, "transverse", "tab:orange"),
        ]
        fig, ax = plt.subplots(figsize=(9, 7))
        for values, label, color in top_groups:
            ax.scatter(values[:, 0] * args.scale_m_per_unit, values[:, 1] * args.scale_m_per_unit, s=2, label=label, c=color)
        angle = np.linspace(0, 2 * np.pi, 300)
        circle_metric = (circle.center + circle.radius * np.column_stack((np.cos(angle), np.sin(angle)))) * args.scale_m_per_unit
        ax.plot(circle_metric[:, 0], circle_metric[:, 1], "r-", label="fitted base circle")
        center_metric = circle.center * args.scale_m_per_unit
        direction_metric = rail_direction * 0.18
        ax.arrow(*center_metric, *direction_metric, color="red", width=0.002, length_includes_head=True)
        ax.set_aspect("equal"); ax.grid(True); ax.legend(); ax.set_xlabel("plane axis 1 (m)"); ax.set_ylabel("plane axis 2 (m)")
        ax.set_title("Reconstruction 01: provisional top-plane primitive fit")
        fig.tight_layout(); fig.savefig(args.output_dir / "base_fit_01_top.png", dpi=180); plt.close(fig)
        fig = plt.figure(figsize=(9, 7)); ax3 = fig.add_subplot(111, projection="3d")
        for values, color in [(base_xyz[base_keep], "0.7"), (rail_groups[0], "tab:blue"), (rail_groups[1], "tab:cyan"), (transverse_xyz[transverse_keep], "tab:orange")]:
            ax3.scatter(values[:, 0], values[:, 1], values[:, 2], s=2, c=color)
        axes = np.asarray([p_x, p_y, normal]) / args.scale_m_per_unit * 0.15
        for vector, color, label in zip(axes, ["r", "g", "b"], ["P +x", "P +y", "P +z"]):
            ax3.quiver(*p_origin, *vector, color=color, label=label)
        ax3.legend(); ax3.set_title("Reconstruction 01: provisional P-axis fit")
        fig.tight_layout(); fig.savefig(args.output_dir / "base_fit_01_oblique.png", dpi=180); plt.close(fig)
        markdown = f"""# Reconstruction 01 base fit\n\n- Status: **provisional; not accepted as final T_P_R**.\n- Table-plane inliers: {int(table.inliers.sum())}; RMS {result['table_plane_R01']['rms_m']*1000:.2f} mm.\n- Fitted base diameter: {fitted_diameter_m*1000:.2f} mm; manufacturer reference 124.00 mm; residual {(fitted_diameter_m-nominal_diameter)*1000:+.2f} mm.\n- Circle inliers: {int(circle.inliers.sum())}/{len(base_xy)}; radial RMS {result['base_outer_circle']['rms_radial_m']*1000:.2f} mm.\n- Fitted rail centerline spacing: {spacing_m*1000:.2f} mm; selected candidate `{selected_hypothesis}` by residual only.\n- Mounting PCD: **not fitted**; only two bolt heads were safely visible.\n- P origin uses the fitted circle center plus the user-approximate 50 mm mounting-plane height. It is not a surveyed mounting-pattern center.\n"""
        (args.output_dir / "base_fit_01.md").write_text(markdown, encoding="utf-8")
        print(f"Base fit written to: {args.output_dir}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
