#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json, write_yaml
from digital_twin.registration.transforms import (
    apply_similarity,
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
    ransac_similarity,
    umeyama_similarity,
)


def register(data: dict, *, target_frame: str, use_ransac: bool, threshold_m: float, iterations: int, seed: int) -> dict:
    correspondences = data.get("correspondences", [])
    if len(correspondences) < 3:
        raise ValueError("At least three non-collinear correspondences are required.")
    target_key = "target_xyz_m"
    if not all(target_key in item for item in correspondences):
        legacy_key = "robot_base_xyz_m" if target_frame == "B" else "physical_reference_xyz_m"
        target_key = legacy_key
    for index, item in enumerate(correspondences):
        if item.get("selection_method") == "manual":
            missing_manual = [key for key in ("image_frame_source", "physical_landmark_description", "reviewer_note") if not item.get(key)]
            if missing_manual:
                raise ValueError(f"Manual correspondence {index} is missing: {', '.join(missing_manual)}")
    source = np.asarray([item["reconstruction_xyz"] for item in correspondences], dtype=float)
    target = np.asarray([item[target_key] for item in correspondences], dtype=float)
    uncertainties = np.asarray([float(item.get("uncertainty_m", 0.001) or 0.001) for item in correspondences])
    if np.any(uncertainties <= 0):
        raise ValueError("Correspondence uncertainties must be positive.")
    weights = 1.0 / uncertainties**2
    result = (
        ransac_similarity(source, target, threshold=threshold_m, iterations=iterations, seed=seed, weights=weights)
        if use_ransac
        else umeyama_similarity(source, target, weights=weights)
    )
    transformed = apply_similarity(source, result.scale, result.rotation, result.translation)
    rank = int(np.linalg.matrix_rank(source - source.mean(axis=0)))
    singular = np.linalg.svd(source - source.mean(axis=0), compute_uv=False)
    warnings = []
    input_status = str(data.get("status", "unspecified"))
    if "provisional" in input_status:
        warnings.append("Input correspondences are provisional and may be statistically correlated; a low fit residual is not independent calibration evidence.")
    if rank < 3:
        warnings.append("Correspondences are coplanar; out-of-plane registration uncertainty may be weak.")
    if len(singular) >= 2 and singular[-1] / max(singular[0], 1e-12) < 0.02:
        warnings.append("Correspondence geometry is nearly degenerate in at least one dimension.")
    records = []
    for index, item in enumerate(correspondences):
        records.append({
            "name": item.get("name", f"point_{index}"),
            "reconstruction_xyz": source[index].tolist(),
            "transformed_xyz_m": transformed[index].tolist(),
            "target_xyz_m": target[index].tolist(),
            "target_frame": target_frame,
            "uncertainty_m": float(uncertainties[index]),
            "error_m": float(result.residuals[index]),
            "inlier": bool(result.inliers[index]),
            "selection_method": item.get("selection_method", "unspecified"),
            "image_frame_source": item.get("image_frame_source"),
            "physical_landmark_description": item.get("physical_landmark_description"),
            "reviewer_note": item.get("reviewer_note"),
        })
    return {
        "transform": f"T_{target_frame}_R",
        "target_frame": target_frame,
        "matrix_convention": "column_vector",
        "scale": result.scale,
        "rotation_matrix": result.rotation.tolist(),
        "quaternion_xyzw": matrix_to_quaternion_xyzw(result.rotation).tolist(),
        "translation_m": result.translation.tolist(),
        "similarity_matrix": result.matrix.tolist(),
        "determinant_rotation": float(np.linalg.det(result.rotation)),
        "handedness": "preserved" if np.linalg.det(result.rotation) > 0 else "reflected",
        "correspondence_count": len(correspondences),
        "inlier_count": int(result.inliers.sum()),
        "rms_error_m": result.rms_error,
        "max_error_m": result.max_error,
        "warnings": warnings,
        "input_correspondence_status": input_status,
        "input_limitations": data.get("limitations", []),
        "correspondences": records,
    }


def compose_to_B(registration: dict, transform_config: dict) -> dict:
    if registration.get("target_frame") != "P":
        raise ValueError("Composition through P requires a T_P_R registration.")
    item = transform_config.get("transforms", {}).get("T_B_P", transform_config.get("T_B_P"))
    if not item:
        raise ValueError("Parent transform configuration does not contain T_B_P.")
    translation, quaternion = item.get("translation_m"), item.get("quaternion_xyzw")
    if not isinstance(translation, list) or not isinstance(quaternion, list) or any(value is None for value in translation + quaternion):
        return {"transform": "T_B_R", "status": "unresolved", "reason": "T_B_P contains null fields", "composition": "T_B_R = T_B_P * T_P_R"}
    R_B_P = quaternion_xyzw_to_matrix(quaternion)
    R_P_R = np.asarray(registration["rotation_matrix"], dtype=float)
    t_B_P = np.asarray(translation, dtype=float)
    t_P_R = np.asarray(registration["translation_m"], dtype=float)
    rotation = R_B_P @ R_P_R
    composed_translation = R_B_P @ t_P_R + t_B_P
    return {
        "transform": "T_B_R",
        "status": "composed",
        "source": "T_B_P * T_P_R",
        "scale": registration["scale"],
        "rotation_matrix": rotation.tolist(),
        "quaternion_xyzw": matrix_to_quaternion_xyzw(rotation).tolist(),
        "translation_m": composed_translation.tolist(),
    }


def write_visualization(path: Path, result: dict) -> None:
    points = []
    for item in result["correspondences"]:
        points.append((*item["transformed_xyz_m"], 255, 0, 0))
        points.append((*item["target_xyz_m"], 0, 255, 0))
    header = ["ply", "format ascii 1.0", f"element vertex {len(points)}", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "end_header"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(header + [" ".join(map(str, point)) for point in points]) + "\n", encoding="utf-8")


def write_overlays(directory: Path, result: dict) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    transformed = np.asarray([item["transformed_xyz_m"] for item in result["correspondences"]], float)
    target = np.asarray([item["target_xyz_m"] for item in result["correspondences"]], float)
    for first, second, name, labels in [(0, 1, "top", ("P x (m)", "P y (m)")), (0, 2, "side", ("P x (m)", "P z (m)"))]:
        fig, axis = plt.subplots(figsize=(7, 6))
        axis.scatter(target[:, first], target[:, second], marker="+", s=100, label="P targets", c="tab:green")
        axis.scatter(transformed[:, first], transformed[:, second], marker="x", s=60, label="transformed R", c="tab:red")
        for source, destination in zip(transformed, target):
            axis.plot([source[first], destination[first]], [source[second], destination[second]], "k-", linewidth=0.7)
        axis.set_aspect("equal", adjustable="datalim"); axis.grid(True); axis.legend(); axis.set_xlabel(labels[0]); axis.set_ylabel(labels[1])
        axis.set_title(f"{result['transform']} provisional correspondence overlay ({name})")
        fig.tight_layout(); fig.savefig(directory / f"T_P_R_{name}.png", dpi=180); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Estimate T_P_R or T_B_R with weighted Umeyama alignment and optional RANSAC.")
    parser.add_argument("--input", type=Path, required=True, help="JSON/YAML correspondences.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--visualization", type=Path, help="Optional colored correspondence PLY (red=transformed R, green=selected B/P target).")
    parser.add_argument("--overlay-dir", type=Path, help="Optional top/side correspondence overlay PNG directory.")
    parser.add_argument("--ransac", action="store_true")
    parser.add_argument("--ransac-threshold-m", type=float, default=0.01)
    parser.add_argument("--ransac-iterations", type=int, default=1000)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--target-frame", choices=["B", "P"], default="B", help="Register R directly to B or to visible physical frame P.")
    parser.add_argument("--parent-transform-config", type=Path, help="Optional transforms YAML/JSON; composes T_B_R when target is P and T_B_P is known.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        result = register(load_structured(args.input), target_frame=args.target_frame, use_ransac=args.ransac, threshold_m=args.ransac_threshold_m, iterations=args.ransac_iterations, seed=args.seed)
        if args.target_frame == "P":
            result["T_B_R_composition"] = (
                compose_to_B(result, load_structured(args.parent_transform_config))
                if args.parent_transform_config
                else {"transform": "T_B_R", "status": "unresolved", "reason": "T_B_P not supplied", "composition": "T_B_R = T_B_P * T_P_R"}
            )
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        (write_yaml if args.output.suffix.lower() in {".yaml", ".yml"} else write_json)(args.output, result)
        if args.visualization:
            write_visualization(args.visualization, result)
        if args.overlay_dir:
            write_overlays(args.overlay_dir, result)
        print(f"Registration written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
