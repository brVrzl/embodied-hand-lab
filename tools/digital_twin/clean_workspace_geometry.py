#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial import cKDTree

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json


def load_points(path: Path) -> tuple[np.ndarray, np.ndarray | None]:
    if not path.is_file():
        raise FileNotFoundError(f"Geometry input does not exist: {path}")
    if path.suffix.lower() == ".npy":
        return np.asarray(np.load(path), dtype=float), None
    if path.suffix.lower() == ".npz":
        data = np.load(path)
        return np.asarray(data["points"], dtype=float), np.asarray(data["colors"]) if "colors" in data else None
    loaded = trimesh.load(path, process=False)
    if isinstance(loaded, trimesh.Scene):
        loaded = loaded.dump(concatenate=True)
    points = np.asarray(getattr(loaded, "vertices", None), dtype=float)
    colors = getattr(getattr(loaded, "visual", None), "vertex_colors", None)
    return points, None if colors is None else np.asarray(colors)


def clean_points(points: np.ndarray, *, crop_min: np.ndarray | None, crop_max: np.ndarray | None,
                 remove_box_min: np.ndarray | None, remove_box_max: np.ndarray | None,
                 voxel_size: float | None, outlier_neighbors: int, outlier_std: float) -> tuple[np.ndarray, np.ndarray]:
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or not np.all(np.isfinite(values)):
        raise ValueError("Input points must be a finite array with shape (N, 3).")
    mask = np.ones(len(values), dtype=bool)
    if crop_min is not None or crop_max is not None:
        if crop_min is None or crop_max is None:
            raise ValueError("Both crop minimum and maximum are required.")
        mask &= np.all((values >= crop_min) & (values <= crop_max), axis=1)
    if remove_box_min is not None or remove_box_max is not None:
        if remove_box_min is None or remove_box_max is None:
            raise ValueError("Both robot-removal box minimum and maximum are required.")
        mask &= ~np.all((values >= remove_box_min) & (values <= remove_box_max), axis=1)
    kept_indices = np.flatnonzero(mask)
    values = values[mask]
    if voxel_size is not None:
        if voxel_size <= 0:
            raise ValueError("Voxel size must be positive.")
        keys = np.floor(values / voxel_size).astype(np.int64)
        _, selected = np.unique(keys, axis=0, return_index=True)
        selected.sort()
        values, kept_indices = values[selected], kept_indices[selected]
    if outlier_neighbors > 0 and len(values) > outlier_neighbors:
        distances, _ = cKDTree(values).query(values, k=outlier_neighbors + 1)
        mean_distance = distances[:, 1:].mean(axis=1)
        threshold = float(mean_distance.mean() + outlier_std * mean_distance.std())
        local_mask = mean_distance <= threshold
        values, kept_indices = values[local_mask], kept_indices[local_mask]
    return values, kept_indices


def main() -> None:
    parser = argparse.ArgumentParser(description="Crop, de-noise, voxelize and explicitly remove robot/clutter boxes from point geometry.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="Output .ply or .npz.")
    parser.add_argument("--config", type=Path, help="Optional JSON/YAML values for crop_min/max and robot_remove_min/max.")
    parser.add_argument("--crop-min", nargs=3, type=float)
    parser.add_argument("--crop-max", nargs=3, type=float)
    parser.add_argument("--robot-remove-min", nargs=3, type=float)
    parser.add_argument("--robot-remove-max", nargs=3, type=float)
    parser.add_argument("--voxel-size", type=float)
    parser.add_argument("--outlier-neighbors", type=int, default=12)
    parser.add_argument("--outlier-std", type=float, default=2.5)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        config = load_structured(args.config) if args.config else {}
        def vec(cli, key):
            value = cli if cli is not None else config.get(key)
            return None if value is None else np.asarray(value, dtype=float)
        crop_min, crop_max = vec(args.crop_min, "crop_min"), vec(args.crop_max, "crop_max")
        remove_min, remove_max = vec(args.robot_remove_min, "robot_remove_min"), vec(args.robot_remove_max, "robot_remove_max")
        points, colors = load_points(args.input)
        cleaned, indices = clean_points(points, crop_min=crop_min, crop_max=crop_max, remove_box_min=remove_min,
                                        remove_box_max=remove_max, voxel_size=args.voxel_size,
                                        outlier_neighbors=args.outlier_neighbors, outlier_std=args.outlier_std)
        report = {"input": str(args.input), "input_count": len(points), "output_count": len(cleaned), "frame": config.get("frame"), "operations": {"crop_min": None if crop_min is None else crop_min.tolist(), "crop_max": None if crop_max is None else crop_max.tolist(), "robot_remove_min": None if remove_min is None else remove_min.tolist(), "robot_remove_max": None if remove_max is None else remove_max.tolist(), "voxel_size": args.voxel_size, "outlier_neighbors": args.outlier_neighbors, "outlier_std": args.outlier_std}}
        if args.dry_run:
            print(json.dumps(report, indent=2)); return
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output_colors = None if colors is None else colors[indices]
        if args.output.suffix.lower() == ".npz":
            np.savez_compressed(args.output, points=cleaned, **({} if output_colors is None else {"colors": output_colors}))
        elif args.output.suffix.lower() == ".ply":
            trimesh.PointCloud(cleaned, colors=output_colors).export(args.output)
        else:
            raise ValueError("Output must be .ply or .npz.")
        write_json(args.output.with_suffix(args.output.suffix + ".report.json"), report)
        print(f"Cleaned geometry written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
