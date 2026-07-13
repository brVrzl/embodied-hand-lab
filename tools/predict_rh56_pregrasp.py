#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from pregrasp import GeometryAwarePregraspPredictor, geometry_from_point_cloud, load_primitive_config


def main() -> None:
    parser = argparse.ArgumentParser(description="Predict RH56 pregrasp candidates from object geometry.")
    parser.add_argument("--point-cloud", type=Path, help="Numpy .npy file with shape (N, 3).")
    parser.add_argument("--geometry-json", type=Path, help="JSON file with either 'points' or 'extents_xyz_m'.")
    parser.add_argument("--primitive-config", type=Path, help="Optional YAML primitive config.")
    parser.add_argument("--task-mode", default="pick", choices=["pick", "hold", "push", "pull", "pre_align"])
    parser.add_argument("--top-k", type=int, default=3)
    parser.add_argument("--frame-id", default="camera")
    args = parser.parse_args()

    geometry = _load_geometry(args.point_cloud, args.geometry_json, frame_id=args.frame_id)
    primitives = load_primitive_config(args.primitive_config) if args.primitive_config else None
    predictor = GeometryAwarePregraspPredictor(primitives)
    candidates = predictor.predict(geometry, task_mode=args.task_mode, top_k=args.top_k)
    print(
        json.dumps(
            {
                "geometry": geometry.to_dict(),
                "task_mode": args.task_mode,
                "candidates": [candidate.to_dict() for candidate in candidates],
            },
            ensure_ascii=False,
            indent=2,
        )
    )


def _load_geometry(point_cloud: Path | None, geometry_json: Path | None, *, frame_id: str):
    if bool(point_cloud) == bool(geometry_json):
        raise SystemExit("Provide exactly one of --point-cloud or --geometry-json.")
    if point_cloud:
        if not point_cloud.exists():
            raise SystemExit(f"Point cloud file does not exist: {point_cloud}")
        return geometry_from_point_cloud(np.load(point_cloud), frame_id=frame_id)

    if not geometry_json.exists():
        raise SystemExit(f"Geometry JSON file does not exist: {geometry_json}")
    data: dict[str, Any] = json.loads(geometry_json.read_text(encoding="utf-8"))
    if "points" in data:
        return geometry_from_point_cloud(data["points"], frame_id=data.get("frame_id", frame_id), shape_hint=data.get("shape_hint"))
    if "extents_xyz_m" not in data:
        raise SystemExit("--geometry-json must contain either 'points' or 'extents_xyz_m'.")

    extents = np.asarray(data["extents_xyz_m"], dtype=np.float64)
    if extents.shape != (3,):
        raise SystemExit("'extents_xyz_m' must contain exactly 3 values.")
    half = extents / 2.0
    corners = np.asarray(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    centroid = np.asarray(data.get("centroid_xyz", [0.0, 0.0, 0.0]), dtype=np.float64)
    return geometry_from_point_cloud(
        corners + centroid,
        frame_id=data.get("frame_id", frame_id),
        percentile=100.0,
        shape_hint=data.get("shape_hint"),
    )


if __name__ == "__main__":
    main()
