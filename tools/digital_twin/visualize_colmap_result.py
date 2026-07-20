#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import write_json


def _lines(path: Path) -> list[str]:
    return [line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip() and not line.startswith("#")]


def load_model(model: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[str]]:
    point_rows = [line.split() for line in _lines(model / "points3D.txt")]
    points = np.asarray([[float(value) for value in row[1:4]] for row in point_rows], dtype=float)
    colors = np.asarray([[int(value) for value in row[4:7]] for row in point_rows], dtype=float) / 255.0
    image_data = (model / "images.txt").read_text(encoding="utf-8").splitlines()
    pose_lines = []
    expect_pose = True
    for raw in image_data:
        if raw.startswith("#"):
            continue
        if expect_pose:
            if raw.strip():
                pose_lines.append(raw.strip().split())
                expect_pose = False
        else:
            expect_pose = True
    centers, directions, names = [], [], []
    for row in pose_lines:
        qw, qx, qy, qz = [float(value) for value in row[1:5]]
        rotation_world_to_camera = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        translation = np.asarray([float(value) for value in row[5:8]])
        centers.append(-rotation_world_to_camera.T @ translation)
        directions.append(rotation_world_to_camera.T @ np.asarray([0.0, 0.0, 1.0]))
        names.append(row[9])
    return points, colors, np.asarray(centers), np.asarray(directions), names


def robust_mask(points: np.ndarray) -> np.ndarray:
    if not len(points):
        return np.zeros(0, dtype=bool)
    center = np.median(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    return distances <= np.quantile(distances, 0.995)


def pca_coordinates(points: np.ndarray, cameras: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    combined = np.vstack((points, cameras))
    origin = np.median(points, axis=0)
    _u, _s, vt = np.linalg.svd(combined - origin, full_matrices=False)
    basis = vt.T
    return (points - origin) @ basis, (cameras - origin) @ basis, basis


def save_views(points: np.ndarray, colors: np.ndarray, cameras: np.ndarray, directions: np.ndarray, output_dir: Path, max_points: int) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    mask = robust_mask(points); points, colors = points[mask], colors[mask]
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=int)
        points, colors = points[indices], colors[indices]
    pca_points, pca_cameras, basis = pca_coordinates(points, cameras)
    outputs = {}
    fig = plt.figure(figsize=(10, 8))
    axis = fig.add_subplot(111, projection="3d")
    axis.scatter(*points.T, s=0.25, c=colors, alpha=0.6)
    axis.plot(*cameras.T, "r.-", linewidth=1, markersize=3, label="registered cameras")
    axis.set_xlabel("R.x"); axis.set_ylabel("R.y"); axis.set_zlabel("R.z"); axis.legend()
    axis.set_title("COLMAP sparse model and camera trajectory (arbitrary R scale/orientation)")
    fig.tight_layout(); outputs["sparse_with_cameras"] = str(output_dir / "sparse_with_cameras.png")
    fig.savefig(outputs["sparse_with_cameras"], dpi=180); plt.close(fig)
    fig = plt.figure(figsize=(10, 8)); axis = fig.add_subplot(111, projection="3d")
    axis.scatter(*points.T, s=0.2, c=colors, alpha=0.35)
    stride = max(1, len(cameras) // 30)
    extent = np.linalg.norm(np.quantile(points, 0.9, axis=0) - np.quantile(points, 0.1, axis=0))
    length = max(extent * 0.035, 1e-6)
    axis.quiver(*cameras[::stride].T, *directions[::stride].T, length=length, normalize=True, color="red", linewidth=0.8)
    axis.plot(*cameras.T, color="red", linewidth=0.7)
    axis.set_xlabel("R.x"); axis.set_ylabel("R.y"); axis.set_zlabel("R.z")
    axis.set_title("Registered camera forward axes/frustum-direction debug view")
    fig.tight_layout(); outputs["camera_frustums"] = str(output_dir / "camera_frustums.png")
    fig.savefig(outputs["camera_frustums"], dpi=180); plt.close(fig)
    for name, axes in (("pca_top", (0, 1)), ("pca_side", (0, 2))):
        fig, axis = plt.subplots(figsize=(10, 7))
        axis.scatter(pca_points[:, axes[0]], pca_points[:, axes[1]], s=0.3, c=colors, alpha=0.5)
        axis.plot(pca_cameras[:, axes[0]], pca_cameras[:, axes[1]], "r.-", linewidth=1, markersize=3)
        axis.set_aspect("equal", adjustable="datalim"); axis.grid(alpha=0.2)
        axis.set_title(f"{name.replace('_', ' ').title()} — PCA debug projection, not B-frame alignment")
        path = output_dir / f"{name}.png"; fig.tight_layout(); fig.savefig(path, dpi=180); plt.close(fig); outputs[name] = str(path)
    return {"outputs": outputs, "pca_basis_columns_in_R": basis.tolist(), "warning": "PCA top/side labels are visualization aids only; T_P_R and T_B_R are unresolved."}


def main() -> None:
    parser = argparse.ArgumentParser(description="Render deterministic debug views of a COLMAP text sparse model and camera trajectory.")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-points", type=int, default=50000)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if not args.model.is_dir():
            raise FileNotFoundError(f"COLMAP text model does not exist: {args.model}")
        if args.dry_run:
            print(json.dumps({"model": str(args.model), "output_dir": str(args.output_dir), "max_points": args.max_points}, indent=2)); return
        points, colors, cameras, directions, names = load_model(args.model)
        report = save_views(points, colors, cameras, directions, args.output_dir, args.max_points)
        report.update({"model": str(args.model), "point_count": len(points), "camera_count": len(cameras), "registered_images": names})
        write_json(args.output_dir / "visualization_report.json", report)
        print(f"COLMAP visualizations written to: {args.output_dir}")
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
