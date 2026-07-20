#!/usr/bin/env python3
"""Register two independent COLMAP text models through cross-video SIFT tracks."""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json
from digital_twin.registration.transforms import apply_similarity, matrix_to_quaternion_xyzw, ransac_similarity


def _data_lines(path: Path, *, preserve_empty: bool = False) -> list[str]:
    result = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("#"):
            continue
        if line or preserve_empty:
            result.append(line)
    return result


def load_model(model: Path) -> tuple[dict[str, np.ndarray], dict[int, np.ndarray]]:
    """Return image-name -> point IDs by keypoint row and point-ID -> XYZ."""
    lines = _data_lines(model / "images.txt", preserve_empty=True)
    if len(lines) % 2:
        raise ValueError(f"Malformed COLMAP images.txt: {model / 'images.txt'}")
    image_points: dict[str, np.ndarray] = {}
    for offset in range(0, len(lines), 2):
        pose = lines[offset].split()
        observations = lines[offset + 1].split()
        if len(pose) < 10 or len(observations) % 3:
            raise ValueError(f"Malformed image record at data line {offset + 1}")
        image_points[pose[9]] = np.asarray(
            [int(observations[index + 2]) for index in range(0, len(observations), 3)], dtype=np.int64
        )
    points: dict[int, np.ndarray] = {}
    for line in _data_lines(model / "points3D.txt"):
        fields = line.split()
        points[int(fields[0])] = np.asarray(fields[1:4], dtype=np.float64)
    return image_points, points


class DescriptorDatabase:
    def __init__(self, path: Path):
        self.connection = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        self.images = {
            str(name): int(image_id)
            for image_id, name in self.connection.execute("SELECT image_id, name FROM images")
        }

    def close(self) -> None:
        self.connection.close()

    def descriptors(self, name: str) -> np.ndarray:
        image_id = self.images.get(name)
        if image_id is None:
            raise KeyError(f"Image {name!r} is absent from descriptor database")
        row = self.connection.execute(
            "SELECT rows, cols, data FROM descriptors WHERE image_id=?", (image_id,)
        ).fetchone()
        if row is None:
            return np.empty((0, 128), dtype=np.uint8)
        rows, cols, blob = row
        return np.frombuffer(blob, dtype=np.uint8).reshape(int(rows), int(cols))

    def keypoints(self, name: str) -> np.ndarray:
        image_id = self.images.get(name)
        if image_id is None:
            raise KeyError(f"Image {name!r} is absent from keypoint database")
        row = self.connection.execute(
            "SELECT rows, cols, data FROM keypoints WHERE image_id=?", (image_id,)
        ).fetchone()
        if row is None:
            return np.empty((0, 2), dtype=np.float32)
        rows, cols, blob = row
        return np.frombuffer(blob, dtype=np.float32).reshape(int(rows), int(cols))[:, :2]

    def features(self, name: str, polygons: list[np.ndarray] | None = None) -> tuple[np.ndarray, np.ndarray]:
        descriptors = self.descriptors(name)
        indices = np.arange(len(descriptors), dtype=np.int64)
        if not polygons or not len(descriptors):
            return descriptors, indices
        xy = self.keypoints(name)
        if len(xy) != len(descriptors):
            raise ValueError(f"Keypoint/descriptor row mismatch for {name}")
        keep = np.ones(len(xy), dtype=bool)
        for polygon in polygons:
            contour = np.asarray(polygon, dtype=np.float32)
            keep &= np.asarray([cv2.pointPolygonTest(contour, tuple(map(float, point)), False) < 0 for point in xy])
        return descriptors[keep], indices[keep]


def load_board_masks(detections_path: Path | None, frame_manifest_path: Path | None, expansion: float) -> dict[str, list[np.ndarray]]:
    if detections_path is None and frame_manifest_path is None:
        return {}
    if detections_path is None or frame_manifest_path is None:
        raise ValueError("Board masking requires both a detection manifest and a frame manifest.")
    detections = load_structured(detections_path)
    manifest = load_structured(frame_manifest_path)
    names_by_time = {
        round(float(row["source_timestamp_sec"]), 3): str(row["frame_filename"])
        for row in manifest["frames"] if row.get("accepted")
    }
    masks: dict[str, list[np.ndarray]] = {}
    for frame in detections["frames"]:
        name = names_by_time.get(round(float(frame["timestamp_sec"]), 3))
        if not name:
            continue
        for detection in frame["detections"]:
            if len(detection.get("charuco_corner_pixels", [])) < 4:
                continue
            hull = cv2.convexHull(np.asarray(detection["charuco_corner_pixels"], dtype=np.float32)).reshape(-1, 2)
            center = hull.mean(axis=0)
            masks.setdefault(name, []).append(center + expansion * (hull - center))
    return masks


def evenly_spaced(names: list[str], maximum: int) -> list[str]:
    ordered = sorted(names)
    if maximum <= 0 or len(ordered) <= maximum:
        return ordered
    indices = np.unique(np.rint(np.linspace(0, len(ordered) - 1, maximum)).astype(int))
    return [ordered[index] for index in indices]


def coarse_score(first: np.ndarray, second: np.ndarray, samples: int, seed: int) -> int:
    if not len(first) or not len(second):
        return 0
    rng = np.random.default_rng(seed)
    a = first[rng.choice(len(first), min(samples, len(first)), replace=False)]
    b = second[rng.choice(len(second), min(samples * 3, len(second)), replace=False)]
    matches = cv2.BFMatcher(cv2.NORM_L2).match(a, b)
    return sum(match.distance < 260.0 for match in matches)


def mutual_ratio_matches(first: np.ndarray, second: np.ndarray, ratio: float) -> list[tuple[int, int, float]]:
    if len(first) < 2 or len(second) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    forward_knn = matcher.knnMatch(first, second, k=2)
    reverse_knn = matcher.knnMatch(second, first, k=2)
    forward = {
        pair[0].queryIdx: (pair[0].trainIdx, float(pair[0].distance))
        for pair in forward_knn if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance
    }
    reverse = {
        pair[0].queryIdx: pair[0].trainIdx
        for pair in reverse_knn if len(pair) == 2 and pair[0].distance < ratio * pair[1].distance
    }
    return [(query, train, distance) for query, (train, distance) in forward.items() if reverse.get(train) == query]


def collect_correspondences(
    model_a: dict[str, np.ndarray], points_a: dict[int, np.ndarray], db_a: DescriptorDatabase,
    model_b: dict[str, np.ndarray], points_b: dict[int, np.ndarray], db_b: DescriptorDatabase,
    *, max_images_a: int, max_images_b: int, top_pairs: int, coarse_samples: int, ratio: float, seed: int,
    masks_a: dict[str, list[np.ndarray]] | None = None, masks_b: dict[str, list[np.ndarray]] | None = None,
) -> tuple[np.ndarray, np.ndarray, list[dict], list[dict]]:
    names_a = evenly_spaced(list(model_a), max_images_a)
    names_b = evenly_spaced(list(model_b), max_images_b)
    features_a = {name: db_a.features(name, (masks_a or {}).get(name)) for name in names_a}
    features_b = {name: db_b.features(name, (masks_b or {}).get(name)) for name in names_b}
    descriptors_a = {name: value[0] for name, value in features_a.items()}
    descriptors_b = {name: value[0] for name, value in features_b.items()}
    candidate_pairs = []
    for index_a, name_a in enumerate(names_a):
        scored = []
        for index_b, name_b in enumerate(names_b):
            score = coarse_score(descriptors_a[name_a], descriptors_b[name_b], coarse_samples, seed + index_a * 1009 + index_b)
            scored.append((score, name_b))
        for score, name_b in sorted(scored, reverse=True)[:top_pairs]:
            candidate_pairs.append({"image_a": name_a, "image_b": name_b, "coarse_score": score})

    best_by_point_pair: dict[tuple[int, int], dict] = {}
    for pair in candidate_pairs:
        name_a, name_b = pair["image_a"], pair["image_b"]
        matches = mutual_ratio_matches(descriptors_a[name_a], descriptors_b[name_b], ratio)
        point_ids_a, point_ids_b = model_a[name_a], model_b[name_b]
        original_indices_a, original_indices_b = features_a[name_a][1], features_b[name_b][1]
        retained = 0
        for filtered_a, filtered_b, distance in matches:
            keypoint_a, keypoint_b = int(original_indices_a[filtered_a]), int(original_indices_b[filtered_b])
            if keypoint_a >= len(point_ids_a) or keypoint_b >= len(point_ids_b):
                continue
            point_a, point_b = int(point_ids_a[keypoint_a]), int(point_ids_b[keypoint_b])
            if point_a < 0 or point_b < 0 or point_a not in points_a or point_b not in points_b:
                continue
            key = (point_a, point_b)
            item = {
                "point_id_a": point_a, "point_id_b": point_b, "descriptor_distance": distance,
                "image_a": name_a, "image_b": name_b,
            }
            if key not in best_by_point_pair or distance < best_by_point_pair[key]["descriptor_distance"]:
                best_by_point_pair[key] = item
            retained += 1
        pair["mutual_ratio_matches"] = len(matches)
        pair["triangulated_matches"] = retained
    records = list(best_by_point_pair.values())
    source = np.asarray([points_a[item["point_id_a"]] for item in records], dtype=np.float64)
    target = np.asarray([points_b[item["point_id_b"]] for item in records], dtype=np.float64)
    return source, target, records, candidate_pairs


def write_correspondence_ply(path: Path, source_in_target: np.ndarray, target: np.ndarray, inliers: np.ndarray) -> None:
    rows = []
    for transformed, reference, keep in zip(source_in_target, target, inliers):
        color_a = (0, 220, 0) if keep else (255, 120, 0)
        color_b = (0, 120, 255) if keep else (255, 0, 0)
        rows.extend([(*transformed, *color_a), (*reference, *color_b)])
    header = ["ply", "format ascii 1.0", f"element vertex {len(rows)}", "property float x", "property float y", "property float z", "property uchar red", "property uchar green", "property uchar blue", "end_header"]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(header + [" ".join(map(str, row)) for row in rows]) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Estimate T_Rb_Ra between independent COLMAP models using cross-video SIFT/3D correspondences.")
    parser.add_argument("--model-a", type=Path, required=True, help="Source COLMAP text model (Ra).")
    parser.add_argument("--database-a", type=Path, required=True)
    parser.add_argument("--model-b", type=Path, required=True, help="Target COLMAP text model (Rb).")
    parser.add_argument("--database-b", type=Path, required=True)
    parser.add_argument("--frame-a", default="R01")
    parser.add_argument("--frame-b", default="R02")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--visualization", type=Path)
    parser.add_argument("--max-images-a", type=int, default=12)
    parser.add_argument("--max-images-b", type=int, default=24)
    parser.add_argument("--top-pairs", type=int, default=3)
    parser.add_argument("--coarse-samples", type=int, default=192)
    parser.add_argument("--ratio", type=float, default=0.72)
    parser.add_argument("--detections-a", type=Path, help="Optional ChArUco detections used to mask repeated board texture.")
    parser.add_argument("--frame-manifest-a", type=Path)
    parser.add_argument("--detections-b", type=Path, help="Optional ChArUco detections used to mask repeated board texture.")
    parser.add_argument("--frame-manifest-b", type=Path)
    parser.add_argument("--board-mask-expansion", type=float, default=1.35)
    parser.add_argument("--ransac-threshold-target", type=float, default=0.08, help="In target reconstruction units.")
    parser.add_argument("--ransac-iterations", type=int, default=10000)
    parser.add_argument("--target-metric-scale", type=float, help="Optional metres per target reconstruction unit for metric residual reporting.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    db_a = db_b = None
    try:
        for path in (args.model_a, args.model_b):
            if not path.is_dir() or not all((path / name).is_file() for name in ("images.txt", "points3D.txt")):
                raise FileNotFoundError(f"Incomplete COLMAP text model: {path}")
        for path in (args.database_a, args.database_b):
            if not path.is_file():
                raise FileNotFoundError(f"COLMAP database not found: {path}")
        model_a, points_a = load_model(args.model_a)
        model_b, points_b = load_model(args.model_b)
        db_a, db_b = DescriptorDatabase(args.database_a), DescriptorDatabase(args.database_b)
        masks_a = load_board_masks(args.detections_a, args.frame_manifest_a, args.board_mask_expansion)
        masks_b = load_board_masks(args.detections_b, args.frame_manifest_b, args.board_mask_expansion)
        source, target, records, pairs = collect_correspondences(
            model_a, points_a, db_a, model_b, points_b, db_b,
            max_images_a=args.max_images_a, max_images_b=args.max_images_b, top_pairs=args.top_pairs,
            coarse_samples=args.coarse_samples, ratio=args.ratio, seed=args.seed,
            masks_a=masks_a, masks_b=masks_b,
        )
        if len(source) < 3:
            raise ValueError(f"Only {len(source)} unique triangulated cross-model correspondences were found.")
        fit = ransac_similarity(source, target, threshold=args.ransac_threshold_target, iterations=args.ransac_iterations, seed=args.seed)
        transformed = apply_similarity(source, fit.scale, fit.rotation, fit.translation)
        for item, residual, keep in zip(records, fit.residuals, fit.inliers):
            item["residual_target_units"] = float(residual)
            item["inlier"] = bool(keep)
        metric_scale = args.target_metric_scale
        result = {
            "schema_version": 1,
            "transform": f"T_{args.frame_b}_{args.frame_a}",
            "convention": f"maps coordinates in {args.frame_a} into {args.frame_b}",
            "method": "mutual_ratio_SIFT_3D_correspondences_plus_RANSAC_Umeyama",
            "charuco_texture_masked": bool(masks_a or masks_b),
            "scale_Rb_per_Ra": fit.scale,
            "rotation_matrix": fit.rotation.tolist(),
            "quaternion_xyzw": matrix_to_quaternion_xyzw(fit.rotation).tolist(),
            "translation_Rb_units": fit.translation.tolist(),
            "determinant_rotation": float(np.linalg.det(fit.rotation)),
            "candidate_correspondence_count": len(records),
            "inlier_count": int(fit.inliers.sum()),
            "inlier_ratio": float(fit.inliers.mean()),
            "rms_target_units": fit.rms_error,
            "max_inlier_target_units": fit.max_error,
            "target_metric_scale_m_per_unit": metric_scale,
            "rms_m": None if metric_scale is None else fit.rms_error * metric_scale,
            "max_inlier_m": None if metric_scale is None else fit.max_error * metric_scale,
            "selected_image_pair_diagnostics": pairs,
            "correspondences": records,
            "acceptance": "provisional_requires_visual_fixed_geometry_review",
            "warning": "Descriptor agreement may include repeated ChArUco patterns or robot surfaces; metric residual and visual fixed-geometry overlays are mandatory before fusion.",
        }
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        write_json(args.output, result)
        if args.visualization:
            write_correspondence_ply(args.visualization, transformed, target, fit.inliers)
        print(f"Cross-model registration written to: {args.output}")
    except (FileNotFoundError, KeyError, sqlite3.Error, ValueError) as exc:
        parser.error(str(exc))
    finally:
        if db_a is not None:
            db_a.close()
        if db_b is not None:
            db_b.close()


if __name__ == "__main__":
    main()
