#!/usr/bin/env python3
"""Associate detected ChArUco corners with COLMAP sparse points and emit scale references."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.calibration.charuco import load_board_specs, make_board
from digital_twin.io import load_structured, write_json


def _data_lines(path: Path, preserve_empty: bool = False) -> list[str]:
    lines = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        stripped = raw.strip()
        if stripped.startswith("#"):
            continue
        if stripped or preserve_empty:
            lines.append(stripped)
    return lines


def load_points(path: Path) -> dict[int, np.ndarray]:
    result = {}
    for line in _data_lines(path):
        fields = line.split()
        result[int(fields[0])] = np.asarray(fields[1:4], dtype=float)
    return result


def load_image_observations(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    lines = _data_lines(path, preserve_empty=True)
    if len(lines) % 2:
        raise ValueError("COLMAP images.txt has an odd number of data lines.")
    result = {}
    for offset in range(0, len(lines), 2):
        pose = lines[offset].split()
        observations = lines[offset + 1].split()
        xy = np.asarray([[float(observations[i]), float(observations[i + 1])] for i in range(0, len(observations), 3)], dtype=float)
        point_ids = np.asarray([int(observations[i + 2]) for i in range(0, len(observations), 3)], dtype=np.int64)
        keep = point_ids >= 0
        result[pose[9]] = (xy[keep], point_ids[keep])
    return result


def _nearest_point_ids(corners: np.ndarray, observations: np.ndarray, point_ids: np.ndarray, threshold_px: float) -> dict[int, int]:
    if not len(observations):
        return {}
    matches = {}
    for index, corner in enumerate(corners):
        distances = np.linalg.norm(observations - corner, axis=1)
        nearest = int(np.argmin(distances))
        if distances[nearest] <= threshold_px:
            matches[index] = int(point_ids[nearest])
    return matches


def _frame_lookup(frame_manifest: dict) -> dict[float, str]:
    return {
        round(float(row["source_timestamp_sec"]), 3): str(row["frame_filename"])
        for row in frame_manifest["frames"] if row.get("accepted")
    }


def _cluster_board_sizes(observations: list[dict], expected_ratio: float) -> dict:
    values = np.asarray([item["median_reconstruction_square"] for item in observations], np.float32)
    if len(values) < 4:
        return {"status": "insufficient_observations", "observed_ratio": None, "outlier_count": 0}
    median = float(np.median(values))
    # Nearest-SIFT association can occasionally select a geometrically unrelated
    # sparse point. Only discard catastrophic order-of-magnitude failures here;
    # ordinary board/reconstruction disagreement must remain visible downstream.
    plausible = (values >= median / 2.5) & (values <= median * 2.5)
    for item, keep in zip(observations, plausible):
        item["association_outlier"] = not bool(keep)
    filtered = values[plausible]
    if len(filtered) < 4:
        return {"status": "insufficient_non_outlier_observations", "observed_ratio": None, "outlier_count": int((~plausible).sum())}
    _compactness, labels, centers = cv2.kmeans(
        filtered.reshape(-1, 1), 2, None,
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7),
        20, cv2.KMEANS_PP_CENTERS,
    )
    centers = centers.reshape(-1)
    large_label, small_label = int(np.argmax(centers)), int(np.argmin(centers))
    ratio = float(centers[large_label] / centers[small_label])
    status = "consistent_with_A3_A4_sizes" if 0.85 * expected_ratio <= ratio <= 1.15 * expected_ratio else "size_clusters_disagree_with_A3_A4_ratio"
    for item, label in zip([item for item in observations if not item["association_outlier"]], labels.reshape(-1)):
        item["metric_board_assignment"] = "A3" if int(label) == large_label else "A4"
        item["assignment_method"] = "two_cluster_reconstructed_square_size"
    return {
        "status": status,
        "expected_ratio_A3_to_A4": expected_ratio,
        "observed_ratio": ratio,
        "cluster_centers_reconstruction_units": {"A3": float(centers[large_label]), "A4": float(centers[small_label])},
        "outlier_count": int((~plausible).sum()),
    }


def extract(args: argparse.Namespace) -> dict:
    specs = load_board_specs(args.boards)
    spec_by_group = {"A3": max(specs, key=lambda item: item.square_length_m), "A4": min(specs, key=lambda item: item.square_length_m)}
    board = make_board(specs[0])
    chessboard_points = np.asarray(board.getChessboardCorners(), dtype=float)
    points = load_points(args.model / "points3D.txt")
    image_observations = load_image_observations(args.model / "images.txt")
    detection_manifest = load_structured(args.detections)
    frame_lookup = _frame_lookup(load_structured(args.frame_manifest))
    board_observations = []
    for frame in detection_manifest["frames"]:
        timestamp = round(float(frame["timestamp_sec"]), 3)
        image_name = frame_lookup.get(timestamp)
        if image_name not in image_observations:
            continue
        sfm_xy, sfm_point_ids = image_observations[image_name]
        for detection_index, detection in enumerate(frame["detections"]):
            if not detection.get("accepted"):
                continue
            pixels = np.asarray(detection["charuco_corner_pixels"], dtype=float)
            corner_ids = np.asarray(detection["charuco_corner_ids"], dtype=int)
            nearest = _nearest_point_ids(pixels, sfm_xy, sfm_point_ids, args.max_pixel_distance)
            point_by_corner_id = {int(corner_ids[index]): points[point_id] for index, point_id in nearest.items() if point_id in points}
            distances = []
            point_pairs = []
            ids = sorted(point_by_corner_id)
            for first_index, first in enumerate(ids):
                for second in ids[first_index + 1:]:
                    physical_grid_distance = float(np.linalg.norm(chessboard_points[first] - chessboard_points[second]) / specs[0].square_length_m)
                    if not np.isclose(physical_grid_distance, 1.0, atol=1e-6):
                        continue
                    distance = float(np.linalg.norm(point_by_corner_id[first] - point_by_corner_id[second]))
                    if distance > 0:
                        distances.append(distance)
                        point_pairs.append([first, second])
            if len(distances) >= args.minimum_adjacent_pairs:
                board_observations.append({
                    "observation_id": f"{image_name}:detection_{detection_index}",
                    "image_name": image_name,
                    "timestamp_sec": timestamp,
                    "detected_corner_count": len(corner_ids),
                    "matched_sparse_corner_count": len(point_by_corner_id),
                    "adjacent_pair_count": len(distances),
                    "median_reconstruction_square": float(np.median(distances)),
                    "mad_reconstruction_square": float(np.median(np.abs(np.asarray(distances) - np.median(distances)))),
                    "center_reconstruction": np.median(np.asarray(list(point_by_corner_id.values())), axis=0).tolist(),
                    "point_pairs": point_pairs,
                    "detector_identity_status": detection.get("identity_status"),
                    "detector_candidate_board_names": detection.get("candidate_board_names"),
                })
    expected_ratio = spec_by_group["A3"].square_length_m / spec_by_group["A4"].square_length_m
    clustering = _cluster_board_sizes(board_observations, expected_ratio)
    a4_observations = [
        item for item in board_observations
        if item.get("metric_board_assignment") == "A4" and not item.get("association_outlier", False)
    ]
    instance_clustering = {"status": "insufficient_A4_observations", "instances": []}
    if len(a4_observations) >= 4:
        centers = np.asarray([item["center_reconstruction"] for item in a4_observations], np.float32)
        _compactness, labels, spatial_centers = cv2.kmeans(
            centers, 2, None,
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7),
            20, cv2.KMEANS_PP_CENTERS,
        )
        order = sorted(range(2), key=lambda index: tuple(spatial_centers[index].tolist()))
        names = {label: f"Board_A4_{rank + 1}" for rank, label in enumerate(order)}
        for item, label in zip(a4_observations, labels.reshape(-1)):
            item["physical_instance"] = names[int(label)]
            item["instance_assignment_method"] = "spatial_cluster_in_reconstruction"
        instance_clustering = {
            "status": "two_A4_spatial_clusters_assigned",
            "instances": [names[index] for index in order],
            "cluster_centers_reconstruction": {names[index]: spatial_centers[index].tolist() for index in order},
            "warning": "Instance numbering is deterministic spatial ordering, not marker-ID identity.",
        }
    for item in board_observations:
        if item.get("metric_board_assignment") == "A3":
            item["physical_instance"] = "Board_A3_1"
            item["instance_assignment_method"] = "confirmed_single_A3_plus_size_cluster"
    references = []
    for item in board_observations:
        group = item.get("metric_board_assignment")
        if group not in spec_by_group or clustering.get("status") != "consistent_with_A3_A4_sizes":
            continue
        spec = spec_by_group[group]
        # Use within-observation dispersion as a conservative uncertainty proxy;
        # the known print dimension itself is user-confirmed.
        relative_mad = item["mad_reconstruction_square"] / max(item["median_reconstruction_square"], 1e-12)
        references.append({
            "name": item["observation_id"],
            "group": group,
            "physical_instance": item.get("physical_instance"),
            "observation_id": item["observation_id"],
            "source": "ChArUco corner to COLMAP sparse-point association",
            "reconstruction_distance": item["median_reconstruction_square"],
            "known_distance_m": spec.square_length_m,
            "uncertainty_m": max(spec.square_length_m * relative_mad, args.minimum_uncertainty_m),
        })
    return {
        "schema_version": 1,
        "model": str(args.model),
        "max_sparse_corner_pixel_distance": args.max_pixel_distance,
        "minimum_adjacent_pairs": args.minimum_adjacent_pairs,
        "board_observation_count": len(board_observations),
        "reference_count": len(references),
        "identity_clustering": clustering,
        "physical_instance_clustering": instance_clustering,
        "board_observations": board_observations,
        "references": references,
        "limitations": [
            "ChArUco corners are associated to nearby triangulated SIFT observations, not directly constrained in COLMAP bundle adjustment.",
            "A3/A4 identity is inferred from two reconstructed square-size clusters because marker IDs 0-16 are reused.",
            "Metric scale must be rejected if cluster ratio or per-board residual diagnostics disagree.",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Extract A3/A4 ChArUco scale references from a COLMAP text model and detection manifest.")
    parser.add_argument("--model", type=Path, required=True, help="COLMAP text model directory.")
    parser.add_argument("--detections", type=Path, required=True)
    parser.add_argument("--frame-manifest", type=Path, required=True)
    parser.add_argument("--boards", type=Path, default=Path("digital_twin/configs/charuco_boards.yaml"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-pixel-distance", type=float, default=3.0)
    parser.add_argument("--minimum-adjacent-pairs", type=int, default=2)
    parser.add_argument("--minimum-uncertainty-m", type=float, default=0.0002)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if not args.model.is_dir():
            raise FileNotFoundError(f"Text model directory not found: {args.model}")
        result = extract(args)
        if args.dry_run:
            print(json.dumps(result, indent=2)); return
        write_json(args.output, result)
        print(f"ChArUco scale references written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
