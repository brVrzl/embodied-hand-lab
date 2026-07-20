from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from digital_twin.io import load_structured


@dataclass(frozen=True)
class BoardSpec:
    name: str
    dictionary: str
    squares_x: int
    squares_y: int
    square_length_m: float
    marker_length_m: float
    paper_size_m: tuple[float, float]

    @property
    def internal_corner_count(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)

    @property
    def marker_count(self) -> int:
        return (self.squares_x * self.squares_y) // 2


@dataclass
class Detection:
    cluster_index: int
    marker_ids: np.ndarray
    marker_corners: list[np.ndarray]
    charuco_ids: np.ndarray
    charuco_corners: np.ndarray
    square_pixel_scale: float
    homography_rms_px: float | None
    coverage_ratio: float
    foreshortening_ratio: float | None
    center_xy: tuple[float, float]
    candidate_board_names: list[str]
    identity_status: str
    accepted: bool
    rejection_reason: str | None


def dictionary_from_name(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"OpenCV does not define ArUco dictionary {name!r}.")
    return cv2.aruco.getPredefinedDictionary(int(getattr(cv2.aruco, name)))


def load_board_specs(path: str | Path) -> list[BoardSpec]:
    data = load_structured(path)
    specs = []
    for raw in data.get("boards", []):
        spec = BoardSpec(
            name=str(raw["name"]),
            dictionary=str(raw["dictionary"]),
            squares_x=int(raw["squaresX"]),
            squares_y=int(raw["squaresY"]),
            square_length_m=float(raw["square_length_m"]),
            marker_length_m=float(raw["marker_length_m"]),
            paper_size_m=tuple(float(value) for value in raw["paper_size_m"]),
        )
        if spec.squares_x < 2 or spec.squares_y < 2:
            raise ValueError(f"Board {spec.name} needs at least 2x2 squares.")
        if not 0 < spec.marker_length_m < spec.square_length_m:
            raise ValueError(f"Board {spec.name} marker length must be between zero and square length.")
        specs.append(spec)
    if not specs:
        raise ValueError("Board configuration contains no boards.")
    if len({spec.dictionary for spec in specs}) != 1:
        raise ValueError("This detector currently requires all configured boards to use one dictionary.")
    grids = {(spec.squares_x, spec.squares_y) for spec in specs}
    if len(grids) != 1:
        raise ValueError("This detector currently requires configured boards to share one square grid.")
    return specs


def make_board(spec: BoardSpec):
    return cv2.aruco.CharucoBoard(
        (spec.squares_x, spec.squares_y),
        spec.square_length_m,
        spec.marker_length_m,
        dictionary_from_name(spec.dictionary),
    )


def _marker_side(corner: np.ndarray) -> float:
    points = np.asarray(corner, dtype=np.float32).reshape(4, 2)
    return float(np.mean(np.linalg.norm(np.roll(points, -1, axis=0) - points, axis=1)))


def _union_find_clusters(corners: list[np.ndarray], factor: float) -> list[list[int]]:
    count = len(corners)
    centers = np.asarray([np.asarray(corner).reshape(4, 2).mean(axis=0) for corner in corners])
    sizes = np.asarray([_marker_side(corner) for corner in corners])
    parents = list(range(count))

    def find(index: int) -> int:
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    def union(first: int, second: int) -> None:
        first_root, second_root = find(first), find(second)
        if first_root != second_root:
            parents[second_root] = first_root

    for first in range(count):
        for second in range(first + 1, count):
            distance = float(np.linalg.norm(centers[first] - centers[second]))
            if distance <= factor * max(sizes[first], sizes[second]):
                union(first, second)
    groups: dict[int, list[int]] = {}
    for index in range(count):
        groups.setdefault(find(index), []).append(index)
    return list(groups.values())


def _split_duplicate_cluster(indices: list[int], ids: np.ndarray, corners: list[np.ndarray]) -> list[list[int]]:
    cluster_ids = ids[indices].reshape(-1)
    _, counts = np.unique(cluster_ids, return_counts=True)
    multiplicity = int(counts.max(initial=1))
    if multiplicity <= 1:
        return [indices]
    centers = np.asarray([np.asarray(corners[index]).reshape(4, 2).mean(axis=0) for index in indices], np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 0.1)
    _compactness, labels, _centers = cv2.kmeans(
        centers,
        multiplicity,
        None,
        criteria,
        10,
        cv2.KMEANS_PP_CENTERS,
    )
    return [[indices[position] for position in np.flatnonzero(labels.reshape(-1) == label)] for label in range(multiplicity)]


def cluster_markers(
    corners: list[np.ndarray], ids: np.ndarray, *, distance_factor: float = 3.8
) -> list[list[int]]:
    if ids is None or not len(corners):
        return []
    clusters = []
    for initial in _union_find_clusters(corners, distance_factor):
        clusters.extend(_split_duplicate_cluster(initial, ids, corners))
    clusters = [cluster for cluster in clusters if cluster]
    clusters.sort(
        key=lambda cluster: tuple(
            np.mean([np.asarray(corners[index]).reshape(4, 2).mean(axis=0) for index in cluster], axis=0)
        )
    )
    return clusters


def _grid_points(ids: np.ndarray, squares_x: int) -> np.ndarray:
    internal_width = squares_x - 1
    values = ids.reshape(-1).astype(int)
    return np.column_stack((values % internal_width + 1, values // internal_width + 1)).astype(np.float32)


def _project_homography(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    homogeneous = np.column_stack((points, np.ones(len(points))))
    projected = (homography @ homogeneous.T).T
    return projected[:, :2] / projected[:, 2:3]


def _square_scale(corners: np.ndarray, ids: np.ndarray, squares_x: int) -> float:
    points = corners.reshape(-1, 2)
    values = ids.reshape(-1).astype(int)
    index = {value: position for position, value in enumerate(values)}
    internal_width = squares_x - 1
    distances = []
    for value, position in index.items():
        for neighbor in (value + 1, value + internal_width):
            if neighbor not in index:
                continue
            if neighbor == value + 1 and value // internal_width != neighbor // internal_width:
                continue
            distances.append(float(np.linalg.norm(points[position] - points[index[neighbor]])))
    return float(np.median(distances)) if distances else 0.0


def _geometry_metrics(
    corners: np.ndarray, ids: np.ndarray, squares_x: int, squares_y: int, image_shape: tuple[int, ...]
) -> tuple[float | None, float, float | None]:
    if len(corners) < 4:
        return None, 0.0, None
    grid = _grid_points(ids, squares_x)
    pixels = corners.reshape(-1, 2).astype(np.float32)
    homography, _mask = cv2.findHomography(grid, pixels, method=cv2.RANSAC, ransacReprojThreshold=3.0)
    if homography is None:
        return None, 0.0, None
    predicted = _project_homography(homography, grid)
    rms = float(np.sqrt(np.mean(np.sum((predicted - pixels) ** 2, axis=1))))
    outside = np.asarray([[0, 0], [squares_x, 0], [squares_x, squares_y], [0, squares_y]], np.float32)
    polygon = _project_homography(homography, outside).astype(np.float32)
    area = abs(float(cv2.contourArea(polygon)))
    coverage = area / float(image_shape[0] * image_shape[1])
    center = np.asarray([[squares_x / 2, squares_y / 2], [squares_x / 2 + 0.5, squares_y / 2], [squares_x / 2, squares_y / 2 + 0.5]], np.float32)
    center_pixels = _project_homography(homography, center)
    lengths = [np.linalg.norm(center_pixels[1] - center_pixels[0]), np.linalg.norm(center_pixels[2] - center_pixels[0])]
    foreshortening = float(min(lengths) / max(lengths)) if max(lengths) > 0 else None
    return rms, coverage, foreshortening


def _assign_identity(detections: list[Detection], specs: list[BoardSpec]) -> None:
    if len(specs) != 2 or len(detections) < 2:
        for detection in detections:
            detection.candidate_board_names = [spec.name for spec in specs]
            detection.identity_status = "ambiguous_without_intrinsics_or_same_frame_scale_comparison"
        return
    ordered = sorted(detections, key=lambda item: item.square_pixel_scale, reverse=True)
    large, small = max(specs, key=lambda item: item.square_length_m), min(specs, key=lambda item: item.square_length_m)
    expected_ratio = large.square_length_m / small.square_length_m
    positive_smaller_scales = [item.square_pixel_scale for item in ordered[1:] if item.square_pixel_scale > 0]
    if not positive_smaller_scales or ordered[0].square_pixel_scale <= 0:
        for detection in detections:
            detection.candidate_board_names = [spec.name for spec in specs]
            detection.identity_status = "ambiguous_insufficient_corner_scale"
        return
    smaller_median = float(np.median(positive_smaller_scales))
    observed_ratio = ordered[0].square_pixel_scale / smaller_median if smaller_median > 0 else 0
    if 0.78 * expected_ratio <= observed_ratio <= 1.25 * expected_ratio:
        ordered[0].candidate_board_names = [large.name]
        ordered[0].identity_status = "relative_scale_candidate_same_frame"
        for detection in ordered[1:]:
            detection.candidate_board_names = [small.name]
            detection.identity_status = "relative_scale_candidate_same_frame"
    else:
        for detection in detections:
            detection.candidate_board_names = [spec.name for spec in specs]
            detection.identity_status = "ambiguous_relative_scale_inconsistent"


def detect_charuco_instances(
    image: np.ndarray,
    specs: list[BoardSpec],
    *,
    minimum_markers: int = 6,
    minimum_charuco_corners: int = 8,
    cluster_distance_factor: float = 3.8,
) -> tuple[list[Detection], list[np.ndarray], np.ndarray | None, list[np.ndarray]]:
    if image is None or image.ndim not in (2, 3):
        raise ValueError("Image must be a grayscale or BGR array.")
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = dictionary_from_name(specs[0].dictionary)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    marker_corners, marker_ids, rejected = detector.detectMarkers(gray)
    if marker_ids is None:
        return [], marker_corners, None, rejected
    marker_ids = np.asarray(marker_ids, dtype=np.int32).reshape(-1, 1)
    clusters = cluster_markers(marker_corners, marker_ids, distance_factor=cluster_distance_factor)
    canonical = specs[0]
    board = make_board(canonical)
    charuco_detector = cv2.aruco.CharucoDetector(board)
    detections: list[Detection] = []
    for cluster_index, indices in enumerate(clusters):
        cluster_corners = [marker_corners[index] for index in indices]
        cluster_ids = marker_ids[indices]
        if len(np.unique(cluster_ids)) != len(cluster_ids):
            continue
        charuco_corners, charuco_ids, _used_corners, _used_ids = charuco_detector.detectBoard(
            gray, None, None, cluster_corners, cluster_ids
        )
        if charuco_ids is None:
            charuco_ids = np.empty((0, 1), dtype=np.int32)
            charuco_corners = np.empty((0, 1, 2), dtype=np.float32)
        charuco_ids = np.asarray(charuco_ids, dtype=np.int32).reshape(-1, 1)
        charuco_corners = np.asarray(charuco_corners, dtype=np.float32).reshape(-1, 1, 2)
        square_scale = _square_scale(charuco_corners, charuco_ids, canonical.squares_x)
        rms, coverage, foreshortening = _geometry_metrics(
            charuco_corners, charuco_ids, canonical.squares_x, canonical.squares_y, image.shape
        )
        center = np.mean([np.asarray(corner).reshape(4, 2).mean(axis=0) for corner in cluster_corners], axis=0)
        reasons = []
        if len(cluster_ids) < minimum_markers:
            reasons.append("too_few_markers")
        if len(charuco_ids) < minimum_charuco_corners:
            reasons.append("too_few_charuco_corners")
        if rms is not None and rms > 3.0:
            reasons.append("high_homography_residual")
        detections.append(
            Detection(
                cluster_index=cluster_index,
                marker_ids=cluster_ids,
                marker_corners=cluster_corners,
                charuco_ids=charuco_ids,
                charuco_corners=charuco_corners,
                square_pixel_scale=square_scale,
                homography_rms_px=rms,
                coverage_ratio=coverage,
                foreshortening_ratio=foreshortening,
                center_xy=(float(center[0]), float(center[1])),
                candidate_board_names=[],
                identity_status="unassigned",
                accepted=not reasons,
                rejection_reason=";".join(reasons) if reasons else None,
            )
        )
    _assign_identity(detections, specs)
    return detections, marker_corners, marker_ids, rejected


def draw_detections(image: np.ndarray, detections: list[Detection]) -> np.ndarray:
    output = image.copy()
    palette = [(0, 220, 255), (255, 120, 0), (80, 230, 80), (230, 80, 230)]
    for index, detection in enumerate(detections):
        color = palette[index % len(palette)]
        cv2.aruco.drawDetectedMarkers(output, detection.marker_corners, detection.marker_ids, borderColor=color)
        if len(detection.charuco_corners):
            cv2.aruco.drawDetectedCornersCharuco(output, detection.charuco_corners, detection.charuco_ids, color)
        label = (
            f"C{detection.cluster_index} {','.join(detection.candidate_board_names) or 'unknown'} "
            f"M{len(detection.marker_ids)}/C{len(detection.charuco_ids)} "
            f"{'OK' if detection.accepted else 'REJECT'}"
        )
        position = (int(detection.center_xy[0]), int(detection.center_xy[1]))
        cv2.putText(output, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(output, label, position, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return output


def detection_to_dict(detection: Detection) -> dict[str, Any]:
    return {
        "cluster_index": detection.cluster_index,
        "board_name": detection.candidate_board_names[0] if len(detection.candidate_board_names) == 1 else None,
        "candidate_board_names": detection.candidate_board_names,
        "identity_status": detection.identity_status,
        "marker_ids": detection.marker_ids.reshape(-1).astype(int).tolist(),
        "detected_markers": len(detection.marker_ids),
        "charuco_corner_ids": detection.charuco_ids.reshape(-1).astype(int).tolist(),
        "charuco_corner_pixels": detection.charuco_corners.reshape(-1, 2).astype(float).tolist(),
        "detected_charuco_corners": len(detection.charuco_ids),
        "square_pixel_scale": detection.square_pixel_scale,
        "homography_rms_px": detection.homography_rms_px,
        "coverage_ratio": detection.coverage_ratio,
        "foreshortening_ratio": detection.foreshortening_ratio,
        "center_xy": list(detection.center_xy),
        "estimated_pose": None,
        "detection_confidence": {
            "marker_fraction": len(detection.marker_ids) / 17.0,
            "corner_fraction": len(detection.charuco_ids) / 24.0,
        },
        "accepted": detection.accepted,
        "rejection_reason": detection.rejection_reason,
    }


def estimate_detection_pose(
    detection: Detection,
    spec: BoardSpec,
    camera_matrix: np.ndarray,
    distortion_coefficients: np.ndarray | None = None,
) -> dict[str, Any] | None:
    if len(detection.charuco_ids) < 4:
        return None
    board = make_board(spec)
    all_object_points = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    object_points = all_object_points[detection.charuco_ids.reshape(-1)]
    image_points = detection.charuco_corners.reshape(-1, 2).astype(np.float32)
    matrix = np.asarray(camera_matrix, dtype=np.float64)
    distortion = np.zeros(5, dtype=np.float64) if distortion_coefficients is None else np.asarray(distortion_coefficients, dtype=np.float64).reshape(-1)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("Camera matrix must be a finite 3x3 array.")
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, distortion)
    residuals = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    rotation, _ = cv2.Rodrigues(rvec)
    return {
        "transform": f"T_C_{spec.name}",
        "board_name": spec.name,
        "rvec": rvec.reshape(-1).astype(float).tolist(),
        "translation_m": tvec.reshape(-1).astype(float).tolist(),
        "rotation_matrix": rotation.astype(float).tolist(),
        "reprojection_rms_px": float(np.sqrt(np.mean(residuals**2))),
        "reprojection_max_px": float(np.max(residuals)),
        "corner_count": len(image_points),
    }
