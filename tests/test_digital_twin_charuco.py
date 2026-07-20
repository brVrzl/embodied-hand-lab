from __future__ import annotations

from dataclasses import replace

import cv2
import numpy as np

from digital_twin.calibration.charuco import (
    detect_charuco_instances,
    estimate_detection_pose,
    load_board_specs,
    make_board,
)


def board_image(spec, width=500, height=700):
    image = make_board(spec).generateImage((width, height), marginSize=20, borderBits=1)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_generated_5_by_7_board_has_all_markers_and_corners() -> None:
    specs = load_board_specs("digital_twin/configs/charuco_boards.yaml")
    detections, _corners, ids, _rejected = detect_charuco_instances(board_image(specs[0]), specs)
    assert ids is not None and len(ids) == 17
    assert len(detections) == 1
    assert len(detections[0].charuco_ids) == 24
    assert detections[0].accepted


def test_repeated_marker_ids_are_split_into_physical_instances() -> None:
    specs = load_board_specs("digital_twin/configs/charuco_boards.yaml")
    image = board_image(specs[0])
    canvas = np.full((700, 1100, 3), 255, dtype=np.uint8)
    canvas[:, :500] = image
    canvas[:, 600:] = image
    detections, _corners, ids, _rejected = detect_charuco_instances(canvas, specs)
    assert ids is not None and len(ids) == 34
    assert len(detections) == 2
    assert all(len(detection.charuco_ids) == 24 for detection in detections)


def test_reverse_7_by_5_orientation_does_not_interpolate_pattern() -> None:
    specs = load_board_specs("digital_twin/configs/charuco_boards.yaml")
    reversed_specs = [replace(spec, squares_x=7, squares_y=5) for spec in specs]
    detections, *_ = detect_charuco_instances(board_image(specs[0]), reversed_specs)
    assert detections
    assert max(len(detection.charuco_ids) for detection in detections) == 0


def test_pose_estimation_from_synthetic_projected_charuco_corners() -> None:
    specs = load_board_specs("digital_twin/configs/charuco_boards.yaml")
    spec = specs[0]
    board = make_board(spec)
    object_points = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    camera_matrix = np.asarray([[900, 0, 640], [0, 900, 360], [0, 0, 1]], dtype=float)
    rvec = np.asarray([0.15, -0.1, 0.05], dtype=float)
    tvec = np.asarray([0.02, -0.03, 1.2], dtype=float)
    pixels, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, np.zeros(5))
    detection = detect_charuco_instances(board_image(spec), specs)[0][0]
    detection.charuco_ids = np.arange(len(object_points), dtype=np.int32).reshape(-1, 1)
    detection.charuco_corners = pixels.astype(np.float32)
    pose = estimate_detection_pose(detection, spec, camera_matrix, np.zeros(5))
    assert pose is not None
    assert np.allclose(pose["translation_m"], tvec, atol=1e-5)
    assert pose["reprojection_rms_px"] < 1e-3
