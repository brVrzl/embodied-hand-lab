from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import yaml

from digital_twin.calibration.base_geometry import fit_circle_ransac, fit_plane_ransac


ROOT = Path(__file__).resolve().parents[1]


def _load_register_tool():
    path = ROOT / "tools/digital_twin/register_colmap_models.py"
    spec = importlib.util.spec_from_file_location("register_colmap_models", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_plane_and_circle_robust_recovery() -> None:
    rng = np.random.default_rng(4)
    xy = rng.uniform(-1, 1, (300, 2))
    plane_points = np.column_stack((xy, 0.2 * xy[:, 0] - 0.1 * xy[:, 1] + 0.3))
    plane_points += rng.normal(0, 0.0005, plane_points.shape)
    points = np.vstack((plane_points, rng.uniform(-2, 2, (40, 3))))
    plane = fit_plane_ransac(points, threshold=0.003, iterations=2500, seed=2)
    assert plane.inliers.sum() >= 285

    angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
    circle_points = np.column_stack((0.4 + 0.62 * np.cos(angles), -0.7 + 0.62 * np.sin(angles)))
    circle_points += rng.normal(0, 0.001, circle_points.shape)
    circle_points = np.vstack((circle_points, rng.uniform(-2, 2, (50, 2))))
    circle = fit_circle_ransac(circle_points, threshold=0.004, radius_bounds=(0.55, 0.68), iterations=4000, seed=3)
    assert np.allclose(circle.center, [0.4, -0.7], atol=0.003)
    assert np.isclose(circle.radius, 0.62, atol=0.003)
    assert circle.inliers.sum() >= 170


def test_board_mask_maps_timestamp_and_expands_polygon(tmp_path: Path) -> None:
    tool = _load_register_tool()
    detection = {
        "frames": [{"timestamp_sec": 1.25, "detections": [{"charuco_corner_pixels": [[10, 10], [20, 10], [20, 20], [10, 20]]}]}]
    }
    manifest = {"frames": [{"source_timestamp_sec": 1.25, "frame_filename": "frame_000001.jpg", "accepted": True}]}
    detection_path, manifest_path = tmp_path / "detections.yaml", tmp_path / "frames.yaml"
    detection_path.write_text(yaml.safe_dump(detection), encoding="utf-8")
    manifest_path.write_text(yaml.safe_dump(manifest), encoding="utf-8")
    masks = tool.load_board_masks(detection_path, manifest_path, 1.2)
    assert list(masks) == ["frame_000001.jpg"]
    assert np.ptp(masks["frame_000001.jpg"][0], axis=0).min() > 10


def test_base_geometry_and_transform_provenance_configs() -> None:
    geometry = yaml.safe_load((ROOT / "digital_twin/configs/jaka_mini_base_geometry.yaml").read_text())
    transforms = yaml.safe_load((ROOT / "digital_twin/configs/transforms.yaml").read_text())
    assert geometry["fixed_base"]["outer_diameter_m"]["value"] == 0.124
    assert geometry["fixed_base"]["mounting_hole_pitch_circle_diameter_m"]["value"] == 0.110
    assert np.isclose(geometry["installed_orientation"]["hypotheses"][0]["candidate_rail_centerline_spacing_m"], 0.110 / np.sqrt(2))
    assert transforms["transforms"]["T_B_P"]["translation_m"] == [None, None, None]
    assert transforms["transforms"]["T_P_R"]["status"].startswith("provisional")
    assert transforms["transforms"]["T_B_R"]["scale"] is None
