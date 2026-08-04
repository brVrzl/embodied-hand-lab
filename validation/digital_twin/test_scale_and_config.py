from __future__ import annotations

from pathlib import Path

import pytest

from digital_twin.io import load_structured
from tools.digital_twin.build_collision_scene import validate_collision_config
from tools.digital_twin.estimate_scene_scale import estimate_scale
from tools.digital_twin.inspect_colmap_result import inspect_text_model


def test_scale_estimation_with_multiple_references() -> None:
    result = estimate_scale([
        {"name": "a", "reconstruction_distance": 2.0, "known_distance_m": 1.0, "uncertainty_m": 0.001},
        {"name": "b", "reconstruction_distance": 1.0, "known_distance_m": 0.5, "uncertainty_m": 0.002},
        {"name": "bad", "reconstruction_distance": 1.0, "known_distance_m": 3.0, "uncertainty_m": 0.001},
    ])
    assert result["estimated_scale_m_per_reconstruction_unit"] == pytest.approx(0.5)
    assert result["accepted_count"] == 2


def test_transform_yaml_preserves_uncalibrated_nulls() -> None:
    data = load_structured(Path("digital_twin/configs/transforms.yaml"))
    assert data["transforms"]["T_W_B"]["translation_m"] == [0.0, 0.0, 0.0]
    assert data["transforms"]["T_W_B"]["quaternion_xyzw"] == [0.0, 0.0, 1.0, 0.0]
    assert data["transforms"]["T_P_B_operational"]["yaw_deg"] == 180.0
    assert data["transforms"]["T_B_R"]["scale"] is None
    assert data["transforms"]["T_B_C_ext"]["quaternion_xyzw"] == [None] * 4
    assert data["frames"]["physical_jaka_base"] == "P"
    assert data["transforms"]["T_B_P"]["translation_m"] == [None] * 3
    assert data["transforms"]["T_P_R"]["scale"] > 0
    assert data["transforms"]["T_P_R"]["status"].startswith("provisional")
    assert data["transforms"]["T_B_R"]["translation_m"] == [None] * 3
    assert data["transforms"]["T_B_R"]["composition"] == "T_B_R = T_B_P * T_P_R"


def test_board_scale_groups_are_reported_independently() -> None:
    result = estimate_scale([
        {"name": "a3_1", "group": "A3", "reconstruction_distance": 0.5, "known_distance_m": 0.05, "uncertainty_m": 0.0002},
        {"name": "a3_2", "group": "A3", "reconstruction_distance": 0.502, "known_distance_m": 0.05, "uncertainty_m": 0.0002},
        {"name": "a4_1", "group": "A4", "reconstruction_distance": 0.35, "known_distance_m": 0.035, "uncertainty_m": 0.0002},
        {"name": "table", "group": "table", "reconstruction_distance": 13.0, "known_distance_m": 1.38, "uncertainty_m": 0.01},
    ])
    assert result["primary_source_policy"] == "A3_and_A4_only"
    assert result["group_estimates"]["A3"]["observation_count"] == 2
    assert result["group_estimates"]["A4"]["observation_count"] == 1
    assert result["A3_A4_agreement"]["status"] == "agree"


def test_colmap_text_inspection_statistics(tmp_path: Path) -> None:
    (tmp_path / "cameras.txt").write_text("# cameras\n1 PINHOLE 640 480 500 500 320 240\n")
    (tmp_path / "images.txt").write_text(
        "# images\n1 1 0 0 0 0 0 0 1 frame_000000.jpg\n10 20 1 30 40 2\n"
        "2 1 0 0 0 1 0 0 1 frame_000001.jpg\n11 21 1\n"
    )
    (tmp_path / "points3D.txt").write_text(
        "# points\n1 0 0 0 255 0 0 0.5 1 0 2 0\n2 1 0 0 0 255 0 1.5 1 1\n"
    )
    report = inspect_text_model(tmp_path, 2, ["frame_000000.jpg", "frame_000001.jpg"])
    assert report["registration_ratio"] == 1.0
    assert report["point3D_count"] == 2
    assert report["mean_track_length"] == pytest.approx(1.5)
    assert report["mean_reprojection_error_px"] == pytest.approx(1.0)
    assert report["trajectory_continuity_status"] == "all_extracted_images_registered"


def test_collision_scene_validation_accepts_parameterized_box() -> None:
    objects = validate_collision_config({
        "units": "meter", "frame": "B", "objects": [{
            "name": "tabletop", "shape_type": "box", "dimensions": [1.0, 0.6, 0.04],
            "pose_in_B": {"translation_m": [0, 0, -0.02], "quaternion_xyzw": [0, 0, 0, 1]},
            "source": "measurement", "status": "measured", "uncertainty_m": 0.002,
            "collision": True, "provisional": False,
        }],
    })
    assert objects[0]["shape_type"] == "box"


def test_collision_scene_validation_rejects_missing_uncertainty() -> None:
    with pytest.raises(ValueError, match="uncertainty_m"):
        validate_collision_config({"units": "meter", "frame": "B", "objects": [{"name": "bad"}]})


def test_integrated_scene_preserves_provisional_provenance_in_P() -> None:
    data = load_structured(Path("digital_twin/configs/static_scene_provisional.yaml"))
    table = data["objects"][0]
    assert data["engineering_frame"] == "P"
    assert table["dimensions_m"] == [0.73, 1.38, 0.02]
    assert table["pose"]["frame"] == "P"
    assert table["status"] == "operational_provisional"
    assert table["collision"] is True
    assert data["objects"][2]["collision"] is False


def test_collision_scene_validation_accepts_P_frame() -> None:
    data = load_structured(Path("digital_twin/configs/collision_scene.yaml"))
    objects = validate_collision_config(data)
    assert data["frame"] == "P"
    assert {obj["name"] for obj in objects} >= {"tabletop", "rail_positive_y", "rail_negative_y"}
    assert all("pose_in_P" in obj for obj in objects)
