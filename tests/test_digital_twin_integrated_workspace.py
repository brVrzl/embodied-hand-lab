from __future__ import annotations

import os
import subprocess
from pathlib import Path

import mujoco
import numpy as np
import pytest

from digital_twin.io import load_structured


ROOT = Path(__file__).resolve().parents[1]


def _id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    return mujoco.mj_name2id(model, kind, name)


def _model(name: str = "workspace_scene.xml") -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(ROOT / "models/digital_twin" / name))


def test_integrated_scene_loads_robot_hand_and_P_workspace() -> None:
    model = _model(); data = mujoco.MjData(model); data.qpos[:] = model.qpos0; mujoco.mj_forward(model, data)
    base = _id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
    hand = _id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    assert base >= 0 and hand >= 0
    assert np.allclose(data.xpos[base], [0.0, 0.0, 0.0])
    assert np.allclose(model.body_quat[base], [0.0, 0.0, 0.0, 1.0])  # MuJoCo wxyz, yaw=180 deg
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_tabletop") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_rail_positive_y") >= 0
    assert np.allclose(model.opt.gravity, [0.0, 0.0, -9.81])


def test_root_transform_preserves_zero_qpos_and_reference_state() -> None:
    model = _model()
    assert np.allclose(model.qpos0, 0.0)
    jaka = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{index}") for index in range(1, 7)]
    assert all(index >= 0 for index in jaka)
    assert [model.qpos0[model.jnt_qposadr[index]] for index in jaka] == [0.0] * 6
    rh56 = [index for index in range(model.njnt) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index) or "").startswith("rh56_")]
    assert len(rh56) == 12
    assert all(model.qpos0[model.jnt_qposadr[index]] == 0.0 for index in rh56)


def test_palm_and_cable_directions_face_P_negative_x() -> None:
    model = _model(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    base = _id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
    hand = _id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    palm = data.xmat[hand].reshape(3, 3) @ np.array([0.0, 1.0, 0.0])
    cable = data.xmat[base].reshape(3, 3) @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(palm, [-1.0, 0.0, 0.0], atol=5e-6)
    assert np.allclose(cable, [-1.0, 0.0, 0.0], atol=1e-12)
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "rh56_palm_normal") >= 0
    assert _id(model, mujoco.mjtObj.mjOBJ_SITE, "jaka_cable_side_direction") >= 0


def test_mount_transform_is_unchanged() -> None:
    model = _model(); hand = _id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    assert np.allclose(model.body_pos[hand], [0.0, 0.0, 0.009])
    assert np.allclose(model.body_quat[hand], [0.0, 0.707106781, 0.707106781, 0.0], atol=1e-8)


def test_clean_default_has_no_sparse_geometry_and_primitives_collide() -> None:
    model = _model()
    assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, "colmap_sparse_debug") < 0
    for name in ("workspace_tabletop", "workspace_rail_positive_y", "workspace_rail_negative_y"):
        geom = _id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        assert geom >= 0 and model.geom_contype[geom] != 0
    config = load_structured(ROOT / "digital_twin/configs/static_environment.yaml")
    assert config["visual_layers"]["sparse_reconstruction_debug"] is False
    assert config["visual_layers"]["permanent_background"] is False
    assert config["visual_layers"]["cables"] is False


def test_optional_sparse_debug_geometry_has_no_collision() -> None:
    model = _model("workspace_scene_sparse_debug.xml")
    debug = _id(model, mujoco.mjtObj.mjOBJ_GEOM, "colmap_sparse_debug")
    assert debug >= 0
    assert model.geom_contype[debug] == 0
    assert model.geom_conaffinity[debug] == 0


def test_camera_placeholders_are_sites_not_collision_geoms() -> None:
    model = _model()
    for name in ("camera_external_placeholder", "camera_wrist_placeholder", "iphone_reconstruction_camera_placeholder"):
        assert _id(model, mujoco.mjtObj.mjOBJ_SITE, name) >= 0
        assert _id(model, mujoco.mjtObj.mjOBJ_GEOM, name) < 0


def test_segmentation_manifest_is_complete_and_debug_only() -> None:
    manifest = load_structured(ROOT / "artifacts/digital_twin/static_scene/segmentation_manifest.yaml")
    expected = {"table", "aluminium_frame", "permanent_background", "removable_clutter", "robot", "calibration_boards", "cables"}
    assert set(manifest["segment_counts"]) == expected
    assert sum(manifest["segment_counts"].values()) == manifest["source_point_count_after_crop"]
    assert manifest["robot_reconstruction_removed"] is True
    assert manifest["sparse_debug_default_enabled"] is False
    assert manifest["sparse_debug_collision"] is False


def test_scene_package_and_manifest_policy() -> None:
    directory = ROOT / "artifacts/digital_twin/static_scene"
    required = [
        "workspace_scene.ply", "workspace_scene.obj", "workspace_scene.glb",
        "workspace_clean_engineering.png", "workspace_clean_presentation.png",
        "zero_pose_top_verified.png", "orientation_before.png", "orientation_after.png",
        "sparse_debug_optional.png",
    ]
    assert all((directory / name).is_file() for name in required)
    manifest = load_structured(directory / "scene_manifest.yaml")
    assert manifest["world_frame"] == "P"
    assert manifest["robot_source_modified"] is False
    assert manifest["operational_robot_placement"]["yaw_deg"] == 180.0
    assert manifest["sparse_debug_included"] is False
    assert manifest["sparse_debug_collision"] is False


def test_generated_XML_regeneration_consistency(tmp_path: Path) -> None:
    output = tmp_path / "workspace.xml"; manifest = tmp_path / "manifest.yaml"
    command = [
        str(ROOT / ".venv/bin/python"), str(ROOT / "tools/digital_twin/build_mujoco_workspace_scene.py"),
        "--robot-model", str(ROOT / "data/sim_assets/jaka_rh56_visual_coacd.xml"),
        "--static-config", str(ROOT / "digital_twin/configs/static_environment.yaml"),
        "--camera-config", str(ROOT / "digital_twin/configs/camera_placeholders.yaml"),
        "--operational-config", str(ROOT / "digital_twin/configs/robot_operational_placement.yaml"),
        "--output", str(output), "--manifest", str(manifest),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    regenerated = mujoco.MjModel.from_xml_path(str(output))
    checked_in = _model()
    for name in ("jaka_Link_0", "rh56_R_hand_base_link"):
        left = _id(regenerated, mujoco.mjtObj.mjOBJ_BODY, name); right = _id(checked_in, mujoco.mjtObj.mjOBJ_BODY, name)
        assert np.allclose(regenerated.body_pos[left], checked_in.body_pos[right])
        assert np.allclose(regenerated.body_quat[left], checked_in.body_quat[right])
    assert _id(regenerated, mujoco.mjtObj.mjOBJ_GEOM, "colmap_sparse_debug") < 0


def test_clean_scene_rendering_smoke() -> None:
    if os.environ.get("MUJOCO_GL") not in {"egl", "osmesa"}:
        pytest.skip("Headless MuJoCo rendering backend is not configured.")
    model = _model(); data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=120, width=160)
    renderer.update_scene(data, camera="digital_twin_validation_camera")
    image = renderer.render(); renderer.close()
    assert image.shape == (120, 160, 3)
    assert np.isfinite(image).all()


def test_future_object_layer_is_empty_and_extendable() -> None:
    layer = load_structured(ROOT / "digital_twin/configs/object_layer.yaml")
    assert layer["frame"] == "P"
    assert layer["status"] == "ready_empty_layer"
    assert layer["objects"] == []
    assert layer["policy"]["rebuild_static_scene_required"] is False
