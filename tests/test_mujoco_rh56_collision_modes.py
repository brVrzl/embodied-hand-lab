from __future__ import annotations

import xml.etree.ElementTree as ET
import sys
from pathlib import Path

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from mujoco_rh56_grasp_benchmark import _configure_collision_model
from view_mujoco_rh56_pose_contact import _build_pose_xml
from sim_maniskill.rh56_collision import REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS


def _minimal_hand_xml() -> ET.Element:
    return ET.fromstring(
        """
        <mujoco model="hand">
          <worldbody>
            <body name="rh56_R_hand_base_link">
              <geom name="rh56_R_hand_base_link_geom_0" type="mesh" mesh="rh56_R_hand_base_link"/>
              <body name="rh56_R_thumb_distal"/>
              <body name="rh56_R_index_distal"/>
              <body name="rh56_R_middle_distal"/>
              <body name="rh56_R_ring_distal"/>
              <body name="rh56_R_pinky_distal"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )


def test_unifuc_pad_proxy_uses_existing_distal_rectangular_pads() -> None:
    root = _minimal_hand_xml()

    _configure_collision_model(root, collision_mode="unifuc_pad_proxy", include_calibration_markers=True)

    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    assert geoms["index_pad_proxy"].get("type") == "box"
    assert geoms["index_pad_proxy"].get("pos") == "0.0083 0.0250 0.0015"
    assert geoms["middle_pad_proxy"].get("pos") == "0.0064 0.0260 0.0015"
    assert "index_proximal_pad_proxy" not in geoms
    assert "middle_tip_pad_proxy" not in geoms
    assert geoms["index_unifuc_center"].get("contype") == "0"


def test_derived_pose_xml_sets_absolute_meshdir(tmp_path: Path) -> None:
    out_xml = tmp_path / "nested" / "pose.xml"

    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="unifuc_pad_proxy",
    )

    root = ET.parse(out_xml).getroot()
    compiler = root.find("compiler")
    assert compiler is not None
    assert compiler.get("meshdir") == str(Path("data/sim_assets").resolve())


def test_correll_mesh_collision_mode_compiles_and_disables_old_proxy(tmp_path: Path) -> None:
    out_xml = tmp_path / "nested" / "correll_pose.xml"

    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="correll_mesh",
    )

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    thumb_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rh56_R_thumb_distal_correll_collision")
    index_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rh56_R_index_distal_correll_collision")
    old_proxy_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rh56_R_thumb_distal_collision")

    assert thumb_id >= 0
    assert index_id >= 0
    assert old_proxy_id >= 0
    assert model.geom_contype[thumb_id] > 0
    assert model.geom_contype[index_id] > 0
    assert model.geom_contype[old_proxy_id] == 0


def test_visual_coacd_collision_mode_compiles_and_removes_old_proxy(tmp_path: Path) -> None:
    out_xml = tmp_path / "nested" / "visual_coacd_pose.xml"

    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="visual_coacd",
    )

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    palm_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "rh56_R_hand_base_link_visual_coacd_collision_000",
    )
    thumb_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_GEOM,
        "rh56_R_thumb_distal_visual_coacd_collision_000",
    )
    old_proxy_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rh56_R_thumb_distal_collision")

    assert palm_id >= 0
    assert thumb_id >= 0
    assert old_proxy_id == -1
    assert model.geom_contype[palm_id] > 0
    assert model.geom_contype[thumb_id] > 0


def test_visual_coacd_inherits_base_visual_geom_transform_and_removes_correll(tmp_path: Path) -> None:
    out_xml = tmp_path / "nested" / "visual_coacd_pose.xml"

    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="visual_coacd",
    )

    root = ET.parse(out_xml).getroot()
    geoms = {geom.get("name"): geom for geom in root.iter("geom")}
    visual = geoms["rh56_R_hand_base_link_geom_0"]
    collision = geoms["rh56_R_hand_base_link_visual_coacd_collision_000"]

    assert collision.get("pos") == visual.get("pos")
    assert collision.get("quat") == visual.get("quat")
    assert "rh56_R_thumb_distal_correll_collision" not in geoms


def test_visual_coacd_runtime_geometry_inventory_and_inertials(tmp_path: Path) -> None:
    out_xml = tmp_path / "visual_coacd_runtime.xml"
    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="visual_coacd",
    )

    root = ET.parse(out_xml).getroot()
    hand_bodies = {
        body.get("name"): body
        for body in root.iter("body")
        if body.get("name", "").startswith("rh56_R_")
    }
    hand_geoms = [geom for body in hand_bodies.values() for geom in body.findall("geom")]
    coacd = [geom for geom in hand_geoms if "_visual_coacd_collision_" in geom.get("name", "")]
    visuals = [
        geom
        for body_name, body in hand_bodies.items()
        for geom in body.findall("geom")
        if geom.get("name") == f"{body_name}_geom_0" and geom.get("mesh") == body_name
    ]
    non_runtime = [geom for geom in hand_geoms if geom not in coacd and geom not in visuals]

    assert len(coacd) == 148
    assert all(
        int(geom.get("contype", "0")) > 0 and int(geom.get("conaffinity", "0")) > 0
        for geom in coacd
    )
    assert len(visuals) == 13
    assert all(geom.get("contype") == "0" and geom.get("conaffinity") == "0" for geom in visuals)
    assert non_runtime == []
    assert not any("correll" in geom.get("name", "") for geom in hand_geoms)
    assert not any("proxy" in geom.get("name", "") for geom in hand_geoms)

    correll_asset_names = {
        mesh.get("name", "")
        for mesh in root.findall("./asset/mesh")
        if mesh.get("name", "").startswith("rh56_correll_")
    }
    assert correll_asset_names == set()

    model = mujoco.MjModel.from_xml_path(str(out_xml))
    active_non_coacd: list[str] = []
    for geom_id in range(model.ngeom):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom_id])) or ""
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        has_collision_bits = model.geom_contype[geom_id] > 0 or model.geom_conaffinity[geom_id] > 0
        if body_name.startswith("rh56_R_") and has_collision_bits and "_visual_coacd_collision_" not in geom_name:
            active_non_coacd.append(geom_name)
    assert active_non_coacd == []

    base_model = mujoco.MjModel.from_xml_path(str(Path("data/sim_assets/jaka_rh56.xml")))
    for body_name in hand_bodies:
        base_id = mujoco.mj_name2id(base_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        derived_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        assert base_id >= 0 and derived_id >= 0
        np.testing.assert_array_equal(model.body_mass[derived_id], base_model.body_mass[base_id])
        np.testing.assert_array_equal(model.body_inertia[derived_id], base_model.body_inertia[base_id])
        np.testing.assert_array_equal(model.body_ipos[derived_id], base_model.body_ipos[base_id])
        np.testing.assert_array_equal(model.body_iquat[derived_id], base_model.body_iquat[base_id])


def test_visual_coacd_review_references_are_explicit_and_opt_in(tmp_path: Path) -> None:
    out_xml = tmp_path / "visual_coacd_review.xml"
    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="visual_coacd",
        disabled_reference_geometry="all",
    )

    root = ET.parse(out_xml).getroot()
    references = [
        geom for geom in root.iter("geom") if geom.get("name", "").startswith("review_disabled_")
    ]
    assert len(references) == 25
    assert all(geom.get("contype") == "0" for geom in references)
    assert all(geom.get("conaffinity") == "0" for geom in references)
    assert all(geom.get("group") == "4" for geom in references)


def test_visual_coacd_uses_reviewed_internal_exclusions_without_thumb_finger_exclusion(tmp_path: Path) -> None:
    out_xml = tmp_path / "nested" / "visual_coacd_pose.xml"

    _build_pose_xml(
        Path("data/sim_assets/jaka_rh56.xml"),
        out_xml,
        collision_mode="visual_coacd",
    )

    root = ET.parse(out_xml).getroot()
    excluded = {
        tuple(sorted((exclude.get("body1"), exclude.get("body2"))))
        for exclude in root.findall("./contact/exclude")
    }

    expected = {tuple(sorted(pair)) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS}

    assert excluded == expected
    assert tuple(sorted(("rh56_R_index_proximal", "rh56_R_index_distal"))) in excluded
    assert tuple(sorted(("rh56_R_thumb_intermediate", "rh56_R_thumb_distal"))) in excluded
    assert tuple(sorted(("rh56_R_thumb_distal", "rh56_R_index_distal"))) not in excluded
    assert tuple(sorted(("rh56_R_hand_base_link", "rh56_R_index_proximal"))) not in excluded
