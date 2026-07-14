from __future__ import annotations

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from build_rh56_visual_coacd_runtime_asset import build_manifest, build_runtime_xml
from sim_maniskill.rh56_collision import (
    DEFAULT_RH56_COLLISION_MODE,
    REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS,
    patch_rh56_collision_model,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PROJECT_ROOT / "data" / "sim_assets"
SOURCE_XML = ASSET_ROOT / "jaka_rh56.xml"
RUNTIME_XML = ASSET_ROOT / "jaka_rh56_visual_coacd.xml"
MANIFEST_PATH = ASSET_ROOT / "jaka_rh56_visual_coacd.manifest.json"


def _runtime_hand_geoms(root: ET.Element) -> list[ET.Element]:
    return [
        geom
        for body in root.iter("body")
        if body.get("name", "").startswith("rh56_R_")
        for geom in body.findall("geom")
    ]


def test_committed_runtime_asset_matches_reproducible_derivation() -> None:
    expected_xml = build_runtime_xml(SOURCE_XML)
    assert RUNTIME_XML.read_bytes() == expected_xml

    expected_manifest = build_manifest(SOURCE_XML, RUNTIME_XML, expected_xml)
    assert json.loads(MANIFEST_PATH.read_text(encoding="utf-8")) == expected_manifest


def test_runtime_asset_has_only_reviewed_rh56_geometry_and_exclusions() -> None:
    root = ET.parse(RUNTIME_XML).getroot()
    geoms = _runtime_hand_geoms(root)
    coacd = [geom for geom in geoms if "_visual_coacd_collision_" in geom.get("name", "")]
    visuals = [geom for geom in geoms if geom.get("name", "").endswith("_geom_0")]

    assert len(coacd) == 148
    assert all(int(geom.get("contype", "0")) > 0 for geom in coacd)
    assert all(int(geom.get("conaffinity", "0")) > 0 for geom in coacd)
    assert len(visuals) == 13
    assert all(geom.get("contype") == "0" and geom.get("conaffinity") == "0" for geom in visuals)
    assert len(geoms) == len(coacd) + len(visuals)
    assert not any("correll" in geom.get("name", "") or "proxy" in geom.get("name", "") for geom in geoms)

    excluded = {
        tuple(sorted((exclude.get("body1"), exclude.get("body2"))))
        for exclude in root.findall("./contact/exclude")
    }
    assert excluded == {tuple(sorted(pair)) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS}


def test_runtime_asset_preserves_validated_body_mass_and_inertia() -> None:
    source = mujoco.MjModel.from_xml_path(str(SOURCE_XML))
    runtime = mujoco.MjModel.from_xml_path(str(RUNTIME_XML))

    for body_id in range(runtime.nbody):
        body_name = mujoco.mj_id2name(runtime, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        if not body_name.startswith("rh56_R_"):
            continue
        source_id = mujoco.mj_name2id(source, mujoco.mjtObj.mjOBJ_BODY, body_name)
        np.testing.assert_array_equal(runtime.body_mass[body_id], source.body_mass[source_id])
        np.testing.assert_array_equal(runtime.body_inertia[body_id], source.body_inertia[source_id])
        np.testing.assert_array_equal(runtime.body_ipos[body_id], source.body_ipos[source_id])
        np.testing.assert_array_equal(runtime.body_iquat[body_id], source.body_iquat[source_id])


def test_manifest_checksums_cover_current_visual_and_collision_assets() -> None:
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    assert manifest["coacd_hull_count"] == 148
    assert manifest["vendor_visual_geom_count"] == 13
    for relative_path, expected in manifest["sha256"].items():
        content = (PROJECT_ROOT / relative_path).read_bytes()
        assert hashlib.sha256(content).hexdigest() == expected


def test_default_patch_and_runtime_configs_select_visual_coacd() -> None:
    assert DEFAULT_RH56_COLLISION_MODE == "visual_coacd"
    root = ET.parse(SOURCE_XML).getroot()
    patch_rh56_collision_model(root)
    names = [geom.get("name", "") for geom in _runtime_hand_geoms(root)]
    assert sum("_visual_coacd_collision_" in name for name in names) == 148
    assert not any("correll" in name or "proxy" in name for name in names)

    for relative_path in (
        "configs/teleop/hebi_mobile_io_jaka_rh56_mount_v2.yaml",
        "configs/teleop/hebi_mobile_io_jaka_rh56.yaml",
        "configs/teleop/xbox_jaka_rh56.yaml",
    ):
        config = yaml.safe_load((PROJECT_ROOT / relative_path).read_text(encoding="utf-8"))
        assert config["shadow"]["mjcf_path"] == "data/sim_assets/jaka_rh56_visual_coacd.xml"

    lift_config = yaml.safe_load(
        (PROJECT_ROOT / "configs/sim/mujoco_jaka_rh56_tennis_ball_lift.yaml").read_text(encoding="utf-8")
    )
    assert lift_config["collision_mode"] == "visual_coacd"
