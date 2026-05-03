from __future__ import annotations

import argparse
import importlib
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


BASE_XML = Path("data/sim_assets/jaka_rh56.xml")
DEFAULT_MANIFEST = Path("data/external/maniskill_ycb_mujoco_assets.json")
OUT_DIR = Path("data/mujoco_ycb_preview")
TABLE_TOP_Z = 0.80
OBJECT_CENTER_XY = (-0.035, -0.570)


def _load_manifest(path: Path) -> dict[str, dict[str, Any]]:
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}. Run tools/prepare_maniskill_ycb_mujoco_assets.py first.")
    data = json.loads(path.read_text(encoding="utf-8"))
    return {item["id"]: item for item in data["objects"]}


def _ensure_asset(root: ET.Element) -> ET.Element:
    asset = root.find("asset")
    if asset is not None:
        return asset
    asset = ET.Element("asset")
    root.insert(0, asset)
    return asset


def _add_ycb_scene(
    root: ET.Element,
    record: dict[str, Any],
    *,
    object_xy: tuple[float, float],
    table_top_z: float,
) -> None:
    asset = _ensure_asset(root)
    mesh_name = f"ycb_{record['id']}_collision"
    ET.SubElement(asset, "mesh", {"name": mesh_name, "file": str(Path(record["collision_obj"]).resolve())})

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody")

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "ycb_preview_table",
            "type": "box",
            "pos": f"{object_xy[0]:.6f} {object_xy[1]:.6f} {table_top_z - 0.020:.6f}",
            "size": "0.42 0.32 0.020",
            "rgba": "0.72 0.66 0.56 1",
            "friction": "1.4 0.05 0.003",
            "condim": "4",
            "contype": "1",
            "conaffinity": "7",
        },
    )
    body_z = table_top_z + float(record["table_body_z_offset_m"])
    object_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "ycb_preview_object_body",
            "pos": f"{object_xy[0]:.6f} {object_xy[1]:.6f} {body_z:.6f}",
        },
    )
    ET.SubElement(object_body, "freejoint", {"name": "ycb_preview_object_freejoint"})
    ET.SubElement(
        object_body,
        "geom",
        {
            "name": "ycb_preview_object",
            "type": "mesh",
            "mesh": mesh_name,
            "density": f"{float(record['density']):.6f}",
            "rgba": "0.95 0.46 0.16 1",
            "friction": "1.8 0.08 0.004",
            "condim": "4",
            "priority": "2",
            "contype": "1",
            "conaffinity": "6",
            "solref": "0.004 1",
            "solimp": "0.92 0.98 0.002",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "ycb_preview_camera",
            "mode": "fixed",
            "pos": "-0.30 -0.76 1.10",
            "xyaxes": "0.96 -0.29 0 0.18 0.60 0.78",
            "fovy": "38",
        },
    )
    root.set("model", f"jaka_rh56_ycb_preview_{record['id']}")


def build_scene_xml(
    *,
    base_xml: Path,
    manifest: Path,
    object_id: str,
    out_xml: Path,
    object_xy: tuple[float, float],
    table_top_z: float,
) -> dict[str, Any]:
    records = _load_manifest(manifest)
    if object_id not in records:
        raise ValueError(f"Unknown object {object_id}; choices={sorted(records)}")
    record = records[object_id]
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    _add_ycb_scene(root, record, object_xy=object_xy, table_top_z=table_top_z)
    ET.indent(root, space="  ")
    out_xml.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    return {
        "object_id": object_id,
        "xml": str(out_xml.resolve()),
        "category": record["category"],
        "bbox_size_m": record["bbox_size_m"],
        "object_body_pos": [object_xy[0], object_xy[1], table_top_z + float(record["table_body_z_offset_m"])],
    }


def _contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int]:
    counts = {"object_table": 0, "object_robot": 0, "robot_table": 0, "total": int(data.ncon)}
    for idx in range(data.ncon):
        contact = data.contact[idx]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "",
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "",
        ]
        joined = " ".join(names)
        if "ycb_preview_object" in joined and "ycb_preview_table" in joined:
            counts["object_table"] += 1
        if "ycb_preview_object" in joined and ("rh56" in joined or "jaka" in joined):
            counts["object_robot"] += 1
        if "ycb_preview_table" in joined and ("rh56" in joined or "jaka" in joined):
            counts["robot_table"] += 1
    return counts


def run_scene(xml_path: Path, *, duration: float, viewer: bool) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    start = time.time()
    last_print = -1.0
    if viewer:
        mujoco_viewer = importlib.import_module("mujoco.viewer")
        with mujoco_viewer.launch_passive(model, data) as handle:
            while handle.is_running() and time.time() - start < duration:
                mujoco.mj_step(model, data)
                handle.sync()
                time.sleep(model.opt.timestep)
    else:
        while data.time < duration:
            mujoco.mj_step(model, data)
            if data.time - last_print >= 0.25:
                last_print = data.time
                body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ycb_preview_object_body")
                pos = data.xpos[body_id].copy()
                print(
                    f"t={data.time:.3f} object=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
                    f"contacts={_contact_summary(model, data)}",
                    flush=True,
                )
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "ycb_preview_object_body")
    pos = data.xpos[body_id].copy()
    return {
        "final_object_pos": [round(float(value), 6) for value in pos],
        "contacts": _contact_summary(model, data),
        "sim_time": round(float(data.time), 6),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Preview a converted ManiSkill YCB object in the JAKA RH56 MuJoCo scene.")
    parser.add_argument("--object", default="002_master_chef_can")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--manifest", default=str(DEFAULT_MANIFEST))
    parser.add_argument("--out-xml", default=None)
    parser.add_argument("--object-xy", nargs=2, type=float, default=list(OBJECT_CENTER_XY))
    parser.add_argument("--table-top-z", type=float, default=TABLE_TOP_Z)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--viewer", action="store_true")
    args = parser.parse_args()

    out_xml = Path(args.out_xml) if args.out_xml else OUT_DIR / f"{args.object}.xml"
    summary = build_scene_xml(
        base_xml=Path(args.base_xml),
        manifest=Path(args.manifest),
        object_id=args.object,
        out_xml=out_xml,
        object_xy=(float(args.object_xy[0]), float(args.object_xy[1])),
        table_top_z=float(args.table_top_z),
    )
    result = run_scene(out_xml, duration=float(args.duration), viewer=bool(args.viewer))
    print(json.dumps({**summary, **result}, indent=2))


if __name__ == "__main__":
    main()
