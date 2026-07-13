from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import trimesh

from sim_maniskill.rh56_collision import (
    REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS,
    VISUAL_COACD_SOURCE_STEMS,
    patch_rh56_visual_coacd_collision_model,
)
from mujoco_rh56_grasp_benchmark import _physical_norm_to_mujoco_ctrl
from view_mujoco_rh56_pose_contact import _set_hand_qpos_from_ctrl


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIM_ASSET_ROOT = PROJECT_ROOT / "data" / "sim_assets"
BASE_XML = SIM_ASSET_ROOT / "jaka_rh56.xml"
SOURCE_DIR = SIM_ASSET_ROOT / "meshes" / "rh56"
COACD_DIR = SIM_ASSET_ROOT / "meshes" / "rh56_collision_visual_coacd"

SOURCE_FILES = {
    "rh56_R_hand_base_link": "R_hand_base_link.STL",
    "rh56_R_thumb_proximal_base": "R_thumb_proximal_base.STL",
    "rh56_R_thumb_proximal": "R_thumb_proximal.STL",
    "rh56_R_thumb_intermediate": "R_thumb_intermediate.STL",
    "rh56_R_thumb_distal": "R_thumb_distal.STL",
    "rh56_R_index_proximal": "R_index_proximal.STL",
    "rh56_R_index_distal": "R_index_distal.STL",
    "rh56_R_middle_proximal": "R_middle_proximal.STL",
    "rh56_R_middle_distal": "R_middle_distal.STL",
    "rh56_R_ring_proximal": "R_ring_proximal.STL",
    "rh56_R_ring_distal": "R_ring_distal.STL",
    "rh56_R_pinky_proximal": "R_pinky_proximal.STL",
    "rh56_R_pinky_distal": "R_pinky_distal.STL",
}


def _set_meshdir(root: ET.Element, meshdir: Path) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", str(meshdir))


def _build_visual_coacd_xml(out_xml: Path) -> Path:
    tree = ET.parse(BASE_XML)
    root = tree.getroot()
    _set_meshdir(root, SIM_ASSET_ROOT)
    patch_rh56_visual_coacd_collision_model(root, asset_root=SIM_ASSET_ROOT)
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    return out_xml


def _build_visual_mesh_contact_xml(out_xml: Path) -> Path:
    tree = ET.parse(BASE_XML)
    root = tree.getroot()
    _set_meshdir(root, SIM_ASSET_ROOT)
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if not name.startswith("rh56_R_"):
            continue
        if geom.get("type") == "mesh" and name.endswith("_geom_0"):
            geom.set("contype", "2")
            geom.set("conaffinity", "3")
            geom.set("group", "3")
        else:
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    return out_xml


def _quat_to_matrix(quat: str | None) -> np.ndarray:
    if not quat:
        return np.eye(3)
    w, x, y, z = np.fromstring(quat, sep=" ", dtype=np.float64)
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _pos_vector(pos: str | None) -> np.ndarray:
    if not pos:
        return np.zeros(3, dtype=np.float64)
    return np.fromstring(pos, sep=" ", dtype=np.float64)


def _transform_vertices(vertices: np.ndarray, *, pos: str | None, quat: str | None) -> np.ndarray:
    rot = _quat_to_matrix(quat)
    trans = _pos_vector(pos)
    return vertices @ rot.T + trans


def _aabb(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return points.min(axis=0), points.max(axis=0)


def _load_vertices(path: Path) -> np.ndarray:
    mesh = trimesh.load(path, force="mesh")
    return np.asarray(mesh.vertices, dtype=np.float64)


def _body_visual_transform(root: ET.Element, body_name: str) -> dict[str, str | None]:
    body = next(body for body in root.iter("body") if body.get("name") == body_name)
    for geom in body.findall("geom"):
        if geom.get("type") == "mesh" and geom.get("name", "").endswith("_geom_0"):
            return {"pos": geom.get("pos"), "quat": geom.get("quat")}
    return {"pos": None, "quat": None}


def _body_collision_transforms(root: ET.Element, body_name: str) -> list[dict[str, str | None]]:
    body = next(body for body in root.iter("body") if body.get("name") == body_name)
    return [
        {"mesh": geom.get("mesh"), "pos": geom.get("pos"), "quat": geom.get("quat")}
        for geom in body.findall("geom")
        if "visual_coacd" in geom.get("name", "")
    ]


def audit_alignment(xml_path: Path) -> dict[str, Any]:
    base_root = ET.parse(BASE_XML).getroot()
    patched_root = ET.parse(xml_path).getroot()
    rows: dict[str, Any] = {}
    for body_name, source_filename in SOURCE_FILES.items():
        visual_transform = _body_visual_transform(base_root, body_name)
        visual_vertices = _transform_vertices(
            _load_vertices(SOURCE_DIR / source_filename),
            pos=visual_transform["pos"],
            quat=visual_transform["quat"],
        )
        collision_vertices: list[np.ndarray] = []
        collision_transforms = _body_collision_transforms(patched_root, body_name)
        for transform in collision_transforms:
            mesh_name = str(transform["mesh"])
            part_file = mesh_name.replace("rh56_visual_coacd_", "") + ".stl"
            collision_vertices.append(
                _transform_vertices(
                    _load_vertices(COACD_DIR / part_file),
                    pos=transform["pos"],
                    quat=transform["quat"],
                )
            )
        all_collision = np.concatenate(collision_vertices, axis=0)
        vmin, vmax = _aabb(visual_vertices)
        cmin, cmax = _aabb(all_collision)
        inside = np.all((visual_vertices >= cmin - 1e-6) & (visual_vertices <= cmax + 1e-6), axis=1)
        centroid_error = float(np.linalg.norm(visual_vertices.mean(axis=0) - all_collision.mean(axis=0)))
        rows[body_name] = {
            "visual_pos": visual_transform["pos"] or "0 0 0",
            "visual_quat": visual_transform["quat"] or "1 0 0 0",
            "collision_part_count": len(collision_transforms),
            "collision_transforms_match_visual": all(
                (transform["pos"] or None) == visual_transform["pos"]
                and (transform["quat"] or None) == visual_transform["quat"]
                for transform in collision_transforms
            ),
            "visual_centroid": visual_vertices.mean(axis=0).round(6).tolist(),
            "collision_centroid": all_collision.mean(axis=0).round(6).tolist(),
            "centroid_error_m": centroid_error,
            "aabb_min_error_m": np.abs(vmin - cmin).round(6).tolist(),
            "aabb_max_error_m": np.abs(vmax - cmax).round(6).tolist(),
            "aabb_surface_coverage_fraction": float(np.mean(inside)),
        }
    return rows


def _body_parent_map(model: mujoco.MjModel) -> dict[str, str]:
    result: dict[str, str] = {}
    for body_id in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
        parent = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.body_parentid[body_id])) or ""
        result[name] = parent
    return result


def _hand_qpos_joint_names(model: mujoco.MjModel) -> list[str]:
    return [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, idx) or ""
        for idx in range(model.njnt)
        if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, idx) or "").startswith("rh56_R_")
    ]


def _set_independent_joint_limit_pose(model: mujoco.MjModel, data: mujoco.MjData, values: np.ndarray) -> dict[str, float]:
    q_config: dict[str, float] = {}
    hand_joints = _hand_qpos_joint_names(model)
    for value, joint_name in zip(values, hand_joints, strict=True):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qpos_addr = int(model.jnt_qposadr[joint_id])
        lo, hi = model.jnt_range[joint_id]
        q = float(lo + value * (hi - lo))
        data.qpos[qpos_addr] = q
        q_config[joint_name] = q
    mujoco.mj_forward(model, data)
    return q_config


def _set_actuator_reachable_pose(model: mujoco.MjModel, data: mujoco.MjData, values: np.ndarray) -> dict[str, float]:
    ctrl = _physical_norm_to_mujoco_ctrl(values.astype(float).tolist())
    _set_hand_qpos_from_ctrl(data, ctrl)
    mujoco.mj_forward(model, data)
    return {
        "index": float(values[0]),
        "middle": float(values[1]),
        "ring": float(values[2]),
        "pinky": float(values[3]),
        "thumb_close": float(values[4]),
        "thumb_lateral": float(values[5]),
    }


def _visual_body_pairs_for_pose(model: mujoco.MjModel, data: mujoco.MjData) -> set[tuple[str, str]]:
    pairs: set[tuple[str, str]] = set()
    for idx in range(data.ncon):
        contact = data.contact[idx]
        body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[int(contact.geom1)])) or ""
        body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[int(contact.geom2)])) or ""
        if body1.startswith("rh56_R_") and body2.startswith("rh56_R_") and body1 != body2:
            pairs.add(tuple(sorted((body1, body2))))
    return pairs


def _contact_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    visual_pairs: set[tuple[str, str]],
    q_config: dict[str, float],
) -> list[dict[str, Any]]:
    parents = _body_parent_map(model)
    rows: list[dict[str, Any]] = []
    for idx in range(data.ncon):
        contact = data.contact[idx]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom1])) or ""
        body2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[geom2])) or ""
        if not (body1.startswith("rh56_R_") and body2.startswith("rh56_R_") and body1 != body2):
            continue
        body_pair = tuple(sorted((body1, body2)))
        parent_child = parents.get(body1) == body2 or parents.get(body2) == body1
        rows.append(
            {
                "body1": body1,
                "body2": body2,
                "geom1": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1) or "",
                "geom2": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2) or "",
                "penetration_depth_m": max(0.0, -float(contact.dist)),
                "contact_position_world_m": np.asarray(contact.pos, dtype=np.float64).round(6).tolist(),
                "joint_configuration": q_config,
                "parent_child": parent_child,
                "reviewed_internal_excluded_pair": body_pair in {
                    tuple(sorted(pair)) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS
                },
                "original_visual_meshes_also_intersect": body_pair in visual_pairs,
            }
        )
    return rows


def audit_self_contacts(
    coacd_xml: Path,
    visual_xml: Path,
    *,
    independent_samples: int,
    actuator_samples: int,
    seed: int,
    max_rows: int,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(coacd_xml))
    visual_model = mujoco.MjModel.from_xml_path(str(visual_xml))
    data = mujoco.MjData(model)
    visual_data = mujoco.MjData(visual_model)
    rng = np.random.default_rng(seed)
    rows: dict[str, list[dict[str, Any]]] = {"independent_joint_limit": [], "actuator_reachable": []}

    def run_sample(kind: str, values: np.ndarray) -> None:
        mujoco.mj_resetData(model, data)
        mujoco.mj_resetData(visual_model, visual_data)
        if kind == "independent_joint_limit":
            q_config = _set_independent_joint_limit_pose(model, data, values)
            _set_independent_joint_limit_pose(visual_model, visual_data, values)
        else:
            q_config = _set_actuator_reachable_pose(model, data, values)
            _set_actuator_reachable_pose(visual_model, visual_data, values)
        visual_pairs = _visual_body_pairs_for_pose(visual_model, visual_data)
        rows[kind].extend(_contact_rows(model, data, visual_pairs, q_config))

    independent_values = [np.zeros(12), np.ones(12), np.full(12, 0.5)]
    independent_values.extend(rng.random(12) for _ in range(independent_samples))
    actuator_values = [np.zeros(6), np.ones(6), np.full(6, 0.5)]
    actuator_values.extend(rng.random(6) for _ in range(actuator_samples))
    for values in independent_values:
        run_sample("independent_joint_limit", values)
    for values in actuator_values:
        run_sample("actuator_reachable", values)

    summary: dict[str, Any] = {}
    for kind, kind_rows in rows.items():
        unique_body_pairs = sorted({tuple(sorted((row["body1"], row["body2"]))) for row in kind_rows})
        unique_geom_pairs = sorted({tuple(sorted((row["geom1"], row["geom2"]))) for row in kind_rows})
        summary[kind] = {
            "sample_count": (independent_samples + 3) if kind == "independent_joint_limit" else (actuator_samples + 3),
            "contact_rows": len(kind_rows),
            "unique_body_pairs": [list(pair) for pair in unique_body_pairs],
            "unique_geom_pair_count": len(unique_geom_pairs),
            "max_penetration_depth_m": max((row["penetration_depth_m"] for row in kind_rows), default=0.0),
            "rows": kind_rows[:max_rows],
        }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 1 RH56 visual-CoACD collision audit.")
    parser.add_argument("--out-dir", type=Path, default=Path("/tmp/rh56_visual_coacd_stage1"))
    parser.add_argument("--independent-samples", type=int, default=64)
    parser.add_argument("--actuator-samples", type=int, default=128)
    parser.add_argument("--seed", type=int, default=17)
    parser.add_argument("--max-contact-rows", type=int, default=200)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    coacd_xml = _build_visual_coacd_xml(args.out_dir / "rh56_visual_coacd.xml")
    visual_xml = _build_visual_mesh_contact_xml(args.out_dir / "rh56_visual_mesh_contacts.xml")
    report = {
        "coacd_xml": str(coacd_xml),
        "visual_mesh_contact_xml": str(visual_xml),
        "reviewed_internal_excluded_body_pairs": [list(pair) for pair in REVIEWED_INTERNAL_EXCLUDED_BODY_PAIRS],
        "alignment": audit_alignment(coacd_xml),
        "self_contacts": audit_self_contacts(
            coacd_xml,
            visual_xml,
            independent_samples=args.independent_samples,
            actuator_samples=args.actuator_samples,
            seed=args.seed,
            max_rows=args.max_contact_rows,
        ),
    }
    out_report = args.out_dir / "stage1_audit.json"
    out_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({k: report[k] for k in ("coacd_xml", "visual_mesh_contact_xml")}, indent=2))
    for kind, summary in report["self_contacts"].items():
        print(
            kind,
            "samples=",
            summary["sample_count"],
            "contacts=",
            summary["contact_rows"],
            "unique_body_pairs=",
            summary["unique_body_pairs"],
            "max_penetration_m=",
            round(summary["max_penetration_depth_m"], 6),
        )
    print(f"wrote {out_report}")


if __name__ == "__main__":
    main()
