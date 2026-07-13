from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

from sim_maniskill.rh56_collision import patch_rh56_correll_collision_model, patch_rh56_visual_coacd_collision_model

BASE_XML = Path("data/sim_assets/jaka_rh56.xml")
OUT_DIR = Path("data/mujoco_grasp_benchmark")
COLLISION_MODES = ("correll_mesh", "visual_coacd", "proxy", "mesh", "mesh_proxy", "unifuc_pad_proxy")

ARM_ACTUATOR_NAMES = [f"jaka_joint_{idx}_act" for idx in range(1, 7)]
HAND_ACTUATOR_NAMES = [
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
]
TIP_BODY_NAMES = [
    "rh56_R_thumb_distal",
    "rh56_R_index_distal",
    "rh56_R_middle_distal",
    "rh56_R_ring_distal",
    "rh56_R_pinky_distal",
]


@dataclass(frozen=True)
class ObjectSpec:
    name: str
    geom_type: str
    size: tuple[float, ...]
    mass: float
    rgba: str
    friction: str = "1.8 0.08 0.004"

    @property
    def half_height(self) -> float:
        if self.geom_type == "sphere":
            return self.size[0]
        if self.geom_type == "cylinder":
            return self.size[1]
        return self.size[2]

    @property
    def planar_width(self) -> float:
        if self.geom_type == "sphere":
            return 2.0 * self.size[0]
        if self.geom_type == "cylinder":
            return 2.0 * self.size[0]
        return 2.0 * min(self.size[0], self.size[1])


OBJECTS: dict[str, ObjectSpec] = {
    "foam_cube": ObjectSpec("foam_cube", "box", (0.018, 0.018, 0.018), 0.018, "0.85 0.30 0.18 1"),
    "paper_box": ObjectSpec("paper_box", "box", (0.026, 0.018, 0.016), 0.025, "0.75 0.48 0.30 1"),
    "light_cylinder": ObjectSpec("light_cylinder", "cylinder", (0.017, 0.035), 0.030, "0.20 0.52 0.85 1"),
    "round_ball": ObjectSpec("round_ball", "sphere", (0.019,), 0.024, "0.25 0.68 0.36 1"),
}


def _load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _ids(model: mujoco.MjModel, obj_type: mujoco.mjtObj, names: list[str]) -> np.ndarray:
    found: list[int] = []
    for name in names:
        idx = mujoco.mj_name2id(model, obj_type, name)
        if idx < 0:
            raise KeyError(f"Missing {obj_type} named {name}")
        found.append(idx)
    return np.asarray(found, dtype=np.int32)


def _body_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise KeyError(f"Missing body {name}")
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy()


def _geom_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise KeyError(f"Missing geom {name}")
    return np.asarray(data.geom_xpos[geom_id], dtype=np.float64).copy()


SEMANTIC_CONTACT_BODIES = {
    "thumb": (
        "rh56_R_thumb_distal",
        "rh56_R_thumb_intermediate",
        "rh56_R_thumb_proximal",
    ),
    "index": (
        "rh56_R_index_distal",
        "rh56_R_index_proximal",
    ),
    "middle": (
        "rh56_R_middle_distal",
        "rh56_R_middle_proximal",
    ),
    "ring_pinky": (
        "rh56_R_ring_distal",
        "rh56_R_ring_proximal",
        "rh56_R_pinky_distal",
        "rh56_R_pinky_proximal",
    ),
}


def _geom_name(model: mujoco.MjModel, geom_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id)) or ""


def _geom_body_name(model: mujoco.MjModel, geom_id: int) -> str:
    body_id = int(model.geom_bodyid[int(geom_id)])
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""


def _semantic_contact_group(model: mujoco.MjModel, geom_id: int) -> str:
    name = _geom_name(model, geom_id)
    body_name = _geom_body_name(model, geom_id)
    if name in {"bench_object", "bench_table", "floor"}:
        return name
    if "pad_proxy" in name:
        if "thumb" in name:
            return "thumb"
        if "index" in name:
            return "index"
        if "middle" in name:
            return "middle"
        if "ring" in name or "pinky" in name:
            return "ring_pinky"
    for group, bodies in SEMANTIC_CONTACT_BODIES.items():
        if body_name in bodies:
            return group
    if body_name.startswith("rh56_R_"):
        return "hand_other"
    if body_name.startswith("jaka_"):
        return "arm"
    return ""


def _active_semantic_geom_positions(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    group: str,
) -> np.ndarray:
    bodies = set(SEMANTIC_CONTACT_BODIES[group])
    positions: list[np.ndarray] = []
    for geom_id in range(model.ngeom):
        if not (model.geom_contype[geom_id] or model.geom_conaffinity[geom_id]):
            continue
        if _geom_body_name(model, geom_id) not in bodies:
            continue
        positions.append(np.asarray(data.geom_xpos[geom_id], dtype=np.float64).copy())
    if positions:
        return np.asarray(positions, dtype=np.float64)
    body_positions = [_body_pos(model, data, body_name) for body_name in bodies if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name) >= 0]
    if not body_positions:
        raise KeyError(f"Missing semantic contact group {group!r}")
    return np.asarray(body_positions, dtype=np.float64)


def _semantic_contact_center(model: mujoco.MjModel, data: mujoco.MjData, group: str) -> np.ndarray:
    positions = _active_semantic_geom_positions(model, data, group)
    return np.mean(positions, axis=0)


def _physical_norm_to_mujoco_ctrl(values: list[float]) -> np.ndarray:
    if len(values) != 6:
        raise ValueError("Expected 6 physical RH56 DOF values.")
    pinky, ring, middle, index, thumb_bend, thumb_rotate = [float(np.clip(v, 0.0, 1.0)) for v in values]
    return np.asarray(
        [
            1.10 * thumb_rotate,
            0.50 * thumb_bend,
            1.70 * index,
            1.68 * middle,
            1.70 * ring,
            1.70 * pinky,
        ],
        dtype=np.float64,
    )


def _raw_to_physical_norm(raw: list[int]) -> list[float]:
    if len(raw) != 6:
        raise ValueError("Expected 6 raw RH56 values.")
    return [float(np.clip((1000.0 - value) / 1000.0, 0.0, 1.0)) for value in raw]


def _set_kinematic_pose(model: mujoco.MjModel, data: mujoco.MjData, arm_q: np.ndarray, hand_ctrl: np.ndarray) -> None:
    data.qpos[:6] = arm_q
    thumb_rotate, thumb_bend, index, middle, ring, pinky = hand_ctrl
    data.qpos[6:18] = [
        thumb_rotate,
        thumb_bend,
        0.6 * thumb_bend,
        0.8 * thumb_bend,
        index,
        index,
        middle,
        middle,
        ring,
        ring,
        pinky,
        pinky,
    ]
    data.ctrl[:6] = arm_q
    data.ctrl[6:12] = hand_ctrl
    mujoco.mj_forward(model, data)


def _solve_hand_base_lift_q(
    base_xml: Path,
    *,
    grasp_q: np.ndarray,
    lift_dz: float,
    iterations: int = 120,
    damping: float = 0.04,
    max_step: float = 0.030,
) -> np.ndarray:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    arm_dof_ids = np.asarray(
        [
            model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")]
            for idx in range(1, 7)
        ],
        dtype=np.int32,
    )
    joint_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")
        for idx in range(1, 7)
    ]
    q = grasp_q.astype(np.float64).copy()
    data.qpos[:6] = q
    mujoco.mj_forward(model, data)
    target = data.xpos[body_id].copy() + np.asarray([0.0, 0.0, lift_dz], dtype=np.float64)
    for _ in range(iterations):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        err = target - data.xpos[body_id]
        if np.linalg.norm(err) < 5e-4:
            break
        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        j_arm = jacp[:, arm_dof_ids]
        lhs = j_arm @ j_arm.T + (damping**2) * np.eye(3)
        dq = j_arm.T @ np.linalg.solve(lhs, err)
        q += np.clip(dq, -max_step, max_step)
        for idx, joint_id in enumerate(joint_ids):
            if bool(model.jnt_limited[joint_id]):
                low, high = model.jnt_range[joint_id]
                q[idx] = np.clip(q[idx], low, high)
    return q


def _sample_object_point_cloud(spec: ObjectSpec, n: int = 768) -> np.ndarray:
    rng = np.random.default_rng(7)
    if spec.geom_type == "box":
        sx, sy, sz = spec.size
        faces = rng.integers(0, 6, size=n)
        pts = np.empty((n, 3), dtype=np.float64)
        pts[:, 0] = rng.uniform(-sx, sx, size=n)
        pts[:, 1] = rng.uniform(-sy, sy, size=n)
        pts[:, 2] = rng.uniform(-sz, sz, size=n)
        for idx, face in enumerate(faces):
            axis = face // 2
            sign = -1.0 if face % 2 == 0 else 1.0
            pts[idx, axis] = sign * spec.size[axis]
        return pts
    if spec.geom_type == "cylinder":
        radius, half_height = spec.size
        theta = rng.uniform(0.0, 2.0 * math.pi, size=n)
        z = rng.uniform(-half_height, half_height, size=n)
        pts = np.stack([radius * np.cos(theta), radius * np.sin(theta), z], axis=1)
        cap_mask = rng.random(n) < 0.25
        cap_r = radius * np.sqrt(rng.random(np.count_nonzero(cap_mask)))
        cap_theta = rng.uniform(0.0, 2.0 * math.pi, size=np.count_nonzero(cap_mask))
        pts[cap_mask, 0] = cap_r * np.cos(cap_theta)
        pts[cap_mask, 1] = cap_r * np.sin(cap_theta)
        pts[cap_mask, 2] = rng.choice([-half_height, half_height], size=np.count_nonzero(cap_mask))
        return pts
    if spec.geom_type == "sphere":
        radius = spec.size[0]
        dirs = rng.normal(size=(n, 3))
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        return radius * dirs
    raise ValueError(f"Unsupported geom_type={spec.geom_type}")


def _point_cloud_summary(points: np.ndarray) -> dict[str, Any]:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    extents = maxs - mins
    return {
        "num_points": int(points.shape[0]),
        "min_xyz": mins.round(6).tolist(),
        "max_xyz": maxs.round(6).tolist(),
        "extents_xyz": extents.round(6).tolist(),
        "estimated_planar_width_m": float(min(extents[0], extents[1])),
    }


def _disable_robot_mesh_collisions(root: ET.Element) -> None:
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name.startswith("jaka_") or (name.startswith("rh56_") and geom.get("type") == "mesh"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")


def _configure_collision_model(root: ET.Element, *, collision_mode: str, include_calibration_markers: bool = False) -> None:
    if collision_mode not in COLLISION_MODES:
        raise ValueError(f"Unknown collision_mode={collision_mode}; choices={COLLISION_MODES}")
    if collision_mode == "correll_mesh":
        patch_rh56_correll_collision_model(root)
    elif collision_mode == "visual_coacd":
        patch_rh56_visual_coacd_collision_model(root)
    elif collision_mode == "proxy":
        _disable_robot_mesh_collisions(root)
        _add_fingertip_collision_proxies(root, include_calibration_markers=include_calibration_markers)
    elif collision_mode == "unifuc_pad_proxy":
        _disable_robot_mesh_collisions(root)
        _add_unifuc_pad_collision_proxies(root, include_calibration_markers=include_calibration_markers)
    elif collision_mode == "mesh_proxy":
        _add_fingertip_collision_proxies(root, include_calibration_markers=include_calibration_markers)


def _set_compiler_meshdir(root: ET.Element, base_xml: Path) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", str(base_xml.resolve().parent))


def _tune_actuators_for_grasp_benchmark(root: ET.Element) -> None:
    for actuator in root.iter("position"):
        name = actuator.get("name", "")
        if name.startswith("rh56_R_"):
            actuator.set("kp", "80")
        elif name.startswith("jaka_joint_"):
            actuator.set("kp", "160")


def _find_body(root: ET.Element, name: str) -> ET.Element:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    raise RuntimeError(f"Missing body {name}")


def _set_thumb_mimic_coupling(root: ET.Element, *, pip_multiplier: float, dip_multiplier: float) -> None:
    equality = root.find("equality")
    if equality is None:
        return
    for joint in equality.findall("joint"):
        joint1 = joint.get("joint1", "")
        if joint1.endswith("thumb_PIP_joint"):
            joint.set("polycoef", f"0 {pip_multiplier:.6g} 0 0 0")
        elif joint1.endswith("thumb_DIP_joint"):
            joint.set("polycoef", f"0 {dip_multiplier:.6g} 0 0 0")


FINGERTIP_PROXY_DEFS = {
    # Distal body origins are at the distal joint frames, not always at the
    # usable finger pads. These positions are in the MuJoCo body-local frame,
    # aligned to the compiled distal mesh centers rather than to body origins.
    "rh56_R_thumb_distal": ("thumb_pad_proxy", "0.0000 0.0160 -0.0010", "0.0065"),
    "rh56_R_index_distal": ("index_pad_proxy", "0.0083 0.0250 0.0015", "0.0065"),
    "rh56_R_middle_distal": ("middle_pad_proxy", "0.0064 0.0260 0.0015", "0.0065"),
    "rh56_R_ring_distal": ("ring_pad_proxy", "0.0080 0.0250 0.0015", "0.0063"),
    "rh56_R_pinky_distal": ("pinky_pad_proxy", "0.0079 0.0225 0.0016", "0.0060"),
}

UNIFUC_PAD_PROXY_DEFS = {
    # Existing UniFucGrasp-inspired pad mode. Keep these centers aligned with
    # the already validated project-local distal pad locations; do not infer
    # new body-local coordinates from the external UniFuc URDF directly.
    "rh56_R_thumb_distal": ("thumb_pad_proxy", "0.0000 0.0160 -0.0010", "0.0055 0.0075 0.0015"),
    "rh56_R_index_distal": ("index_pad_proxy", "0.0083 0.0250 0.0015", "0.0056 0.0078 0.0012"),
    "rh56_R_middle_distal": ("middle_pad_proxy", "0.0064 0.0260 0.0015", "0.0054 0.0077 0.0012"),
    "rh56_R_ring_distal": ("ring_pad_proxy", "0.0080 0.0250 0.0015", "0.0058 0.0079 0.0012"),
    "rh56_R_pinky_distal": ("pinky_pad_proxy", "0.0079 0.0225 0.0016", "0.0062 0.0082 0.0012"),
}


def _add_fingertip_collision_proxies(root: ET.Element, *, include_calibration_markers: bool = False) -> None:
    proxy_defs = {
        body_name: [proxy_def] for body_name, proxy_def in FINGERTIP_PROXY_DEFS.items()
    }
    for body_name, geoms in proxy_defs.items():
        body = _find_body(root, body_name)
        for geom_name, pos, radius in geoms:
            if any(geom.get("name") == geom_name for geom in body.iter("geom")):
                continue
            ET.SubElement(
                body,
                "geom",
                {
                    "name": geom_name,
                    "type": "sphere",
                    "pos": pos,
                    "size": radius,
                    "rgba": "0.05 0.85 0.80 0.55",
                    "friction": "2.2 0.12 0.006",
                    "condim": "4",
                    "priority": "2",
                },
            )
        if include_calibration_markers:
            _, active_pos, _ = FINGERTIP_PROXY_DEFS[body_name]
            center_x, _, center_z = active_pos.split()
            marker_offsets = ["0.014", "0.018", "0.022", "0.026"]
            marker_rgba = [
                "1.00 0.88 0.05 0.45",
                "1.00 0.55 0.05 0.45",
                "0.95 0.12 0.08 0.45",
                "0.55 0.10 1.00 0.45",
            ]
            if "thumb" in body_name:
                marker_offsets = ["0.004", "0.008", "0.012", "0.016"]
            proxy_prefix = FINGERTIP_PROXY_DEFS[body_name][0].replace("_pad_proxy", "")
            for idx, (offset_y, rgba) in enumerate(zip(marker_offsets, marker_rgba)):
                marker_name = f"{proxy_prefix}_pad_cal_{idx}"
                if any(geom.get("name") == marker_name for geom in body.iter("geom")):
                    continue
                ET.SubElement(
                    body,
                    "geom",
                    {
                        "name": marker_name,
                        "type": "sphere",
                        "pos": f"{center_x} {offset_y} {center_z}",
                        "size": "0.0035",
                        "rgba": rgba,
                        "contype": "0",
                        "conaffinity": "0",
                    },
                )


def _add_unifuc_pad_collision_proxies(root: ET.Element, *, include_calibration_markers: bool = False) -> None:
    for body_name, (geom_name, pos, size) in UNIFUC_PAD_PROXY_DEFS.items():
        body = _find_body(root, body_name)
        if not any(geom.get("name") == geom_name for geom in body.iter("geom")):
            ET.SubElement(
                body,
                "geom",
                {
                    "name": geom_name,
                    "type": "box",
                    "pos": pos,
                    "size": size,
                    "rgba": "0.05 0.85 0.80 0.55",
                    "friction": "2.2 0.12 0.006",
                    "condim": "4",
                    "priority": "2",
                },
            )
        if include_calibration_markers:
            center_x, center_y, center_z = pos.split()
            marker_name = geom_name.replace("_pad_proxy", "_unifuc_center")
            if not any(geom.get("name") == marker_name for geom in body.iter("geom")):
                ET.SubElement(
                    body,
                    "geom",
                    {
                        "name": marker_name,
                        "type": "sphere",
                        "pos": f"{center_x} {center_y} {center_z}",
                        "size": "0.0025",
                        "rgba": "1.00 0.55 0.05 0.70",
                        "contype": "0",
                        "conaffinity": "0",
                    },
                )


def _add_table_object_camera(
    root: ET.Element,
    *,
    spec: ObjectSpec,
    object_pos: np.ndarray,
    table_top_z: float,
) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody.")

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "bench_table",
            "type": "box",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {table_top_z - 0.02:.6f}",
            "size": "0.42 0.32 0.020",
            "rgba": "0.72 0.66 0.56 1",
            "friction": "1.4 0.05 0.003",
            "condim": "4",
        },
    )
    obj_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "bench_object_body",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {object_pos[2]:.6f}",
        },
    )
    ET.SubElement(obj_body, "freejoint", {"name": "bench_object_freejoint"})
    geom_attrs = {
        "name": "bench_object",
        "type": spec.geom_type,
        "mass": f"{spec.mass:.6f}",
        "rgba": spec.rgba,
        "friction": spec.friction,
        "condim": "4",
        "priority": "1",
    }
    if spec.geom_type == "box":
        geom_attrs["size"] = f"{spec.size[0]:.6f} {spec.size[1]:.6f} {spec.size[2]:.6f}"
    elif spec.geom_type == "cylinder":
        geom_attrs["size"] = f"{spec.size[0]:.6f} {spec.size[1]:.6f}"
    elif spec.geom_type == "sphere":
        geom_attrs["size"] = f"{spec.size[0]:.6f}"
    ET.SubElement(obj_body, "geom", geom_attrs)

    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "bench_close_camera",
            "mode": "fixed",
            "pos": "-0.30 -0.76 0.36",
            "xyaxes": "0.96 -0.29 0 0.14 0.46 0.88",
            "fovy": "40",
        },
    )


def _build_scene_xml(
    base_xml: Path,
    out_xml: Path,
    *,
    spec: ObjectSpec,
    object_pos: np.ndarray,
    table_top_z: float,
    collision_mode: str,
) -> None:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    _set_compiler_meshdir(root, base_xml)
    _tune_actuators_for_grasp_benchmark(root)
    _configure_collision_model(root, collision_mode=collision_mode)
    _add_table_object_camera(root, spec=spec, object_pos=object_pos, table_top_z=table_top_z)
    root.set("model", f"jaka_rh56_{spec.name}_grasp_benchmark")
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)


def _estimate_nominal_object_pos(
    base_xml: Path,
    grasp_q: np.ndarray,
    close_ctrl: np.ndarray,
    spec: ObjectSpec,
    *,
    collision_mode: str,
) -> np.ndarray:
    tree = ET.parse(base_xml)
    root = tree.getroot()
    _set_compiler_meshdir(root, base_xml)
    _configure_collision_model(root, collision_mode=collision_mode)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    data = mujoco.MjData(model)
    _set_kinematic_pose(model, data, grasp_q, close_ctrl)
    thumb = _semantic_contact_center(model, data, "thumb")
    index = _semantic_contact_center(model, data, "index")
    middle = _semantic_contact_center(model, data, "middle")
    center = 0.45 * thumb + 0.35 * index + 0.20 * middle
    center[2] = max(center[2], spec.half_height + 0.026)
    return center


def _make_grasp_candidates(spec: ObjectSpec) -> list[dict[str, Any]]:
    width = spec.planar_width
    if width <= 0.036:
        base_profiles = [
            [0.0, 0.0, 0.18, 0.22, 0.45, 1.0],
            [0.0, 0.0, 0.30, 0.34, 0.52, 1.0],
            [0.05, 0.05, 0.42, 0.46, 0.60, 1.0],
            [0.10, 0.10, 0.55, 0.60, 0.68, 1.0],
        ]
    elif width <= 0.052:
        base_profiles = [
            [0.0, 0.0, 0.10, 0.14, 0.40, 1.0],
            [0.0, 0.0, 0.20, 0.24, 0.48, 1.0],
            [0.04, 0.04, 0.32, 0.36, 0.56, 1.0],
            [0.08, 0.08, 0.45, 0.50, 0.64, 1.0],
        ]
    else:
        base_profiles = [
            [0.05, 0.05, 0.08, 0.12, 0.36, 1.0],
            [0.10, 0.10, 0.18, 0.22, 0.44, 1.0],
            [0.15, 0.15, 0.28, 0.32, 0.52, 1.0],
            [0.20, 0.20, 0.38, 0.42, 0.60, 1.0],
        ]

    offsets = [
        [0.0, 0.0, 0.0],
        [0.010, 0.0, 0.0],
        [-0.010, 0.0, 0.0],
        [0.020, 0.0, 0.0],
        [-0.020, 0.0, 0.0],
        [0.0, 0.010, 0.0],
        [0.0, -0.010, 0.0],
        [0.0, 0.020, 0.0],
        [0.0, -0.020, 0.0],
        [0.010, 0.010, 0.004],
        [-0.010, 0.010, 0.004],
        [0.010, -0.010, 0.004],
        [-0.010, -0.010, 0.004],
        [0.020, 0.010, 0.006],
        [-0.020, 0.010, 0.006],
        [0.020, -0.010, 0.006],
        [-0.020, -0.010, 0.006],
        [0.0, 0.0, 0.010],
    ]
    candidates: list[dict[str, Any]] = []
    for profile_idx, profile in enumerate(base_profiles):
        rotate = [0.0, 0.0, 0.0, 0.0, 0.0, profile[5]]
        for offset_idx, offset in enumerate(offsets):
            candidates.append(
                {
                    "name": f"p{profile_idx}_o{offset_idx}",
                    "physical_close_norm": profile,
                    "physical_rotate_norm": rotate,
                    "object_offset": offset,
                }
            )
    return candidates


def _contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int]:
    counts = {
        "object_thumb": 0,
        "object_index": 0,
        "object_middle": 0,
        "object_ring_pinky": 0,
        "object_table": 0,
        "hand_table": 0,
        "hand_self": 0,
        "max_penetration_mm": 0,
        "total": int(data.ncon),
    }
    max_penetration_m = 0.0
    for idx in range(data.ncon):
        contact = data.contact[idx]
        names = [_geom_name(model, int(contact.geom1)), _geom_name(model, int(contact.geom2))]
        groups = [
            _semantic_contact_group(model, int(contact.geom1)),
            _semantic_contact_group(model, int(contact.geom2)),
        ]
        joined = " ".join(names)
        if contact.dist < 0.0:
            max_penetration_m = max(max_penetration_m, abs(float(contact.dist)))
        if "bench_object" in groups and "thumb" in groups:
            counts["object_thumb"] += 1
        elif "bench_object" in groups and "index" in groups:
            counts["object_index"] += 1
        elif "bench_object" in groups and "middle" in groups:
            counts["object_middle"] += 1
        elif "bench_object" in groups and "ring_pinky" in groups:
            counts["object_ring_pinky"] += 1
        if "bench_object" in groups and "bench_table" in groups:
            counts["object_table"] += 1
        if any(group in {"thumb", "index", "middle", "ring_pinky", "hand_other"} for group in groups) and "bench_table" in groups:
            counts["hand_table"] += 1
        if all(group in {"thumb", "index", "middle", "ring_pinky", "hand_other"} for group in groups):
            counts["hand_self"] += 1
    counts["max_penetration_mm"] = int(round(max_penetration_m * 1000.0))
    return counts


def _object_hand_contact_count(contacts: dict[str, int]) -> int:
    return (
        contacts["object_thumb"]
        + contacts["object_index"]
        + contacts["object_middle"]
        + contacts["object_ring_pinky"]
    )


def _run_candidate(
    xml_path: Path,
    *,
    grasp_q: np.ndarray,
    lift_q: np.ndarray,
    rotate_ctrl: np.ndarray,
    close_ctrl: np.ndarray,
    duration: float,
    success_lift_m: float,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    object_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bench_object_body")
    if object_body < 0:
        raise KeyError("Missing bench_object_body")

    data.qpos[:6] = grasp_q
    data.ctrl[arm_ids] = grasp_q
    data.ctrl[hand_ids] = np.zeros(6)
    mujoco.mj_forward(model, data)
    initial_z = float(data.xpos[object_body][2])
    initial_contacts = _contact_summary(model, data)
    initial_penetration = bool(
        initial_contacts["hand_table"] > 0
        or initial_contacts["hand_self"] > 0
        or _object_hand_contact_count(initial_contacts) > 0
    )

    dt = model.opt.timestep
    contact_log: list[dict[str, Any]] = []
    while data.time < duration:
        t = float(data.time)
        if t < 0.50:
            arm_alpha = 0.0
            hand = np.zeros(6)
        elif t < 1.00:
            arm_alpha = 0.0
            hand = rotate_ctrl
        elif t < 2.20:
            arm_alpha = 0.0
            alpha = (t - 1.00) / 1.20
            hand = (1.0 - alpha) * rotate_ctrl + alpha * close_ctrl
        else:
            arm_alpha = min(1.0, (t - 2.20) / max(0.40, duration - 2.20))
            hand = close_ctrl
        arm_q = (1.0 - arm_alpha) * grasp_q + arm_alpha * lift_q
        data.ctrl[arm_ids] = arm_q
        data.ctrl[hand_ids] = hand
        mujoco.mj_step(model, data)

        if int(data.time / 0.25) != int((data.time - dt) / 0.25):
            contact_log.append(
                {
                    "time": round(float(data.time), 3),
                    "object_pos": data.xpos[object_body].round(5).tolist(),
                    "contacts": _contact_summary(model, data),
                }
            )

    final_pos = data.xpos[object_body].copy()
    contacts = _contact_summary(model, data)
    lift_m = float(final_pos[2] - initial_z)
    opposing_contact = contacts["object_thumb"] > 0 and (
        contacts["object_index"] > 0 or contacts["object_middle"] > 0 or contacts["object_ring_pinky"] > 0
    )
    success = bool(
        lift_m >= success_lift_m
        and opposing_contact
        and contacts["object_table"] == 0
        and contacts["hand_self"] == 0
        and not initial_penetration
    )
    score = (
        100.0 * lift_m
        + 2.0 * contacts["object_thumb"]
        + 2.0 * contacts["object_index"]
        + 2.0 * contacts["object_middle"]
        + contacts["object_ring_pinky"]
        - 2.0 * contacts["object_table"]
        - contacts["hand_table"]
        - 3.0 * contacts["hand_self"]
        - 0.5 * contacts["max_penetration_mm"]
        - (25.0 if initial_penetration else 0.0)
    )
    return {
        "success": success,
        "score": float(score),
        "initial_z": initial_z,
        "final_object_pos": final_pos.round(6).tolist(),
        "lift_m": lift_m,
        "success_lift_m": success_lift_m,
        "initial_contacts": initial_contacts,
        "initial_penetration": initial_penetration,
        "opposing_contact": opposing_contact,
        "final_contacts": contacts,
        "contact_log": contact_log,
    }


def run_benchmark(args: argparse.Namespace) -> dict[str, Any]:
    robot_cfg = _load_yaml(args.robot_config)
    grasp_q = np.asarray(robot_cfg["joint_presets"]["pinch_grasp_box_v2"], dtype=np.float64)
    lift_q = _solve_hand_base_lift_q(Path(args.base_xml), grasp_q=grasp_q, lift_dz=args.lift_dz)

    selected_objects = list(OBJECTS) if args.objects == ["all"] else args.objects
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "base_xml": str(Path(args.base_xml)),
        "grasp_preset": "pinch_grasp_box_v2",
        "lift_strategy": {"type": "hand_base_position_ik", "lift_dz_m": args.lift_dz, "lift_q": lift_q.round(6).tolist()},
        "method": "point-cloud width estimate + RH56 analytical candidate sweep + MuJoCo contact validation",
        "collision_mode": args.collision_mode,
        "success_lift_m": args.success_lift,
        "objects": {},
    }

    for object_name in selected_objects:
        if object_name not in OBJECTS:
            raise ValueError(f"Unknown object {object_name}; choices={sorted(OBJECTS)}")
        spec = OBJECTS[object_name]
        points = _sample_object_point_cloud(spec, n=args.point_count)
        point_summary = _point_cloud_summary(points)
        object_dir = out_dir / object_name
        object_dir.mkdir(parents=True, exist_ok=True)
        np.save(object_dir / "object_point_cloud.npy", points)

        candidates = _make_grasp_candidates(spec)
        candidate_results: list[dict[str, Any]] = []
        for candidate in candidates[: args.max_candidates]:
            close_ctrl = _physical_norm_to_mujoco_ctrl(candidate["physical_close_norm"])
            rotate_ctrl = _physical_norm_to_mujoco_ctrl(candidate["physical_rotate_norm"])
            nominal_pos = _estimate_nominal_object_pos(
                Path(args.base_xml),
                grasp_q,
                close_ctrl,
                spec,
                collision_mode=args.collision_mode,
            )
            object_pos = nominal_pos + np.asarray(candidate["object_offset"], dtype=np.float64)
            object_pos[2] = max(object_pos[2], spec.half_height + 0.016)
            table_top_z = float(object_pos[2] - spec.half_height - 0.002)
            xml_path = object_dir / f"{candidate['name']}.xml"
            _build_scene_xml(
                Path(args.base_xml),
                xml_path,
                spec=spec,
                object_pos=object_pos,
                table_top_z=table_top_z,
                collision_mode=args.collision_mode,
            )
            result = _run_candidate(
                xml_path,
                grasp_q=grasp_q,
                lift_q=lift_q,
                rotate_ctrl=rotate_ctrl,
                close_ctrl=close_ctrl,
                duration=args.duration,
                success_lift_m=args.success_lift,
            )
            candidate_results.append(
                {
                    **candidate,
                    "xml": str(xml_path),
                    "object_pos": object_pos.round(6).tolist(),
                    "table_top_z": table_top_z,
                    "close_ctrl_mujoco": close_ctrl.round(6).tolist(),
                    "rotate_ctrl_mujoco": rotate_ctrl.round(6).tolist(),
                    "result": result,
                }
            )
        candidate_results.sort(
            key=lambda item: (item["result"]["success"], item["result"]["score"]),
            reverse=True,
        )
        object_summary = {
            "spec": {
                "geom_type": spec.geom_type,
                "size": list(spec.size),
                "mass": spec.mass,
                "planar_width_m": spec.planar_width,
            },
            "point_cloud": point_summary,
            "num_candidates": len(candidate_results),
            "num_success": sum(1 for item in candidate_results if item["result"]["success"]),
            "top_candidates": candidate_results[: min(10, len(candidate_results))],
        }
        (object_dir / "candidates.json").write_text(json.dumps(candidate_results, indent=2), encoding="utf-8")
        (object_dir / "summary.json").write_text(json.dumps(object_summary, indent=2), encoding="utf-8")
        summary["objects"][object_name] = object_summary

    (out_dir / "benchmark_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="MuJoCo grasp benchmark for JAKA mini2 + RH56.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--out-dir", default=str(OUT_DIR))
    parser.add_argument("--objects", nargs="+", default=["all"], help=f"Object names or all. Choices: {sorted(OBJECTS)}")
    parser.add_argument("--duration", type=float, default=3.8)
    parser.add_argument("--lift-dz", type=float, default=0.080)
    parser.add_argument("--success-lift", type=float, default=0.020)
    parser.add_argument("--point-count", type=int, default=768)
    parser.add_argument("--max-candidates", type=int, default=72)
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="correll_mesh")
    args = parser.parse_args()

    summary = run_benchmark(args)
    compact = {
        "out_dir": args.out_dir,
        "objects": {
            name: {
                "num_success": item["num_success"],
                "best_score": item["top_candidates"][0]["result"]["score"] if item["top_candidates"] else None,
                "best_success": item["top_candidates"][0]["result"]["success"] if item["top_candidates"] else None,
                "best_lift_m": item["top_candidates"][0]["result"]["lift_m"] if item["top_candidates"] else None,
                "best_contacts": item["top_candidates"][0]["result"]["final_contacts"] if item["top_candidates"] else None,
            }
            for name, item in summary["objects"].items()
        },
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
