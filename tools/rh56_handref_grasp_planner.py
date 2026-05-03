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

from sim_maniskill.rh56_collision import patch_rh56_collision_model


BASE_XML = Path("data/sim_assets/jaka_rh56.xml")
OUT_DIR = Path("data/mujoco_handref_grasps")
ROBOT_CONFIG = Path("configs/robot/jaka_mini2_real.yaml")
TABLE_TOP_Z = 0.80
OBJECT_CENTER_XY = (-0.035, -0.570)

ARM_ACTUATORS = [f"jaka_joint_{idx}_act" for idx in range(1, 7)]
HAND_ACTUATORS = [
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
]

TIP_GEOMS = {
    "thumb": "rh56_R_thumb_distal_collision",
    "index": "rh56_R_index_distal_collision",
    "middle": "rh56_R_middle_distal_collision",
    "ring": "rh56_R_ring_distal_collision",
    "pinky": "rh56_R_pinky_distal_collision",
}


@dataclass(frozen=True)
class ObjectSpec:
    name: str
    dataset: str
    dataset_id: str
    display_name: str
    geom_type: str
    size: tuple[float, ...]
    mass: float
    rgba: str
    family: str
    collision_padding: float
    pose_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    table_half_height: float | None = None
    friction: str = "2.4 0.12 0.006"

    @property
    def half_height(self) -> float:
        if self.table_half_height is not None:
            return self.table_half_height
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

    @property
    def collision_size(self) -> tuple[float, ...]:
        pad = self.collision_padding
        if self.geom_type == "sphere":
            return (self.size[0] + pad,)
        if self.geom_type == "cylinder":
            return (self.size[0] + pad, self.size[1])
        return (self.size[0] + pad, self.size[1] + pad, self.size[2])

    @property
    def visual_size(self) -> tuple[float, ...]:
        return self.size

    @property
    def pose_quat_string(self) -> str:
        return " ".join(f"{value:.8f}" for value in self.pose_quat)


OBJECTS: dict[str, ObjectSpec] = {
    "foam_block_40mm": ObjectSpec(
        "foam_block_40mm",
        "Common tabletop",
        "foam_block_40mm",
        "40 mm lightweight foam block",
        "box",
        (0.020, 0.020, 0.020),
        0.018,
        "0.85 0.30 0.18 1",
        "box_precision_pinch",
        0.003,
        friction="2.0 0.08 0.004",
    ),
    "light_cylinder_36mm": ObjectSpec(
        "light_cylinder_36mm",
        "Common tabletop",
        "light_cylinder_36mm",
        "36 mm lightweight horizontal cylinder",
        "cylinder",
        (0.018, 0.040),
        0.030,
        "0.20 0.52 0.85 1",
        "cylinder_power_envelope",
        0.003,
        (0.70710678, 0.0, 0.70710678, 0.0),
        0.018,
        friction="1.4 0.05 0.002",
    ),
    "light_can_50mm": ObjectSpec(
        "light_can_50mm",
        "Common tabletop",
        "light_can_50mm",
        "50 mm lightweight horizontal can",
        "cylinder",
        (0.025, 0.045),
        0.050,
        "0.78 0.40 0.16 1",
        "cylinder_power_envelope",
        0.003,
        (0.70710678, 0.0, 0.70710678, 0.0),
        0.025,
        friction="1.6 0.06 0.003",
    ),
    "062_dice": ObjectSpec(
        "062_dice",
        "YCB",
        "062_dice",
        "YCB 062 dice",
        "box",
        (0.0081, 0.0081, 0.0081),
        0.0052,
        "0.92 0.92 0.88 1",
        "box_precision_pinch",
        0.002,
    ),
    "009_gelatin_box": ObjectSpec(
        "009_gelatin_box",
        "YCB",
        "009_gelatin_box",
        "YCB 009 gelatin box",
        "box",
        (0.0140, 0.0425, 0.0365),
        0.097,
        "0.76 0.36 0.30 1",
        "box_power_envelope",
        0.003,
        (0.70710678, 0.0, 0.70710678, 0.0),
        0.0140,
    ),
    "061_foam_brick": ObjectSpec(
        "061_foam_brick",
        "YCB",
        "061_foam_brick",
        "YCB 061 foam brick",
        "box",
        (0.0375, 0.0250, 0.0250),
        0.028,
        "0.85 0.30 0.18 1",
        "box_precision_pinch",
        0.004,
    ),
    "004_sugar_box": ObjectSpec(
        "004_sugar_box",
        "YCB",
        "004_sugar_box",
        "YCB 004 sugar box",
        "box",
        (0.0300, 0.0445, 0.0875),
        0.514,
        "0.75 0.48 0.30 1",
        "box_precision_pinch",
        0.004,
    ),
    "005_tomato_soup_can": ObjectSpec(
        "005_tomato_soup_can",
        "YCB",
        "005_tomato_soup_can",
        "YCB 005 tomato soup can",
        "cylinder",
        (0.0330, 0.0505),
        0.349,
        "0.20 0.52 0.85 1",
        "cylinder_power_envelope",
        0.0035,
    ),
    "040_large_marker": ObjectSpec(
        "040_large_marker",
        "YCB",
        "040_large_marker",
        "YCB 040 large marker",
        "cylinder",
        (0.0090, 0.0700),
        0.015,
        "0.15 0.20 0.22 1",
        "thin_cylinder_tripod",
        0.0025,
        (0.70710678, 0.0, 0.70710678, 0.0),
        0.0090,
    ),
    "056_tennis_ball": ObjectSpec(
        "056_tennis_ball",
        "YCB",
        "056_tennis_ball",
        "YCB 056 tennis ball",
        "sphere",
        (0.0330,),
        0.058,
        "0.52 0.78 0.20 1",
        "sphere_containment",
        0.003,
    ),
}

DEFAULT_OBJECTS = ["foam_block_40mm", "light_cylinder_36mm", "light_can_50mm"]

OBJECT_ALIASES: dict[str, str] = {
    "foam_cube": "foam_block_40mm",
    "paper_box": "061_foam_brick",
    "light_cylinder": "light_cylinder_36mm",
    "can": "light_can_50mm",
    "light_can": "light_can_50mm",
    "round_ball": "056_tennis_ball",
}


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _ids(model: mujoco.MjModel, obj_type: mujoco.mjtObj, names: list[str]) -> np.ndarray:
    ids: list[int] = []
    for name in names:
        idx = mujoco.mj_name2id(model, obj_type, name)
        if idx < 0:
            raise KeyError(f"Missing {obj_type} named {name}")
        ids.append(idx)
    return np.asarray(ids, dtype=np.int32)


def _physical_norm_to_mujoco_ctrl(values: list[float] | tuple[float, ...]) -> np.ndarray:
    """Physical order: pinky, ring, middle, index, thumb_bend, thumb_rotate."""

    if len(values) != 6:
        raise ValueError("Expected six RH56 physical normalized values.")
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


def _physical_norm_to_raw(values: list[float] | tuple[float, ...]) -> list[int]:
    return [int(round(1000.0 * (1.0 - float(np.clip(v, 0.0, 1.0))))) for v in values]


def _set_kinematic_hand(model: mujoco.MjModel, data: mujoco.MjData, arm_q: np.ndarray, ctrl: np.ndarray) -> None:
    data.qpos[:6] = arm_q
    thumb_rotate, thumb_bend, index, middle, ring, pinky = ctrl
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
    data.ctrl[6:12] = ctrl
    mujoco.mj_forward(model, data)


def _geom_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
    if geom_id < 0:
        raise KeyError(f"Missing geom {name}")
    return np.asarray(data.geom_xpos[geom_id], dtype=np.float64).copy()


def _hand_base_pos(base_xml: Path, arm_q: np.ndarray) -> np.ndarray:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    data.qpos[:6] = arm_q
    mujoco.mj_forward(model, data)
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy()


def _hand_base_pose(base_xml: Path, arm_q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    data.qpos[:6] = arm_q
    mujoco.mj_forward(model, data)
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy(), np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3).copy()


def _rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def _rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _rpy_matrix(rpy: list[float] | tuple[float, float, float]) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def _orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    return 0.5 * (
        np.cross(current[:, 0], target[:, 0])
        + np.cross(current[:, 1], target[:, 1])
        + np.cross(current[:, 2], target[:, 2])
    )


def _solve_hand_base_target_q(base_xml: Path, seed_q: np.ndarray, target_pos: np.ndarray) -> tuple[np.ndarray, float]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    arm_dofs = np.asarray(
        [
            model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")]
            for idx in range(1, 7)
        ],
        dtype=np.int32,
    )
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}") for idx in range(1, 7)]
    q = seed_q.astype(np.float64).copy()
    final_err = float("inf")
    for _ in range(180):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        err = np.asarray(target_pos, dtype=np.float64) - data.xpos[body_id]
        final_err = float(np.linalg.norm(err))
        if final_err < 7e-4:
            break
        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        j = jacp[:, arm_dofs]
        dq = j.T @ np.linalg.solve(j @ j.T + 0.035**2 * np.eye(3), err)
        q += np.clip(dq, -0.035, 0.035)
        for idx, joint_id in enumerate(joint_ids):
            if bool(model.jnt_limited[joint_id]):
                q[idx] = float(np.clip(q[idx], model.jnt_range[joint_id, 0], model.jnt_range[joint_id, 1]))
    return q, final_err


def _solve_hand_pose_target_q(
    base_xml: Path,
    seed_q: np.ndarray,
    target_pos: np.ndarray,
    target_rot: np.ndarray,
    *,
    rot_weight: float = 0.12,
) -> tuple[np.ndarray, float, float]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    if body_id < 0:
        raise KeyError("Missing rh56_R_hand_base_link")
    arm_dofs = np.asarray(
        [
            model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}")]
            for idx in range(1, 7)
        ],
        dtype=np.int32,
    )
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"jaka_joint_{idx}") for idx in range(1, 7)]
    q = seed_q.astype(np.float64).copy()
    final_pos_err = float("inf")
    final_rot_err = float("inf")
    target_rot = np.asarray(target_rot, dtype=np.float64).reshape(3, 3)
    for _ in range(220):
        data.qpos[:6] = q
        mujoco.mj_forward(model, data)
        current_rot = np.asarray(data.xmat[body_id], dtype=np.float64).reshape(3, 3)
        pos_err = np.asarray(target_pos, dtype=np.float64) - data.xpos[body_id]
        rot_err = _orientation_error(current_rot, target_rot)
        final_pos_err = float(np.linalg.norm(pos_err))
        final_rot_err = float(np.linalg.norm(rot_err))
        if final_pos_err < 8e-4 and final_rot_err < 0.020:
            break
        jacp = np.zeros((3, model.nv), dtype=np.float64)
        jacr = np.zeros((3, model.nv), dtype=np.float64)
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)
        j = np.vstack([jacp[:, arm_dofs], rot_weight * jacr[:, arm_dofs]])
        err = np.concatenate([pos_err, rot_weight * rot_err])
        dq = j.T @ np.linalg.solve(j @ j.T + 0.040**2 * np.eye(6), err)
        q += np.clip(dq, -0.030, 0.030)
        for idx, joint_id in enumerate(joint_ids):
            if bool(model.jnt_limited[joint_id]):
                q[idx] = float(np.clip(q[idx], model.jnt_range[joint_id, 0], model.jnt_range[joint_id, 1]))
    return q, final_pos_err, final_rot_err


def _solve_hand_base_offset_q(base_xml: Path, grasp_q: np.ndarray, xyz_delta: np.ndarray) -> np.ndarray:
    target = _hand_base_pos(base_xml, grasp_q) + np.asarray(xyz_delta, dtype=np.float64)
    q, _ = _solve_hand_base_target_q(base_xml, grasp_q, target)
    return q


def _solve_lift_q(base_xml: Path, grasp_q: np.ndarray, lift_dz: float) -> np.ndarray:
    return _solve_hand_base_offset_q(base_xml, grasp_q, np.asarray([0.0, 0.0, lift_dz], dtype=np.float64))


def _solve_approach_q(base_xml: Path, grasp_q: np.ndarray, approach_dy: float) -> np.ndarray:
    return _solve_hand_base_offset_q(base_xml, grasp_q, np.asarray([0.0, approach_dy, 0.0], dtype=np.float64))


def _solve_radial_approach_q(
    base_xml: Path,
    grasp_q: np.ndarray,
    target_hand_pos: np.ndarray,
    object_pos: np.ndarray,
    approach_distance: float,
    approach_dy: float,
) -> np.ndarray:
    approach_target = np.asarray(target_hand_pos, dtype=np.float64).copy()
    radial = approach_target[:2] - np.asarray(object_pos, dtype=np.float64)[:2]
    norm = float(np.linalg.norm(radial))
    if norm < 1e-6:
        radial = np.asarray([0.0, -1.0], dtype=np.float64)
    else:
        radial = radial / norm
    approach_target[:2] += radial * float(approach_distance)
    approach_target[1] += float(approach_dy)
    q, _ = _solve_hand_base_target_q(base_xml, grasp_q, approach_target)
    return q


def _prepare_robot_xml(base_xml: Path) -> ET.Element:
    root = ET.parse(base_xml).getroot()
    patch_rh56_collision_model(root)
    for actuator in root.iter("position"):
        name = actuator.get("name", "")
        if name.startswith("rh56_R_"):
            actuator.set("kp", "100")
        elif name.startswith("jaka_joint_"):
            actuator.set("kp", "180")
    for geom in root.iter("geom"):
        name = geom.get("name", "")
        if name.startswith("jaka_") or (name.startswith("rh56_R_") and geom.get("type") == "mesh"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    return root


def _shift_robot_base(root: ET.Element, z_offset: float) -> None:
    if abs(z_offset) < 1e-9:
        return
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody")
    robot_base = worldbody.find("./body[@name='jaka_Link_0']")
    if robot_base is None:
        raise RuntimeError("Missing jaka_Link_0")
    pos = [float(value) for value in robot_base.get("pos", "0 0 0").split()]
    while len(pos) < 3:
        pos.append(0.0)
    pos[2] += z_offset
    robot_base.set("pos", f"{pos[0]:.6f} {pos[1]:.6f} {pos[2]:.6f}")


def _format_size(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.6f}" for value in values)


def _add_object_geom_size(attrs: dict[str, str], geom_type: str, size: tuple[float, ...]) -> None:
    if geom_type == "box":
        attrs["size"] = _format_size((size[0], size[1], size[2]))
    elif geom_type == "cylinder":
        attrs["size"] = _format_size((size[0], size[1]))
    elif geom_type == "sphere":
        attrs["size"] = _format_size((size[0],))


def _add_scene(root: ET.Element, spec: ObjectSpec, object_pos: np.ndarray, table_top_z: float) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "bench_table",
            "type": "box",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {table_top_z - 0.020:.6f}",
            "size": "0.42 0.32 0.020",
            "rgba": "0.72 0.66 0.56 1",
            "friction": "1.4 0.05 0.003",
            "condim": "4",
            "contype": "1",
            "conaffinity": "7",
        },
    )
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "bench_object_body",
            "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {object_pos[2]:.6f}",
            "quat": spec.pose_quat_string,
        },
    )
    ET.SubElement(body, "freejoint", {"name": "bench_object_freejoint"})
    visual_attrs = {
        "name": "bench_object_visual",
        "type": spec.geom_type,
        "rgba": spec.rgba,
        "contype": "0",
        "conaffinity": "0",
        "group": "1",
        "density": "0",
    }
    _add_object_geom_size(visual_attrs, spec.geom_type, spec.visual_size)
    ET.SubElement(body, "geom", visual_attrs)

    collision_attrs = {
        "name": "bench_object",
        "type": spec.geom_type,
        "mass": f"{spec.mass:.6f}",
        "rgba": "0.05 0.65 1.00 0.18",
        "friction": spec.friction,
        "condim": "4",
        "priority": "2",
        "contype": "1",
        "conaffinity": "6",
        "solref": "0.004 1",
        "solimp": "0.92 0.98 0.002",
        "group": "3",
    }
    _add_object_geom_size(collision_attrs, spec.geom_type, spec.collision_size)
    ET.SubElement(body, "geom", collision_attrs)
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


def _write_scene_xml(
    base_xml: Path,
    out_xml: Path,
    spec: ObjectSpec,
    object_pos: np.ndarray,
    table_top_z: float,
    scene_z_offset: float,
) -> None:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    root = _prepare_robot_xml(base_xml)
    _shift_robot_base(root, scene_z_offset)
    _add_scene(root, spec, object_pos, table_top_z)
    root.set("model", f"rh56_handref_{spec.name}")
    ET.indent(root, space="  ")
    out_xml.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")


def _tip_positions(base_xml: Path, grasp_q: np.ndarray, physical_close: list[float]) -> dict[str, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    _set_kinematic_hand(model, data, grasp_q, _physical_norm_to_mujoco_ctrl(physical_close))
    return {key: _geom_pos(model, data, geom_name) for key, geom_name in TIP_GEOMS.items()}


def _object_center_from_plan(base_xml: Path, grasp_q: np.ndarray, spec: ObjectSpec, candidate: dict[str, Any]) -> np.ndarray:
    tips = _tip_positions(base_xml, grasp_q, candidate["physical_close_norm"])
    if "cylinder" in candidate["family"] or "sphere" in candidate["family"] or "power" in candidate["family"]:
        opposing = 0.35 * tips["index"] + 0.35 * tips["middle"] + 0.15 * tips["ring"] + 0.15 * tips["pinky"]
        center = 0.50 * tips["thumb"] + 0.50 * opposing
    else:
        center = 0.45 * tips["thumb"] + 0.35 * tips["index"] + 0.20 * tips["middle"]
    center = center + np.asarray(candidate["object_offset"], dtype=np.float64)
    center[2] = max(center[2] + float(candidate["z_drop"]), spec.half_height + 0.002)
    return center


def _fixed_object_pos(spec: ObjectSpec, table_clearance: float, object_xy: tuple[float, float]) -> np.ndarray:
    return np.asarray(
        [
            float(object_xy[0]),
            float(object_xy[1]),
            spec.half_height + float(table_clearance),
        ],
        dtype=np.float64,
    )


def _wrist_pose_deltas(spec: ObjectSpec) -> list[dict[str, Any]]:
    if spec.family == "box_power_envelope":
        return [
            {"name": "box_side_center", "delta": [0.0, 0.0, 0.004], "rpy": [0.0, 0.0, 0.0]},
            {"name": "box_side_yaw_left", "delta": [0.012, -0.006, 0.010], "rpy": [0.0, 0.0, 0.55]},
            {"name": "box_side_yaw_right", "delta": [-0.012, -0.006, 0.010], "rpy": [0.0, 0.0, -0.55]},
            {"name": "box_top_canted", "delta": [0.0, -0.006, 0.020], "rpy": [0.45, 0.0, 0.0]},
        ]
    if spec.family == "thin_cylinder_tripod":
        return [
            {"name": "axis_center", "delta": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            {"name": "thumb_side", "delta": [0.012, -0.010, 0.004], "rpy": [0.0, 0.0, 0.70]},
            {"name": "index_side", "delta": [-0.012, -0.010, 0.004], "rpy": [0.0, 0.0, -0.70]},
            {"name": "top_tripod", "delta": [0.0, -0.006, 0.018], "rpy": [0.45, 0.0, 0.0]},
        ]
    if spec.family == "sphere_containment":
        return [
            {"name": "palm_cup", "delta": [0.0, 0.0, 0.020], "rpy": [0.35, 0.0, 0.0]},
            {"name": "thumb_high", "delta": [0.012, -0.004, 0.016], "rpy": [0.25, 0.0, 0.50]},
            {"name": "index_high", "delta": [-0.012, -0.004, 0.016], "rpy": [0.25, 0.0, -0.50]},
            {"name": "low_envelope", "delta": [0.0, 0.010, 0.008], "rpy": [-0.25, 0.0, 0.0]},
        ]
    if spec.family == "cylinder_power_envelope":
        return [
            {"name": "power_center", "delta": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
            {"name": "power_axis_left", "delta": [0.018, -0.006, 0.006], "rpy": [0.0, 0.0, 0.65]},
            {"name": "power_axis_right", "delta": [-0.018, -0.006, 0.006], "rpy": [0.0, 0.0, -0.65]},
            {"name": "power_high", "delta": [0.0, -0.006, 0.018], "rpy": [0.35, 0.0, 0.0]},
        ]
    return [
        {"name": "precision_center", "delta": [0.0, 0.0, 0.0], "rpy": [0.0, 0.0, 0.0]},
        {"name": "precision_yaw_left", "delta": [0.014, -0.004, 0.006], "rpy": [0.0, 0.0, 0.50]},
        {"name": "precision_yaw_right", "delta": [-0.014, -0.004, 0.006], "rpy": [0.0, 0.0, -0.50]},
        {"name": "precision_high", "delta": [0.0, -0.004, 0.016], "rpy": [0.35, 0.0, 0.0]},
    ]


def _candidate_families(spec: ObjectSpec) -> list[dict[str, Any]]:
    if spec.family == "box_power_envelope":
        physical_profiles = [
            [0.35, 0.35, 0.55, 0.58, 0.65, 1.00],
            [0.50, 0.50, 0.62, 0.66, 0.70, 1.00],
            [0.65, 0.65, 0.70, 0.74, 0.74, 0.90],
            [0.80, 0.80, 0.78, 0.82, 0.76, 0.75],
        ]
        z_drops = [-0.002, -0.006, -0.012, -0.020]
        lateral_offsets = [[0.0, 0.0, 0.0], [0.010, 0.0, 0.0], [-0.010, 0.0, 0.0], [0.0, 0.010, 0.0], [0.0, -0.010, 0.0]]
    elif spec.family == "cylinder_power_envelope":
        physical_profiles = [
            [0.50, 0.50, 0.50, 0.50, 0.55, 1.00],
            [0.65, 0.65, 0.65, 0.65, 0.55, 1.00],
            [0.80, 0.80, 0.80, 0.80, 0.65, 1.00],
            [0.65, 0.65, 0.65, 0.65, 0.75, 0.65],
            [0.50, 0.50, 0.50, 0.50, 0.75, 0.55],
        ]
        z_drops = [-0.006, -0.014, -0.024, -0.034]
        lateral_offsets = [[0.0, 0.0, 0.0], [0.010, 0.0, 0.0], [-0.010, 0.0, 0.0], [0.0, 0.010, 0.0]]
    elif spec.family == "thin_cylinder_tripod":
        physical_profiles = [
            [0.10, 0.10, 0.52, 0.58, 0.66, 1.00],
            [0.25, 0.25, 0.60, 0.66, 0.70, 1.00],
            [0.45, 0.45, 0.62, 0.68, 0.72, 0.85],
            [0.65, 0.65, 0.65, 0.70, 0.75, 0.70],
        ]
        z_drops = [-0.004, -0.010, -0.016, -0.024]
        lateral_offsets = [[0.0, 0.0, 0.0], [0.006, 0.0, 0.0], [-0.006, 0.0, 0.0], [0.0, 0.006, 0.0], [0.0, -0.006, 0.0]]
    elif spec.family == "sphere_containment":
        physical_profiles = [
            [0.45, 0.45, 0.55, 0.60, 0.68, 1.00],
            [0.60, 0.60, 0.62, 0.68, 0.72, 0.95],
            [0.72, 0.72, 0.70, 0.74, 0.76, 0.85],
            [0.82, 0.82, 0.78, 0.80, 0.78, 0.70],
        ]
        z_drops = [-0.008, -0.016, -0.024, -0.032]
        lateral_offsets = [[0.0, 0.0, 0.0], [0.007, 0.0, 0.0], [-0.007, 0.0, 0.0], [0.0, 0.007, 0.0], [0.0, -0.007, 0.0]]
    else:
        physical_profiles = [
            [0.00, 0.00, 0.42, 0.45, 0.60, 1.00],
            [0.00, 0.00, 0.50, 0.52, 0.65, 1.00],
            [0.10, 0.10, 0.55, 0.60, 0.68, 1.00],
            [0.20, 0.20, 0.60, 0.65, 0.70, 1.00],
            [0.55, 0.55, 0.65, 0.70, 0.62, 0.75],
        ]
        z_drops = [-0.004, -0.010, -0.016, -0.024]
        lateral_offsets = [[0.0, 0.0, 0.0], [0.008, 0.0, 0.0], [-0.008, 0.0, 0.0], [0.0, 0.008, 0.0], [0.0, -0.008, 0.0]]

    candidates: list[dict[str, Any]] = []
    width_margin = spec.planar_width + 2.0 * spec.collision_padding + 0.008
    wrist_poses = _wrist_pose_deltas(spec)
    for profile_idx, physical in enumerate(physical_profiles):
        for wrist_idx, wrist_pose in enumerate(wrist_poses):
            for z_idx, z_drop in enumerate(z_drops):
                for offset_idx, offset in enumerate(lateral_offsets):
                    candidates.append(
                        {
                            "name": f"{spec.family}_p{profile_idx}_w{wrist_idx}_z{z_idx}_o{offset_idx}",
                            "family": spec.family,
                            "wrist_pose_name": wrist_pose["name"],
                            "wrist_delta": wrist_pose["delta"],
                            "wrist_rpy": wrist_pose["rpy"],
                            "object_width_m": spec.planar_width,
                            "target_proxy_width_m": width_margin,
                            "physical_rotate_norm": [0.0, 0.0, 0.0, 0.0, 0.0, physical[5]],
                            "physical_close_norm": physical,
                            "object_offset": offset,
                            "z_drop": z_drop,
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
        "total": int(data.ncon),
    }
    for idx in range(data.ncon):
        c = data.contact[idx]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom1)) or "",
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(c.geom2)) or "",
        ]
        joined = " ".join(names)
        if "bench_object" in joined and "thumb" in joined:
            counts["object_thumb"] += 1
        elif "bench_object" in joined and "index" in joined:
            counts["object_index"] += 1
        elif "bench_object" in joined and "middle" in joined:
            counts["object_middle"] += 1
        elif "bench_object" in joined and ("ring" in joined or "pinky" in joined):
            counts["object_ring_pinky"] += 1
        if "bench_object" in joined and "bench_table" in joined:
            counts["object_table"] += 1
        if "rh56_R_" in joined and "bench_table" in joined:
            counts["hand_table"] += 1
        if names[0].startswith("rh56_R_") and names[1].startswith("rh56_R_"):
            counts["hand_self"] += 1
    return counts


def _opposing_contact(contacts: dict[str, int]) -> bool:
    return contacts["object_thumb"] > 0 and (
        contacts["object_index"] > 0 or contacts["object_middle"] > 0 or contacts["object_ring_pinky"] > 0
    )


def _family_contact_quality(family: str, contacts: dict[str, int]) -> tuple[bool, float]:
    thumb = contacts["object_thumb"]
    index = contacts["object_index"]
    middle = contacts["object_middle"]
    ring_pinky = contacts["object_ring_pinky"]
    object_contacts = thumb + index + middle + ring_pinky
    if family == "box_precision_pinch":
        ok = thumb > 0 and (index > 0 or middle > 0) and ring_pinky <= 1 and object_contacts <= 9
        score = 2.5 * min(thumb, 2) + 2.0 * min(index + middle, 3) - 1.5 * ring_pinky - 0.8 * max(0, object_contacts - 7)
        return ok, score
    if "cylinder" in family or "power" in family or "sphere" in family:
        ok = thumb > 0 and (index + middle + ring_pinky) >= 2
        score = 1.5 * min(thumb, 3) + min(index + middle, 4) + 0.8 * min(ring_pinky, 4)
        return ok, score
    ok = _opposing_contact(contacts)
    score = 2.0 * thumb + 2.0 * index + 2.0 * middle + ring_pinky
    return ok, score


def _classify_failure(result: dict[str, Any]) -> str:
    if result["success"]:
        return "success"
    final_contacts = result["final_contacts"]
    if result["initial_penetration"]:
        return "initial_penetration"
    if final_contacts["hand_table"] > 0:
        return "table_collision"
    if result["max_xy_displacement_m"] > 0.055 and result["max_lift_m"] < result["success_lift_m"]:
        return "pushed_away"
    if not result["family_contact_ok"]:
        if result["max_lift_m"] >= result["success_lift_m"] and final_contacts["object_table"] > 0:
            return "slip_out"
        return "no_opposing_contact"
    if final_contacts["object_table"] > 0:
        return "object_still_on_table"
    if result["max_lift_m"] >= result["success_lift_m"] and result["lift_m"] < result["success_lift_m"]:
        return "slip_out"
    return "insufficient_lift"


def _run_candidate(
    xml_path: Path,
    approach_q: np.ndarray,
    grasp_q: np.ndarray,
    lift_q: np.ndarray,
    rotate_ctrl: np.ndarray,
    close_ctrl: np.ndarray,
    family: str,
    duration: float,
    success_lift_m: float,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATORS)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATORS)
    object_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bench_object_body")
    data.qpos[:6] = approach_q
    data.ctrl[arm_ids] = approach_q
    data.ctrl[hand_ids] = np.zeros(6)
    mujoco.mj_forward(model, data)
    initial_pos = data.xpos[object_body].copy()
    initial_z = float(data.xpos[object_body][2])
    max_z = initial_z
    max_xy_displacement = 0.0
    initial_contacts = _contact_summary(model, data)
    initial_penetration = bool(
        initial_contacts["hand_table"] > 0
        or initial_contacts["object_thumb"] > 0
        or initial_contacts["object_index"] > 0
        or initial_contacts["object_middle"] > 0
        or initial_contacts["object_ring_pinky"] > 0
    )

    contact_log: list[dict[str, Any]] = []
    while data.time < duration:
        t = float(data.time)
        if t < 0.45:
            arm = approach_q
            hand = np.zeros(6)
        elif t < 0.95:
            alpha = (t - 0.45) / 0.50
            arm = (1.0 - alpha) * approach_q + alpha * grasp_q
            hand = rotate_ctrl
        elif t < 1.70:
            arm = grasp_q
            alpha = (t - 0.95) / 0.75
            hand = (1.0 - alpha) * rotate_ctrl + alpha * (0.72 * close_ctrl + 0.28 * rotate_ctrl)
        elif t < 2.70:
            arm = grasp_q
            alpha = (t - 1.70) / 1.00
            hand = (1.0 - alpha) * (0.72 * close_ctrl + 0.28 * rotate_ctrl) + alpha * close_ctrl
        else:
            alpha = min(1.0, (t - 2.70) / max(0.50, duration - 2.70))
            arm = (1.0 - alpha) * grasp_q + alpha * lift_q
            hand = close_ctrl
        data.ctrl[arm_ids] = arm
        data.ctrl[hand_ids] = hand
        mujoco.mj_step(model, data)
        current_pos = data.xpos[object_body].copy()
        max_z = max(max_z, float(current_pos[2]))
        max_xy_displacement = max(max_xy_displacement, float(np.linalg.norm(current_pos[:2] - initial_pos[:2])))
        if int(data.time / 0.25) != int((data.time - model.opt.timestep) / 0.25):
            contact_log.append(
                {
                    "time": round(float(data.time), 3),
                    "object_pos": data.xpos[object_body].round(5).tolist(),
                    "contacts": _contact_summary(model, data),
                }
            )

    final_pos = data.xpos[object_body].copy()
    final_contacts = _contact_summary(model, data)
    contact_ok, contact_quality = _family_contact_quality(family, final_contacts)
    lift_m = float(final_pos[2] - initial_z)
    success = bool(
        lift_m >= success_lift_m
        and contact_ok
        and final_contacts["object_table"] == 0
        and not initial_penetration
    )
    score = (
        100.0 * lift_m
        + contact_quality
        - 2.0 * final_contacts["object_table"]
        - final_contacts["hand_table"]
        - (25.0 if initial_penetration else 0.0)
    )
    result = {
        "success": success,
        "score": float(score),
        "initial_object_pos": initial_pos.round(6).tolist(),
        "initial_z": initial_z,
        "final_object_pos": final_pos.round(6).tolist(),
        "lift_m": lift_m,
        "max_lift_m": float(max_z - initial_z),
        "max_xy_displacement_m": max_xy_displacement,
        "success_lift_m": success_lift_m,
        "initial_contacts": initial_contacts,
        "initial_penetration": initial_penetration,
        "opposing_contact": _opposing_contact(final_contacts),
        "family_contact_ok": contact_ok,
        "family_contact_quality": contact_quality,
        "final_contacts": final_contacts,
        "contact_log": contact_log,
    }
    result["failure_mode"] = _classify_failure(result)
    return result


def _replay_record(object_name: str, spec: ObjectSpec, candidate: dict[str, Any], rank: int) -> dict[str, Any]:
    result = candidate["result"]
    return {
        "schema": "rh56_handref_replay_v0",
        "object": object_name,
        "dataset": spec.dataset,
        "dataset_id": spec.dataset_id,
        "family": candidate["family"],
        "rank": rank,
        "candidate_name": candidate["name"],
        "xml": candidate["xml"],
        "success": result["success"],
        "failure_mode": result["failure_mode"],
        "lift_m": result["lift_m"],
        "max_lift_m": result["max_lift_m"],
        "max_xy_displacement_m": result["max_xy_displacement_m"],
        "wrist_pose_name": candidate["wrist_pose_name"],
        "wrist_delta": candidate["wrist_delta"],
        "wrist_rpy": candidate["wrist_rpy"],
        "object_pos": candidate["object_pos"],
        "table_top_z": candidate["table_top_z"],
        "grasp_q": candidate["grasp_q"],
        "approach_q": candidate["approach_q"],
        "lift_q": candidate["lift_q"],
        "physical_rotate_raw": candidate["physical_rotate_raw"],
        "physical_close_raw": candidate["physical_close_raw"],
        "physical_close_norm": candidate["physical_close_norm"],
        "rotate_ctrl_mujoco": candidate["rotate_ctrl_mujoco"],
        "close_ctrl_mujoco": candidate["close_ctrl_mujoco"],
        "ik_error_m": candidate["ik_error_m"],
        "ik_rot_error": candidate["ik_rot_error"],
        "final_contacts": result["final_contacts"],
        "initial_contacts": result["initial_contacts"],
    }


def _write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.write_text("\n".join(json.dumps(row, sort_keys=True) for row in rows) + ("\n" if rows else ""), encoding="utf-8")


def _write_baseline_v0(out_dir: Path, summary: dict[str, Any], replay_records: list[dict[str, Any]]) -> None:
    baseline_dir = out_dir / "baseline_v0"
    baseline_dir.mkdir(parents=True, exist_ok=True)
    objects: dict[str, Any] = {}
    for object_name, object_summary in summary["objects"].items():
        if not object_summary["top_candidates"]:
            continue
        best = object_summary["top_candidates"][0]
        objects[object_name] = {
            "object": object_name,
            "family": object_summary["spec"]["family"],
            "success": best["result"]["success"],
            "failure_mode": best["result"]["failure_mode"],
            "lift_m": best["result"]["lift_m"],
            "max_lift_m": best["result"]["max_lift_m"],
            "candidate_name": best["name"],
            "wrist_pose_name": best["wrist_pose_name"],
            "wrist_delta": best["wrist_delta"],
            "wrist_rpy": best["wrist_rpy"],
            "xml": best["xml"],
            "grasp_q": best["grasp_q"],
            "approach_q": best["approach_q"],
            "lift_q": best["lift_q"],
            "physical_close_raw": best["physical_close_raw"],
            "physical_close_norm": best["physical_close_norm"],
            "final_contacts": best["result"]["final_contacts"],
        }
    baseline = {
        "schema": "rh56_baseline_v0",
        "method": summary["method"],
        "object_dataset": summary["object_dataset"],
        "success_lift_m": summary["success_lift_m"],
        "table_top_z_m": summary["table_top_z_m"],
        "objects": objects,
    }
    (baseline_dir / "baseline_summary.json").write_text(json.dumps(baseline, indent=2), encoding="utf-8")
    baseline_presets = {
        "metadata": {name: {k: v for k, v in item.items() if k not in {"grasp_q", "approach_q", "lift_q"}} for name, item in objects.items()},
        "arm_q": {name: {"approach": item["approach_q"], "grasp": item["grasp_q"], "lift": item["lift_q"]} for name, item in objects.items()},
        "hand_raw": {name: item["physical_close_raw"] for name, item in objects.items()},
    }
    (baseline_dir / "presets.yaml").write_text(yaml.safe_dump(baseline_presets, sort_keys=True), encoding="utf-8")
    _write_jsonl(baseline_dir / "success_replays.jsonl", [row for row in replay_records if row["success"]])


def run_planner(args: argparse.Namespace) -> dict[str, Any]:
    robot_cfg = _load_yaml(Path(args.robot_config))
    seed_grasp_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    seed_hand_base_pos, seed_hand_base_rot = _hand_base_pose(Path(args.base_xml), seed_grasp_q)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    object_names = DEFAULT_OBJECTS if args.objects == ["all"] else [OBJECT_ALIASES.get(name, name) for name in args.objects]

    summary: dict[str, Any] = {
        "method": "object-conditioned wrist pose sampling + hand-ref staged hybrid close + MuJoCo validation",
        "object_dataset": "YCB canonical object subset with analytic MuJoCo collision proxies",
        "base_xml": str(args.base_xml),
        "seed_arm_preset": args.arm_preset,
        "approach_dy_m": args.approach_dy,
        "approach_distance_m": args.approach_distance,
        "object_center_xy_m": [args.object_x, args.object_y],
        "table_top_z_m": args.table_height,
        "lift_dz_m": args.lift_dz,
        "success_lift_m": args.success_lift,
        "objects": {},
    }
    preset_export: dict[str, Any] = {"gesture_presets": {}, "metadata": {}}
    replay_records: list[dict[str, Any]] = []

    for object_name in object_names:
        if object_name not in OBJECTS:
            raise ValueError(f"Unknown object {object_name}; choices={sorted(OBJECTS)}")
        spec = OBJECTS[object_name]
        object_dir = out_dir / object_name
        object_dir.mkdir(parents=True, exist_ok=True)
        results: list[dict[str, Any]] = []
        for candidate in _candidate_families(spec)[: args.max_candidates]:
            object_pos_unshifted = _fixed_object_pos(spec, args.table_clearance, (args.object_x, args.object_y))
            reference_object_center = _object_center_from_plan(Path(args.base_xml), seed_grasp_q, spec, candidate)
            target_hand_pos = (
                object_pos_unshifted
                + seed_hand_base_pos
                - reference_object_center
                + np.asarray(candidate["wrist_delta"], dtype=np.float64)
            )
            target_hand_rot = seed_hand_base_rot @ _rpy_matrix(candidate["wrist_rpy"])
            grasp_q, ik_error, ik_rot_error = _solve_hand_pose_target_q(
                Path(args.base_xml),
                seed_grasp_q,
                target_hand_pos,
                target_hand_rot,
            )
            approach_q = _solve_radial_approach_q(
                Path(args.base_xml),
                grasp_q,
                target_hand_pos,
                object_pos_unshifted,
                args.approach_distance,
                args.approach_dy,
            )
            lift_q = _solve_lift_q(Path(args.base_xml), grasp_q, args.lift_dz)
            planned_table_top_z = float(object_pos_unshifted[2] - spec.half_height - args.table_clearance)
            scene_z_offset = float(args.table_height - planned_table_top_z)
            object_pos = object_pos_unshifted + np.asarray([0.0, 0.0, scene_z_offset], dtype=np.float64)
            table_top_z = float(args.table_height)
            xml_path = object_dir / f"{candidate['name']}.xml"
            _write_scene_xml(Path(args.base_xml), xml_path, spec, object_pos, table_top_z, scene_z_offset)
            rotate_ctrl = _physical_norm_to_mujoco_ctrl(candidate["physical_rotate_norm"])
            close_ctrl = _physical_norm_to_mujoco_ctrl(candidate["physical_close_norm"])
            result = _run_candidate(
                xml_path,
                approach_q,
                grasp_q,
                lift_q,
                rotate_ctrl,
                close_ctrl,
                candidate["family"],
                duration=args.duration,
                success_lift_m=args.success_lift,
            )
            candidate_score = float(result["score"] - 60.0 * ik_error - 6.0 * ik_rot_error)
            results.append(
                {
                    **candidate,
                    "xml": str(xml_path),
                    "object_pos": object_pos.round(6).tolist(),
                    "object_pos_unshifted": object_pos_unshifted.round(6).tolist(),
                    "reference_object_center": reference_object_center.round(6).tolist(),
                    "target_hand_base_pos": target_hand_pos.round(6).tolist(),
                    "target_hand_base_rot": target_hand_rot.round(6).tolist(),
                    "ik_error_m": ik_error,
                    "ik_rot_error": ik_rot_error,
                    "candidate_score": candidate_score,
                    "table_top_z": table_top_z,
                    "planned_table_top_z": planned_table_top_z,
                    "scene_z_offset": scene_z_offset,
                    "physical_rotate_raw": _physical_norm_to_raw(candidate["physical_rotate_norm"]),
                    "physical_close_raw": _physical_norm_to_raw(candidate["physical_close_norm"]),
                    "grasp_q": grasp_q.round(6).tolist(),
                    "approach_q": approach_q.round(6).tolist(),
                    "lift_q": lift_q.round(6).tolist(),
                    "rotate_ctrl_mujoco": rotate_ctrl.round(6).tolist(),
                    "close_ctrl_mujoco": close_ctrl.round(6).tolist(),
                    "result": result,
                }
            )
        results.sort(key=lambda item: (item["result"]["success"], item["candidate_score"]), reverse=True)
        replay_records.extend(_replay_record(object_name, spec, item, rank) for rank, item in enumerate(results))
        object_summary = {
            "spec": {
                "dataset": spec.dataset,
                "dataset_id": spec.dataset_id,
                "display_name": spec.display_name,
                "geom_type": spec.geom_type,
                "visual_size": list(spec.visual_size),
                "collision_size": list(spec.collision_size),
                "collision_padding_m": spec.collision_padding,
                "mass": spec.mass,
                "family": spec.family,
                "planar_width_m": spec.planar_width,
            },
            "num_candidates": len(results),
            "num_success": sum(1 for item in results if item["result"]["success"]),
            "top_candidates": results[: min(10, len(results))],
        }
        (object_dir / "candidates.json").write_text(json.dumps(results, indent=2), encoding="utf-8")
        (object_dir / "summary.json").write_text(json.dumps(object_summary, indent=2), encoding="utf-8")
        summary["objects"][object_name] = object_summary
        if results:
            best = results[0]
            preset_export["gesture_presets"][f"{object_name}_handref_rotate"] = best["physical_rotate_raw"]
            preset_export["gesture_presets"][f"{object_name}_handref_close"] = best["physical_close_raw"]
            preset_export["metadata"][object_name] = {
                "family": best["family"],
                "success": best["result"]["success"],
                "lift_m": best["result"]["lift_m"],
                "wrist_pose_name": best["wrist_pose_name"],
                "wrist_delta": best["wrist_delta"],
                "wrist_rpy": best["wrist_rpy"],
                "object_offset": best["object_offset"],
                "z_drop": best["z_drop"],
                "ik_error_m": best["ik_error_m"],
                "ik_rot_error": best["ik_rot_error"],
                "expected_contacts": best["result"]["final_contacts"],
            }

    (out_dir / "benchmark_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (out_dir / "handref_presets.yaml").write_text(yaml.safe_dump(preset_export, sort_keys=True), encoding="utf-8")
    _write_jsonl(out_dir / "replay_dataset.jsonl", replay_records)
    _write_baseline_v0(out_dir, summary, replay_records)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="RH56 hand-ref analytical grasp planner in MuJoCo.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default=str(ROBOT_CONFIG))
    parser.add_argument("--out-dir", default=str(OUT_DIR))
    object_choices = sorted([*OBJECTS, *OBJECT_ALIASES])
    parser.add_argument("--objects", nargs="+", default=["all"], help=f"Object names or all. Choices: {object_choices}")
    parser.add_argument("--arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--approach-dy", type=float, default=0.0)
    parser.add_argument("--approach-distance", type=float, default=0.0)
    parser.add_argument("--lift-dz", type=float, default=0.120)
    parser.add_argument("--success-lift", type=float, default=0.050)
    parser.add_argument("--object-x", type=float, default=OBJECT_CENTER_XY[0])
    parser.add_argument("--object-y", type=float, default=OBJECT_CENTER_XY[1])
    parser.add_argument("--table-height", type=float, default=TABLE_TOP_Z, help="World z of the tabletop surface in meters.")
    parser.add_argument("--table-clearance", type=float, default=0.004)
    parser.add_argument("--max-candidates", type=int, default=80)
    args = parser.parse_args()

    summary = run_planner(args)
    compact = {
        "out_dir": args.out_dir,
        "objects": {
            name: {
                "family": item["spec"]["family"],
                "num_success": item["num_success"],
                "best_success": item["top_candidates"][0]["result"]["success"] if item["top_candidates"] else None,
                "best_lift_m": item["top_candidates"][0]["result"]["lift_m"] if item["top_candidates"] else None,
                "best_wrist_pose": item["top_candidates"][0]["wrist_pose_name"] if item["top_candidates"] else None,
                "best_ik_error_m": item["top_candidates"][0]["ik_error_m"] if item["top_candidates"] else None,
                "best_ik_rot_error": item["top_candidates"][0]["ik_rot_error"] if item["top_candidates"] else None,
                "best_close_norm": item["top_candidates"][0]["physical_close_norm"] if item["top_candidates"] else None,
                "best_close_raw": item["top_candidates"][0]["physical_close_raw"] if item["top_candidates"] else None,
                "best_contacts": item["top_candidates"][0]["result"]["final_contacts"] if item["top_candidates"] else None,
            }
            for name, item in summary["objects"].items()
        },
    }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
