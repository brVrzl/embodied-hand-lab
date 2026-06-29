from __future__ import annotations

import argparse
import dataclasses
import importlib
import json
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from embodiment_core.config import load_yaml
from jaka_driver_adapter.palm_target_ik import DEFAULT_MJCF, PalmTargetIkState
from teleop_tools.hebi_mobile_io import HebiMobileIOClient, HebiMobileIOSnapshot, quat_conjugate_wxyz, rotate_vector_wxyz
from teleop_tools.hebi_rviz_shadow import _actual_palm_pose_from_state, _relative_config_from_config
from teleop_tools.relative_pose_lag_follow import RelativePoseLagFollower


DEFAULT_CONFIG = "configs/teleop/hebi_mobile_io_jaka_rh56.yaml"
PALM_DOWN_QUATERNION_WXYZ = (0.7071067811865476, -0.7071067811865475, 0.0, 0.0)
TARGET_MARKER_BODY = "hebi_shadow_target_marker"
IK_TARGET_MARKER_BODY = "hebi_shadow_ik_target_marker"
PHONE_CAMERA_RAY_BODY = "hebi_shadow_phone_camera_ray"
PHONE_AXIS_RAY_BODIES = {
    "+X": "hebi_shadow_phone_axis_x_ray",
    "-X": "hebi_shadow_phone_axis_neg_x_ray",
    "+Y": "hebi_shadow_phone_axis_y_ray",
    "-Y": "hebi_shadow_phone_axis_neg_y_ray",
    "+Z": "hebi_shadow_phone_axis_z_ray",
    "-Z": "hebi_shadow_phone_axis_neg_z_ray",
}
TARGET_MARKER_XML = Path("data/mujoco_debug/hebi_mujoco_shadow_target.xml")


def _ensure_marker_axes(
    body: ET.Element,
    *,
    prefix: str,
    length: float,
    radius: float,
    alpha: float,
) -> None:
    axes = {
        "x": ("1.0 0.05 0.05", f"0 0 0 {length} 0 0"),
        "y": ("0.05 0.85 0.15", f"0 0 0 0 {length} 0"),
        "z": ("0.15 0.35 1.0", f"0 0 0 0 0 {length}"),
    }
    existing = {geom.get("name") for geom in body.findall("geom")}
    for axis, (rgb, fromto) in axes.items():
        name = f"{prefix}_{axis}_axis"
        if name in existing:
            continue
        ET.SubElement(
            body,
            "geom",
            {
                "name": name,
                "type": "capsule",
                "fromto": fromto,
                "size": f"{radius:.4f}",
                "rgba": f"{rgb} {alpha:.2f}",
                "group": "5",
                "contype": "0",
                "conaffinity": "0",
            },
        )


def _ensure_grid_floor(root: ET.Element, worldbody: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    if not any(texture.get("name") == "hebi_shadow_floor_grid" for texture in asset.findall("texture")):
        ET.SubElement(
            asset,
            "texture",
            {
                "name": "hebi_shadow_floor_grid",
                "type": "2d",
                "builtin": "checker",
                "width": "512",
                "height": "512",
                "rgb1": "0.86 0.86 0.86",
                "rgb2": "0.58 0.58 0.58",
            },
        )
    if not any(material.get("name") == "hebi_shadow_floor_grid_mat" for material in asset.findall("material")):
        ET.SubElement(
            asset,
            "material",
            {
                "name": "hebi_shadow_floor_grid_mat",
                "texture": "hebi_shadow_floor_grid",
                "texrepeat": "12 12",
                "reflectance": "0.05",
                "rgba": "1 1 1 1",
            },
        )
    floor = next((geom for geom in worldbody.findall("geom") if geom.get("name") == "floor"), None)
    if floor is not None:
        floor.set("material", "hebi_shadow_floor_grid_mat")
        floor.set("rgba", "1 1 1 1")


def _remove_marker_axes(body: ET.Element, *, prefix: str) -> None:
    for geom in list(body.findall("geom")):
        name = geom.get("name", "")
        if name.startswith(f"{prefix}_") and name.endswith("_axis"):
            body.remove(geom)


def _build_target_marker_xml(base_xml: str | Path, out_xml: str | Path) -> Path:
    base_xml = Path(base_xml).resolve()
    out_xml = Path(out_xml).resolve()
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(base_xml.parent))
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MJCF is missing <worldbody>.")
    _ensure_grid_floor(root, worldbody)
    target_marker = next(
        (body for body in worldbody.iter("body") if body.get("name") == TARGET_MARKER_BODY),
        None,
    )
    if target_marker is None:
        marker = ET.SubElement(
            worldbody,
            "body",
            {
                "name": TARGET_MARKER_BODY,
                "mocap": "true",
                "pos": "0 0 0",
            },
        )
        target_marker = marker
        ET.SubElement(
            marker,
            "geom",
            {
                "name": "hebi_shadow_target_marker_sphere",
                "type": "sphere",
                "size": "0.018",
                "rgba": "0.0 0.55 1.0 0.85",
                "group": "5",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    _remove_marker_axes(target_marker, prefix="hebi_shadow_target_marker")
    ik_target_marker = next(
        (body for body in worldbody.iter("body") if body.get("name") == IK_TARGET_MARKER_BODY),
        None,
    )
    if ik_target_marker is None:
        marker = ET.SubElement(
            worldbody,
            "body",
            {
                "name": IK_TARGET_MARKER_BODY,
                "mocap": "true",
                "pos": "0 0 0",
            },
        )
        ik_target_marker = marker
        ET.SubElement(
            marker,
            "geom",
            {
                "name": "hebi_shadow_ik_target_marker_sphere",
                "type": "sphere",
                "size": "0.012",
                "rgba": "0.1 0.9 0.25 0.9",
                "group": "5",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    _remove_marker_axes(ik_target_marker, prefix="hebi_shadow_ik_target_marker")
    if not any(body.get("name") == PHONE_CAMERA_RAY_BODY for body in worldbody.iter("body")):
        marker = ET.SubElement(
            worldbody,
            "body",
            {
                "name": PHONE_CAMERA_RAY_BODY,
                "mocap": "true",
                "pos": "0 0 0",
            },
        )
        ET.SubElement(
            marker,
            "geom",
            {
                "name": "hebi_shadow_phone_camera_ray_capsule",
                "type": "capsule",
                "fromto": "0 0 0 0 0 0.16",
                "size": "0.005",
                "rgba": "1.0 0.85 0.05 0.9",
                "group": "5",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    axis_styles = {
        "+X": ("0.55 0.0 0.0 0.90", "0.14"),
        "-X": ("1.0 0.1 0.1 0.45", "0.08"),
        "+Y": ("0.0 0.45 0.1 0.90", "0.14"),
        "-Y": ("0.1 0.9 0.2 0.45", "0.08"),
        "+Z": ("0.15 0.35 1.0 0.85", "0.12"),
        "-Z": ("0.0 0.12 0.55 0.65", "0.09"),
    }
    for axis_name, body_name in PHONE_AXIS_RAY_BODIES.items():
        if any(body.get("name") == body_name for body in worldbody.iter("body")):
            continue
        rgba, length = axis_styles[axis_name]
        marker = ET.SubElement(
            worldbody,
            "body",
            {
                "name": body_name,
                "mocap": "true",
                "pos": "0 0 0",
            },
        )
        ET.SubElement(
            marker,
            "geom",
            {
                "name": f"{body_name}_capsule",
                "type": "capsule",
                "fromto": f"0 0 0 0 0 {length}",
                "size": "0.0035",
                "rgba": rgba,
                "group": "5",
                "contype": "0",
                "conaffinity": "0",
            },
        )
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    return out_xml


def _unit_vector(values: list[float] | np.ndarray) -> np.ndarray:
    vector = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        return np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    return vector / norm


def _quat_align_z_to_vector_wxyz(direction: list[float] | np.ndarray) -> np.ndarray:
    target = _unit_vector(direction)
    source = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if dot > 1.0 - 1e-9:
        return np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    if dot < -1.0 + 1e-9:
        return np.asarray([0.0, 1.0, 0.0, 0.0], dtype=np.float64)
    axis = np.cross(source, target)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-9)
    angle = float(np.arccos(dot))
    half = angle * 0.5
    return np.asarray([np.cos(half), *(axis * np.sin(half))], dtype=np.float64)


def _axis_from_name(name: str) -> tuple[float, float, float]:
    normalized = name.strip().lower()
    axes = {
        "+x": (1.0, 0.0, 0.0),
        "x": (1.0, 0.0, 0.0),
        "-x": (-1.0, 0.0, 0.0),
        "+y": (0.0, 1.0, 0.0),
        "y": (0.0, 1.0, 0.0),
        "-y": (0.0, -1.0, 0.0),
        "+z": (0.0, 0.0, 1.0),
        "z": (0.0, 0.0, 1.0),
        "-z": (0.0, 0.0, -1.0),
    }
    if normalized not in axes:
        raise ValueError(f"Unsupported phone camera axis {name!r}. Use one of +X,-X,+Y,-Y,+Z,-Z.")
    return axes[normalized]


def _phone_camera_axis_from_config(config: dict[str, Any]) -> tuple[float, float, float]:
    relative_cfg = config.get("relative_pose_lag_follow", {})
    return tuple(float(v) for v in relative_cfg.get("phone_back_camera_axis", [0.0, 0.0, -1.0]))


def _phone_axis_direction_world(
    phone_quaternion_wxyz: list[float],
    phone_axis: tuple[float, float, float],
    *,
    convention: str,
    project_to_floor: bool = False,
) -> np.ndarray:
    quat = (
        quat_conjugate_wxyz(phone_quaternion_wxyz)
        if convention == "world-to-phone"
        else phone_quaternion_wxyz
    )
    direction = np.asarray(rotate_vector_wxyz(quat, phone_axis), dtype=np.float64)
    if project_to_floor:
        horizontal = direction.copy()
        horizontal[2] = 0.0
        if float(np.linalg.norm(horizontal)) > 1e-6:
            direction = horizontal
    return _unit_vector(direction)


def _phone_back_camera_direction_world(
    phone_quaternion_wxyz: list[float],
    phone_back_camera_axis: tuple[float, float, float],
    *,
    convention: str,
) -> np.ndarray:
    return _phone_axis_direction_world(
        phone_quaternion_wxyz,
        phone_back_camera_axis,
        convention=convention,
    )


def _phone_to_world_quaternion(
    phone_quaternion_wxyz: list[float],
    *,
    convention: str,
) -> np.ndarray:
    return (
        quat_conjugate_wxyz(phone_quaternion_wxyz)
        if convention == "world-to-phone"
        else np.asarray(phone_quaternion_wxyz, dtype=np.float64)
    )


def _phone_local_delta_from_anchor(
    snapshot: HebiMobileIOSnapshot,
    anchor_position_m: np.ndarray,
    anchor_quaternion_wxyz: list[float],
    *,
    convention: str,
) -> np.ndarray:
    world_delta = np.asarray(snapshot.position_m, dtype=np.float64) - anchor_position_m
    phone_to_world = _phone_to_world_quaternion(anchor_quaternion_wxyz, convention=convention)
    return rotate_vector_wxyz(quat_conjugate_wxyz(phone_to_world), world_delta)


def _joint_axis_world(ik_state: PalmTargetIkState, joint_index: int) -> np.ndarray:
    dof_id = int(ik_state.arm_dof_ids[joint_index])
    return _unit_vector(ik_state.data.xaxis[dof_id])


def _tool_axes_from_joint6_red_axis(
    ik_state: PalmTargetIkState,
    reference_quaternion_wxyz: list[float] | np.ndarray | None,
) -> dict[str, np.ndarray]:
    red = _joint_axis_world(ik_state, 5)
    reference_quat = (
        ik_state.current_palm_quaternion_wxyz
        if reference_quaternion_wxyz is None
        else np.asarray(reference_quaternion_wxyz, dtype=np.float64)
    )
    blue_seed = np.asarray(rotate_vector_wxyz(reference_quat, [0.0, 0.0, 1.0]), dtype=np.float64)
    blue = blue_seed - red * float(np.dot(blue_seed, red))
    if float(np.linalg.norm(blue)) <= 1e-6:
        blue_seed = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
        if abs(float(np.dot(blue_seed, red))) > 0.95:
            blue_seed = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
        blue = blue_seed - red * float(np.dot(blue_seed, red))
    blue = _unit_vector(blue)
    green = _unit_vector(np.cross(blue, red))
    blue = _unit_vector(np.cross(red, green))
    return {
        "+X": red,
        "-X": -red,
        "+Y": green,
        "-Y": -green,
        "+Z": blue,
        "-Z": -blue,
    }


def _limit_arm_joint_motion(
    ik_state: PalmTargetIkState,
    previous_joints_rad: np.ndarray,
    previous_joint_velocity_rad_s: np.ndarray,
    *,
    dt: float,
    max_joint_velocity_rad_s: float,
    max_joint_acceleration_rad_s2: float,
) -> tuple[bool, list[float], np.ndarray]:
    velocity_limit = abs(float(max_joint_velocity_rad_s))
    acceleration_limit = abs(float(max_joint_acceleration_rad_s2))
    if velocity_limit <= 0.0 or dt <= 0.0:
        return False, ik_state.arm_joints_rad.tolist(), np.zeros_like(previous_joint_velocity_rad_s)
    dt = max(float(dt), 1e-3)
    desired_joints = ik_state.arm_joints_rad.copy()
    desired_velocity = (desired_joints - previous_joints_rad) / dt
    if acceleration_limit > 0.0 and np.isfinite(acceleration_limit):
        max_velocity_delta = acceleration_limit * dt
        desired_velocity = np.clip(
            desired_velocity,
            previous_joint_velocity_rad_s - max_velocity_delta,
            previous_joint_velocity_rad_s + max_velocity_delta,
        )
    if velocity_limit > 0.0 and np.isfinite(velocity_limit):
        desired_velocity = np.clip(desired_velocity, -velocity_limit, velocity_limit)
    limited_joints = previous_joints_rad + desired_velocity * dt
    limited = bool(np.any(np.abs(limited_joints - desired_joints) > 1e-9))
    if limited:
        ik_state.arm_joints_rad[:] = limited_joints
        ik_state._clip_arm_joints()
        ik_state._forward()
    return limited, desired_joints.astype(float).tolist(), desired_velocity


def _shadow_state_from_config(
    config: dict[str, Any],
    *,
    orientation_ik_weight: float,
    target_workspace_radius_m: float | None,
) -> PalmTargetIkState:
    shadow_cfg = config.get("shadow", {})
    mjcf_path = _build_target_marker_xml(
        shadow_cfg.get("mjcf_path", DEFAULT_MJCF),
        TARGET_MARKER_XML,
    )
    return PalmTargetIkState(
        mjcf_path=mjcf_path,
        initial_arm_joints_rad=shadow_cfg.get("initial_arm_joints_rad", [0.0] * 6),
        ik_gain=float(shadow_cfg.get("ik_gain", 0.7)),
        ik_damping=float(shadow_cfg.get("ik_damping", 0.05)),
        ik_max_step_rad=float(shadow_cfg.get("ik_max_step_rad", 0.08)),
        ik_iterations=int(shadow_cfg.get("ik_iterations", 20)),
        target_workspace_radius_m=(
            float(shadow_cfg.get("target_workspace_radius_m", 0.08))
            if target_workspace_radius_m is None
            else float(target_workspace_radius_m)
        ),
        orientation_ik_weight=orientation_ik_weight,
    )


def _relaxed_relative_config(config: dict[str, Any], args: argparse.Namespace):
    base = _relative_config_from_config(config)
    phone_back_camera_axis = (
        _axis_from_name(args.phone_camera_axis)
        if args.phone_camera_axis
        else base.phone_back_camera_axis
    )
    orientation_anchor = (
        PALM_DOWN_QUATERNION_WXYZ
        if args.orientation_anchor_mode == "flat-palm"
        else None
    )
    target_filter_time_constant_sec = (
        base.target_filter_time_constant_sec
        if args.target_filter_time_constant_sec is None
        else float(args.target_filter_time_constant_sec)
    )
    target_update_deadband_m = (
        base.target_update_deadband_m
        if args.target_update_deadband_m is None
        else float(args.target_update_deadband_m)
    )
    target_update_release_m = (
        base.target_update_release_m
        if args.target_update_release_m is None
        else float(args.target_update_release_m)
    )
    return dataclasses.replace(
        base,
        target_response_mode="direct",
        orientation_control_enabled=not bool(args.position_only),
        orientation_mapping_mode=str(args.orientation_mapping_mode or base.orientation_mapping_mode),
        orientation_scale=float(args.orientation_scale),
        phone_back_camera_axis=phone_back_camera_axis,
        phone_quaternion_convention=args.phone_quaternion_convention,
        orientation_anchor_quaternion_wxyz=orientation_anchor,
        position_scale=float(args.position_scale),
        workspace_min_m=(
            (-2.0, -2.0, -2.0)
            if args.workspace_mode == "unrestricted"
            else base.workspace_min_m
        ),
        workspace_max_m=(
            (2.0, 2.0, 2.0)
            if args.workspace_mode == "unrestricted"
            else base.workspace_max_m
        ),
        freeze_when_phone_still=not bool(args.disable_freeze_when_phone_still),
        max_pos_tracking_error_warn_m=10.0,
        max_pos_tracking_error_pause_m=10.0,
        max_rot_tracking_error_warn_rad=10.0,
        max_rot_tracking_error_pause_rad=10.0,
        max_q_tracking_error_pause_rad=10.0,
        phone_jump_reject_translation_m=float(args.phone_jump_reject_translation_m),
        phone_jump_reject_rotation_rad=float(args.phone_jump_reject_rotation_rad),
        max_target_velocity_m_s=float(args.max_target_velocity_m_s),
        max_target_acceleration_m_s2=float(args.max_target_acceleration_m_s2),
        max_target_jump_m=float(args.max_target_jump_m),
        target_filter_time_constant_sec=target_filter_time_constant_sec,
        target_update_deadband_m=target_update_deadband_m,
        target_update_release_m=target_update_release_m,
    )


def run_shadow(args: argparse.Namespace) -> dict[str, Any]:
    config = load_yaml(args.config)
    phone_back_camera_axis = (
        _axis_from_name(args.phone_camera_axis)
        if args.phone_camera_axis
            else _phone_camera_axis_from_config(config)
    )
    hebi_cfg = config.get("hebi", {})
    ik_state = _shadow_state_from_config(
        config,
        orientation_ik_weight=args.orientation_ik_weight,
        target_workspace_radius_m=args.target_workspace_radius_m,
    )
    follower = RelativePoseLagFollower(_relaxed_relative_config(config, args))
    client = HebiMobileIOClient(
        family=str(hebi_cfg.get("family", "HEBI")),
        name=str(hebi_cfg.get("name", "mobileIO")),
        lookup_wait_sec=float(hebi_cfg.get("lookup_wait_sec", 2.0)),
        setup_ui=bool(hebi_cfg.get("setup_ui", True)),
        max_stale_feedback_sec=float(hebi_cfg.get("max_stale_feedback_sec", 0.25)),
    )
    connect_state: dict[str, Any] = {
        "connected": False,
        "error": "waiting_for_hebi",
    }

    def _connect_hebi_background() -> None:
        connect_deadline = time.time() + max(0.0, float(args.wait_for_hebi_sec))
        while time.time() <= connect_deadline:
            try:
                client.connect()
                connect_state["connected"] = True
                connect_state["error"] = "ok"
                return
            except RuntimeError as exc:
                connect_state["error"] = str(exc)
                time.sleep(1.0)
        connect_state["error"] = "hebi_connect_timeout"

    threading.Thread(target=_connect_hebi_background, daemon=True).start()
    mujoco_viewer = importlib.import_module("mujoco.viewer")
    records: list[dict[str, Any]] = []
    q_current = ik_state.arm_joints_rad.tolist()
    last_joint_velocity_rad_s = np.zeros(6, dtype=np.float64)
    tool_position_anchor_m: np.ndarray | None = None
    tool_axes_anchor: dict[str, np.ndarray] | None = None
    phone_position_anchor_m: np.ndarray | None = None
    phone_quaternion_anchor_wxyz: list[float] | None = None
    start = time.time()
    last_tick = start
    tick_period = 1.0 / max(float(args.hz), 1.0)
    jsonl_path = Path(args.jsonl_out) if args.jsonl_out else None
    if jsonl_path is not None:
        jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        jsonl_path.write_text("", encoding="utf-8")

    with mujoco_viewer.launch_passive(
        ik_state.model,
        ik_state.data,
        show_left_ui=bool(args.show_left_ui),
        show_right_ui=bool(args.show_right_ui),
    ) as handle:
        target_body_id = mujoco.mj_name2id(ik_state.model, mujoco.mjtObj.mjOBJ_BODY, TARGET_MARKER_BODY)
        target_mocap_id = int(ik_state.model.body_mocapid[target_body_id]) if target_body_id >= 0 else -1
        ik_target_body_id = mujoco.mj_name2id(
            ik_state.model, mujoco.mjtObj.mjOBJ_BODY, IK_TARGET_MARKER_BODY
        )
        ik_target_mocap_id = (
            int(ik_state.model.body_mocapid[ik_target_body_id]) if ik_target_body_id >= 0 else -1
        )
        phone_camera_body_id = mujoco.mj_name2id(
            ik_state.model, mujoco.mjtObj.mjOBJ_BODY, PHONE_CAMERA_RAY_BODY
        )
        phone_camera_mocap_id = (
            int(ik_state.model.body_mocapid[phone_camera_body_id])
            if phone_camera_body_id >= 0
            else -1
        )
        phone_axis_mocap_ids = {}
        for axis_name, body_name in PHONE_AXIS_RAY_BODIES.items():
            body_id = mujoco.mj_name2id(ik_state.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
            phone_axis_mocap_ids[axis_name] = (
                int(ik_state.model.body_mocapid[body_id]) if body_id >= 0 else -1
            )
        handle.opt.geomgroup[5] = 1
        handle.cam.azimuth = -130
        handle.cam.elevation = -25
        handle.cam.distance = float(args.camera_distance)
        handle.cam.lookat[:] = [-0.18, 0.0, 0.20]
        while handle.is_running() and time.time() - start < float(args.duration_sec):
            now = time.time()
            if connect_state["connected"]:
                snapshot = client.read(timeout_ms=float(hebi_cfg.get("read_timeout_ms", 10.0)))
            else:
                snapshot = HebiMobileIOSnapshot(
                    timestamp_sec=now,
                    position_m=[0.0, 0.0, 0.0],
                    quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
                    valid=False,
                    reason=str(connect_state.get("error", "waiting_for_hebi")),
                )
            actual = _actual_palm_pose_from_state(ik_state)
            output = follower.step(snapshot, actual, q_current, timestamp_sec=now)
            dt = max(0.0, min(now - last_tick, 0.1))
            palm_target_position_m = output.palm_target_position_m
            position_mapping_mode = str(args.position_mapping_mode)
            phone_local_delta_m = None
            if (
                position_mapping_mode == "tool_frame"
                and output.command_deadman
                and follower.phone_anchor_pose is not None
            ):
                if (
                    tool_position_anchor_m is None
                    or tool_axes_anchor is None
                    or phone_position_anchor_m is None
                    or phone_quaternion_anchor_wxyz is None
                ):
                    tool_position_anchor_m = ik_state.current_palm_position_m.copy()
                    tool_axes_anchor = _tool_axes_from_joint6_red_axis(
                        ik_state,
                        output.palm_target_quaternion_wxyz,
                    )
                    phone_position_anchor_m = np.asarray(
                        follower.phone_anchor_pose.position_m,
                        dtype=np.float64,
                    )
                    phone_quaternion_anchor_wxyz = list(follower.phone_anchor_pose.quaternion_wxyz)
                phone_local_delta_m = _phone_local_delta_from_anchor(
                    snapshot,
                    phone_position_anchor_m,
                    phone_quaternion_anchor_wxyz,
                    convention=args.phone_quaternion_convention,
                )
                if tool_axes_anchor is not None:
                    palm_target_position_m = (
                        tool_position_anchor_m
                        + float(args.position_scale)
                        * (
                            phone_local_delta_m[0] * tool_axes_anchor["+X"]
                            + phone_local_delta_m[1] * tool_axes_anchor["+Y"]
                            + phone_local_delta_m[2] * tool_axes_anchor["+Z"]
                        )
                    ).astype(float).tolist()
            phone_camera_direction_world = _phone_back_camera_direction_world(
                snapshot.quaternion_wxyz,
                phone_back_camera_axis,
                convention=args.phone_quaternion_convention,
            )
            phone_axis_directions_world = _tool_axes_from_joint6_red_axis(
                ik_state,
                output.palm_target_quaternion_wxyz,
            )
            camera_ray_origin = (
                np.asarray(palm_target_position_m, dtype=np.float64)
                if palm_target_position_m is not None
                else ik_state.current_palm_position_m
            )
            if phone_camera_mocap_id >= 0:
                ik_state.data.mocap_pos[phone_camera_mocap_id] = camera_ray_origin
                ik_state.data.mocap_quat[phone_camera_mocap_id] = _quat_align_z_to_vector_wxyz(
                    phone_camera_direction_world
                )
            for axis_name, mocap_id in phone_axis_mocap_ids.items():
                if mocap_id < 0:
                    continue
                ik_state.data.mocap_pos[mocap_id] = camera_ray_origin
                ik_state.data.mocap_quat[mocap_id] = _quat_align_z_to_vector_wxyz(
                    phone_axis_directions_world[axis_name]
                )
            ik_joint_velocity_limited = False
            ik_desired_joints_before_velocity_limit = ik_state.arm_joints_rad.astype(float).tolist()
            if output.command_deadman and palm_target_position_m is not None:
                if target_mocap_id >= 0:
                    ik_state.data.mocap_pos[target_mocap_id] = palm_target_position_m
                    if output.palm_target_quaternion_wxyz is not None:
                        ik_state.data.mocap_quat[target_mocap_id] = output.palm_target_quaternion_wxyz
                previous_joints = ik_state.arm_joints_rad.copy()
                ik_state.apply_position_target(
                    palm_target_position_m=palm_target_position_m,
                    palm_target_quaternion_wxyz=(
                        None if args.position_only else output.palm_target_quaternion_wxyz
                    ),
                    wrist_roll_velocity_rad_s=0.0,
                    dt=dt,
                )
                (
                    ik_joint_velocity_limited,
                    ik_desired_joints_before_velocity_limit,
                    last_joint_velocity_rad_s,
                ) = _limit_arm_joint_motion(
                    ik_state,
                    previous_joints,
                    last_joint_velocity_rad_s,
                    dt=dt,
                    max_joint_velocity_rad_s=args.max_joint_velocity_rad_s,
                    max_joint_acceleration_rad_s2=args.max_joint_acceleration_rad_s2,
                )
                if ik_target_mocap_id >= 0:
                    ik_state.data.mocap_pos[ik_target_mocap_id] = ik_state.target_palm_position_m
                    ik_state.data.mocap_quat[ik_target_mocap_id] = ik_state.current_palm_quaternion_wxyz
                q_current = ik_state.arm_joints_rad.tolist()
            else:
                tool_position_anchor_m = None
                tool_axes_anchor = None
                phone_position_anchor_m = None
                phone_quaternion_anchor_wxyz = None
                if target_mocap_id >= 0:
                    ik_state.data.mocap_pos[target_mocap_id] = ik_state.current_palm_position_m
                    ik_state.data.mocap_quat[target_mocap_id] = ik_state.current_palm_quaternion_wxyz
                if ik_target_mocap_id >= 0:
                    ik_state.data.mocap_pos[ik_target_mocap_id] = ik_state.current_palm_position_m
                    ik_state.data.mocap_quat[ik_target_mocap_id] = ik_state.current_palm_quaternion_wxyz
                follower.reset()
                last_joint_velocity_rad_s[:] = 0.0
            handle.set_texts(
                (
                    None,
                    mujoco.mjtGridPos.mjGRID_TOPLEFT,
                    "HEBI",
                    (
                        f"B1={bool(snapshot.raw_inputs.get('b1', False))} "
                        f"deadman={output.command_deadman} "
                        f"reason={output.log.get('reason', 'ok')}"
                    ),
                )
            )
            handle.sync()
            record = {
                "elapsed_sec": round(now - start, 4),
                "dt": round(dt, 6),
                "deadman": output.command_deadman,
                "snapshot_valid": snapshot.valid,
                "snapshot_reason": snapshot.reason,
                "phone_position_m": snapshot.position_m,
                "phone_quaternion_wxyz": snapshot.quaternion_wxyz,
                "phone_step_translation_m": output.log.get("phone_step_translation_m"),
                "phone_step_rotation_rad": output.log.get("phone_step_rotation_rad"),
                "phone_back_camera_direction_world": phone_camera_direction_world.round(6).tolist(),
                "phone_axis_directions_world": {
                    axis_name: direction.round(6).tolist()
                    for axis_name, direction in phone_axis_directions_world.items()
                },
                "axis_visual_mode": "joint6_tool_frame",
                "phone_back_camera_axis": list(phone_back_camera_axis),
                "phone_quaternion_convention": args.phone_quaternion_convention,
                "position_mapping_mode": position_mapping_mode,
                "phone_local_delta_m": (
                    None if phone_local_delta_m is None else phone_local_delta_m.round(6).tolist()
                ),
                "target_position_m": palm_target_position_m,
                "target_quaternion_wxyz": output.palm_target_quaternion_wxyz,
                "target_filtered": output.log.get("target_filtered"),
                "target_velocity_limited": output.log.get("target_velocity_limited"),
                "target_acceleration_limited": output.log.get("target_acceleration_limited"),
                "target_deadband_hold": output.log.get("target_deadband_hold"),
                "ik_target_position_m": ik_state.target_palm_position_m.round(6).tolist(),
                "ik_target_workspace_limited": ik_state.target_workspace_limited,
                "ik_joint_limit_limited": ik_state.joint_limit_limited,
                "ik_joint_velocity_limited": ik_joint_velocity_limited,
                "ik_joint_velocity_rad_s": last_joint_velocity_rad_s.round(6).tolist(),
                "ik_limited_joint_indices_1_based": ik_state.limited_joint_indices_1_based,
                "ik_desired_q_before_velocity_limit": ik_desired_joints_before_velocity_limit,
                "ik_target_error_m": round(ik_state.target_error_m, 6),
                "q_current": q_current,
                "palm_position_m": ik_state.current_palm_position_m.round(6).tolist(),
                "palm_quaternion_wxyz": ik_state.current_palm_quaternion_wxyz.round(6).tolist(),
                "palm_target_rotation_error_rad": ik_state.target_rotation_error_rad,
                "reason": output.log.get("reason", "ok"),
            }
            records.append(record)
            if jsonl_path is not None:
                with jsonl_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(record, ensure_ascii=False) + "\n")
            last_tick = now
            sleep_s = tick_period - (time.time() - now)
            if sleep_s > 0.0:
                time.sleep(sleep_s)

    if args.jsonl_out and jsonl_path is None:
        out_path = Path(args.jsonl_out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        with out_path.open("w", encoding="utf-8") as handle:
            for record in records:
                handle.write(json.dumps(record, ensure_ascii=False) + "\n")

    active = [record for record in records if record["deadman"]]
    return {
        "duration_sec": float(args.duration_sec),
        "records": len(records),
        "active_records": len(active),
        "orientation_scale": float(args.orientation_scale),
        "orientation_anchor_mode": args.orientation_anchor_mode,
        "position_only": bool(args.position_only),
        "position_scale": float(args.position_scale),
        "workspace_mode": args.workspace_mode,
        "target_workspace_radius_m": args.target_workspace_radius_m,
        "phone_back_camera_axis": list(phone_back_camera_axis),
        "phone_quaternion_convention": args.phone_quaternion_convention,
        "max_joint_velocity_rad_s": float(args.max_joint_velocity_rad_s),
        "jsonl_out": args.jsonl_out,
        "final_q": q_current,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive a MuJoCo JAKA+RH56 shadow from real HEBI Mobile I/O pose.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--orientation-scale", type=float, default=1.0)
    parser.add_argument(
        "--orientation-mapping-mode",
        choices=["relative", "axis_mapped_relative", "mounted_device", "phone_back_camera"],
        default=None,
    )
    parser.add_argument("--orientation-anchor-mode", choices=["current", "flat-palm"], default="current")
    parser.add_argument("--position-only", action="store_true")
    parser.add_argument("--phone-camera-axis", choices=["+X", "-X", "+Y", "-Y", "+Z", "-Z"])
    parser.add_argument(
        "--phone-quaternion-convention",
        choices=["body-to-world", "world-to-phone"],
        default="body-to-world",
    )
    parser.add_argument("--position-scale", type=float, default=0.35)
    parser.add_argument("--position-mapping-mode", choices=["config", "tool_frame"], default="config")
    parser.add_argument("--orientation-ik-weight", type=float, default=0.7)
    parser.add_argument("--max-joint-velocity-rad-s", type=float, default=0.45)
    parser.add_argument("--max-joint-acceleration-rad-s2", type=float, default=1.50)
    parser.add_argument("--camera-distance", type=float, default=0.65)
    parser.add_argument("--show-left-ui", action="store_true")
    parser.add_argument("--show-right-ui", action="store_true")
    parser.add_argument("--workspace-mode", choices=["config", "unrestricted"], default="config")
    parser.add_argument(
        "--target-workspace-radius-m",
        type=float,
        default=0.0,
        help="IK target radius around the initial palm. Use -1 to keep the value from config.",
    )
    parser.add_argument("--phone-jump-reject-translation-m", type=float, default=0.25)
    parser.add_argument("--phone-jump-reject-rotation-rad", type=float, default=6.3)
    parser.add_argument("--max-target-velocity-m-s", type=float, default=0.25)
    parser.add_argument("--max-target-acceleration-m-s2", type=float, default=2.0)
    parser.add_argument("--max-target-jump-m", type=float, default=0.05)
    parser.add_argument("--target-filter-time-constant-sec", type=float, default=None)
    parser.add_argument("--target-update-deadband-m", type=float, default=None)
    parser.add_argument("--target-update-release-m", type=float, default=None)
    parser.add_argument("--disable-freeze-when-phone-still", action="store_true")
    parser.add_argument("--jsonl-out", default="data/checks/hebi_mujoco_shadow_live.jsonl")
    parser.add_argument("--wait-for-hebi-sec", type=float, default=0.0)
    args = parser.parse_args()
    if args.target_workspace_radius_m < 0.0:
        args.target_workspace_radius_m = None
    print(json.dumps(run_shadow(args), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
