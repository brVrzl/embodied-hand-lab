from __future__ import annotations

import argparse
import json
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageTk

from embodiment_core.config import load_yaml
from mujoco_rh56_grasp_benchmark import _configure_collision_model


ARM_PREGRASP_QPOS = np.asarray([0.123, 0.429, 1.496, -1.447, -0.019, -2.164], dtype=np.float64)
HAND_OPEN_CTRL = np.zeros(6, dtype=np.float64)
ARM_ACTUATOR_NAMES = [f"jaka_joint_{idx}_act" for idx in range(1, 7)]
HAND_ACTUATOR_NAMES = [
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
]


def _as_vec3(value: Any, *, field: str) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64).reshape(-1)
    if arr.size != 3:
        raise ValueError(f"{field} must contain 3 values, got {arr.size}.")
    return arr


def _fmt(values: np.ndarray | list[float] | tuple[float, ...]) -> str:
    return " ".join(f"{float(value):.6g}" for value in values)


def _camera_xyaxes(eye: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # MuJoCo fixed camera points along local -Z. z_axis is therefore the camera
    # backward axis in world coordinates.
    z_axis = eye - target
    z_norm = np.linalg.norm(z_axis)
    if z_norm <= 1e-9:
        raise ValueError("Camera eye and target must differ.")
    z_axis = z_axis / z_norm
    up = np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(float(np.dot(up, z_axis))) > 0.98:
        up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    x_axis = np.cross(up, z_axis)
    x_axis = x_axis / max(np.linalg.norm(x_axis), 1e-9)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / max(np.linalg.norm(y_axis), 1e-9)
    return x_axis, y_axis


def _find_worldbody(root: ET.Element) -> ET.Element:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MJCF is missing <worldbody>.")
    return worldbody


def _ensure_compiler_meshdir(root: ET.Element, base_xml: Path) -> None:
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(base_xml.parent))


def _ensure_visual_environment(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    if not any(texture.get("name") == "workspace_skybox" for texture in asset.findall("texture")):
        ET.SubElement(
            asset,
            "texture",
            {
                "name": "workspace_skybox",
                "type": "skybox",
                "builtin": "gradient",
                "rgb1": "0.78 0.80 0.82",
                "rgb2": "0.96 0.96 0.94",
                "width": "512",
                "height": "3072",
            },
        )
    if not any(texture.get("name") == "workspace_table_texture" for texture in asset.findall("texture")):
        ET.SubElement(
            asset,
            "texture",
            {
                "name": "workspace_table_texture",
                "type": "2d",
                "builtin": "checker",
                "rgb1": "0.68 0.58 0.43",
                "rgb2": "0.78 0.70 0.56",
                "width": "512",
                "height": "512",
            },
        )
    if not any(material.get("name") == "workspace_table_mat" for material in asset.findall("material")):
        ET.SubElement(
            asset,
            "material",
            {
                "name": "workspace_table_mat",
                "texture": "workspace_table_texture",
                "texrepeat": "9 5",
                "texuniform": "true",
                "rgba": "0.78 0.70 0.56 1",
                "specular": "0.18",
                "shininess": "0.20",
            },
        )
    visual = root.find("visual")
    if visual is None:
        visual = ET.SubElement(root, "visual")
    headlight = visual.find("headlight")
    if headlight is None:
        headlight = ET.SubElement(visual, "headlight")
    headlight.set("ambient", "0.55 0.55 0.55")
    headlight.set("diffuse", "0.45 0.45 0.45")
    headlight.set("specular", "0.12 0.12 0.12")
    rgba = visual.find("rgba")
    if rgba is None:
        rgba = ET.SubElement(visual, "rgba")
    rgba.set("haze", "0.88 0.90 0.92 1")


def _tune_actuators(root: ET.Element) -> None:
    for actuator in root.iter("position"):
        name = actuator.get("name", "")
        if name.startswith("jaka_joint_"):
            actuator.set("kp", "160")
        elif name.startswith("rh56_R_"):
            actuator.set("kp", "80")


def _add_workspace_scene(root: ET.Element, config: dict[str, Any]) -> None:
    worldbody = _find_worldbody(root)
    scene_cfg = config.get("scene", {})
    floor_cfg = scene_cfg.get("floor", {})
    floor = next((geom for geom in worldbody.iter("geom") if geom.get("name") == "floor"), None)
    if floor is not None:
        floor.set("pos", f"0 0 {float(floor_cfg.get('z_m', -1.0)):.6g}")
        floor.set("rgba", _fmt(floor_cfg.get("rgba", [0.62, 0.62, 0.60, 1.0])))
        floor.set("size", "3 3 0.1")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "workspace_back_wall",
            "type": "box",
            "pos": "-0.56 0.34 0.32",
            "size": "0.72 0.015 0.42",
            "rgba": "0.80 0.80 0.78 1",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "workspace_left_wall",
            "type": "box",
            "pos": "-0.90 0.00 0.32",
            "size": "0.015 0.36 0.42",
            "rgba": "0.74 0.75 0.74 1",
            "contype": "0",
            "conaffinity": "0",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "workspace_area_light",
            "pos": "-0.42 -0.25 0.85",
            "dir": "0.2 0.2 -1",
            "diffuse": "0.7 0.7 0.7",
            "specular": "0.1 0.1 0.1",
        },
    )

    table_cfg = scene_cfg.get("table", {})
    table_center = _as_vec3(table_cfg.get("center_xyz_m", [-0.265, 0.0, -0.02]), field="table.center_xyz_m")
    table_size = _as_vec3(table_cfg.get("size_xyz_m", [0.60, 0.30, 0.02]), field="table.size_xyz_m")
    table_rgba = table_cfg.get("rgba", [0.72, 0.66, 0.56, 1.0])
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "workspace_table",
            "type": "box",
            "pos": _fmt(table_center),
            "size": _fmt(table_size),
            "rgba": _fmt(table_rgba),
            "material": "workspace_table_mat",
            "friction": "1.4 0.05 0.003",
            "condim": "4",
        },
    )

    ball_cfg = scene_cfg.get("tennis_ball", {})
    ball_center = _as_vec3(ball_cfg.get("center_xyz_m", [-0.12, 0.0, 0.0335]), field="tennis_ball.center_xyz_m")
    ball_radius = float(ball_cfg.get("radius_m", 0.0335))
    ball_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "tennis_ball_body",
            "pos": _fmt(ball_center),
        },
    )
    ET.SubElement(ball_body, "freejoint", {"name": "tennis_ball_freejoint"})
    ET.SubElement(
        ball_body,
        "geom",
        {
            "name": "tennis_ball",
            "type": "sphere",
            "size": f"{ball_radius:.6g}",
            "mass": f"{float(ball_cfg.get('mass_kg', 0.058)):.6g}",
            "rgba": _fmt(ball_cfg.get("rgba", [0.78, 0.92, 0.18, 1.0])),
            "friction": "1.8 0.08 0.004",
            "condim": "4",
            "priority": "1",
        },
    )

    frame_cfg = scene_cfg.get("frame", {})
    if bool(frame_cfg.get("enabled", True)):
        _add_fixture_frame(worldbody, config)


def _add_fixture_frame(worldbody: ET.Element, config: dict[str, Any]) -> None:
    scene_cfg = config.get("scene", {})
    table_cfg = scene_cfg.get("table", {})
    frame_cfg = scene_cfg.get("frame", {})
    table_center = _as_vec3(table_cfg.get("center_xyz_m", [-0.265, 0.0, -0.02]), field="table.center_xyz_m")
    table_size = _as_vec3(table_cfg.get("size_xyz_m", [0.60, 0.30, 0.02]), field="table.size_xyz_m")
    side = float(frame_cfg.get("profile_side_m", 0.03))
    rgba = _fmt(frame_cfg.get("rgba", [0.75, 0.75, 0.72, 1.0]))
    table_top_z = float(table_center[2] + table_size[2])
    rail_z = table_top_z + side / 2.0
    x_left = table_center[0] - table_size[0] + side / 2.0
    y_back = table_center[1] + table_size[1] - side / 2.0
    rails = [
        ("camera_left_upright", [x_left, y_back, table_top_z + 0.265], [side / 2.0, side / 2.0, 0.265]),
    ]
    mount_cfg = frame_cfg.get("robot_mount", {})
    if bool(mount_cfg.get("enabled", True)):
        center_xy = np.asarray(mount_cfg.get("center_xy_m", [0.0, 0.0]), dtype=np.float64)
        rail_length = float(mount_cfg.get("base_rail_length_m", 0.32))
        rail_spacing = float(mount_cfg.get("base_rail_spacing_m", 0.10))
        cross_length = float(mount_cfg.get("cross_rail_length_m", 0.28))
        cross_spacing = float(mount_cfg.get("cross_rail_spacing_m", 0.24))
        bolt_spacing = float(mount_cfg.get("bolt_spacing_along_rail_m", 0.18))
        bolt_radius = float(mount_cfg.get("bolt_radius_m", 0.007))
        top_z = float(mount_cfg.get("top_z_m", 0.0))
        mount_z = top_z - side / 2.0
        x0 = center_xy[0] - rail_spacing / 2.0
        x1 = center_xy[0] + rail_spacing / 2.0
        y0 = center_xy[1] - cross_spacing / 2.0
        y1 = center_xy[1] + cross_spacing / 2.0
        rails.extend(
            [
                ("robot_mount_base_rail_left", [x0, center_xy[1], mount_z], [side / 2.0, rail_length / 2.0, side / 2.0]),
                ("robot_mount_base_rail_right", [x1, center_xy[1], mount_z], [side / 2.0, rail_length / 2.0, side / 2.0]),
                ("robot_mount_cross_rail_front", [center_xy[0], y0, mount_z], [cross_length / 2.0, side / 2.0, side / 2.0]),
                ("robot_mount_cross_rail_back", [center_xy[0], y1, mount_z], [cross_length / 2.0, side / 2.0, side / 2.0]),
            ]
        )
        bolt_y0 = center_xy[1] - bolt_spacing / 2.0
        bolt_y1 = center_xy[1] + bolt_spacing / 2.0
        for idx, (bolt_x, bolt_y) in enumerate(((x0, bolt_y0), (x0, bolt_y1), (x1, bolt_y0), (x1, bolt_y1)), start=1):
            ET.SubElement(
                worldbody,
                "geom",
                {
                    "name": f"robot_mount_bolt_{idx}",
                    "type": "cylinder",
                    "pos": _fmt([bolt_x, bolt_y, top_z + 0.002]),
                    "size": f"{bolt_radius:.6g} 0.004",
                    "rgba": "0.12 0.12 0.12 1",
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
    for name, pos, size in rails:
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": name,
                "type": "box",
                "pos": _fmt(pos),
                "size": _fmt(size),
                "rgba": rgba,
                "contype": "0",
                "conaffinity": "0",
            },
        )


def _add_cameras(root: ET.Element, config: dict[str, Any]) -> list[str]:
    worldbody = _find_worldbody(root)
    camera_names: list[str] = []
    scene_cfg = config.get("scene", {})
    table_cfg = scene_cfg.get("table", {})
    table_center = _as_vec3(table_cfg.get("center_xyz_m", [-0.265, 0.0, -0.02]), field="table.center_xyz_m")
    table_size = _as_vec3(table_cfg.get("size_xyz_m", [0.60, 0.30, 0.02]), field="table.size_xyz_m")
    table_top_z = float(table_center[2] + table_size[2])
    viewer_cfg = config.get("viewer", {})
    panel_width = float(viewer_cfg.get("panel_width", 560))
    panel_height = float(viewer_cfg.get("panel_height_per_camera", 315))
    aspect = panel_width / max(panel_height, 1.0)
    for name, camera_cfg in (config.get("cameras", {}) or {}).items():
        eye = _as_vec3(camera_cfg.get("eye_xyz_m"), field=f"cameras.{name}.eye_xyz_m")
        target = _as_vec3(camera_cfg.get("target_xyz_m"), field=f"cameras.{name}.target_xyz_m")
        x_axis, y_axis = _camera_xyaxes(eye, target)
        fovy_deg = float(camera_cfg.get("fovy_deg", 45.0))
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": str(name),
                "mode": "fixed",
                "pos": _fmt(eye),
                "xyaxes": f"{_fmt(x_axis)} {_fmt(y_axis)}",
                "fovy": f"{fovy_deg:.6g}",
            },
        )
        if bool(camera_cfg.get("show_model", True)):
            _add_camera_visual_model(
                worldbody,
                name=str(name),
                eye=eye,
                target=target,
                x_axis=x_axis,
                y_axis=y_axis,
                fovy_deg=fovy_deg,
                aspect=aspect,
                table_top_z=table_top_z,
            )
        camera_names.append(str(name))
    return camera_names


def _add_camera_visual_model(
    worldbody: ET.Element,
    *,
    name: str,
    eye: np.ndarray,
    target: np.ndarray,
    x_axis: np.ndarray,
    y_axis: np.ndarray,
    fovy_deg: float,
    aspect: float,
    table_top_z: float,
) -> None:
    forward = target - eye
    distance = float(np.linalg.norm(forward))
    if distance <= 1e-9:
        return
    forward = forward / distance
    marker_rgba = "0.05 0.18 0.24 1"
    ray_rgba = "0.0 0.55 1.0 0.45"
    post_rgba = "0.35 0.35 0.33 0.85"
    visual_attrs = {"group": "5", "contype": "0", "conaffinity": "0"}
    label = name.replace(" ", "_")

    if eye[2] > table_top_z + 0.05:
        post_half_height = max(float(eye[2] - table_top_z) / 2.0, 0.01)
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"{label}_camera_mount_post",
                "type": "cylinder",
                "pos": _fmt([eye[0], eye[1], table_top_z + post_half_height]),
                "size": f"0.006 {post_half_height:.6g}",
                "rgba": post_rgba,
                **visual_attrs,
            },
        )

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": f"{label}_camera_body",
            "type": "box",
            "pos": _fmt(eye),
            "size": "0.035 0.018 0.014",
            "rgba": marker_rgba,
            **visual_attrs,
        },
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": f"{label}_camera_target_marker",
            "type": "sphere",
            "pos": _fmt(target),
            "size": "0.010",
            "rgba": "0.0 0.55 1.0 0.45",
            **visual_attrs,
        },
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": f"{label}_camera_aim_ray",
            "type": "capsule",
            "fromto": f"{_fmt(eye)} {_fmt(target)}",
            "size": "0.0015",
            "rgba": ray_rgba,
            **visual_attrs,
        },
    )

    frustum_length = min(0.22, max(0.12, distance * 0.30))
    half_height = math.tan(math.radians(fovy_deg) / 2.0) * frustum_length
    half_width = half_height * aspect
    center = eye + forward * frustum_length
    corners = [
        center + x_sign * half_width * x_axis + y_sign * half_height * y_axis
        for x_sign in (-1.0, 1.0)
        for y_sign in (-1.0, 1.0)
    ]
    for idx, corner in enumerate(corners, start=1):
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"{label}_camera_frustum_{idx}",
                "type": "capsule",
                "fromto": f"{_fmt(eye)} {_fmt(corner)}",
                "size": "0.0018",
                "rgba": ray_rgba,
                **visual_attrs,
            },
        )


def build_workspace_xml(config: dict[str, Any]) -> dict[str, Any]:
    base_xml = Path(config.get("base_xml", "data/sim_assets/jaka_rh56.xml")).resolve()
    out_xml = Path(config.get("out_xml", "data/mujoco_debug/tennis_ball_lift_workspace.xml")).resolve()
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    _ensure_compiler_meshdir(root, base_xml)
    _ensure_visual_environment(root)
    _tune_actuators(root)
    _configure_collision_model(root, collision_mode=str(config.get("collision_mode", "visual_coacd")))
    _add_workspace_scene(root, config)
    camera_names = _add_cameras(root, config)
    root.set("model", "jaka_rh56_tennis_ball_lift_workspace")
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    summary = {
        "base_xml": str(base_xml),
        "out_xml": str(out_xml),
        "camera_names": camera_names,
        "workspace_config": config.get("workspace_config"),
        "collision_mode": config.get("collision_mode", "visual_coacd"),
    }
    (out_xml.parent / "tennis_ball_lift_workspace_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    return summary


def _ids(model: mujoco.MjModel, obj_type: mujoco.mjtObj, names: list[str]) -> np.ndarray:
    found: list[int] = []
    for name in names:
        idx = mujoco.mj_name2id(model, obj_type, name)
        if idx < 0:
            raise KeyError(f"Missing {obj_type} named {name}")
        found.append(idx)
    return np.asarray(found, dtype=np.int32)


def _set_initial_state(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    data.qpos[:6] = ARM_PREGRASP_QPOS
    data.ctrl[arm_ids] = ARM_PREGRASP_QPOS
    data.ctrl[hand_ids] = HAND_OPEN_CTRL
    mujoco.mj_forward(model, data)


def _camera_panel_rgb(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    renderers: list[mujoco.Renderer],
    camera_ids: list[int],
    camera_names: list[str],
) -> np.ndarray:
    panels: list[np.ndarray] = []
    for renderer, camera_id, camera_name in zip(renderers, camera_ids, camera_names, strict=True):
        renderer._scene_option.geomgroup[5] = 0
        renderer.update_scene(data, camera=camera_id)
        rgb = renderer.render()
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 180, 34), fill=(0, 0, 0))
        draw.text((12, 9), camera_name, fill=(255, 255, 255))
        panels.append(np.asarray(image))
    return np.vstack(panels)


class CameraPanelWindow:
    def __init__(self, *, title: str, geometry: str) -> None:
        import tkinter as tk

        self.tk = tk
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry(geometry)
        self.root.attributes("-topmost", True)
        self.label = tk.Label(self.root)
        self.label.pack()
        self._photo: ImageTk.PhotoImage | None = None
        self.closed = False
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self) -> None:
        self.closed = True
        self.root.withdraw()

    def update(self, rgb: np.ndarray) -> None:
        if self.closed:
            return
        self._photo = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.label.configure(image=self._photo)
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        if not self.closed:
            self.root.destroy()
            self.closed = True


def run_viewer(config: dict[str, Any], *, xml_path: str | Path, camera_panel: bool) -> None:
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    _set_initial_state(model, data)
    viewer_cfg = config.get("viewer", {})
    duration_sec = float(viewer_cfg.get("duration_sec", 0.0))
    target_fps = float(viewer_cfg.get("fps", 30))
    sleep_s = 1.0 / max(target_fps, 1.0)

    camera_names = list((config.get("cameras", {}) or {}).keys())
    camera_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
        for name in camera_names
    ]
    renderers: list[mujoco.Renderer] = []
    panel_window: CameraPanelWindow | None = None
    if camera_panel:
        panel_width = int(viewer_cfg.get("panel_width", 640))
        panel_height = int(viewer_cfg.get("panel_height_per_camera", 360))
        renderers = [mujoco.Renderer(model, height=panel_height, width=panel_width) for _ in camera_ids]
        panel_geometry = f"{panel_width}x{panel_height * max(1, len(camera_ids))}{viewer_cfg.get('panel_geometry', '+20+40')}"
        panel_window = CameraPanelWindow(title="MuJoCo fixed cameras", geometry=panel_geometry)

    start = time.time()
    with mujoco.viewer.launch_passive(model, data) as handle:
        handle.opt.geomgroup[5] = 1
        handle.cam.azimuth = -130
        handle.cam.elevation = -25
        handle.cam.distance = 0.85
        handle.cam.lookat[:] = [-0.18, 0.0, 0.16]
        while handle.is_running():
            if duration_sec > 0 and time.time() - start >= duration_sec:
                break
            mujoco.mj_step(model, data)
            handle.sync()
            if camera_panel and renderers and panel_window is not None and not panel_window.closed:
                panel = _camera_panel_rgb(
                    model,
                    data,
                    renderers=renderers,
                    camera_ids=camera_ids,
                    camera_names=camera_names,
                )
                panel_window.update(panel)
            time.sleep(sleep_s)

    for renderer in renderers:
        renderer.close()
    if panel_window is not None:
        panel_window.close()


def export_snapshot(config: dict[str, Any], *, xml_path: str | Path, output_dir: Path) -> dict[str, str]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    _set_initial_state(model, data)
    output_dir.mkdir(parents=True, exist_ok=True)
    viewer_cfg = config.get("viewer", {})
    width = int(viewer_cfg.get("panel_width", 640))
    height = int(viewer_cfg.get("panel_height_per_camera", 360))
    paths: dict[str, str] = {}
    for camera_name in (config.get("cameras", {}) or {}):
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if camera_id < 0:
            continue
        renderer = mujoco.Renderer(model, height=height, width=width)
        renderer._scene_option.geomgroup[5] = 0
        renderer.update_scene(data, camera=camera_id)
        rgb = renderer.render()
        renderer.close()
        path = output_dir / f"{camera_name}.png"
        cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        paths[camera_name] = str(path)
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Preview the MuJoCo tennis-ball lift workspace with two fixed cameras.")
    parser.add_argument("--config", default="configs/sim/mujoco_jaka_rh56_tennis_ball_lift.yaml")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--no-viewer", action="store_true")
    parser.add_argument("--no-camera-panel", action="store_true")
    parser.add_argument("--snapshot-dir", default="data/previews/mujoco_tennis_ball_lift")
    args = parser.parse_args()

    config = load_yaml(args.config)
    summary = build_workspace_xml(config)
    snapshot_paths = export_snapshot(config, xml_path=summary["out_xml"], output_dir=Path(args.snapshot_dir))
    print(json.dumps({**summary, "snapshot_paths": snapshot_paths}, indent=2))
    if args.build_only or args.no_viewer:
        return
    run_viewer(config, xml_path=summary["out_xml"], camera_panel=not args.no_camera_panel)


if __name__ == "__main__":
    main()
