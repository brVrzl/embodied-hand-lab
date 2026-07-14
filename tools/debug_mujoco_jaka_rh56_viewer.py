from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import imageio.v3 as iio
import mujoco
import numpy as np

from mujoco_rh56_grasp_benchmark import COLLISION_MODES, _configure_collision_model


BASE_XML = Path("data/sim_assets/jaka_rh56.xml")
DEBUG_DIR = Path("data/mujoco_debug")
PREGRASP_QPOS = np.asarray(
    [0.123, 0.429, 1.496, -1.447, -0.019, -2.164] + [0.0] * 12,
    dtype=np.float64,
)
HAND_CLOSE_CTRL = np.asarray([0.75, 0.45, 1.25, 1.25, 1.25, 1.25], dtype=np.float64)
HAND_OPEN_CTRL = np.zeros(6, dtype=np.float64)
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


def _body_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise KeyError(f"Missing body {name}")
    return np.asarray(data.xpos[body_id], dtype=np.float64)


def _pregrasp_positions(base_xml: Path) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    data.qpos[: PREGRASP_QPOS.size] = PREGRASP_QPOS
    data.ctrl[:6] = PREGRASP_QPOS[:6]
    data.ctrl[6:12] = HAND_OPEN_CTRL
    mujoco.mj_forward(model, data)
    tip_positions = {name: _body_pos(model, data, name).tolist() for name in TIP_BODY_NAMES}
    tip_array = np.asarray(list(tip_positions.values()), dtype=np.float64)
    palm = _body_pos(model, data, "rh56_R_hand_base_link")
    cube_pos = np.mean([tip_array[0], tip_array[1], tip_array[2]], axis=0)
    cube_pos = 0.65 * cube_pos + 0.35 * palm
    return {
        "tips": tip_positions,
        "palm": palm.tolist(),
        "cube_in_hand_pos": cube_pos.tolist(),
        "table_cube_pos": [-0.10, 0.0, 0.025],
    }


def _add_debug_world(root: ET.Element, scenario: str, cube_pos: list[float]) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Base MJCF does not contain <worldbody>.")

    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "debug_table",
            "type": "box",
            "pos": "-0.10 0.0 -0.025",
            "size": "0.45 0.32 0.025",
            "rgba": "0.70 0.64 0.55 1",
            "friction": "1.2 0.02 0.001",
        },
    )
    cube_body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "debug_cube_body",
            "pos": f"{cube_pos[0]:.6f} {cube_pos[1]:.6f} {cube_pos[2]:.6f}",
        },
    )
    ET.SubElement(cube_body, "freejoint", {"name": "debug_cube_freejoint"})
    ET.SubElement(
        cube_body,
        "geom",
        {
            "name": "debug_cube",
            "type": "box",
            "size": "0.02 0.02 0.02",
            "mass": "0.03",
            "rgba": "1 0 0 1",
            "friction": "1.4 0.04 0.002",
            "condim": "4",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "debug_close",
            "mode": "fixed",
            "pos": "-0.43 -0.42 0.34",
            "xyaxes": "0.85 -0.52 0 0.24 0.40 0.88",
            "fovy": "35",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "debug_front",
            "mode": "fixed",
            "pos": "-0.50 -0.70 0.30",
            "xyaxes": "0.90 -0.43 0 0.17 0.35 0.92",
            "fovy": "45",
        },
    )
    root.set("model", f"jaka_rh56_mujoco_debug_{scenario}")


def build_debug_xml(base_xml: str | Path, out_xml: str | Path, *, scenario: str, collision_mode: str) -> dict[str, Any]:
    base_xml = Path(base_xml).resolve()
    out_xml = Path(out_xml)
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    positions = _pregrasp_positions(base_xml)
    cube_pos = positions["cube_in_hand_pos"] if scenario == "cube_in_hand" else positions["table_cube_pos"]
    tree = ET.parse(base_xml)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    compiler.set("meshdir", str(base_xml.parent))
    _configure_collision_model(root, collision_mode=collision_mode, include_calibration_markers=False)
    _add_debug_world(root, scenario=scenario, cube_pos=cube_pos)
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)
    summary = {
        "base_xml": str(base_xml.resolve()),
        "debug_xml": str(out_xml.resolve()),
        "scenario": scenario,
        "collision_mode": collision_mode,
        "cube_pos": cube_pos,
        **positions,
    }
    (out_xml.parent / "debug_scene_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def _actuator_ids(model: mujoco.MjModel, names: list[str]) -> list[int]:
    ids: list[int] = []
    for name in names:
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if actuator_id < 0:
            raise KeyError(f"Missing actuator {name}")
        ids.append(actuator_id)
    return ids


def _set_initial_state(model: mujoco.MjModel, data: mujoco.MjData, *, scenario: str) -> None:
    data.qpos[: PREGRASP_QPOS.size] = PREGRASP_QPOS
    data.ctrl[:6] = PREGRASP_QPOS[:6]
    data.ctrl[6:12] = HAND_OPEN_CTRL
    if scenario == "hand_close":
        cube_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "debug_cube_freejoint")
        if cube_joint >= 0:
            qpos_addr = model.jnt_qposadr[cube_joint]
            data.qpos[qpos_addr : qpos_addr + 7] = [0.0, 0.0, -0.5, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)


def _contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int]:
    counts = {"cube_hand": 0, "cube_table": 0, "hand_table": 0, "hand_self": 0, "total": int(data.ncon)}
    for idx in range(data.ncon):
        contact = data.contact[idx]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "",
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "",
        ]
        joined = " ".join(names)
        if "debug_cube" in joined and "rh56" in joined:
            counts["cube_hand"] += 1
        if "debug_cube" in joined and "debug_table" in joined:
            counts["cube_table"] += 1
        if "rh56" in joined and "debug_table" in joined:
            counts["hand_table"] += 1
        if names[0].startswith("rh56_R_") and names[1].startswith("rh56_R_"):
            counts["hand_self"] += 1
    return counts


def _step_control(model: mujoco.MjModel, data: mujoco.MjData, *, elapsed: float, cycle_period: float) -> None:
    hand_alpha = 0.5 - 0.5 * np.cos(2.0 * np.pi * min(1.0, (elapsed % cycle_period) / cycle_period))
    if elapsed > cycle_period:
        hand_alpha = 1.0
    data.ctrl[:6] = PREGRASP_QPOS[:6]
    data.ctrl[6:12] = (1.0 - hand_alpha) * HAND_OPEN_CTRL + hand_alpha * HAND_CLOSE_CTRL


def _print_status(model: mujoco.MjModel, data: mujoco.MjData, sim_time: float) -> None:
    cube_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "debug_cube_body")
    cube_pos = data.xpos[cube_body].copy() if cube_body >= 0 else np.zeros(3)
    contacts = _contact_summary(model, data)
    hand_qpos = data.qpos[6:18].copy()
    print(
        f"t={sim_time:6.3f} cube=({cube_pos[0]:+.3f},{cube_pos[1]:+.3f},{cube_pos[2]:+.3f}) "
        f"hand_mcp={np.round(hand_qpos[[0,1,4,6,8,10]], 3).tolist()} contacts={contacts}",
        flush=True,
    )


def run_debug(
    xml_path: str | Path,
    *,
    scenario: str,
    duration: float,
    cycle_period: float,
    viewer: bool,
    record_mp4: str | None,
    width: int,
    height: int,
    fps: int,
) -> None:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    _set_initial_state(model, data, scenario=scenario)

    renderer = None
    frames: list[np.ndarray] = []
    if record_mp4:
        renderer = mujoco.Renderer(model, height=height, width=width)
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "debug_close")

    last_print = -1.0
    step_count = 0
    if viewer:
        mujoco_viewer = importlib.import_module("mujoco.viewer")
        handle = mujoco_viewer.launch_passive(model, data)
        handle.cam.azimuth = -120
        handle.cam.elevation = -20
        handle.cam.distance = 0.55
        handle.cam.lookat[:] = [-0.52, -0.08, 0.18]
        wall_start = time.time()
        duration_limited = duration > 0.0
        while handle.is_running() and (not duration_limited or data.time < duration):
            _step_control(model, data, elapsed=data.time, cycle_period=cycle_period)
            mujoco.mj_step(model, data)
            if data.time - last_print >= 0.5:
                _print_status(model, data, data.time)
                last_print = data.time
            handle.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - wall_start - data.time)))

        # On Thor's VNC llvmpipe stack, MuJoCo/GLFW can crash during native viewer teardown.
        # The interactive viewer has no Python cleanup to preserve, so exit before finalizers run.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    else:
        while data.time < duration:
            _step_control(model, data, elapsed=data.time, cycle_period=cycle_period)
            mujoco.mj_step(model, data)
            if data.time - last_print >= 0.5:
                _print_status(model, data, data.time)
                last_print = data.time
            if renderer is not None and step_count % max(1, int(round(1.0 / (fps * model.opt.timestep)))) == 0:
                renderer.update_scene(data, camera=camera_id)
                frames.append(renderer.render().copy())
            step_count += 1

    if renderer is not None:
        out_path = Path(record_mp4)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(out_path, frames, fps=fps)
        print(f"Wrote {len(frames)} frames to {out_path}")
        renderer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Debug JAKA mini2 + RH56 grasp geometry in MuJoCo.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--out-xml", default=str(DEBUG_DIR / "jaka_rh56_debug.xml"))
    parser.add_argument("--scenario", choices=["hand_close", "cube_in_hand", "table_cube"], default="cube_in_hand")
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="visual_coacd")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--cycle-period", type=float, default=3.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--record-mp4", default=None)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    summary = build_debug_xml(args.base_xml, args.out_xml, scenario=args.scenario, collision_mode=args.collision_mode)
    print(json.dumps(summary, indent=2))
    run_debug(
        args.out_xml,
        scenario=args.scenario,
        duration=args.duration,
        cycle_period=args.cycle_period,
        viewer=args.viewer,
        record_mp4=args.record_mp4,
        width=args.width,
        height=args.height,
        fps=args.fps,
    )


if __name__ == "__main__":
    main()
