from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import imageio.v3 as iio
import mujoco
import numpy as np
import yaml


BASE_XML = Path("data/sim_assets/jaka_rh56.xml")
OUT_DIR = Path("data/mujoco_debug/pinch_box_v1")

HAND_ACTUATOR_NAMES = [
    "rh56_R_thumb_MCP_joint1_act",
    "rh56_R_thumb_MCP_joint2_act",
    "rh56_R_index_MCP_joint_act",
    "rh56_R_middle_MCP_joint_act",
    "rh56_R_ring_MCP_joint_act",
    "rh56_R_pinky_MCP_joint_act",
]

TIP_BODIES = [
    "rh56_R_thumb_distal",
    "rh56_R_index_distal",
    "rh56_R_middle_distal",
    "rh56_R_ring_distal",
    "rh56_R_pinky_distal",
]

PALM_BODY = "rh56_R_hand_base_link"


def _load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _body_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if body_id < 0:
        raise KeyError(f"Missing body {name}")
    return np.asarray(data.xpos[body_id], dtype=np.float64).copy()


def _ctrl_ids(model: mujoco.MjModel) -> list[int]:
    ids = []
    for name in HAND_ACTUATOR_NAMES:
        idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if idx < 0:
            raise KeyError(f"Missing actuator {name}")
        ids.append(idx)
    return ids


def _physical_norm_to_mujoco_ctrl(values: list[float]) -> np.ndarray:
    """Map real debug physical order to MuJoCo actuator order/ranges.

    Real debug order:
      0 pinky, 1 ring, 2 middle, 3 index, 4 thumb_bend, 5 thumb_rotate.
    MuJoCo actuator order:
      thumb_rotate, thumb_bend, index, middle, ring, pinky.
    """

    if len(values) != 6:
        raise ValueError("Expected 6 physical DOF values.")
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


def _set_arm_hand(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    arm_q: np.ndarray,
    hand_ctrl: np.ndarray,
) -> None:
    data.qpos[:6] = arm_q
    # Kinematic pose-setting for geometry estimates. MuJoCo equality constraints
    # normally couple distal joints during dynamics, but mj_forward() alone does
    # not move qpos from position actuator targets.
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
    hand_ids = _ctrl_ids(model)
    for ctrl_id, value in zip(hand_ids, hand_ctrl, strict=True):
        data.ctrl[ctrl_id] = value
    mujoco.mj_forward(model, data)


def _estimate_box_pose(
    base_xml: Path,
    *,
    grasp_arm_q: np.ndarray,
    thumb_rotate_ctrl: np.ndarray,
    close_ctrl: np.ndarray,
    box_size: np.ndarray,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(base_xml))
    data = mujoco.MjData(model)
    _set_arm_hand(model, data, arm_q=grasp_arm_q, hand_ctrl=thumb_rotate_ctrl)
    rotate_tips = {name: _body_pos(model, data, name) for name in TIP_BODIES}

    _set_arm_hand(model, data, arm_q=grasp_arm_q, hand_ctrl=close_ctrl)
    close_tips = {name: _body_pos(model, data, name) for name in TIP_BODIES}
    palm = _body_pos(model, data, PALM_BODY)

    # Put the paper box at the local pinch centroid. This is not a perception result;
    # it is a geometry probe for the current successful real-world primitive.
    thumb = close_tips["rh56_R_thumb_distal"]
    index = close_tips["rh56_R_index_distal"]
    middle = close_tips["rh56_R_middle_distal"]
    pinch_center = 0.45 * thumb + 0.35 * index + 0.20 * middle
    box_pos = pinch_center.copy()
    box_pos[2] = max(box_pos[2], box_size[2] + 0.015)

    camera_lookat = 0.5 * (palm + box_pos)
    return {
        "box_pos": box_pos.tolist(),
        "palm": palm.tolist(),
        "rotate_tips": {key: value.tolist() for key, value in rotate_tips.items()},
        "close_tips": {key: value.tolist() for key, value in close_tips.items()},
        "camera_lookat": camera_lookat.tolist(),
    }


def _add_world(root: ET.Element, *, box_pos: list[float], box_size: list[float]) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody.")
    table_half_thickness = 0.015
    table_top_z = float(box_pos[2] - box_size[2] - 0.003)
    table_center_z = table_top_z - table_half_thickness
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "pinch_debug_table",
            "type": "box",
            "pos": f"{box_pos[0]:.6f} {box_pos[1]:.6f} {table_center_z:.6f}",
            "size": "0.35 0.28 0.015",
            "rgba": "0.68 0.62 0.54 1",
            "friction": "1.2 0.03 0.001",
            "condim": "4",
        },
    )
    body = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "pinch_box_body",
            "pos": f"{box_pos[0]:.6f} {box_pos[1]:.6f} {box_pos[2]:.6f}",
        },
    )
    ET.SubElement(body, "freejoint", {"name": "pinch_box_freejoint"})
    ET.SubElement(
        body,
        "geom",
        {
            "name": "pinch_box",
            "type": "box",
            "size": f"{box_size[0]:.6f} {box_size[1]:.6f} {box_size[2]:.6f}",
            "mass": "0.025",
            "rgba": "0.85 0.35 0.15 1",
            "friction": "1.5 0.05 0.003",
            "condim": "4",
        },
    )
    ET.SubElement(
        worldbody,
        "camera",
        {
            "name": "pinch_debug_camera",
            "mode": "fixed",
            "pos": "-0.35 -0.72 0.38",
            "xyaxes": "0.94 -0.34 0 0.17 0.47 0.87",
            "fovy": "42",
        },
    )


def _build_xml(base_xml: Path, out_xml: Path, *, box_pos: list[float], box_size: list[float]) -> None:
    out_xml.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(base_xml)
    root = tree.getroot()
    _add_world(root, box_pos=box_pos, box_size=box_size)
    root.set("model", "jaka_rh56_pinch_box_pose_debug")
    tree.write(out_xml, encoding="utf-8", xml_declaration=False)


def _contact_summary(model: mujoco.MjModel, data: mujoco.MjData) -> dict[str, int]:
    counts = {
        "box_thumb": 0,
        "box_index": 0,
        "box_middle": 0,
        "box_other_hand": 0,
        "box_table": 0,
        "hand_table": 0,
        "total": int(data.ncon),
    }
    for idx in range(data.ncon):
        contact = data.contact[idx]
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "",
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "",
        ]
        joined = " ".join(names)
        if "pinch_box" in joined and "thumb" in joined:
            counts["box_thumb"] += 1
        elif "pinch_box" in joined and "index" in joined:
            counts["box_index"] += 1
        elif "pinch_box" in joined and "middle" in joined:
            counts["box_middle"] += 1
        elif "pinch_box" in joined and "rh56" in joined:
            counts["box_other_hand"] += 1
        if "pinch_box" in joined and "pinch_debug_table" in joined:
            counts["box_table"] += 1
        if "rh56" in joined and "pinch_debug_table" in joined:
            counts["hand_table"] += 1
    return counts


def _run_sim(
    xml_path: Path,
    *,
    grasp_arm_q: np.ndarray,
    lift_arm_q: np.ndarray,
    rotate_ctrl: np.ndarray,
    close_ctrl: np.ndarray,
    duration: float,
    record_mp4: Path | None,
    width: int,
    height: int,
    fps: int,
) -> dict[str, Any]:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    hand_ids = _ctrl_ids(model)
    data.qpos[:6] = grasp_arm_q
    data.ctrl[:6] = grasp_arm_q
    data.ctrl[hand_ids] = np.zeros(6)
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=height, width=width) if record_mp4 else None
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "pinch_debug_camera")
    frames: list[np.ndarray] = []
    dt = model.opt.timestep
    contact_log: list[dict[str, Any]] = []
    box_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pinch_box_body")

    while data.time < duration:
        t = data.time
        if t < 0.7:
            arm_alpha = 0.0
            hand = np.zeros(6)
        elif t < 1.4:
            arm_alpha = 0.0
            hand = rotate_ctrl
        elif t < 2.3:
            arm_alpha = 0.0
            hand_alpha = (t - 1.4) / 0.9
            hand = (1.0 - hand_alpha) * rotate_ctrl + hand_alpha * close_ctrl
        else:
            arm_alpha = min(1.0, (t - 2.3) / max(0.5, duration - 2.3))
            hand = close_ctrl
        arm_q = (1.0 - arm_alpha) * grasp_arm_q + arm_alpha * lift_arm_q
        data.ctrl[:6] = arm_q
        data.ctrl[hand_ids] = hand
        mujoco.mj_step(model, data)

        if int(data.time / 0.25) != int((data.time - dt) / 0.25):
            contact_log.append(
                {
                    "time": round(float(data.time), 3),
                    "box_pos": data.xpos[box_body].round(4).tolist() if box_body >= 0 else None,
                    "contacts": _contact_summary(model, data),
                }
            )
        if renderer is not None and len(frames) % max(1, int(round(1.0 / (fps * dt)))) == 0:
            renderer.update_scene(data, camera=camera_id)
            frames.append(renderer.render().copy())

    if renderer is not None and record_mp4 is not None:
        record_mp4.parent.mkdir(parents=True, exist_ok=True)
        iio.imwrite(record_mp4, frames, fps=fps)
        renderer.close()

    return {
        "final_box_pos": data.xpos[box_body].tolist() if box_body >= 0 else None,
        "final_contacts": _contact_summary(model, data),
        "contact_log": contact_log,
        "record_mp4": str(record_mp4) if record_mp4 else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulate current RH56 top-down pinch primitive in MuJoCo.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--hand-config", default="configs/hand/rh56_real.yaml")
    parser.add_argument("--out-dir", default=str(OUT_DIR))
    parser.add_argument("--duration", type=float, default=4.5)
    parser.add_argument("--box-size", nargs=3, type=float, default=[0.022, 0.018, 0.018])
    parser.add_argument("--box-offset", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--sweep", action="store_true", help="Sweep small box offsets and score candidate contacts.")
    parser.add_argument("--record-mp4", action="store_true")
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()

    robot_cfg = _load_yaml(args.robot_config)
    hand_cfg = _load_yaml(args.hand_config)
    grasp_q = np.asarray(robot_cfg["joint_presets"]["pinch_grasp_box_v2"], dtype=np.float64)
    lift_q = np.asarray(robot_cfg["joint_presets"]["pinch_lift_box_v1"], dtype=np.float64)
    rotate_raw = [int(v) for v in hand_cfg["gesture_presets"]["pinch_box_thumb_rotate_v2"]]
    close_raw = [int(v) for v in hand_cfg["gesture_presets"]["pinch_box_v4"]]
    rotate_ctrl = _physical_norm_to_mujoco_ctrl(_raw_to_physical_norm(rotate_raw))
    close_ctrl = _physical_norm_to_mujoco_ctrl(_raw_to_physical_norm(close_raw))
    box_size = np.asarray(args.box_size, dtype=np.float64)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    estimate = _estimate_box_pose(
        Path(args.base_xml),
        grasp_arm_q=grasp_q,
        thumb_rotate_ctrl=rotate_ctrl,
        close_ctrl=close_ctrl,
        box_size=box_size,
    )

    def run_one(offset: np.ndarray, name: str) -> dict[str, Any]:
        box_pos = (np.asarray(estimate["box_pos"], dtype=np.float64) + offset).tolist()
        xml_path = out_dir / f"{name}.xml"
        _build_xml(Path(args.base_xml), xml_path, box_pos=box_pos, box_size=box_size.tolist())
        result = _run_sim(
            xml_path,
            grasp_arm_q=grasp_q,
            lift_arm_q=lift_q,
            rotate_ctrl=rotate_ctrl,
            close_ctrl=close_ctrl,
            duration=args.duration,
            record_mp4=(out_dir / f"{name}.mp4") if args.record_mp4 and name == "pinch_box_pose_debug" else None,
            width=args.width,
            height=args.height,
            fps=args.fps,
        )
        final_z = float(result["final_box_pos"][2]) if result["final_box_pos"] is not None else -1.0
        score = (
            10.0 * final_z
            + result["final_contacts"].get("box_thumb", 0)
            + result["final_contacts"].get("box_index", 0)
            + result["final_contacts"].get("box_middle", 0)
            - 0.25 * result["final_contacts"].get("box_table", 0)
            - 0.25 * result["final_contacts"].get("hand_table", 0)
        )
        return {
            "name": name,
            "xml": str(xml_path),
            "box_pos": box_pos,
            "box_offset": offset.tolist(),
            "score": score,
            "result": result,
        }

    if args.sweep:
        candidates: list[dict[str, Any]] = []
        offsets = []
        for dx in [-0.018, -0.009, 0.0, 0.009, 0.018]:
            for dy in [-0.018, -0.009, 0.0, 0.009, 0.018]:
                for dz in [-0.006, 0.0, 0.006]:
                    offsets.append(np.asarray([dx, dy, dz], dtype=np.float64))
        for idx, offset in enumerate(offsets):
            candidates.append(run_one(offset, f"sweep_{idx:03d}"))
        candidates.sort(key=lambda item: item["score"], reverse=True)
        summary = {
            "grasp_preset": "pinch_grasp_box_v2",
            "lift_preset": "pinch_lift_box_v1",
            "hand_stages": ["pinch_box_thumb_rotate_v2", "pinch_box_v4"],
            "rotate_raw": rotate_raw,
            "close_raw": close_raw,
            "rotate_ctrl_mujoco": rotate_ctrl.tolist(),
            "close_ctrl_mujoco": close_ctrl.tolist(),
            "box_size": box_size.tolist(),
            "estimate": estimate,
            "top_candidates": candidates[:10],
        }
        (out_dir / "pinch_box_pose_sweep.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(json.dumps(summary, indent=2))
        return

    offset = np.asarray(args.box_offset, dtype=np.float64)
    one = run_one(offset, "pinch_box_pose_debug")
    xml_path = Path(one["xml"])
    result = one["result"]
    summary = {
        "xml": str(xml_path),
        "grasp_preset": "pinch_grasp_box_v2",
        "lift_preset": "pinch_lift_box_v1",
        "hand_stages": ["pinch_box_thumb_rotate_v2", "pinch_box_v4"],
        "rotate_raw": rotate_raw,
        "close_raw": close_raw,
        "rotate_ctrl_mujoco": rotate_ctrl.tolist(),
        "close_ctrl_mujoco": close_ctrl.tolist(),
        "box_size": box_size.tolist(),
        "estimate": estimate,
        "result": result,
    }
    (out_dir / "pinch_box_pose_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
