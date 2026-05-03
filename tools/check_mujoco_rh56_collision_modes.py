from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from mujoco_rh56_grasp_benchmark import ARM_ACTUATOR_NAMES, HAND_ACTUATOR_NAMES, _ids, _load_yaml, _physical_norm_to_mujoco_ctrl
from view_mujoco_rh56_pose_contact import PHYSICAL_POSES, _build_pose_xml, _set_hand_qpos_from_ctrl


def _contact_pairs(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    pairs: list[dict[str, Any]] = []
    for idx in range(data.ncon):
        contact = data.contact[idx]
        geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        pairs.append(
            {
                "geom1": geom1,
                "geom2": geom2,
                "dist": float(contact.dist),
                "pos": np.asarray(contact.pos, dtype=np.float64).round(6).tolist(),
            }
        )
    return pairs


def _summarize_pairs(pairs: list[dict[str, Any]]) -> dict[str, int]:
    counts = {
        "total": len(pairs),
        "hand_self": 0,
        "hand_floor_or_table": 0,
        "proxy_contact": 0,
        "mesh_contact": 0,
    }
    for pair in pairs:
        a = str(pair["geom1"])
        b = str(pair["geom2"])
        joined = f"{a} {b}"
        if a.startswith("rh56_R_") and b.startswith("rh56_R_"):
            counts["hand_self"] += 1
        if "rh56_R_" in joined and ("floor" in joined or "bench_table" in joined):
            counts["hand_floor_or_table"] += 1
        if "pad_proxy" in joined:
            counts["proxy_contact"] += 1
        if "rh56_R_" in joined and "pad_proxy" not in joined:
            counts["mesh_contact"] += 1
    return counts


def check_modes(args: argparse.Namespace) -> dict[str, Any]:
    robot_cfg = _load_yaml(args.robot_config)
    arm_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    report: dict[str, Any] = {
        "base_xml": str(args.base_xml),
        "arm_preset": args.arm_preset,
        "modes": {},
    }
    for collision_mode in args.collision_modes:
        out_xml = Path(args.out_dir) / f"pose_collision_{collision_mode}.xml"
        _build_pose_xml(
            Path(args.base_xml),
            out_xml,
            thumb_coupling=args.thumb_coupling,
            collision_mode=collision_mode,
        )
        model = mujoco.MjModel.from_xml_path(str(out_xml))
        data = mujoco.MjData(model)
        arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
        hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
        mode_rows: dict[str, Any] = {}
        for pose_name, physical_norm in PHYSICAL_POSES.items():
            mujoco.mj_resetData(model, data)
            hand_ctrl = _physical_norm_to_mujoco_ctrl(physical_norm)
            data.qpos[:6] = arm_q
            _set_hand_qpos_from_ctrl(data, hand_ctrl, thumb_coupling=args.thumb_coupling)
            data.ctrl[arm_ids] = arm_q
            data.ctrl[hand_ids] = hand_ctrl
            mujoco.mj_forward(model, data)
            pairs = _contact_pairs(model, data)
            mode_rows[pose_name] = {
                "physical_norm": physical_norm,
                "mujoco_ctrl": hand_ctrl.round(6).tolist(),
                "summary": _summarize_pairs(pairs),
                "pairs": pairs[: args.max_pairs],
            }
        report["modes"][collision_mode] = {
            "xml": str(out_xml),
            "poses": mode_rows,
        }
    out_report = Path(args.out_dir) / "pose_collision_report.json"
    out_report.parent.mkdir(parents=True, exist_ok=True)
    out_report.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Static RH56 MuJoCo collision diagnostics across collision modes.")
    parser.add_argument("--base-xml", default="data/sim_assets/jaka_rh56.xml")
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--arm-preset", default="upright")
    parser.add_argument("--thumb-coupling", choices=["urdf", "xacro", "gazebo_plugin"], default="urdf")
    parser.add_argument(
        "--collision-modes",
        nargs="+",
        choices=["proxy", "mesh", "mesh_proxy", "unifuc_pad_proxy"],
        default=["proxy", "mesh", "mesh_proxy", "unifuc_pad_proxy"],
    )
    parser.add_argument("--out-dir", default="data/collision_diagnostics/pose_modes")
    parser.add_argument("--max-pairs", type=int, default=20)
    args = parser.parse_args()
    report = check_modes(args)
    compact: dict[str, Any] = {}
    for mode, mode_data in report["modes"].items():
        compact[mode] = {
            pose: payload["summary"] for pose, payload in mode_data["poses"].items()
        }
    print(json.dumps(compact, indent=2))


if __name__ == "__main__":
    main()
