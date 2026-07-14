#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_rh56_grasp_benchmark import (  # noqa: E402
    ARM_ACTUATOR_NAMES,
    BASE_XML,
    COLLISION_MODES,
    HAND_ACTUATOR_NAMES,
    _ids,
    _load_yaml,
    _physical_norm_to_mujoco_ctrl,
)
from pregrasp import evaluate_rh56_hardware_constraints, load_primitive_config, rh56_default_primitives  # noqa: E402
from view_mujoco_rh56_pose_contact import (  # noqa: E402
    PHYSICAL_POSES,
    THUMB_COUPLINGS,
    _build_pose_xml,
    _set_hand_qpos_from_ctrl,
)

REQUIRED_PAD_PROXIES = {
    "thumb_pad_proxy",
    "index_pad_proxy",
    "middle_pad_proxy",
    "ring_pad_proxy",
    "pinky_pad_proxy",
}


def _canonical_to_physical_norm(command: list[float]) -> list[float]:
    # canonical: [index, middle, ring, pinky, thumb_close, thumb_lateral]
    return [command[3], command[2], command[1], command[0], command[4], command[5]]


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
                "dist_m": float(contact.dist),
                "penetration_mm": max(0.0, -float(contact.dist) * 1000.0),
            }
        )
    return pairs


def _classify_contacts(pairs: list[dict[str, Any]]) -> dict[str, Any]:
    hand_self = 0
    hand_table = 0
    max_penetration_mm = 0.0
    for pair in pairs:
        joined = f"{pair['geom1']} {pair['geom2']}"
        max_penetration_mm = max(max_penetration_mm, float(pair["penetration_mm"]))
        if "bench_table" in joined or "floor" in joined:
            if "rh56_R_" in joined or "pad_proxy" in joined:
                hand_table += 1
        if joined.count("rh56_R_") >= 2 or (
            "pad_proxy" in joined and ("rh56_R_" in joined or joined.count("pad_proxy") >= 2)
        ):
            hand_self += 1
    return {
        "total": len(pairs),
        "hand_self": hand_self,
        "hand_table": hand_table,
        "max_penetration_mm": round(max_penetration_mm, 3),
    }


def _audit_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    arm_q: np.ndarray,
    arm_ids: np.ndarray,
    hand_ids: np.ndarray,
    thumb_coupling: str,
    physical_norm: list[float],
) -> dict[str, Any]:
    mujoco.mj_resetData(model, data)
    hand_ctrl = _physical_norm_to_mujoco_ctrl(physical_norm)
    data.qpos[:6] = arm_q
    _set_hand_qpos_from_ctrl(data, hand_ctrl, thumb_coupling=thumb_coupling)
    data.ctrl[arm_ids] = arm_q
    data.ctrl[hand_ids] = hand_ctrl
    mujoco.mj_forward(model, data)
    pairs = _contact_pairs(model, data)
    summary = _classify_contacts(pairs)
    return {
        "physical_norm": np.asarray(physical_norm, dtype=np.float64).round(6).tolist(),
        "mujoco_ctrl": hand_ctrl.round(6).tolist(),
        "summary": summary,
        "pairs": pairs[:12],
    }


def audit(args: argparse.Namespace) -> dict[str, Any]:
    out_xml = Path(args.out_xml)
    _build_pose_xml(
        Path(args.base_xml),
        out_xml,
        thumb_coupling=args.thumb_coupling,
        collision_mode=args.collision_mode,
    )
    model = mujoco.MjModel.from_xml_path(str(out_xml))
    data = mujoco.MjData(model)
    robot_cfg = _load_yaml(args.robot_config)
    arm_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    geom_names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, idx) or ""
        for idx in range(model.ngeom)
    }

    primitives = (
        load_primitive_config(args.primitive_config)
        if args.primitive_config
        else rh56_default_primitives()
    )
    pose_rows: dict[str, Any] = {}
    for name, physical_norm in PHYSICAL_POSES.items():
        pose_rows[name] = _audit_pose(
            model,
            data,
            arm_q=arm_q,
            arm_ids=arm_ids,
            hand_ids=hand_ids,
            thumb_coupling=args.thumb_coupling,
            physical_norm=physical_norm,
        )

    primitive_rows: dict[str, Any] = {}
    blockers: list[str] = []
    for primitive in primitives:
        row = _audit_pose(
            model,
            data,
            arm_q=arm_q,
            arm_ids=arm_ids,
            hand_ids=hand_ids,
            thumb_coupling=args.thumb_coupling,
            physical_norm=_canonical_to_physical_norm(primitive.hand_command),
        )
        summary = row["summary"]
        hardware = evaluate_rh56_hardware_constraints(
            primitive.hand_command,
            max_thumb_index_blocking_risk=args.max_thumb_index_blocking_risk,
        )
        blocked = bool(
            summary["hand_self"] > args.max_hand_self_contacts
            or summary["hand_table"] > 0
            or summary["max_penetration_mm"] > args.max_penetration_mm
            or not hardware.feasible
        )
        row["hardware_constraints"] = hardware.to_dict()
        row["dataset_ready"] = not blocked
        if blocked:
            blockers.append(primitive.name)
        primitive_rows[primitive.name] = row

    missing_proxies = (
        sorted(REQUIRED_PAD_PROXIES - geom_names)
        if args.collision_mode == "unifuc_pad_proxy"
        else []
    )
    if missing_proxies:
        blockers.append("missing_pad_proxies")

    return {
        "base_xml": str(args.base_xml),
        "audit_xml": str(out_xml),
        "collision_mode": args.collision_mode,
        "thumb_coupling": args.thumb_coupling,
        "arm_preset": args.arm_preset,
        "thresholds": {
            "max_penetration_mm": args.max_penetration_mm,
            "max_hand_self_contacts": args.max_hand_self_contacts,
            "max_thumb_index_blocking_risk": args.max_thumb_index_blocking_risk,
        },
        "missing_required_pad_proxies": missing_proxies,
        "reference_poses": pose_rows,
        "pregrasp_primitives": primitive_rows,
        "dataset_blockers": blockers,
        "dataset_ready": len(blockers) == 0,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Audit RH56 MuJoCo collision readiness before dataset generation.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--arm-preset", default="upright")
    parser.add_argument("--primitive-config", default="configs/pregrasp/rh56_pregrasp.yaml")
    parser.add_argument("--out-xml", default="data/collision_diagnostics/rh56_collision_readiness.xml")
    parser.add_argument("--output", default="data/collision_diagnostics/rh56_collision_readiness.json")
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="visual_coacd")
    parser.add_argument("--thumb-coupling", choices=sorted(THUMB_COUPLINGS), default="urdf")
    parser.add_argument("--max-penetration-mm", type=float, default=1.5)
    parser.add_argument("--max-hand-self-contacts", type=int, default=0)
    parser.add_argument("--max-thumb-index-blocking-risk", type=float, default=0.70)
    parser.add_argument("--fail-on-blocker", action="store_true")
    args = parser.parse_args()

    report = audit(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")

    for name, row in report["pregrasp_primitives"].items():
        summary = row["summary"]
        print(
            f"primitive={name:16s} ready={row['dataset_ready']} "
            f"contacts={summary['total']} hand_self={summary['hand_self']} "
            f"max_penetration_mm={summary['max_penetration_mm']}"
        )
    print(f"dataset_ready={report['dataset_ready']} blockers={report['dataset_blockers']}")
    print(f"wrote {output}")
    if args.fail_on_blocker and not report["dataset_ready"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
