from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

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
from view_mujoco_rh56_pose_contact import (  # noqa: E402
    DEFAULT_CODEBOOK,
    POSE_XML,
    THUMB_COUPLINGS,
    _build_pose_xml,
    _load_codebook,
    _set_hand_qpos_from_ctrl,
)


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


def _count_contacts(pairs: list[dict[str, Any]]) -> dict[str, int]:
    counts = {
        "total": len(pairs),
        "hand_self": 0,
        "proxy_contact": 0,
        "mesh_contact": 0,
        "table": 0,
    }
    for pair in pairs:
        joined = f"{pair['geom1']} {pair['geom2']}"
        if "bench_table" in joined:
            counts["table"] += 1
        if "collision" in joined or "pad_proxy" in joined:
            counts["proxy_contact"] += 1
        if "geom_0" in joined:
            counts["mesh_contact"] += 1
        if joined.count("rh56_R_") >= 2:
            counts["hand_self"] += 1
    return counts


def check(args: argparse.Namespace) -> dict[str, Any]:
    out_xml = Path(args.out_xml)
    _build_pose_xml(
        Path(args.base_xml),
        out_xml,
        thumb_coupling=args.thumb_coupling,
        collision_mode=args.collision_mode,
    )
    centroids, metadata = _load_codebook(Path(args.codebook))
    model = mujoco.MjModel.from_xml_path(str(out_xml))
    data = mujoco.MjData(model)
    robot_cfg = _load_yaml(args.robot_config)
    arm_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    arm_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, ARM_ACTUATOR_NAMES)
    hand_ids = _ids(model, mujoco.mjtObj.mjOBJ_ACTUATOR, HAND_ACTUATOR_NAMES)
    occupancy = metadata.get("sampled_code_occupancy") or []

    results: list[dict[str, Any]] = []
    for code_idx, physical_norm in enumerate(centroids):
        hand_ctrl = _physical_norm_to_mujoco_ctrl(physical_norm)
        mujoco.mj_resetData(model, data)
        data.qpos[:6] = arm_q
        _set_hand_qpos_from_ctrl(data, hand_ctrl, thumb_coupling=args.thumb_coupling)
        data.ctrl[arm_ids] = arm_q
        data.ctrl[hand_ids] = hand_ctrl
        mujoco.mj_forward(model, data)
        pairs = _contact_pairs(model, data)
        results.append(
            {
                "code": code_idx,
                "marker": "anchor/rare" if (code_idx < len(occupancy) and occupancy[code_idx] == 0.0) or float(physical_norm[5]) > 0.3 else "data",
                "occupancy": float(occupancy[code_idx]) if code_idx < len(occupancy) else 0.0,
                "physical_norm": np.asarray(physical_norm).round(6).tolist(),
                "mujoco_ctrl": hand_ctrl.round(6).tolist(),
                "counts": _count_contacts(pairs),
                "pairs": pairs,
            }
        )
    return {
        "codebook": str(args.codebook),
        "collision_mode": args.collision_mode,
        "thumb_coupling": args.thumb_coupling,
        "results": results,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Check static MuJoCo contacts for RH56 hand codebook poses.")
    parser.add_argument("--codebook", default=str(DEFAULT_CODEBOOK))
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--out-xml", default=str(POSE_XML))
    parser.add_argument("--thumb-coupling", choices=sorted(THUMB_COUPLINGS), default="urdf")
    parser.add_argument("--collision-mode", choices=COLLISION_MODES, default="proxy")
    parser.add_argument("--output", default="data/collision_diagnostics/rh56_codebook_contacts.json")
    args = parser.parse_args()

    result = check(args)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    for item in result["results"]:
        counts = item["counts"]
        print(
            f"code={item['code']:02d} marker={item['marker']} occ={item['occupancy']:.6f} "
            f"total={counts['total']} hand_self={counts['hand_self']} proxy={counts['proxy_contact']} "
            f"physical={item['physical_norm']}"
        )
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
