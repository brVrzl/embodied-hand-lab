#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mujoco_rh56_grasp_benchmark import (  # noqa: E402
    BASE_XML,
    OBJECTS,
    ObjectSpec,
    _build_scene_xml,
    _estimate_nominal_object_pos,
    _load_yaml,
    _physical_norm_to_mujoco_ctrl,
    _point_cloud_summary,
    _run_candidate,
    _sample_object_point_cloud,
    _solve_hand_base_lift_q,
)
from pregrasp import (  # noqa: E402
    evaluate_rh56_hardware_constraints,
    geometry_from_point_cloud,
    load_primitive_config,
    rh56_default_primitives,
)
from rh56_driver.hand_schema import CANONICAL_HAND_ORDER  # noqa: E402

DATASET_SCHEMA_VERSION = "rh56_pregrasp_mujoco_v0.1"


def _canonical_to_physical_norm(command: list[float]) -> list[float]:
    # canonical: [index, middle, ring, pinky, thumb_close, thumb_lateral]
    return [command[3], command[2], command[1], command[0], command[4], command[5]]


def _rotate_only_command(command: list[float]) -> list[float]:
    return [0.0, 0.0, 0.0, 0.0, 0.0, command[5]]


def _candidate_offsets(radius_m: float) -> list[list[float]]:
    r = float(radius_m)
    return [
        [0.0, 0.0, 0.0],
        [r, 0.0, 0.0],
        [-r, 0.0, 0.0],
        [0.0, r, 0.0],
        [0.0, -r, 0.0],
        [r, r, 0.004],
        [-r, r, 0.004],
        [r, -r, 0.004],
        [-r, -r, 0.004],
    ]


def _selected_objects(names: list[str]) -> list[tuple[str, ObjectSpec]]:
    selected = list(OBJECTS) if names == ["all"] else names
    rows: list[tuple[str, ObjectSpec]] = []
    for name in selected:
        if name not in OBJECTS:
            raise ValueError(f"Unknown object {name}; choices={sorted(OBJECTS)}")
        rows.append((name, OBJECTS[name]))
    return rows


def generate_dataset(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    scene_dir = out_dir / "scenes"
    scene_dir.mkdir(parents=True, exist_ok=True)
    samples_path = out_dir / "samples.jsonl"
    if samples_path.exists():
        samples_path.unlink()

    robot_cfg = _load_yaml(args.robot_config)
    grasp_q = np.asarray(robot_cfg["joint_presets"][args.arm_preset], dtype=np.float64)
    lift_q = _solve_hand_base_lift_q(Path(args.base_xml), grasp_q=grasp_q, lift_dz=args.lift_dz)
    primitives = load_primitive_config(args.primitive_config) if args.primitive_config else rh56_default_primitives()
    offsets = _candidate_offsets(args.offset_radius_m)[: args.offsets_per_primitive]

    manifest: dict[str, Any] = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "created_time": time.time(),
        "base_xml": str(args.base_xml),
        "robot_config": str(args.robot_config),
        "arm_preset": args.arm_preset,
        "collision_mode": args.collision_mode,
        "canonical_hand_order": list(CANONICAL_HAND_ORDER),
        "primitive_config": str(args.primitive_config) if args.primitive_config else "default",
        "lift_dz_m": args.lift_dz,
        "success_lift_m": args.success_lift,
        "objects": {},
        "num_samples": 0,
        "num_success": 0,
        "num_filtered_hardware": 0,
    }

    sample_index = 0
    with samples_path.open("a", encoding="utf-8") as handle:
        for object_name, spec in _selected_objects(args.objects):
            object_dir = out_dir / "objects" / object_name
            object_dir.mkdir(parents=True, exist_ok=True)
            points = _sample_object_point_cloud(spec, n=args.point_count)
            point_cloud_path = object_dir / "object_point_cloud.npy"
            np.save(point_cloud_path, points)
            geometry = geometry_from_point_cloud(points, frame_id="object", shape_hint=spec.geom_type)
            point_summary = _point_cloud_summary(points)
            object_success = 0
            object_samples = 0

            for primitive in primitives:
                hardware = evaluate_rh56_hardware_constraints(
                    primitive.hand_command,
                    max_thumb_index_blocking_risk=args.max_thumb_index_blocking_risk,
                )
                if not hardware.feasible:
                    manifest["num_filtered_hardware"] += len(offsets)
                    continue

                close_physical = _canonical_to_physical_norm(primitive.hand_command)
                rotate_physical = _rotate_only_command(primitive.hand_command)
                close_ctrl = _physical_norm_to_mujoco_ctrl(close_physical)
                rotate_ctrl = _physical_norm_to_mujoco_ctrl(rotate_physical)
                nominal_pos = _estimate_nominal_object_pos(
                    Path(args.base_xml),
                    grasp_q,
                    close_ctrl,
                    spec,
                    collision_mode=args.collision_mode,
                )
                for offset_idx, offset in enumerate(offsets):
                    object_pos = nominal_pos + np.asarray(offset, dtype=np.float64)
                    object_pos[2] = max(object_pos[2], spec.half_height + 0.016)
                    table_top_z = float(object_pos[2] - spec.half_height - 0.002)
                    xml_path = scene_dir / object_name / f"{primitive.name}_o{offset_idx}.xml"
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
                    sample = {
                        "schema_version": DATASET_SCHEMA_VERSION,
                        "sample_index": sample_index,
                        "object": {
                            "name": object_name,
                            "geom_type": spec.geom_type,
                            "size": list(spec.size),
                            "mass": spec.mass,
                            "point_cloud_path": str(point_cloud_path),
                            "point_summary": point_summary,
                            "geometry": geometry.to_dict(),
                        },
                        "candidate": {
                            "primitive": primitive.to_dict(),
                            "offset_index": offset_idx,
                            "object_offset_xyz_m": offset,
                            "object_spawn_xyz": object_pos.round(6).tolist(),
                            "hardware_constraints": hardware.to_dict(),
                            "close_physical_norm": np.asarray(close_physical).round(6).tolist(),
                            "rotate_physical_norm": np.asarray(rotate_physical).round(6).tolist(),
                            "close_mujoco_ctrl": close_ctrl.round(6).tolist(),
                            "rotate_mujoco_ctrl": rotate_ctrl.round(6).tolist(),
                        },
                        "simulation": result,
                        "label": {
                            "success": bool(result["success"]),
                            "score": float(result["score"]),
                            "failure_mode": _failure_mode(result),
                        },
                    }
                    handle.write(json.dumps(sample) + "\n")
                    sample_index += 1
                    object_samples += 1
                    if result["success"]:
                        object_success += 1

            manifest["objects"][object_name] = {
                "num_samples": object_samples,
                "num_success": object_success,
                "point_cloud_path": str(point_cloud_path),
            }
            manifest["num_samples"] += object_samples
            manifest["num_success"] += object_success

    (out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return manifest


def _failure_mode(result: dict[str, Any]) -> str:
    if result["success"]:
        return "none"
    contacts = result["final_contacts"]
    if result["initial_penetration"]:
        return "initial_penetration"
    if contacts["hand_self"] > 0:
        return "hand_self_collision"
    if contacts["object_table"] > 0:
        return "object_on_table"
    if not result["opposing_contact"]:
        return "missing_opposing_contact"
    if result["lift_m"] < result["success_lift_m"]:
        return "insufficient_lift"
    return "unknown"


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate a MuJoCo RH56 pregrasp dataset from audited primitives.")
    parser.add_argument("--base-xml", default=str(BASE_XML))
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--primitive-config", default="configs/pregrasp/rh56_pregrasp.yaml")
    parser.add_argument("--objects", nargs="+", default=["all"])
    parser.add_argument("--out-dir", default="data/rh56_pregrasp_dataset_v0")
    parser.add_argument("--collision-mode", default="unifuc_pad_proxy")
    parser.add_argument("--point-count", type=int, default=768)
    parser.add_argument("--offset-radius-m", type=float, default=0.010)
    parser.add_argument("--offsets-per-primitive", type=int, default=5)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--lift-dz", type=float, default=0.075)
    parser.add_argument("--success-lift", type=float, default=0.025)
    parser.add_argument("--max-thumb-index-blocking-risk", type=float, default=0.70)
    args = parser.parse_args()

    manifest = generate_dataset(args)
    print(
        json.dumps(
            {
                "out_dir": args.out_dir,
                "num_samples": manifest["num_samples"],
                "num_success": manifest["num_success"],
                "num_filtered_hardware": manifest["num_filtered_hardware"],
                "objects": manifest["objects"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
