from __future__ import annotations

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_maniskill.rh56_collision_validation import (  # noqa: E402
    ACTUATOR_NAMES,
    ARM_ACTUATOR_NAMES,
    CANONICAL_PHYSICAL_POSES,
    CommandProfile,
    canonical_target_ctrl,
    classify_representation_comparison,
    default_command_profiles,
    run_trajectory_validation,
    write_trajectory_artifacts,
)
from view_mujoco_rh56_pose_contact import _build_pose_xml  # noqa: E402


COLLISION_MODES = ("visual_coacd", "correll_mesh", "unifuc_pad_proxy")
STRATEGIES = ("simultaneous", "thumb_first", "finger_first", "iterative_incremental")


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _add_stage2_object(
    root: ET.Element,
    *,
    object_name: str = "round_ball",
    object_pos: tuple[float, float, float] = (-0.11, -0.50, 0.050),
    include_table: bool = True,
    gravity_compensated: bool = False,
    table_top_z: float = 0.024,
    table_half_size_xy: tuple[float, float] = (0.18, 0.16),
) -> None:
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("Missing worldbody.")
    if any(geom.get("name") == "stage2_table" for geom in root.iter("geom")):
        return
    if include_table:
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "stage2_table",
                "type": "box",
                "pos": f"{object_pos[0]:.6f} {object_pos[1]:.6f} {table_top_z - 0.006:.6f}",
                "size": f"{table_half_size_xy[0]:.6f} {table_half_size_xy[1]:.6f} 0.006",
                "friction": "1.4 0.05 0.003",
                "condim": "4",
                "rgba": "0.72 0.66 0.56 1",
            },
        )
    specs = {
        "round_ball": {"type": "sphere", "size": "0.022", "mass": "0.030", "rgba": "0.25 0.68 0.36 1"},
        "foam_cube": {"type": "box", "size": "0.018 0.018 0.018", "mass": "0.018", "rgba": "0.85 0.30 0.18 1"},
    }
    if object_name not in specs:
        raise ValueError(f"Unknown Stage 2 object {object_name!r}; choices={sorted(specs)}")
    obj = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "stage2_object_body",
            "pos": " ".join(f"{value:.9f}" for value in object_pos),
            **({"gravcomp": "1"} if gravity_compensated else {}),
        },
    )
    ET.SubElement(obj, "freejoint", {"name": "stage2_object_freejoint"})
    ET.SubElement(
        obj,
        "geom",
        {
            "name": "stage2_object",
            "type": specs[object_name]["type"],
            "size": specs[object_name]["size"],
            "mass": specs[object_name]["mass"],
            "friction": "1.8 0.08 0.004",
            "condim": "4",
            "priority": "1",
            "rgba": specs[object_name]["rgba"],
        },
    )


def _build_stage2_xml(
    *,
    base_xml: Path,
    out_xml: Path,
    collision_mode: str,
    include_object: bool,
    object_name: str = "round_ball",
    object_pos: tuple[float, float, float] = (-0.11, -0.50, 0.050),
    include_object_table: bool = True,
    gravity_compensated_object: bool = False,
    object_table_top_z: float = 0.024,
    object_table_half_size_xy: tuple[float, float] = (0.18, 0.16),
) -> None:
    _build_pose_xml(base_xml, out_xml, collision_mode=collision_mode)
    if include_object:
        tree = ET.parse(out_xml)
        root = tree.getroot()
        _add_stage2_object(
            root,
            object_name=object_name,
            object_pos=object_pos,
            include_table=include_object_table,
            gravity_compensated=gravity_compensated_object,
            table_top_z=object_table_top_z,
            table_half_size_xy=object_table_half_size_xy,
        )
        root.set("model", f"rh56_stage2a_{collision_mode}_object")
        tree.write(out_xml, encoding="utf-8", xml_declaration=False)


def _arm_qpos(robot_config: Path, arm_preset: str) -> np.ndarray:
    robot_cfg = _load_yaml(robot_config)
    return np.asarray(robot_cfg["joint_presets"][arm_preset], dtype=np.float64)


def _profile_with_timeout(profile: CommandProfile, timeout_scale: float) -> CommandProfile:
    if timeout_scale == 1.0:
        return profile
    return CommandProfile(
        name=profile.name,
        max_velocity_ctrl_per_s=profile.max_velocity_ctrl_per_s,
        max_accel_ctrl_per_s2=profile.max_accel_ctrl_per_s2,
        settle_seconds=profile.settle_seconds,
        hold_seconds=profile.hold_seconds,
        timeout_seconds=profile.timeout_seconds * timeout_scale,
        error_tolerance_ctrl=profile.error_tolerance_ctrl,
        progress_window_seconds=profile.progress_window_seconds,
        progress_epsilon_ctrl=profile.progress_epsilon_ctrl,
        persistent_contact_seconds=profile.persistent_contact_seconds,
        transient_contact_seconds=profile.transient_contact_seconds,
        force_blockage_threshold=profile.force_blockage_threshold,
        hybrid_slowdown_error_ctrl=profile.hybrid_slowdown_error_ctrl,
        hybrid_near_contact_scale=profile.hybrid_near_contact_scale,
    )


def _is_blocked_or_forbidden(row: dict[str, Any]) -> bool:
    return bool(row["blocked"] or row["outcome"] == "forbidden_structural_collision")


def _reference_rows(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    return [row for row in rows if row["collision_mode"] in {"correll_mesh", "unifuc_pad_proxy"}]


def _annotate_mode_comparison(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    comparisons: list[dict[str, Any]] = []
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in results:
        key = (row["scene"], row["target_name"], row["strategy"])
        grouped.setdefault(key, []).append(row)

    for (scene, target_name, strategy), rows in grouped.items():
        by_mode = {row["collision_mode"]: row for row in rows}
        visual = by_mode.get("visual_coacd")
        refs = _reference_rows(rows)
        if visual is None or not refs:
            continue

        refs_with_self_contact = [
            row
            for row in refs
            if row["first_blocking_pair"] or row["first_forbidden_pair"] or row["max_rh56_self_penetration_m"] > 0.0
        ]
        evidence = visual.get("original_visual_diagnostic", {})
        comparison_semantics = classify_representation_comparison(
            visual_coacd=visual,
            references=refs,
            original_visual_intersects=evidence.get("intersects"),
            original_visual_gap_m=evidence.get("minimum_surface_distance_m"),
        )
        classification = comparison_semantics["classification"]
        reason = comparison_semantics["reason"]

        comparison = {
            "scene": scene,
            "target_name": target_name,
            "strategy": strategy,
            "classification": classification,
            "reason": reason,
            "root_cause_classification": comparison_semantics.get("root_cause_classification"),
            "reference_modes_are_ground_truth": False,
            "visual_coacd": {
                "outcome": visual["outcome"],
                "blockage_kind": visual["blockage_kind"],
                "first_blocking_pair": visual["first_blocking_pair"],
                "first_forbidden_pair": visual["first_forbidden_pair"],
                "first_contact_time": visual["first_contact_time"],
                "max_rh56_self_penetration_m": visual["max_rh56_self_penetration_m"],
                "final_target_error": visual["final_target_error"],
            },
            "references": [
                {
                    "collision_mode": row["collision_mode"],
                    "outcome": row["outcome"],
                    "blockage_kind": row["blockage_kind"],
                    "first_blocking_pair": row["first_blocking_pair"],
                    "first_forbidden_pair": row["first_forbidden_pair"],
                    "first_contact_time": row["first_contact_time"],
                    "max_rh56_self_penetration_m": row["max_rh56_self_penetration_m"],
                    "final_target_error": row["final_target_error"],
                }
                for row in refs
            ],
            "reference_self_contact_or_penetration_seen": bool(refs_with_self_contact),
        }
        comparisons.append(comparison)
        for row in rows:
            row["mode_comparison_classification"] = classification
            row["mode_comparison_reason"] = reason
    return comparisons


def _path_dependent_outcomes(results: list[dict[str, Any]]) -> None:
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in results:
        key = (row["collision_mode"], row["scene"], row["target_name"])
        grouped.setdefault(key, []).append(row)
    for rows in grouped.values():
        any_reached = any(row["reached"] for row in rows)
        if not any_reached:
            continue
        for row in rows:
            if row["outcome"] == "blocked":
                row["path_dependent_obstruction"] = True
                row["interpretation"] = (
                    "This command order blocked, but another reviewed strategy reached the same target "
                    "under the same collision mode. Treat as path/order dependent until reviewed."
                )
            else:
                row["path_dependent_obstruction"] = False


def run(args: argparse.Namespace) -> dict[str, Any]:
    out_dir = Path(args.out_dir)
    xml_dir = out_dir / "xml"
    xml_dir.mkdir(parents=True, exist_ok=True)
    arm_q = _arm_qpos(Path(args.robot_config), args.arm_preset)
    profiles = default_command_profiles()
    profile = _profile_with_timeout(profiles[args.profile], args.timeout_scale)
    scenes = ["object_free"] + (["object_present"] if args.include_object else [])
    results: list[dict[str, Any]] = []

    for scene in scenes:
        include_object = scene == "object_present"
        for collision_mode in args.collision_modes:
            xml_path = xml_dir / f"{scene}_{collision_mode}.xml"
            _build_stage2_xml(
                base_xml=Path(args.base_xml),
                out_xml=xml_path,
                collision_mode=collision_mode,
                include_object=include_object,
            )
            for target_name in args.targets:
                target_ctrl = canonical_target_ctrl(target_name)
                for strategy in args.strategies:
                    model = mujoco.MjModel.from_xml_path(str(xml_path))
                    result = run_trajectory_validation(
                        model,
                        collision_mode=collision_mode,
                        target_name=target_name,
                        target_ctrl=target_ctrl,
                        strategy=strategy,
                        profile=profile,
                        arm_qpos=arm_q,
                        sample_stride=args.sample_stride,
                    )
                    run_dir = out_dir / scene / collision_mode / target_name / strategy / profile.name
                    reproduction = (
                        f".venv/bin/python tools/validate_rh56_visual_coacd_stage2.py "
                        f"--out-dir {out_dir} --collision-modes {collision_mode} --targets {target_name} "
                        f"--strategies {strategy} --profile {profile.name}"
                    )
                    if include_object:
                        reproduction += " --include-object"
                    write_trajectory_artifacts(result, run_dir, reproduction_command=reproduction)
                    row = result.summary_dict(include_samples=False, include_contacts=False)
                    row.update(
                        {
                            "scene": scene,
                            "xml": str(xml_path),
                            "artifact_dir": str(run_dir),
                            "target_physical_norm": CANONICAL_PHYSICAL_POSES[target_name],
                            "actuator_order": list(ACTUATOR_NAMES),
                            "arm_actuator_names": list(ARM_ACTUATOR_NAMES),
                            "visual_mesh_intersection": {
                                "status": "not_evaluated_stage2a",
                                "reason": (
                                    "This smoke validator compares collision modes on identical dynamic "
                                    "trajectories; exact vendor visual mesh triangle intersection remains "
                                    "a separate diagnostic."
                                ),
                            },
                        }
                    )
                    results.append(row)

    _path_dependent_outcomes(results)
    mode_comparisons = _annotate_mode_comparison(results)
    report = {
        "schema": "rh56_visual_coacd_stage2a_dynamic_validation_v0.2",
        "base_xml": str(args.base_xml),
        "robot_config": str(args.robot_config),
        "arm_preset": args.arm_preset,
        "collision_modes": args.collision_modes,
        "targets": args.targets,
        "strategies": args.strategies,
        "profile": profile.name,
        "profile_details": {
            "max_velocity_ctrl_per_s": list(profile.max_velocity_ctrl_per_s),
            "max_accel_ctrl_per_s2": list(profile.max_accel_ctrl_per_s2),
            "settle_seconds": profile.settle_seconds,
            "hold_seconds": profile.hold_seconds,
            "timeout_seconds": profile.timeout_seconds,
            "error_tolerance_ctrl": profile.error_tolerance_ctrl,
            "hybrid_slowdown_error_ctrl": profile.hybrid_slowdown_error_ctrl,
            "hybrid_near_contact_scale": profile.hybrid_near_contact_scale,
        },
        "known_speed_mapping": {
            "hardware_raw_speed_units": (
                "available as RH56 SPEED_SET registers and config defaults such as 500 or 800"
            ),
            "raw_speed_to_rad_s": "unavailable in repository; this validator does not invent it",
            "nominal_profile_basis": (
                "teleop delta_limit=0.05 normalized units at command_hz=15 mapped to MuJoCo "
                "actuator ranges"
            ),
        },
        "mode_comparisons": mode_comparisons,
        "stage2a_limitations": [
            (
                "Exact vendor visual-mesh triangle intersection is still reported as a diagnostic "
                "placeholder, not as a CI gate."
            ),
            (
                "A blocked command order is not treated as a collision-mesh defect when another "
                "reviewed order reaches the same target."
            ),
            "Final Stage 2B CI gates remain intentionally unimplemented.",
        ],
        "results": results,
    }
    (out_dir / "stage2a_summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description="Stage 2A dynamic RH56 visual_coacd trajectory validator.")
    parser.add_argument("--base-xml", default="data/sim_assets/jaka_rh56.xml")
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--arm-preset", default="upright")
    parser.add_argument("--out-dir", default="/tmp/rh56_visual_coacd_stage2a")
    parser.add_argument("--collision-modes", nargs="+", choices=COLLISION_MODES, default=list(COLLISION_MODES))
    parser.add_argument("--targets", nargs="+", choices=sorted(CANONICAL_PHYSICAL_POSES), default=["sim_best_pinch", "power_close"])
    parser.add_argument("--strategies", nargs="+", choices=STRATEGIES, default=list(STRATEGIES))
    parser.add_argument("--profile", choices=sorted(default_command_profiles()), default="slow_validation")
    parser.add_argument("--timeout-scale", type=float, default=1.0)
    parser.add_argument("--sample-stride", type=int, default=5)
    parser.add_argument("--include-object", action="store_true")
    args = parser.parse_args()

    report = run(args)
    compact = [
        {
            "scene": row["scene"],
            "mode": row["collision_mode"],
            "target": row["target_name"],
            "strategy": row["strategy"],
            "outcome": row["outcome"],
            "blockage_kind": row["blockage_kind"],
            "first_blocking_pair": row["first_blocking_pair"],
            "max_penetration_m": row["max_penetration_m"],
            "max_rh56_self_penetration_m": row["max_rh56_self_penetration_m"],
            "final_target_error": row["final_target_error"],
            "mode_comparison_classification": row.get("mode_comparison_classification"),
            "artifact_dir": row["artifact_dir"],
        }
        for row in report["results"]
    ]
    print(json.dumps({"out_dir": args.out_dir, "results": compact}, indent=2))


if __name__ == "__main__":
    main()
