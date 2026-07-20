#!/usr/bin/env python3
"""Run a deterministic, simulation-only collision sweep in the clean P-world scene."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Mapping, Sequence

import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from digital_twin.collision_sweep import (  # noqa: E402
    ARM_ACTUATOR_NAMES,
    HAND_ACTUATOR_NAMES,
    PALM_BODY,
    PoseSample,
    actuator_ids,
    angle_degrees,
    bounded_duration,
    canonical_pair,
    contact_rows,
    deterministic_arm_samples,
    early_termination_reason,
    enforce_noncolliding_layers,
    minimum_robot_environment_distance,
    name_or_id,
    object_id,
    set_static_state,
    smoothstep_interpolation,
    update_consecutive_contact_durations,
    verify_operational_scene,
    write_csv,
)
from digital_twin.io import load_structured, write_json, write_yaml  # noqa: E402


STATIC_FIELDS = [
    "sample_id", "pose_name", "source", "role", "arm_qpos_rad", "hand_ctrl_rad", "palm_position_P_m",
    "palm_normal_P", "minimum_environment_distance_m", "nearest_environment_pair", "contact_count",
    "baseline_contact_count", "review_count", "warn_count", "fail_count", "status", "finite",
]
DYNAMIC_FIELDS = [
    "trajectory_id", "name", "source", "start_pose", "target_pose", "duration_s", "steps", "throughput_steps_s",
    "first_contact_time_s", "contact_duration_max_s", "max_penetration_m", "peak_normal_force_n", "baseline_contacts",
    "review_events", "warn_events", "fail_events", "solver_warning_count", "nonfinite", "early_termination", "status",
]
CONTACT_FIELDS = [
    "phase", "sample_id", "trajectory_id", "trajectory_name", "simulation_time_s", "step", "body_a", "body_b",
    "geom_a", "geom_b", "contact_position_m", "contact_normal_world", "penetration_depth_m", "normal_force_n",
    "tangent_force_1_n", "tangent_force_2_n", "tangent_resultant_n", "contact_duration_s", "category", "status",
    "baseline", "qpos", "qvel", "ctrl",
]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _joint_limits(model: mujoco.MjModel, margin_deg: float) -> np.ndarray:
    margin = math.radians(margin_deg)
    limits = []
    for name in [f"jaka_joint_{i}" for i in range(1, 7)]:
        joint = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        low, high = map(float, model.jnt_range[joint])
        limits.append([low + margin, high - margin])
    return np.asarray(limits)


def _load_repository_poses(model: mujoco.MjModel, robot_config_path: Path) -> list[PoseSample]:
    payload = load_structured(robot_config_path)
    limits = _joint_limits(model, float(payload.get("safety", {}).get("joint_limit_margin_deg", 5.0)))
    rows: list[PoseSample] = []
    for name, values in payload.get("joint_presets", {}).items():
        q = np.asarray(values, dtype=np.float64)
        if q.shape != (6,) or np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
            continue
        role = "nominal_safe_repository" if name in {"upright", "teleop_ready"} else "repository_task_pose"
        rows.append(PoseSample(name, tuple(q), (0.0,) * 6, f"{robot_config_path}:{name}", role))
    tennis = np.asarray([0.123, 0.429, 1.496, -1.447, -0.019, -2.164])
    rows.append(PoseSample("tennis_pregrasp", tuple(tennis), (0.0,) * 6, "tools/preview_mujoco_tennis_ball_lift.py:ARM_PREGRASP_QPOS", "repository_task_pose"))
    return rows


def _hand_samples() -> list[PoseSample]:
    zero = (0.0,) * 6
    closed = (0.75, 0.45, 1.25, 1.25, 1.25, 1.25)
    rows = [
        PoseSample("rh56_open", zero, zero, "tools/debug_mujoco_jaka_rh56_viewer.py:HAND_OPEN_CTRL", "hand_actuator_valid"),
        PoseSample("rh56_closed", zero, closed, "tools/debug_mujoco_jaka_rh56_viewer.py:HAND_CLOSE_CTRL", "hand_actuator_valid"),
        PoseSample("rh56_half_flexion", zero, tuple(0.5 * np.asarray(closed)), "derived_midpoint_of_repository_open_close", "hand_actuator_valid"),
    ]
    for axis, value in enumerate(closed):
        target = np.zeros(6); target[axis] = 0.5 * value
        rows.append(PoseSample(f"rh56_finger_axis_{axis}_half", zero, tuple(target), "one_actuator_at_a_time_repository_close_target", "hand_actuator_valid"))
    return rows


def _dedupe_samples(samples: Sequence[PoseSample]) -> list[PoseSample]:
    # Preserve semantically distinct named poses even when their numerical state
    # is identical (for example ``zero`` and ``rh56_open``).  Only duplicate
    # names are suppressed.
    seen: set[str] = set(); result: list[PoseSample] = []
    for sample in samples:
        if sample.name not in seen:
            seen.add(sample.name); result.append(sample)
    return result


def _status_from_rows(rows: Sequence[Mapping[str, Any]]) -> str:
    statuses = {str(row["status"]) for row in rows}
    for candidate in ("FAIL", "WARN", "REVIEW", "ALLOWED", "BASELINE"):
        if candidate in statuses:
            return candidate
    return "PASS"


def _evaluate_static(
    model: mujoco.MjModel,
    config: Mapping[str, Any],
    samples: Sequence[PoseSample],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    data = mujoco.MjData(model); rows: list[dict[str, Any]] = []; contacts: list[dict[str, Any]] = []; states: list[dict[str, Any]] = []
    proximity = float(config["thresholds"]["proximity_m"])
    palm_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, PALM_BODY)
    for index, sample in enumerate(samples):
        set_static_state(model, data, sample.arm_qpos, sample.hand_ctrl)
        finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.xpos).all())
        context = {"phase": "static", "sample_id": index, "trajectory_id": "", "trajectory_name": "", "step": 0}
        active = contact_rows(model, data, config, context=context)
        contacts.extend(active)
        counts = Counter(row["status"] for row in active)
        distance, nearest = minimum_robot_environment_distance(model, data, config, proximity)
        palm_rotation = data.xmat[palm_id].reshape(3, 3)
        row = {
            "sample_id": index, "pose_name": sample.name, "source": sample.source, "role": sample.role,
            "arm_qpos_rad": list(sample.arm_qpos), "hand_ctrl_rad": list(sample.hand_ctrl),
            "palm_position_P_m": data.xpos[palm_id].tolist(), "palm_normal_P": (palm_rotation @ [0.0, 1.0, 0.0]).tolist(),
            "minimum_environment_distance_m": distance, "nearest_environment_pair": nearest, "contact_count": len(active),
            "baseline_contact_count": counts["BASELINE"], "review_count": counts["REVIEW"], "warn_count": counts["WARN"],
            "fail_count": counts["FAIL"], "status": "FAIL" if not finite else _status_from_rows(active), "finite": finite,
        }
        rows.append(row)
        states.append({"sample_id": index, "name": sample.name, "qpos": data.qpos.tolist(), "row": row})
    return rows, contacts, states


def _select_directional_aliases(static_rows: Sequence[Mapping[str, Any]], samples: Sequence[PoseSample]) -> dict[str, int]:
    candidates = [
        (index, row) for index, row in enumerate(static_rows)
        if row["finite"] and row["fail_count"] == 0 and row["pose_name"].startswith(("oat_", "pair_", "halton_"))
    ]
    if not candidates:
        raise RuntimeError("No collision-free deterministic arm candidates are available for trajectory selection.")
    def select(key: Any, reverse: bool = False) -> int:
        return sorted(candidates, key=lambda item: key(item[1]), reverse=reverse)[0][0]
    aliases = {
        "arm_extended_P_negative_x": select(lambda row: row["palm_position_P_m"][0]),
        "arm_extended_P_positive_x": select(lambda row: row["palm_position_P_m"][0], True),
        "left_lateral_reach_P_positive_y": select(lambda row: row["palm_position_P_m"][1], True),
        "right_lateral_reach_P_negative_y": select(lambda row: row["palm_position_P_m"][1]),
        "low_tabletop_approach": select(lambda row: abs(row["palm_position_P_m"][2] - 0.02)),
        "wrist_down": select(lambda row: row["palm_normal_P"][2]),
        "rail_approach_positive_y": select(lambda row: abs(row["palm_position_P_m"][1] - 0.038890873)),
        "rail_approach_negative_y": select(lambda row: abs(row["palm_position_P_m"][1] + 0.038890873)),
    }
    # Elbow-low is selected separately with the hand/palm z proxy because static rows intentionally remain compact.
    aliases["elbow_low"] = select(lambda row: row["palm_position_P_m"][2])
    return aliases


def _warning_count(data: mujoco.MjData) -> int:
    try:
        return int(sum(int(item.number) for item in data.warning))
    except Exception:
        return 0


def _run_dynamic(
    model: mujoco.MjModel,
    config: Mapping[str, Any],
    name: str,
    start: PoseSample,
    target: PoseSample,
    trajectory_id: int,
) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, Any]]:
    dynamic = config["dynamic"]; dt = float(model.opt.timestep)
    start_ctrl = np.r_[start.arm_qpos, start.hand_ctrl]; target_ctrl = np.r_[target.arm_qpos, target.hand_ctrl]
    max_velocity = np.r_[[float(dynamic["max_arm_velocity_rad_s"])] * 6, [float(dynamic["max_hand_velocity_rad_s"])] * 6]
    max_acceleration = np.r_[[float(dynamic["max_arm_acceleration_rad_s2"])] * 6, [float(dynamic["max_hand_acceleration_rad_s2"])] * 6]
    duration = bounded_duration(start_ctrl, target_ctrl, base_duration_s=float(dynamic["base_duration_s"]), max_velocity=max_velocity, max_acceleration=max_acceleration)
    move_steps = max(2, int(math.ceil(duration / dt)) + 1)
    controls = smoothstep_interpolation(start_ctrl, target_ctrl, move_steps)
    settle_steps = int(math.ceil(float(dynamic["settle_s"]) / dt)); hold_steps = int(math.ceil(float(dynamic["hold_s"]) / dt))
    data = mujoco.MjData(model); set_static_state(model, data, start.arm_qpos, start.hand_ctrl)
    arm_ids = actuator_ids(model, ARM_ACTUATOR_NAMES); hand_ids = actuator_ids(model, HAND_ACTUATOR_NAMES)
    all_ids = np.r_[arm_ids, hand_ids]
    timeline: list[dict[str, Any]] = []; consecutive: dict[tuple[str, str], float] = {}
    first_contact_time: float | None = None; max_depth = 0.0; peak_force = 0.0; max_duration = 0.0
    snapshots: dict[str, Any] = {}; early = ""; start_wall = time.perf_counter(); total_steps = 0
    for step in range(settle_steps + move_steps + hold_steps):
        if step < settle_steps:
            command = start_ctrl
        elif step < settle_steps + move_steps:
            command = controls[step - settle_steps]
        else:
            command = target_ctrl
        data.ctrl[all_ids] = command
        mujoco.mj_step(model, data); total_steps += 1
        active_pairs = []
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            active_pairs.append(canonical_pair(
                name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)),
                name_or_id(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)),
            ))
        consecutive = update_consecutive_contact_durations(active_pairs, consecutive, dt)
        context = {"phase": "dynamic", "sample_id": "", "trajectory_id": trajectory_id, "trajectory_name": name, "step": step}
        active = contact_rows(model, data, config, context=context, consecutive_duration=consecutive)
        if active and first_contact_time is None:
            first_contact_time = float(data.time)
        for row in active:
            timeline.append(row)
            if not row["baseline"]:
                max_depth = max(max_depth, row["penetration_depth_m"])
                peak_force = max(peak_force, abs(row["normal_force_n"]))
                max_duration = max(max_duration, row["contact_duration_s"])
            if row["status"] in {"WARN", "FAIL"} and "first_issue" not in snapshots:
                snapshots["first_issue"] = {"qpos": data.qpos.tolist(), "detail": row}
            if not row["baseline"] and row["penetration_depth_m"] >= max_depth:
                snapshots["maximum_penetration"] = {"qpos": data.qpos.tolist(), "detail": row}
        early = early_termination_reason(data.qpos, data.qvel, active, float(config["thresholds"]["catastrophic_penetration_m"])) or ""
        if early == "nonfinite_state":
            snapshots["first_fail"] = {"qpos": data.qpos.tolist(), "detail": "nonfinite_state"}
        if early:
            break
    elapsed = max(time.perf_counter() - start_wall, 1e-9); counts = Counter(row["status"] for row in timeline)
    nonfinite = early == "nonfinite_state"; status = "FAIL" if nonfinite else _status_from_rows(timeline)
    result = {
        "trajectory_id": trajectory_id, "name": name, "source": f"actuator_smoothstep:{start.source}->{target.source}",
        "start_pose": start.name, "target_pose": target.name, "duration_s": float(data.time), "steps": total_steps,
        "throughput_steps_s": total_steps / elapsed, "first_contact_time_s": first_contact_time,
        "contact_duration_max_s": max_duration, "max_penetration_m": max_depth, "peak_normal_force_n": peak_force,
        "baseline_contacts": counts["BASELINE"], "review_events": counts["REVIEW"], "warn_events": counts["WARN"],
        "fail_events": counts["FAIL"], "solver_warning_count": _warning_count(data), "nonfinite": nonfinite,
        "early_termination": early, "status": status,
    }
    return result, timeline, snapshots


def _render_snapshot(
    model: mujoco.MjModel,
    snapshot: Mapping[str, Any],
    path: Path,
    camera: str,
    label: str,
    renderer: mujoco.Renderer | None = None,
) -> str | None:
    try:
        import cv2
        import imageio.v3 as iio
        data = mujoco.MjData(model); data.qpos[:] = np.asarray(snapshot["qpos"], dtype=np.float64); mujoco.mj_forward(model, data)
        active_renderer = renderer or mujoco.Renderer(model, height=720, width=960)
        active_renderer.update_scene(data, camera=camera); image = active_renderer.render()
        if renderer is None:
            active_renderer.close()
        image = np.ascontiguousarray(image)
        cv2.putText(image, label[:110], (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 60, 60), 2, cv2.LINE_AA)
        path.parent.mkdir(parents=True, exist_ok=True); iio.imwrite(path, image)
        return None
    except Exception as exc:
        return f"{type(exc).__name__}: {exc}"


def _pair_summary(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        groups[canonical_pair(row["geom_a"], row["geom_b"])].append(row)
    result = []
    for pair, values in sorted(groups.items()):
        statuses = Counter(row["status"] for row in values); categories = Counter(row["category"] for row in values)
        result.append({
            "geom_pair": list(pair), "category": categories.most_common(1)[0][0], "samples": len(values),
            "first_contact_time_s": min(float(row["simulation_time_s"]) for row in values),
            "peak_penetration_m": max(float(row["penetration_depth_m"]) for row in values),
            "peak_normal_force_n": max(abs(float(row["normal_force_n"])) for row in values),
            "max_contact_duration_s": max(float(row["contact_duration_s"]) for row in values),
            "baseline": all(bool(row["baseline"]) for row in values), "status_counts": dict(statuses),
        })
    return result


def _aggregate_events(rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Collapse per-step rows into one review event per configuration/status."""
    groups: dict[tuple[Any, ...], list[Mapping[str, Any]]] = defaultdict(list)
    for row in rows:
        context_id = row.get("sample_id") if row.get("phase") == "static" else row.get("trajectory_id")
        key = (row.get("phase"), context_id, row["status"])
        groups[key].append(row)
    events: list[dict[str, Any]] = []
    for event_id, (key, values) in enumerate(sorted(groups.items(), key=lambda item: str(item[0]))):
        first = min(values, key=lambda row: (float(row["simulation_time_s"]), int(row.get("step", 0))))
        maximum = max(values, key=lambda row: float(row["penetration_depth_m"]))
        events.append({
            "event_id": event_id, "phase": key[0], "sample_id": first.get("sample_id"),
            "trajectory_id": first.get("trajectory_id"), "trajectory_name": first.get("trajectory_name"),
            "geom_pairs": sorted({canonical_pair(row["geom_a"], row["geom_b"]) for row in values}),
            "body_pairs": sorted({canonical_pair(row["body_a"], row["body_b"]) for row in values}),
            "categories": sorted({str(row["category"]) for row in values}),
            "status": first["status"], "first_contact_time_s": float(first["simulation_time_s"]),
            "contact_sample_count": len(values), "peak_penetration_m": float(maximum["penetration_depth_m"]),
            "peak_normal_force_n": max(abs(float(row["normal_force_n"])) for row in values),
            "max_contact_duration_s": max(float(row["contact_duration_s"]) for row in values),
            "first_contact_qpos": first["qpos"], "maximum_penetration_qpos": maximum["qpos"],
            "first_contact_position_m": first["contact_position_m"], "maximum_contact_position_m": maximum["contact_position_m"],
        })
    return events


def _diagnostics(static_rows: Sequence[Mapping[str, Any]], contacts: Sequence[Mapping[str, Any]]) -> list[dict[str, str]]:
    diagnostics: list[dict[str, str]] = []
    zero = next(row for row in static_rows if row["pose_name"] == "zero")
    zero_environment = [row for row in contacts if row.get("sample_id") == zero["sample_id"] and row["category"] in {"arm_table_contact", "hand_table_contact", "arm_aluminium_contact", "hand_aluminium_contact", "robot_floor_contact"}]
    if zero_environment:
        diagnostics.append({"status": "FAIL", "rule": "zero_pose_environment_contact", "interpretation": "Scene placement regression or incorrect table/rail/root geometry."})
    table_samples = {row["sample_id"] for row in contacts if row["category"].endswith("_table_contact")}
    if len(table_samples) > max(5, int(0.35 * len(static_rows))):
        diagnostics.append({"status": "WARN", "rule": "widespread_table_contact", "interpretation": "Repeated unrelated table contacts may indicate table-height or root-placement error."})
    pos = {row["sample_id"] for row in contacts if row["geom_a"] == "workspace_rail_positive_y" or row["geom_b"] == "workspace_rail_positive_y"}
    neg = {row["sample_id"] for row in contacts if row["geom_a"] == "workspace_rail_negative_y" or row["geom_b"] == "workspace_rail_negative_y"}
    if len(pos & neg) > 3:
        diagnostics.append({"status": "WARN", "rule": "symmetric_rail_contacts", "interpretation": "Simultaneous symmetric rail collisions may indicate conservative rail primitives or rail-spacing error."})
    if not diagnostics:
        diagnostics.append({"status": "PASS", "rule": "placement_pattern_screen", "interpretation": "No widespread placement or mirror-error pattern was detected."})
    return diagnostics


def _write_summary(output: Path, summary: Mapping[str, Any]) -> None:
    counts = summary["contact_category_counts"]
    markdown = f"""# Offline MuJoCo joint-space collision characterization

This is a deterministic **simulation-only characterization**, not hardware or industrial safety certification.

- Result: **{summary['status']}**
- Digital-twin maturity: **{summary['digital_twin_maturity']}**
- Scene: `{summary['scene']}` (World=P, operational root yaw 180°)
- Static configurations: {summary['static_configuration_count']}
- Dynamic trajectories: {summary['dynamic_trajectory_count']}
- Total dynamic steps: {summary['total_dynamic_steps']}
- Aggregate throughput: {summary['simulation_throughput_steps_s']:.1f} steps/s
- Numerical stability: {summary['numerically_stable']}
- Maximum environment penetration: {summary['maximum_environment_penetration_m'] * 1000:.3f} mm
- Peak environment normal force: {summary['peak_environment_normal_force_n']:.3f} N

## Contact categories

| Category | Recorded contact samples |
|---|---:|
"""
    for category, count in sorted(counts.items()):
        markdown += f"| {category} | {count} |\n"
    markdown += "\n## Acceptance\n\n"
    for key, item in summary["acceptance"].items():
        markdown += f"- **{item['status']}** `{key}`: {item['detail']}\n"
    markdown += "\n## Diagnostics\n\n"
    for item in summary["diagnostics"]:
        markdown += f"- **{item['status']}** `{item['rule']}`: {item['interpretation']}\n"
    markdown += "\n## First issues\n\n"
    markdown += f"- First WARN configuration: `{summary['first_warn_configuration']}`\n"
    markdown += f"- First FAIL configuration: `{summary['first_fail_configuration']}`\n"
    markdown += f"- First failing trajectory: `{summary['first_failing_trajectory']}`\n"
    markdown += "\nThe sweep did not modify scene geometry, T_F_H, joint zero definitions, calibrated T_B_P, or the operational 180° yaw.\n"
    (output / "summary.md").write_text(markdown, encoding="utf-8")


def run(args: argparse.Namespace) -> dict[str, Any]:
    scene = args.scene.resolve(); classification_path = args.classification.resolve(); operational_path = args.operational_config.resolve(); output = args.output.resolve()
    for path in (scene, classification_path, operational_path, args.robot_config.resolve()):
        if not path.is_file(): raise FileNotFoundError(f"Required input does not exist: {path}")
    config = load_structured(classification_path); operational = load_structured(operational_path)
    model = mujoco.MjModel.from_xml_path(str(scene)); placement = verify_operational_scene(model, operational, config); layers = enforce_noncolliding_layers(model, config)
    if args.dry_run:
        return {"dry_run": True, "scene": str(scene), "model": {"nq": model.nq, "nu": model.nu, "ngeom": model.ngeom}, "placement": placement, "layers": layers}
    output.mkdir(parents=True, exist_ok=True); (output / "visualizations").mkdir(exist_ok=True)
    limits = _joint_limits(model, float(config["sampling"]["joint_limit_margin_deg"]))
    deterministic = deterministic_arm_samples(limits, halton_samples=args.halton_samples if args.halton_samples is not None else int(config["sampling"]["halton_samples"]), halton_skip=int(config["sampling"]["halton_skip"]))
    samples = [PoseSample(name, tuple(q), (0.0,) * 6, source) for name, q, source in deterministic]
    samples.extend(_load_repository_poses(model, args.robot_config.resolve())); samples.extend(_hand_samples()); samples = _dedupe_samples(samples)
    static_rows, static_contacts, static_states = _evaluate_static(model, config, samples)
    aliases = _select_directional_aliases(static_rows, samples)
    for alias, sample_index in aliases.items():
        original = samples[sample_index]; samples.append(PoseSample(alias, original.arm_qpos, original.hand_ctrl, f"deterministic_FK_alias_of:{original.name}", "derived_collision_free_directional"))
    static_rows, static_contacts, static_states = _evaluate_static(model, config, samples)
    sample_by_name = {sample.name: sample for sample in samples}
    alias_samples = {name: samples[index] for name, index in aliases.items()}
    zero = sample_by_name["zero"]
    trajectory_specs = [
        ("zero_to_forward_P_negative_x", zero, alias_samples["arm_extended_P_negative_x"]),
        ("zero_to_low_tabletop_approach", zero, alias_samples["low_tabletop_approach"]),
        ("zero_to_left_lateral_reach", zero, alias_samples["left_lateral_reach_P_positive_y"]),
        ("zero_to_right_lateral_reach", zero, alias_samples["right_lateral_reach_P_negative_y"]),
        ("zero_to_wrist_down", zero, alias_samples["wrist_down"]),
        ("zero_to_rh56_open", zero, sample_by_name["rh56_open"]),
        ("rh56_open_to_closed", sample_by_name["rh56_open"], sample_by_name["rh56_closed"]),
        ("rh56_closed_to_open", sample_by_name["rh56_closed"], sample_by_name["rh56_open"]),
        ("return_to_home", alias_samples["wrist_down"], zero),
    ]
    if args.quick:
        trajectory_specs = trajectory_specs[:2]
    dynamic_rows: list[dict[str, Any]] = []; dynamic_contacts: list[dict[str, Any]] = []
    sweep_wall = time.perf_counter()
    for trajectory_id, (name, start, target) in enumerate(trajectory_specs):
        row, contacts, _trajectory_snapshots = _run_dynamic(model, config, name, start, target, trajectory_id)
        dynamic_rows.append(row); dynamic_contacts.extend(contacts)
    sweep_elapsed = max(time.perf_counter() - sweep_wall, 1e-9)
    all_contacts = static_contacts + dynamic_contacts; pair_summary = _pair_summary(all_contacts)
    warnings = [row for row in all_contacts if row["status"] == "WARN"]
    failures = [row for row in all_contacts if row["status"] == "FAIL"]
    warning_events = _aggregate_events(warnings); failing_events = _aggregate_events(failures)
    baseline = [row for row in all_contacts if row["baseline"]]
    # Each compact WARN/FAIL event receives first-contact and peak-penetration
    # top/oblique views.  Joint state and contact details remain in the event JSON.
    snapshots: dict[str, dict[str, Any]] = {}
    for event in warning_events + failing_events:
        prefix = f"event_{event['status'].lower()}_{event['phase']}_{event['event_id']:03d}"
        snapshots[f"{prefix}_first_contact"] = {"qpos": event["first_contact_qpos"], "detail": event}
        snapshots[f"{prefix}_maximum_penetration"] = {"qpos": event["maximum_penetration_qpos"], "detail": event}
    # Compute all aggregate evidence before releasing the large per-contact list.
    category_counts = Counter(row["category"] for row in all_contacts)
    category_event_keys = {
        (
            row["phase"], row.get("sample_id") if row["phase"] == "static" else row.get("trajectory_id"),
            canonical_pair(row["geom_a"], row["geom_b"]), row["category"], row["status"],
        )
        for row in all_contacts if not row["baseline"]
    }
    unique_category_events = Counter(key[3] for key in category_event_keys)
    environment_categories = {"arm_table_contact", "hand_table_contact", "arm_aluminium_contact", "hand_aluminium_contact", "robot_floor_contact"}
    env_rows = [row for row in all_contacts if row["category"] in environment_categories]
    zero_row = next(row for row in static_rows if row["pose_name"] == "zero")
    zero_env = [row for row in static_contacts if row["sample_id"] == zero_row["sample_id"] and row["category"] in environment_categories]
    zero_baseline = [row for row in baseline if row.get("phase") == "static" and row.get("sample_id") == zero_row["sample_id"]]
    diagnostics = _diagnostics(static_rows, static_contacts)
    max_environment_penetration = max((float(row["penetration_depth_m"]) for row in env_rows), default=0.0)
    peak_environment_force = max((abs(float(row["normal_force_n"])) for row in env_rows), default=0.0)

    # Write compact products before the large timeline, then release the latter
    # before headless rendering.  This keeps peak memory bounded without dropping
    # any per-contact record.
    write_csv(output / "static_samples.csv", static_rows, STATIC_FIELDS)
    write_csv(output / "dynamic_trajectories.csv", dynamic_rows, DYNAMIC_FIELDS)
    pair_fields = ["geom_pair", "category", "samples", "first_contact_time_s", "peak_penetration_m", "peak_normal_force_n", "max_contact_duration_s", "baseline", "status_counts"]
    write_csv(output / "contact_pair_summary.csv", pair_summary, pair_fields)
    write_json(output / "baseline_contacts.json", {
        "zero_pose_manifold_contact_count": len(zero_baseline),
        "recorded_contact_sample_count": len(baseline),
        "unique_pair_count": len({canonical_pair(row['geom_a'], row['geom_b']) for row in baseline}),
        "pairs": [row for row in pair_summary if row["baseline"]],
        "zero_pose_contacts": zero_baseline,
        "note": "Every active baseline contact remains in contact_timeline.csv; this file is the compact baseline summary.",
    })
    write_json(output / "warning_events.json", warning_events); write_json(output / "failing_events.json", failing_events)
    write_yaml(output / "sampled_qpos.yaml", {"frame": "P", "samples": [{"name": sample.name, "arm_qpos_rad": [float(value) for value in sample.arm_qpos], "hand_ctrl_rad": [float(value) for value in sample.hand_ctrl], "source": sample.source, "role": sample.role} for sample in samples], "directional_alias_indices": {key: int(value) for key, value in aliases.items()}})
    write_yaml(output / "sweep_config_snapshot.yaml", config)
    write_csv(output / "contact_timeline.csv", all_contacts, CONTACT_FIELDS)
    nominal_dynamic_fails = [row for row in dynamic_rows if row["status"] == "FAIL"]
    acceptance = {
        "zero_pose_environment_free": {"status": "PASS" if not zero_env else "FAIL", "detail": f"{len(zero_env)} environment contacts."},
        "nominal_trajectories_no_fail": {"status": "PASS" if not nominal_dynamic_fails else "FAIL", "detail": f"{len(nominal_dynamic_fails)} failing trajectories."},
        "no_robot_floor_contact": {"status": "PASS" if category_counts["robot_floor_contact"] == 0 else "FAIL", "detail": f"{category_counts['robot_floor_contact']} contact samples."},
        "localized_environment_contacts": {"status": "PASS" if not any(item["status"] == "WARN" for item in diagnostics) else "WARN", "detail": "Pattern diagnostics reported below."},
        "numerical_stability": {"status": "PASS" if all(not row["nonfinite"] and row["solver_warning_count"] == 0 for row in dynamic_rows) else "FAIL", "detail": "No non-finite state or MuJoCo warning required."},
        "operational_yaw_preserved": {"status": "PASS", "detail": "180 degrees verified before execution."},
        "debug_geometry_noncolliding": {"status": "PASS", "detail": "Clean scene and forbidden-prefix policy verified."},
        "camera_placeholders_noncolliding": {"status": "PASS", "detail": "Required placeholders are sites, not geoms."},
        "baseline_separated": {"status": "PASS", "detail": f"{len(baseline)} samples retained as BASELINE."},
    }
    simulation_ready = all(item["status"] == "PASS" for item in acceptance.values())
    total_steps = sum(int(row["steps"]) for row in dynamic_rows)
    summary = {
        "schema_version": 1, "purpose": "offline_mujoco_collision_characterization_not_hardware_safety_certification",
        "scene": str(scene.relative_to(ROOT) if scene.is_relative_to(ROOT) else scene), "scene_sha256": _sha256(scene),
        "status": "PASS" if simulation_ready else "BLOCKED", "digital_twin_maturity": "Simulation Ready" if simulation_ready else "Integrated Workspace",
        "static_configuration_count": len(static_rows), "dynamic_trajectory_count": len(dynamic_rows), "total_dynamic_steps": total_steps,
        "simulation_throughput_steps_s": total_steps / sweep_elapsed, "known_baseline_contact_samples": len(baseline),
        "contact_category_counts": dict(category_counts), "contact_category_event_counts": dict(unique_category_events),
        "known_zero_pose_baseline_manifold_contact_count": sum(1 for row in static_contacts if row["sample_id"] == zero_row["sample_id"] and row["baseline"]),
        "maximum_environment_penetration_m": max_environment_penetration,
        "peak_environment_normal_force_n": peak_environment_force,
        "first_warn_configuration": next((row["pose_name"] for row in static_rows if row["status"] == "WARN"), None),
        "first_fail_configuration": next((row["pose_name"] for row in static_rows if row["status"] == "FAIL"), None),
        "first_failing_trajectory": next((row["name"] for row in dynamic_rows if row["status"] == "FAIL"), None),
        "first_warning_trajectory": next((row["name"] for row in dynamic_rows if row["status"] == "WARN"), None),
        "numerically_stable": acceptance["numerical_stability"]["status"] == "PASS", "acceptance": acceptance,
        "diagnostics": diagnostics, "placement_verification": placement, "layer_verification": layers,
        "render_errors": {},
        "contact_force_convention": "mujoco.mj_contactForce returns [normal, tangent1, tangent2, torque1, torque2, torque3] in the contact frame; contact.frame[:3] records the world-space normal. Forces are simulated constraint forces, not hardware measurements.",
    }
    import gc
    del all_contacts, static_contacts, dynamic_contacts, env_rows, warnings, failures, baseline
    gc.collect()
    render_errors: dict[str, str] = {}
    if not args.skip_render:
        renderer: mujoco.Renderer | None = None
        try:
            renderer = mujoco.Renderer(model, height=720, width=960)
            for event_name, snapshot in snapshots.items():
                safe_name = "".join(char if char.isalnum() or char in "_-" else "_" for char in event_name)
                detail = snapshot.get("detail", snapshot.get("row", {})); label = f"{event_name}: {detail.get('status', 'contact issue') if isinstance(detail, dict) else detail}"
                for view, camera in (("top", "workspace_top_camera"), ("oblique", "workspace_oblique_camera")):
                    error = _render_snapshot(model, snapshot, output / "visualizations" / f"{safe_name}_{view}.png", camera, label, renderer)
                    if error: render_errors[f"{event_name}:{view}"] = error
        except Exception as exc:
            render_errors["renderer_initialization"] = f"{type(exc).__name__}: {exc}"
        finally:
            if renderer is not None:
                renderer.close()
    summary["render_errors"] = render_errors
    write_json(output / "summary.json", summary); _write_summary(output, summary)
    metadata = {
        "executed_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "command": sys.argv,
        "python": sys.version, "platform": platform.platform(), "machine": platform.machine(), "mujoco_version": mujoco.__version__,
        "scene_sha256": _sha256(scene), "classification_sha256": _sha256(classification_path),
        "operational_config_sha256": _sha256(operational_path), "wall_seconds": sweep_elapsed, "render_backend": os.environ.get("MUJOCO_GL"),
        "render_errors": render_errors,
    }
    write_json(output / "execution_metadata.json", metadata)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=ROOT / "models/digital_twin/workspace_scene.xml")
    parser.add_argument("--classification", type=Path, default=ROOT / "digital_twin/configs/collision_classification.yaml")
    parser.add_argument("--operational-config", type=Path, default=ROOT / "digital_twin/configs/robot_operational_placement.yaml")
    parser.add_argument("--robot-config", type=Path, default=ROOT / "configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/digital_twin/collision_sweep")
    parser.add_argument("--halton-samples", type=int, help="Override deterministic Halton sample count.")
    parser.add_argument("--quick", action="store_true", help="Run only two dynamic trajectories; intended for integration testing.")
    parser.add_argument("--skip-render", action="store_true", help="Skip diagnostic rendering while retaining numerical outputs.")
    parser.add_argument("--dry-run", action="store_true", help="Validate inputs and operational placement without running samples.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = run(args)
    except Exception as exc:
        print(f"collision sweep failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
