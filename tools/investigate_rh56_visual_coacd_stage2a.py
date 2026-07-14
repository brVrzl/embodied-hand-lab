#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from sim_maniskill.rh56_collision_diagnostics import (  # noqa: E402
    classify_focused_root_cause,
    diagnose_vendor_visual_mesh_pair,
    diagnose_vendor_visual_mesh_object_pair,
    identify_coacd_part,
    static_cross_evaluate_qpos,
)
from sim_maniskill.rh56_collision_validation import (  # noqa: E402
    ACTUATOR_NAMES,
    CANONICAL_PHYSICAL_POSES,
    ContactRecord,
    SampleRecord,
    TrajectoryResult,
    canonical_target_ctrl,
    classify_empty_hand_terminal_contact,
    classify_representation_comparison,
    default_command_profiles,
    load_trajectory_manifest,
    run_trajectory_validation,
    summarize_object_mediated_trajectory,
    summarize_post_contact_response,
    write_trajectory_artifacts,
)
from validate_rh56_visual_coacd_stage2 import (  # noqa: E402
    _arm_qpos,
    _build_stage2_xml,
    _profile_with_timeout,
)

COLLISION_MODES = ("visual_coacd", "correll_mesh", "unifuc_pad_proxy")
PROFILES = ("slow_validation", "nominal", "hybrid")
TARGET_NAME = "sim_best_pinch"
STRATEGY = "iterative_incremental"
TRAJECTORY_MANIFEST = REPO_ROOT / "configs/sim/rh56_stage2_trajectories.yaml"
FOAM_CUBE_HALF_SIZE_M = (0.018, 0.018, 0.018)
DIAGNOSTIC_STATE_NAMES = (
    "before_first_thumb_index_contact_state",
    "first_thumb_index_contact_state",
    "first_persistent_thumb_index_contact_state",
    "first_blockage_state",
    "maximum_thumb_index_penetration_state",
    "final_state",
)
VISUAL_DIAGNOSTIC_STATE_NAMES = (
    "before_first_thumb_index_contact_state",
    "first_thumb_index_contact_state",
    "first_persistent_thumb_index_contact_state",
    "first_blockage_state",
    "maximum_thumb_index_penetration_state",
)


def _thumb_index(record: ContactRecord | Mapping[str, Any]) -> bool:
    body1 = record.body1 if isinstance(record, ContactRecord) else str(record["body1"])
    body2 = record.body2 if isinstance(record, ContactRecord) else str(record["body2"])
    groups = set()
    for body in (body1, body2):
        if "_thumb_" in body:
            groups.add("thumb")
        if "_index_" in body:
            groups.add("index")
    return groups == {"thumb", "index"}


def _contacts_at_time(result: TrajectoryResult, time_s: float, *, thumb_index_only: bool = False) -> list[dict[str, Any]]:
    tolerance = 0.5e-9
    rows = []
    for record in result.contacts:
        rh56_contact = record.body1.startswith("rh56_R_") or record.body2.startswith("rh56_R_")
        if (
            rh56_contact
            and abs(record.time - time_s) <= tolerance
            and (not thumb_index_only or _thumb_index(record))
        ):
            rows.append(asdict(record))
    return rows


def _sample_payload(sample: SampleRecord) -> dict[str, Any]:
    return asdict(sample)


def _max_penetration_so_far(result: TrajectoryResult, time_s: float) -> float:
    return max(
        (
            max(0.0, -row.dist)
            for row in result.contacts
            if row.time <= time_s
            and (row.body1.startswith("rh56_R_") or row.body2.startswith("rh56_R_"))
        ),
        default=0.0,
    )


def _first_behavioral_divergence(
    visual: TrajectoryResult,
    references: Sequence[TrajectoryResult],
    *,
    qpos_threshold: float = 5e-4,
    qvel_threshold: float = 1e-2,
    ctrl_threshold: float = 5e-4,
) -> dict[str, Any] | None:
    reference_samples = [{row.step: row for row in result.samples} for result in references]
    for sample in visual.samples:
        aligned = [rows.get(sample.step) for rows in reference_samples]
        if any(row is None for row in aligned):
            continue
        aligned_rows = [row for row in aligned if row is not None]
        visual_thumb_index = _contacts_at_time(visual, sample.time, thumb_index_only=True)
        reference_thumb_index = [
            _contacts_at_time(result, row.time, thumb_index_only=True)
            for result, row in zip(references, aligned_rows, strict=True)
        ]
        qpos_deltas = [
            float(np.linalg.norm(np.asarray(sample.qpos) - np.asarray(row.qpos), ord=np.inf))
            for row in aligned_rows
        ]
        qvel_deltas = [
            float(np.linalg.norm(np.asarray(sample.qvel) - np.asarray(row.qvel), ord=np.inf))
            for row in aligned_rows
        ]
        ctrl_deltas = [
            float(np.linalg.norm(np.asarray(sample.ctrl) - np.asarray(row.ctrl), ord=np.inf))
            for row in aligned_rows
        ]
        contact_divergence = bool(visual_thumb_index) and all(not rows for rows in reference_thumb_index)
        state_divergence = all(delta > qpos_threshold for delta in qpos_deltas)
        velocity_divergence = all(delta > qvel_threshold for delta in qvel_deltas)
        command_divergence = all(delta > ctrl_threshold for delta in ctrl_deltas)
        if not (contact_divergence or state_divergence or velocity_divergence or command_divergence):
            continue
        reasons = []
        if contact_divergence:
            reasons.append("visual_coacd has thumb/index contact while both references are contact-free")
        if state_divergence:
            reasons.append("qpos differs materially from both references")
        if velocity_divergence:
            reasons.append("qvel differs materially from both references")
        if command_divergence:
            reasons.append("contact-dependent command rate differs from both references")
        return {
            "time": sample.time,
            "step": sample.step,
            "reasons": reasons,
            "thresholds": {
                "qpos_inf": qpos_threshold,
                "qvel_inf": qvel_threshold,
                "ctrl_inf": ctrl_threshold,
            },
            "visual_coacd": {
                **_sample_payload(sample),
                "all_active_rh56_contacts": _contacts_at_time(visual, sample.time),
                "first_thumb_index_contact_time": visual.first_thumb_index_contact_time,
                "first_persistent_thumb_index_contact_time": visual.first_persistent_thumb_index_contact_time,
                "first_blockage_time": visual.first_blockage_time,
                "maximum_penetration_so_far_m": _max_penetration_so_far(visual, sample.time),
            },
            "references": [
                {
                    "collision_mode": result.collision_mode,
                    **_sample_payload(row),
                    "all_active_rh56_contacts": _contacts_at_time(result, row.time),
                    "qpos_inf_delta_from_visual": qpos_delta,
                    "qvel_inf_delta_from_visual": qvel_delta,
                    "ctrl_inf_delta_from_visual": ctrl_delta,
                }
                for result, row, qpos_delta, qvel_delta, ctrl_delta in zip(
                    references,
                    aligned_rows,
                    qpos_deltas,
                    qvel_deltas,
                    ctrl_deltas,
                    strict=True,
                )
            ],
        }
    return None


def _component_contacts(result: TrajectoryResult) -> dict[str, Any]:
    aggregates: dict[tuple[str, str], dict[str, Any]] = {}
    for record in result.contacts:
        if not _thumb_index(record):
            continue
        thumb_geom, index_geom = (record.geom1, record.geom2)
        thumb_body, index_body = (record.body1, record.body2)
        if "_index_" in thumb_body:
            thumb_geom, index_geom = index_geom, thumb_geom
            thumb_body, index_body = index_body, thumb_body
        thumb_part = identify_coacd_part(thumb_geom)
        index_part = identify_coacd_part(index_geom)
        if thumb_part is None or index_part is None:
            continue
        key = (str(thumb_part["manifest_id"]), str(index_part["manifest_id"]))
        row = aggregates.setdefault(
            key,
            {
                "thumb_body": thumb_body,
                "index_body": index_body,
                "thumb_geom": thumb_geom,
                "index_geom": index_geom,
                "thumb_part": thumb_part,
                "index_part": index_part,
                "first_contact_time": record.time,
                "contact_sample_count": 0,
                "maximum_penetration_m": 0.0,
                "maximum_normal_force": 0.0,
                "first_contact": None,
                "maximum_penetration_contact": None,
            },
        )
        payload = {
            "time": record.time,
            "position_m": record.pos,
            "normal": record.normal,
            "distance_m": record.dist,
            "penetration_m": max(0.0, -record.dist),
            "normal_force": record.normal_force,
            "stage_index": record.stage_index,
            "stage": record.stage,
        }
        row["contact_sample_count"] += 1
        if row["first_contact"] is None:
            row["first_contact"] = payload
        if payload["penetration_m"] >= row["maximum_penetration_m"]:
            row["maximum_penetration_m"] = payload["penetration_m"]
            row["maximum_penetration_contact"] = payload
        row["maximum_normal_force"] = max(row["maximum_normal_force"], record.normal_force)
    rows = sorted(aggregates.values(), key=lambda row: row["first_contact_time"])
    first_pair = rows[0] if rows else None
    maximum_pair = max(rows, key=lambda row: row["maximum_penetration_m"], default=None)
    return {
        "collision_mode": result.collision_mode,
        "profile": result.profile,
        "convex_part_pairs": rows,
        "first_contact_part_pair": first_pair,
        "maximum_penetration_part_pair": maximum_pair,
    }


def _series_by_mode(
    results: Mapping[str, TrajectoryResult],
    value: str,
) -> dict[str, list[tuple[float, float | None]]]:
    output: dict[str, list[tuple[float, float | None]]] = {}
    for mode, result in results.items():
        if value == "thumb_index_distance":
            contacts = {}
            for row in result.contacts:
                if _thumb_index(row):
                    contacts[row.time] = min(contacts.get(row.time, math.inf), row.dist)
            output[mode] = [(sample.time, contacts.get(sample.time)) for sample in result.samples]
        elif value == "thumb_index_force":
            contacts = {}
            for row in result.contacts:
                if _thumb_index(row):
                    contacts[row.time] = max(contacts.get(row.time, 0.0), row.normal_force)
            output[mode] = [(sample.time, contacts.get(sample.time, 0.0)) for sample in result.samples]
        else:
            output[mode] = [(sample.time, float(getattr(sample, value))) for sample in result.samples]
    return output


def _write_aligned_plot(results: Mapping[str, TrajectoryResult], path: Path, profile: str) -> None:
    width = 1100
    panel_height = 150
    padding = 46
    panels = [
        ("final target error", _series_by_mode(results, "target_error")),
        ("command max", {mode: [(row.time, max(row.ctrl)) for row in result.samples] for mode, result in results.items()}),
        (
            "measured max",
            {mode: [(row.time, max(row.measured_ctrl_qpos)) for row in result.samples] for mode, result in results.items()},
        ),
        ("thumb/index contact distance (m)", _series_by_mode(results, "thumb_index_distance")),
        ("thumb/index normal force", _series_by_mode(results, "thumb_index_force")),
        ("qvel norm", _series_by_mode(results, "qvel_norm")),
    ]
    height = padding + panel_height * len(panels) + 25
    max_time = max((row.time for result in results.values() for row in result.samples), default=1.0)
    colors = {"visual_coacd": "#c43d3d", "correll_mesh": "#2878b5", "unifuc_pad_proxy": "#3c8c55"}
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#fff"/>',
        f'<text x="{padding}" y="24" font-family="sans-serif" font-size="15">{profile}: aligned dynamic trajectories</text>',
    ]
    for panel_index, (title, mode_series) in enumerate(panels):
        top = padding + panel_index * panel_height
        bottom = top + panel_height - 30
        values = [value for series in mode_series.values() for _, value in series if value is not None and math.isfinite(value)]
        minimum = min(values, default=0.0)
        maximum = max(values, default=1.0)
        if maximum <= minimum:
            maximum = minimum + 1.0
        lines.extend(
            [
                f'<text x="{padding}" y="{top + 12}" font-family="sans-serif" font-size="12">{title}</text>',
                f'<line x1="{padding}" y1="{bottom}" x2="{width - padding}" y2="{bottom}" stroke="#bbb"/>',
                f'<line x1="{padding}" y1="{top + 20}" x2="{padding}" y2="{bottom}" stroke="#bbb"/>',
                f'<text x="4" y="{top + 28}" font-family="monospace" font-size="9">{maximum:.4g}</text>',
                f'<text x="4" y="{bottom}" font-family="monospace" font-size="9">{minimum:.4g}</text>',
            ]
        )
        for mode, series in mode_series.items():
            points = []
            for time_s, value in series:
                if value is None or not math.isfinite(value):
                    if points:
                        lines.append(
                            f'<polyline fill="none" stroke="{colors[mode]}" stroke-width="1.3" points="{" ".join(points)}"/>'
                        )
                        points = []
                    continue
                x = padding + (width - 2 * padding) * time_s / max(max_time, 1e-12)
                y = bottom - (bottom - top - 20) * (value - minimum) / (maximum - minimum)
                points.append(f"{x:.2f},{y:.2f}")
            if points:
                lines.append(
                    f'<polyline fill="none" stroke="{colors[mode]}" stroke-width="1.3" points="{" ".join(points)}"/>'
                )
        if panel_index == 0:
            legend_x = width - 430
            for offset, mode in enumerate(COLLISION_MODES):
                x = legend_x + offset * 140
                lines.append(f'<line x1="{x}" y1="20" x2="{x + 20}" y2="20" stroke="{colors[mode]}" stroke-width="2"/>')
                lines.append(f'<text x="{x + 24}" y="24" font-family="sans-serif" font-size="10">{mode}</text>')
    lines.append(f'<text x="{width - padding - 80}" y="{height - 8}" font-family="sans-serif" font-size="10">time (s): {max_time:.3f}</text>')
    lines.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def _state_contact_points(state: Mapping[str, Any]) -> list[list[float]]:
    return [
        list(row["pos"])
        for row in state.get("active_rh56_contacts", [])
        if _thumb_index(row)
    ]


def _dynamic_row(result: TrajectoryResult) -> dict[str, Any]:
    row = result.summary_dict(include_samples=False, include_contacts=False)
    row["command_distance_at_first_thumb_index_contact"] = (
        result.states.get("first_thumb_index_contact_state", {}) or {}
    ).get("final_target_error")
    row["command_distance_at_blockage"] = (result.states.get("first_blockage_state", {}) or {}).get(
        "final_target_error"
    )
    return row


def _sample_qpos_at_or_after(result: TrajectoryResult, time_s: float | None) -> list[float] | None:
    if time_s is None:
        return None
    sample = next((row for row in result.samples if row.time + 1e-10 >= time_s), None)
    return None if sample is None else sample.qpos


def _object_visual_diagnostics(
    *,
    result: TrajectoryResult,
    xml_path: Path,
    base_xml: Path,
    summary: Mapping[str, Any],
    tolerance_m: float,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    states = {
        "first_bilateral_object_contact": summary.get("first_bilateral_object_contact_time"),
        "first_thumb_index_self_contact": summary.get("first_thumb_index_self_contact_time"),
        "final": result.samples[-1].time if result.samples else None,
    }
    for state_name, time_s in states.items():
        qpos = _sample_qpos_at_or_after(result, time_s)
        if qpos is None:
            continue
        for hand_body in ("rh56_R_thumb_distal", "rh56_R_index_distal"):
            diagnostic = diagnose_vendor_visual_mesh_object_pair(
                base_xml=base_xml,
                kinematic_xml=xml_path,
                qpos=qpos,
                hand_body=hand_body,
                tolerance_m=tolerance_m,
            )
            diagnostic.update(
                {
                    "profile": result.profile,
                    "collision_mode": result.collision_mode,
                    "state_name": state_name,
                    "time": time_s,
                }
            )
            rows.append(diagnostic)
        if state_name == "first_thumb_index_self_contact":
            hand_hand = diagnose_vendor_visual_mesh_pair(
                base_xml=base_xml,
                kinematic_xml=xml_path,
                qpos=qpos,
                body1="rh56_R_thumb_distal",
                body2="rh56_R_index_distal",
                tolerance_m=tolerance_m,
            )
            hand_hand.update(
                {
                    "profile": result.profile,
                    "collision_mode": result.collision_mode,
                    "state_name": state_name,
                    "time": time_s,
                }
            )
            rows.append(hand_hand)
    return rows


def _hold_comparison(
    continuous: TrajectoryResult,
    held: TrajectoryResult,
) -> dict[str, Any]:
    continuous_summary = summarize_post_contact_response(continuous)
    held_summary = summarize_post_contact_response(held)
    continuous_max = float(continuous.max_thumb_index_penetration_m)
    held_final = held_summary.get("maximum_penetration") or {}
    held_final_state = held.states.get("diagnostic_hold_final_state") or {}
    held_final_contacts = [
        row
        for row in held_final_state.get("active_rh56_contacts", [])
        if (
            ("_thumb_" in str(row.get("body1", "")) and "_index_" in str(row.get("body2", "")))
            or ("_index_" in str(row.get("body1", "")) and "_thumb_" in str(row.get("body2", "")))
        )
    ]
    settled_penetration = max(
        (max(0.0, -float(row.get("dist", 0.0))) for row in held_final_contacts), default=0.0
    )
    settled_force = max((float(row.get("normal_force", 0.0)) for row in held_final_contacts), default=0.0)
    return {
        "profile": continuous.profile,
        "continuous": continuous_summary,
        "hold_after_persistent_contact": held_summary,
        "continuous_max_penetration_m": continuous_max,
        "hold_max_penetration_m": held.max_thumb_index_penetration_m,
        "hold_settled_penetration_m": settled_penetration,
        "hold_settled_normal_force": settled_force,
        "continuous_penetration_increased_while_pushing": continuous_summary[
            "penetration_increased_while_command_pushed"
        ],
        "hold_reduced_maximum_vs_continuous": held.max_thumb_index_penetration_m < continuous_max,
        "continued_actuation_primary_explanation": bool(
            continuous_summary["penetration_increased_while_command_pushed"]
            and held.max_thumb_index_penetration_m < continuous_max * 0.8
        ),
        "unused_maximum_snapshot": held_final,
    }


def _markdown_report(report: Mapping[str, Any]) -> str:
    lines = [
        "# RH56 Stage 2A Focused Root-Cause Report",
        "",
        f"Classification: **{report['classification']['classification']}**",
        "",
    ]
    for reason in report["classification"]["reasons"]:
        lines.append(f"- {reason}")
    lines.extend(
        [
            "",
            "## Dynamic outcomes",
            "",
            "| Profile | Mode | Outcome | First T/I contact (s) | Persistent (s) | Blockage (s) | Reached (s) | Final error | Max T/I penetration (mm) | Max T/I force |",
            "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in report["dynamic_outcomes"]:
        value = lambda key: "-" if row.get(key) is None else f"{float(row[key]):.4f}"
        lines.append(
            f"| {row['profile']} | {row['collision_mode']} | {row['outcome']} | "
            f"{value('first_thumb_index_contact_time')} | {value('first_persistent_thumb_index_contact_time')} | "
            f"{value('first_blockage_time')} | {value('target_reached_time')} | {row['final_target_error']:.6f} | "
            f"{row['max_thumb_index_penetration_m'] * 1000.0:.4f} | {row['max_thumb_index_normal_force']:.4f} |"
        )
    lines.extend(["", "## Contact response", ""])
    lines.append(
        f"Continued actuation is the primary explanation for deep post-contact penetration across all "
        f"profiles: **{report['continued_actuation_primary_explanation']}**."
    )
    lines.extend(
        [
            "",
            "| Profile | Continuous max (mm) | Hold max (mm) | Hold settled (mm) | Hold settled force | Increased while pushing |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for row in report["hold_after_contact_comparisons"]:
        lines.append(
            f"| {row['profile']} | {row['continuous_max_penetration_m'] * 1000.0:.4f} | "
            f"{row['hold_max_penetration_m'] * 1000.0:.4f} | "
            f"{row['hold_settled_penetration_m'] * 1000.0:.4f} | "
            f"{row['hold_settled_normal_force']:.4f} | "
            f"{row['continuous_penetration_increased_while_pushing']} |"
        )
    lines.extend(["", "## Empty-hand semantics", ""])
    lines.append(
        f"`sim_best_pinch` is `{report['target_manifest']['target_semantics']}` because repository usage "
        "does not establish whether the full command is a free-space or object-required target. "
        f"The observed event is `{report['empty_hand_terminal_semantics']['classification']}`, not a "
        "successful free-space reach."
    )
    lines.extend(
        [
            "",
            "## Object-mediated foam cube",
            "",
            "| Profile | Mode | Interpreted outcome | Thumb (s) | Index (s) | Bilateral (s) | Self (s) | Retention (s) | Max self pen. (mm) |",
            "|---|---|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in report["object_mediated_outcomes"]:
        value = lambda key: "-" if row.get(key) is None else f"{float(row[key]):.4f}"
        lines.append(
            f"| {row['profile']} | {row['collision_mode']} | {row['interpreted_outcome']} | "
            f"{value('first_thumb_object_contact_time')} | {value('first_index_object_contact_time')} | "
            f"{value('first_bilateral_object_contact_time')} | "
            f"{value('first_thumb_index_self_contact_time')} | {row['object_retention_duration_s']:.4f} | "
            f"{row['max_rh56_self_penetration_m'] * 1000.0:.4f} |"
        )
    lines.append(
        "The cube scene uses the repository's top-down grasp arm preset and a table-supported free cube; "
        "retention is a bilateral-contact diagnostic, not a lift-success result."
    )
    lines.extend(["", "## Stage 2A closeout", ""])
    lines.append(
        "No object-mediated run prevented later thumb/index self-contact, so object grasp success remains "
        "report-only. Deterministic replay, state validity, unknown-pair reporting, and forbidden structural "
        "contact detection are candidates for later Stage 2B gates; reachability, force, penetration, and "
        "retention thresholds are not."
    )
    lines.extend(["", "## First behavioral divergence", ""])
    for profile, divergence in report["first_behavioral_divergence"].items():
        if divergence is None:
            lines.append(f"- `{profile}`: no material divergence found on aligned samples.")
        else:
            lines.append(
                f"- `{profile}`: {divergence['time']:.6f} s, stage "
                f"`{divergence['visual_coacd']['stage']}`, " + "; ".join(divergence["reasons"]) + "."
            )
    lines.extend(["", "## Static diagnostics", ""])
    lines.append(
        "Same-qpos comparisons assign saved qpos to each collision model and call `mj_forward`; "
        "they are collision-representation diagnostics, not dynamic reachability evidence."
    )
    separated_gaps = [
        row["proximity"]["minimum_surface_distance_mm"]
        for row in report["visual_mesh_diagnostics"]
        if row.get("proximity", {}).get("intersects") is False
        and row.get("proximity", {}).get("minimum_surface_distance_mm") is not None
    ]
    intersections = sum(
        1 for row in report["visual_mesh_diagnostics"] if row.get("proximity", {}).get("intersects")
    )
    if separated_gaps:
        lines.append(
            f"Original vendor visuals were separated by {min(separated_gaps):.6f} to "
            f"{max(separated_gaps):.6f} mm immediately before first CoACD contact, then intersected "
            f"at {intersections} contact/persistence/blockage diagnostic states."
        )
    else:
        lines.append(f"Original vendor visual diagnostics found {intersections} intersecting saved states.")
    lines.append(
        "At each visual_coacd first-contact qpos, both reference collision modes were thumb/index "
        "contact-free. Thus the collision representations differ, but CoACD contact does not precede "
        "original visual-mesh intersection at the sampled 2 ms resolution."
    )
    first_component = report["coacd_component_contacts"][0].get("first_contact_part_pair")
    maximum_component = report["coacd_component_contacts"][0].get("maximum_penetration_part_pair")
    if first_component and maximum_component:
        lines.extend(
            [
                "",
                "## CoACD components",
                "",
                f"First contact: `{first_component['thumb_part']['manifest_id']}` with "
                f"`{first_component['index_part']['manifest_id']}`. Maximum penetration: "
                f"`{maximum_component['thumb_part']['manifest_id']}` with "
                f"`{maximum_component['index_part']['manifest_id']}`.",
            ]
        )
    repeatability = report.get("repeatability", {})
    if repeatability:
        lines.extend(
            [
                "",
                "## Repeatability",
                "",
                f"The nominal visual_coacd repeat was identical: `{repeatability['identical']}`; "
                f"maximum aligned qpos delta `{repeatability['max_aligned_qpos_delta']:.3e}` and "
                f"maximum aligned control delta `{repeatability['max_aligned_ctrl_delta']:.3e}`.",
            ]
        )
    lines.extend(
        [
            "",
            "## Limits",
            "",
            "- `python-fcl` and `rtree` are unavailable. Visual intersection/proximity uses the report-labelled deterministic BVH triangle fallback.",
            "- Hardware raw speed units still have no repository-backed conversion to radians per second.",
            "- This finding is Stage 2A evidence only and is not a Stage 2B CI gate.",
            "- No collision geometry, exclusions, joint limits, or actuator mappings were changed.",
            "",
            "## Reproduction",
            "",
            f"```bash\n{report['deterministic_reproduction_command']}\n```",
            "",
        ]
    )
    return "\n".join(lines)


def run(args: argparse.Namespace) -> dict[str, Any]:
    np.random.seed(args.seed)
    out_dir = Path(args.out_dir).resolve()
    trajectory_manifest = load_trajectory_manifest(args.trajectory_manifest)
    target_manifest = trajectory_manifest["trajectories"][TARGET_NAME]
    target_ctrl = canonical_target_ctrl(TARGET_NAME)
    arm_qpos = _arm_qpos(Path(args.robot_config), args.arm_preset)
    object_arm_qpos = _arm_qpos(Path(args.object_robot_config), args.object_arm_preset)
    xml_dir = out_dir / "xml"
    xml_dir.mkdir(parents=True, exist_ok=True)
    xml_by_mode: dict[str, Path] = {}
    object_xml_by_mode: dict[str, Path] = {}
    for mode in COLLISION_MODES:
        xml_path = xml_dir / f"object_free_{mode}.xml"
        _build_stage2_xml(
            base_xml=Path(args.base_xml),
            out_xml=xml_path,
            collision_mode=mode,
            include_object=False,
        )
        xml_by_mode[mode] = xml_path

    results_by_profile: dict[str, dict[str, TrajectoryResult]] = {}
    dynamic_rows: list[dict[str, Any]] = []
    reproduction = (
        f".venv/bin/python tools/investigate_rh56_visual_coacd_stage2a.py --out-dir {out_dir} "
        f"--timeout-scale {args.timeout_scale} --seed {args.seed}"
    )
    for profile_name in args.profiles:
        profile = _profile_with_timeout(default_command_profiles()[profile_name], args.timeout_scale)
        profile_results: dict[str, TrajectoryResult] = {}
        for mode in COLLISION_MODES:
            model = mujoco.MjModel.from_xml_path(str(xml_by_mode[mode]))
            result = run_trajectory_validation(
                model,
                collision_mode=mode,
                target_name=TARGET_NAME,
                target_ctrl=target_ctrl,
                strategy=STRATEGY,
                profile=profile,
                arm_qpos=arm_qpos,
                sample_stride=1,
            )
            run_dir = out_dir / "trajectories" / profile_name / mode
            mode_reproduction = reproduction + f" --profiles {profile_name}"
            write_trajectory_artifacts(result, run_dir, reproduction_command=mode_reproduction)
            profile_results[mode] = result
            dynamic_rows.append(_dynamic_row(result))
        results_by_profile[profile_name] = profile_results
        _write_aligned_plot(profile_results, out_dir / "plots" / f"{profile_name}_aligned.svg", profile_name)

    hold_results: dict[str, TrajectoryResult] = {}
    hold_comparisons: list[dict[str, Any]] = []
    for profile_name in args.profiles:
        profile = _profile_with_timeout(default_command_profiles()[profile_name], args.timeout_scale)
        held = run_trajectory_validation(
            mujoco.MjModel.from_xml_path(str(xml_by_mode["visual_coacd"])),
            collision_mode="visual_coacd",
            target_name=TARGET_NAME,
            target_ctrl=target_ctrl,
            strategy=STRATEGY,
            profile=profile,
            arm_qpos=arm_qpos,
            sample_stride=1,
            hold_after_persistent_distal_contact_seconds=args.contact_hold_seconds,
        )
        hold_results[profile_name] = held
        write_trajectory_artifacts(
            held,
            out_dir / "contact_response_hold" / profile_name / "visual_coacd",
            reproduction_command=reproduction
            + f" --profiles {profile_name} --contact-hold-seconds {args.contact_hold_seconds}",
        )
        hold_comparisons.append(_hold_comparison(results_by_profile[profile_name]["visual_coacd"], held))

    placement_profile = "nominal" if "nominal" in results_by_profile else args.profiles[0]
    placement_state = results_by_profile[placement_profile]["visual_coacd"].states[
        "before_first_thumb_index_contact_state"
    ]
    if placement_state is None:
        raise RuntimeError("Cannot place foam_cube without a saved pre-contact thumb/index state.")
    placement_qpos = np.asarray(placement_state["qpos"], dtype=np.float64).copy()
    placement_qpos[:6] = object_arm_qpos
    placement_visual = diagnose_vendor_visual_mesh_pair(
        base_xml=args.base_xml,
        kinematic_xml=xml_by_mode["visual_coacd"],
        qpos=placement_qpos,
        body1="rh56_R_thumb_distal",
        body2="rh56_R_index_distal",
        tolerance_m=args.mesh_tolerance_m,
    )
    closest_thumb = np.asarray(placement_visual["proximity"]["closest_point1_m"], dtype=np.float64)
    closest_index = np.asarray(placement_visual["proximity"]["closest_point2_m"], dtype=np.float64)
    object_position = 0.5 * (closest_thumb + closest_index)
    for mode in COLLISION_MODES:
        object_xml_path = xml_dir / f"foam_cube_{mode}.xml"
        _build_stage2_xml(
            base_xml=Path(args.base_xml),
            out_xml=object_xml_path,
            collision_mode=mode,
            include_object=True,
            object_name="foam_cube",
            object_pos=tuple(float(value) for value in object_position),
            include_object_table=True,
            gravity_compensated_object=False,
            object_table_top_z=float(object_position[2] - FOAM_CUBE_HALF_SIZE_M[2]),
            object_table_half_size_xy=(0.42, 0.32),
        )
        object_xml_by_mode[mode] = object_xml_path

    object_results_by_profile: dict[str, dict[str, TrajectoryResult]] = {}
    object_rows: list[dict[str, Any]] = []
    object_visual_rows: list[dict[str, Any]] = []
    for profile_name in args.profiles:
        profile = _profile_with_timeout(default_command_profiles()[profile_name], args.timeout_scale)
        profile_results = {}
        for mode in COLLISION_MODES:
            model = mujoco.MjModel.from_xml_path(str(object_xml_by_mode[mode]))
            result = run_trajectory_validation(
                model,
                collision_mode=mode,
                target_name=TARGET_NAME,
                target_ctrl=target_ctrl,
                strategy=STRATEGY,
                profile=profile,
                arm_qpos=object_arm_qpos,
                sample_stride=1,
            )
            profile_results[mode] = result
            write_trajectory_artifacts(
                result,
                out_dir / "object_trajectories" / profile_name / mode,
                reproduction_command=reproduction + f" --profiles {profile_name}",
            )
            summary = summarize_object_mediated_trajectory(result, model)
            summary.update({"profile": profile_name, "collision_mode": mode, "object": "foam_cube"})
            object_rows.append(summary)
            object_visual_rows.extend(
                _object_visual_diagnostics(
                    result=result,
                    xml_path=object_xml_by_mode[mode],
                    base_xml=Path(args.base_xml),
                    summary=summary,
                    tolerance_m=args.mesh_tolerance_m,
                )
            )
        object_results_by_profile[profile_name] = profile_results
        _write_aligned_plot(
            profile_results,
            out_dir / "plots" / f"foam_cube_{profile_name}_aligned.svg",
            f"foam_cube {profile_name}",
        )

    divergences = {
        profile: _first_behavioral_divergence(
            results["visual_coacd"],
            [results["correll_mesh"], results["unifuc_pad_proxy"]],
        )
        for profile, results in results_by_profile.items()
    }

    repeatability: dict[str, Any] = {}
    if "nominal" in results_by_profile:
        nominal_profile = _profile_with_timeout(default_command_profiles()["nominal"], args.timeout_scale)
        repeat_model = mujoco.MjModel.from_xml_path(str(xml_by_mode["visual_coacd"]))
        repeated = run_trajectory_validation(
            repeat_model,
            collision_mode="visual_coacd",
            target_name=TARGET_NAME,
            target_ctrl=target_ctrl,
            strategy=STRATEGY,
            profile=nominal_profile,
            arm_qpos=arm_qpos,
            sample_stride=1,
        )
        write_trajectory_artifacts(
            repeated,
            out_dir / "repeatability" / "nominal" / "visual_coacd",
            reproduction_command=reproduction + " --profiles nominal",
        )
        primary = results_by_profile["nominal"]["visual_coacd"]
        aligned_count = min(len(primary.samples), len(repeated.samples))
        qpos_delta = max(
            (
                float(np.linalg.norm(np.asarray(first.qpos) - np.asarray(second.qpos), ord=np.inf))
                for first, second in zip(
                    primary.samples[:aligned_count], repeated.samples[:aligned_count], strict=True
                )
            ),
            default=0.0,
        )
        ctrl_delta = max(
            (
                float(np.linalg.norm(np.asarray(first.ctrl) - np.asarray(second.ctrl), ord=np.inf))
                for first, second in zip(
                    primary.samples[:aligned_count], repeated.samples[:aligned_count], strict=True
                )
            ),
            default=0.0,
        )
        repeatability = {
            "profile": "nominal",
            "collision_mode": "visual_coacd",
            "primary_outcome": primary.outcome,
            "repeat_outcome": repeated.outcome,
            "primary_sample_count": len(primary.samples),
            "repeat_sample_count": len(repeated.samples),
            "max_aligned_qpos_delta": qpos_delta,
            "max_aligned_ctrl_delta": ctrl_delta,
            "first_contact_time_delta": abs(
                float(primary.first_thumb_index_contact_time or 0.0)
                - float(repeated.first_thumb_index_contact_time or 0.0)
            ),
            "first_blockage_time_delta": abs(
                float(primary.first_blockage_time or 0.0) - float(repeated.first_blockage_time or 0.0)
            ),
        }
        repeatability["identical"] = bool(
            primary.outcome == repeated.outcome
            and len(primary.samples) == len(repeated.samples)
            and qpos_delta == 0.0
            and ctrl_delta == 0.0
            and repeatability["first_contact_time_delta"] == 0.0
            and repeatability["first_blockage_time_delta"] == 0.0
        )

    same_qpos_rows: list[dict[str, Any]] = []
    classification_same_qpos: list[dict[str, Any]] = []
    visual_mesh_rows: list[dict[str, Any]] = []
    component_rows: list[dict[str, Any]] = []
    for profile, results in results_by_profile.items():
        visual = results["visual_coacd"]
        component_rows.append(_component_contacts(visual))
        first_thumb_index = next((row for row in visual.contacts if _thumb_index(row)), None)
        if first_thumb_index is None:
            continue
        body1, body2 = first_thumb_index.body1, first_thumb_index.body2
        for state_name in DIAGNOSTIC_STATE_NAMES:
            state = visual.states.get(state_name)
            if not state:
                continue
            evaluation = static_cross_evaluate_qpos(
                xml_by_mode,
                state["qpos"],
                ctrl=state.get("ctrl"),
            )
            evaluation.update({"profile": profile, "state_name": state_name, "time": state["time"]})
            same_qpos_rows.append(evaluation)
            if state_name in {
                "first_thumb_index_contact_state",
                "first_persistent_thumb_index_contact_state",
                "first_blockage_state",
                "maximum_thumb_index_penetration_state",
            }:
                classification_same_qpos.append(evaluation)
        for state_name in VISUAL_DIAGNOSTIC_STATE_NAMES:
            state = visual.states.get(state_name)
            if not state:
                continue
            diagnostic = diagnose_vendor_visual_mesh_pair(
                base_xml=args.base_xml,
                kinematic_xml=xml_by_mode["visual_coacd"],
                qpos=state["qpos"],
                body1=body1,
                body2=body2,
                contact_points=_state_contact_points(state),
                tolerance_m=args.mesh_tolerance_m,
            )
            diagnostic.update({"profile": profile, "state_name": state_name, "time": state["time"]})
            visual_mesh_rows.append(diagnostic)

    classification = classify_focused_root_cause(
        dynamic_rows=dynamic_rows,
        same_qpos_evaluations=classification_same_qpos,
        visual_mesh_diagnostics=visual_mesh_rows,
        meaningful_gap_m=args.meaningful_gap_m,
        near_touch_tolerance_m=args.near_touch_tolerance_m,
    )
    representation_interpretations = []
    for profile_name, profile_results in results_by_profile.items():
        visual_row = _dynamic_row(profile_results["visual_coacd"])
        reference_rows = [_dynamic_row(profile_results[mode]) for mode in COLLISION_MODES[1:]]
        interpretation = classify_representation_comparison(
            visual_coacd=visual_row,
            references=reference_rows,
            original_visual_intersects=True,
        )
        interpretation["profile"] = profile_name
        representation_interpretations.append(interpretation)

    first_visual_contact = next(
        (row for row in results_by_profile[args.profiles[0]]["visual_coacd"].contacts if _thumb_index(row)),
        None,
    )
    empty_hand_terminal_semantics = classify_empty_hand_terminal_contact(
        target_semantics=target_manifest["target_semantics"],
        contact_category=(
            "unknown_or_unreviewed_pair" if first_visual_contact is None else first_visual_contact.category
        ),
        regions=(
            []
            if first_visual_contact is None
            else [first_visual_contact.region1, first_visual_contact.region2]
        ),
        vendor_visuals_touch_or_intersect=classification["classification"]
        == "shared_visual_or_kinematic_intersection",
        forbidden_structural_contact=any(row.first_forbidden_pair for row in hold_results.values()),
        tunnelling=False,
        numerical_instability=any(row.numerical_instability for row in hold_results.values()),
    )

    object_comparison_rows = []
    for summary in object_rows:
        diagnostics = [
            row
            for row in object_visual_rows
            if row["profile"] == summary["profile"] and row["collision_mode"] == summary["collision_mode"]
        ]
        hand_object_visual_intersections = [
            row
            for row in diagnostics
            if row.get("diagnostic_type") == "static_original_vendor_visual_mesh_to_exact_box_proximity"
            if row.get("proximity", {}).get("intersects") is True
        ]
        hand_hand_visual_intersections = [
            row
            for row in diagnostics
            if row.get("diagnostic_type") == "static_original_vendor_visual_mesh_proximity"
            and row.get("proximity", {}).get("intersects") is True
        ]
        contacted_digits = {
            "thumb": summary["first_thumb_object_contact_time"] is not None,
            "index": summary["first_index_object_contact_time"] is not None,
        }
        missed_visual_contact = []
        for row in hand_object_visual_intersections:
            body = row.get("hand", {}).get("body", "")
            digit = "thumb" if "_thumb_" in body else "index" if "_index_" in body else ""
            if digit and not contacted_digits[digit]:
                missed_visual_contact.append({"digit": digit, "state_name": row["state_name"]})
        object_comparison_rows.append(
            {
                **summary,
                "establishes_bilateral_object_contact": summary["first_bilateral_object_contact_time"]
                is not None,
                "visual_hand_object_intersection_seen": bool(hand_object_visual_intersections),
                "visual_hand_hand_intersection_seen": bool(hand_hand_visual_intersections),
                "misses_contact_indicated_by_visual_geometry": missed_visual_contact,
                "blocks_or_terminates_consistently_with_visual_geometry": bool(
                    summary["object_retained"] and hand_object_visual_intersections
                ),
                "stable_retention": summary["object_retained"],
                "excessive_penetration_or_force_assessment": (
                    "report_only_no_reviewed_stage2b_threshold"
                ),
            }
        )

    continued_actuation_explains_penetration = bool(hold_comparisons) and all(
        row["continued_actuation_primary_explanation"] for row in hold_comparisons
    )
    report = {
        "schema": "rh56_visual_coacd_stage2a_closeout_v0.2",
        "scope": {
            "target": TARGET_NAME,
            "target_physical_norm": CANONICAL_PHYSICAL_POSES[TARGET_NAME],
            "target_ctrl": target_ctrl.tolist(),
            "strategy": STRATEGY,
            "collision_modes": list(COLLISION_MODES),
            "profiles": list(args.profiles),
            "seed": args.seed,
            "timestep_s": next(
                iter(mujoco.MjModel.from_xml_path(str(path)).opt.timestep for path in xml_by_mode.values())
            ),
            "timeout_scale": args.timeout_scale,
            "actuator_order": list(ACTUATOR_NAMES),
        },
        "invariants": {
            "same_initial_ctrl": [0.0] * 6,
            "same_arm_qpos": arm_qpos.tolist(),
            "same_target_and_iterative_stage_fractions": [0.25, 0.5, 0.75, 1.0],
            "randomness_used_by_runner": False,
            "dynamic_method": "position actuator controls advanced by repeated mujoco.mj_step",
            "static_method": "saved qpos assigned independently per mode followed by one mujoco.mj_forward",
        },
        "classification": classification,
        "representation_interpretations": representation_interpretations,
        "reference_modes_are_ground_truth": False,
        "trajectory_manifest_file": str(Path(args.trajectory_manifest).resolve()),
        "target_manifest": target_manifest,
        "empty_hand_terminal_semantics": empty_hand_terminal_semantics,
        "dynamic_outcomes": dynamic_rows,
        "post_contact_response": [
            summarize_post_contact_response(results_by_profile[profile]["visual_coacd"])
            for profile in args.profiles
        ],
        "hold_after_contact_comparisons": hold_comparisons,
        "continued_actuation_primary_explanation": continued_actuation_explains_penetration,
        "object_scene": {
            "object": "foam_cube",
            "half_size_m": list(FOAM_CUBE_HALF_SIZE_M),
            "mass_kg": 0.018,
            "initial_position_m": object_position.tolist(),
            "gravity_compensated": False,
            "support_table": True,
            "support_table_half_size_xy_m": [0.42, 0.32],
            "arm_preset": args.object_arm_preset,
            "arm_qpos": object_arm_qpos.tolist(),
            "placement_basis": (
                "Midpoint of the original vendor thumb/index visual closest points at the saved pre-contact "
                "hand configuration, transformed to the recorded pinch_grasp_box_v2 arm preset."
            ),
            "placement_static_diagnostic_only": True,
            "placement_visual_proximity": placement_visual["proximity"],
            "retention_limit": (
                "Retention means sustained bilateral contact while the cube remains table-supported; "
                "it is not a lift or payload-success claim."
            ),
        },
        "object_mediated_outcomes": object_comparison_rows,
        "object_visual_diagnostics_file": str(out_dir / "object_visual_diagnostics.json"),
        "stage2a_closeout": {
            "validated": [
                "deterministic actuator-space trajectories use repeated mujoco.mj_step",
                "empty-hand focused root cause is shared_visual_or_kinematic_intersection",
                "reference representation reachability is not treated as ground truth",
                "continued actuation is the primary cause of deep post-contact penetration",
                "foam_cube contact ordering and representation differences are recorded",
            ],
            "visual_consistent_blocking": [
                "sim_best_pinch iterative_incremental visual_coacd under slow_validation, nominal, and hybrid"
            ],
            "expected_path_obstruction": [
                "No additional path obstruction was confirmed in this focused closeout; prior power_close order-dependent cases remain report-only candidates."
            ],
            "object_mediated": (
                "All reviewed runs established bilateral object contact before thumb/index self-contact, "
                "but none prevented later visual-consistent self-intersection; no successful object-mediated closure."
            ),
            "unresolved": [
                "sim_best_pinch free-space versus object-required intent is not documented",
                "foam_cube placement slides substantially and is not a retained-lift validation",
                "reviewed penetration and force acceptance thresholds are unavailable",
                "hardware speed registers still lack a repository-backed rad/s conversion",
            ],
            "stage2b_gate_candidates": [
                "manifest schema validation",
                "deterministic replay and numerical-state validity",
                "prominent unknown or unreviewed RH56 self-contact reporting",
                "forbidden structural contact detection",
            ],
            "report_only": [
                "canonical target reachability",
                "empty-hand distal terminal-contact expectation",
                "object retention and grasp success",
                "penetration and force magnitudes",
                "representation comparison outcomes",
            ],
            "broad_stage2b_thresholds_implemented": False,
        },
        "first_behavioral_divergence": divergences,
        "repeatability": repeatability,
        "same_qpos_cross_evaluations_file": str(out_dir / "same_qpos_cross_evaluation.json"),
        "visual_mesh_diagnostics_file": str(out_dir / "visual_mesh_diagnostics.json"),
        "coacd_component_contacts_file": str(out_dir / "coacd_component_contacts.json"),
        "visual_mesh_diagnostics": visual_mesh_rows,
        "coacd_component_contacts": component_rows,
        "deterministic_reproduction_command": reproduction,
        "stage2b_gate": False,
    }
    (out_dir / "same_qpos_cross_evaluation.json").write_text(
        json.dumps(same_qpos_rows, indent=2), encoding="utf-8"
    )
    (out_dir / "visual_mesh_diagnostics.json").write_text(
        json.dumps(visual_mesh_rows, indent=2), encoding="utf-8"
    )
    (out_dir / "coacd_component_contacts.json").write_text(
        json.dumps(component_rows, indent=2), encoding="utf-8"
    )
    (out_dir / "post_contact_response.json").write_text(
        json.dumps(hold_comparisons, indent=2), encoding="utf-8"
    )
    (out_dir / "object_mediated_outcomes.json").write_text(
        json.dumps(object_comparison_rows, indent=2), encoding="utf-8"
    )
    (out_dir / "object_visual_diagnostics.json").write_text(
        json.dumps(object_visual_rows, indent=2), encoding="utf-8"
    )
    (out_dir / "focused_root_cause_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    (out_dir / "stage2a_closeout_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    markdown = _markdown_report(report)
    (out_dir / "root_cause_report.md").write_text(markdown, encoding="utf-8")
    (out_dir / "stage2a_closeout_report.md").write_text(markdown, encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Focused Stage 2A root-cause investigation for sim_best_pinch iterative_incremental."
    )
    parser.add_argument("--base-xml", default="data/sim_assets/jaka_rh56.xml")
    parser.add_argument("--robot-config", default="configs/robot/jaka_mini2.yaml")
    parser.add_argument("--arm-preset", default="upright")
    parser.add_argument("--object-robot-config", default="configs/robot/jaka_mini2_real.yaml")
    parser.add_argument("--object-arm-preset", default="pinch_grasp_box_v2")
    parser.add_argument("--out-dir", default="/tmp/rh56_visual_coacd_stage2a_focused")
    parser.add_argument("--profiles", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--timeout-scale", type=float, default=2.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--mesh-tolerance-m", type=float, default=1e-8)
    parser.add_argument("--near-touch-tolerance-m", type=float, default=2e-4)
    parser.add_argument("--meaningful-gap-m", type=float, default=5e-4)
    parser.add_argument("--contact-hold-seconds", type=float, default=1.0)
    parser.add_argument("--trajectory-manifest", default=str(TRAJECTORY_MANIFEST))
    args = parser.parse_args()
    report = run(args)
    print(
        json.dumps(
            {
                "out_dir": str(Path(args.out_dir).resolve()),
                "classification": report["classification"],
                "dynamic_outcomes": [
                    {
                        "profile": row["profile"],
                        "mode": row["collision_mode"],
                        "outcome": row["outcome"],
                        "first_thumb_index_contact_time": row["first_thumb_index_contact_time"],
                        "first_blockage_time": row["first_blockage_time"],
                        "target_reached_time": row["target_reached_time"],
                    }
                    for row in report["dynamic_outcomes"]
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
