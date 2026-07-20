#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json
from digital_twin.registration.transforms import quaternion_xyzw_to_matrix


def check(name: str, status: str, message: str, value: Any = None, target: Any = None) -> dict:
    return {"name": name, "status": status, "message": message, "value": value, "target": target}


def _known(values: Any) -> bool:
    return isinstance(values, list) and values and all(value is not None for value in values)


def validate(args: argparse.Namespace) -> dict:
    transforms = load_structured(args.transforms)
    workspace = load_structured(args.workspace)
    checks = []
    checks.append(check("unit_consistency", "pass" if transforms.get("units", {}).get("length") == "meter" and workspace.get("units") == "meter" else "fail", "Transform/workspace length unit declaration."))
    checks.append(check("transform_direction_consistency", "pass" if transforms.get("matrix_convention") == "column_vector" and str(transforms.get("transform_naming", "")).startswith("T_A_B maps") else "fail", "Expected column vectors and T_A_B maps B into A."))
    for name, item in transforms.get("transforms", {}).items():
        if "translation_R02_units" in item:
            quaternion = item.get("quaternion_xyzw")
            scale = item.get("scale")
            try:
                q = np.asarray(quaternion, dtype=float)
                rotation = quaternion_xyzw_to_matrix(q)
                valid = isinstance(scale, (int, float)) and scale > 0 and np.all(np.isfinite(item["translation_R02_units"]))
                checks.extend([
                    check(f"{name}_similarity_fields", "pass" if valid else "fail", "Reconstruction-to-reconstruction similarity has positive scale and finite target-frame translation."),
                    check(f"{name}_rotation_orthogonality", "pass" if np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) else "fail", "Rotation orthogonality."),
                    check(f"{name}_rotation_determinant", "pass" if np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7) else "fail", "Proper rotation determinant.", float(np.linalg.det(rotation)), 1.0),
                    check(f"{name}_quaternion_normalization", "pass" if np.isclose(np.linalg.norm(q), 1.0, atol=1e-6) else "fail", "Quaternion xyzw norm.", float(np.linalg.norm(q)), 1.0),
                ])
            except (TypeError, ValueError) as exc:
                checks.append(check(f"{name}_transform_validity", "fail", str(exc)))
            continue
        translation, quaternion = item.get("translation_m"), item.get("quaternion_xyzw")
        if not (_known(translation) and _known(quaternion)):
            checks.append(check(f"{name}_mandatory_fields", "missing", f"{name} is {item.get('status')} and remains uncalibrated/unregistered."))
            continue
        try:
            q = np.asarray(quaternion, dtype=float)
            rotation = quaternion_xyzw_to_matrix(q)
            matrix = np.eye(4); matrix[:3, :3] = rotation; matrix[:3, 3] = np.asarray(translation, dtype=float)
            checks.extend([
                check(f"{name}_matrix_dimensions", "pass" if matrix.shape == (4, 4) else "fail", "Homogeneous matrix shape.", list(matrix.shape), [4, 4]),
                check(f"{name}_homogeneous_validity", "pass" if np.allclose(matrix[3], [0, 0, 0, 1]) else "fail", "Homogeneous last row."),
                check(f"{name}_rotation_orthogonality", "pass" if np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7) else "fail", "Rotation orthogonality."),
                check(f"{name}_rotation_determinant", "pass" if np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7) else "fail", "Proper rotation determinant.", float(np.linalg.det(rotation)), 1.0),
                check(f"{name}_quaternion_normalization", "pass" if np.isclose(np.linalg.norm(q), 1.0, atol=1e-6) else "fail", "Quaternion xyzw norm.", float(np.linalg.norm(q)), 1.0),
            ])
        except (TypeError, ValueError) as exc:
            checks.append(check(f"{name}_transform_validity", "fail", str(exc)))
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model); mujoco.mj_forward(model, data)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
    checks.append(check("jaka_base_placement", "pass" if base_id >= 0 and np.allclose(data.xpos[base_id], 0, atol=1e-9) and np.allclose(np.asarray(data.xmat[base_id]).reshape(3,3), np.eye(3), atol=1e-7) else "fail", "jaka_Link_0 must coincide with W origin at zero rotation.", data.xpos[base_id].tolist() if base_id >= 0 else None, [0, 0, 0]))
    gravity = np.asarray(model.opt.gravity)
    checks.append(check("gravity_direction", "pass" if np.allclose(gravity, [0, 0, -9.81], atol=1e-6) else "fail", "Expected -B.z gravity.", gravity.tolist(), [0, 0, -9.81]))
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
    flange_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_6")
    mount_expected = transforms["transforms"]["T_F_H"]
    if hand_id >= 0 and flange_id >= 0:
        local_pos = np.asarray(model.body_pos[hand_id])
        local_wxyz = np.asarray(model.body_quat[hand_id])
        expected_xyzw = np.asarray(mount_expected["quaternion_xyzw"], float)
        expected_wxyz = expected_xyzw[[3,0,1,2]]
        mount_ok = np.allclose(local_pos, mount_expected["translation_m"], atol=1e-9) and (np.allclose(local_wxyz, expected_wxyz, atol=1e-7) or np.allclose(local_wxyz, -expected_wxyz, atol=1e-7))
        checks.append(check("flange_to_hand_transform", "pass" if mount_ok else "fail", "MJCF local hand pose matches repository-defined T_F_H; physical validation remains outstanding.", {"translation_m": local_pos.tolist(), "quaternion_wxyz": local_wxyz.tolist()}))
    else:
        checks.append(check("flange_to_hand_transform", "fail", "Required flange/hand body missing."))
    xml_root = ET.parse(args.model).getroot()
    scale_attributes = [mesh.get("scale") for mesh in xml_root.iter("mesh") if mesh.get("scale")]
    link0 = trimesh.load_mesh(REPO_ROOT / "data/sim_assets/meshes/jaka_minicobo_meshes/Link0.STL", process=False)
    mesh_ok = not scale_attributes and 0.05 < float(max(link0.extents)) < 0.5
    checks.append(check("mesh_scaling", "pass" if mesh_ok else "warn", "No explicit mesh scale in wrapper; Link0 has plausible meter-scale bounds, not metrological validation.", {"explicit_scale_attributes": scale_attributes, "link0_extents_m": np.asarray(link0.extents).tolist()}))
    tabletop = workspace.get("tabletop", {})
    table_pose_known = _known(tabletop.get("center_xyz_m")) and _known(tabletop.get("quaternion_xyzw")) and _known(tabletop.get("size_xyz_m"))
    if table_pose_known:
        table_rotation = quaternion_xyzw_to_matrix(tabletop["quaternion_xyzw"])
        normal = table_rotation[:, 2]
        angular = float(np.degrees(np.arccos(np.clip(normal @ np.asarray([0,0,1]), -1, 1))))
        checks.append(check("tabletop_normal", "pass" if angular < 2 else "warn", "Tabletop +z compared with B +z.", {"normal": normal.tolist(), "angle_deg": angular}, "<2 deg preliminary"))
        checks.append(check("robot_table_penetration", "not_run", "Table dimensions exist, but penetration needs a generated integrated scene at chosen joint configuration."))
        checks.append(check("static_scene_penetration", "not_run", "Requires generated collision scene and overlap checks."))
    else:
        checks.extend([
            check("tabletop_normal", "missing", "Tabletop pose is not measured."),
            check("robot_table_penetration", "missing", "Cannot test without measured tabletop geometry."),
            check("static_scene_penetration", "missing", "Cannot test without static collision objects."),
        ])
    for camera_name in ("T_B_C_ext", "T_F_C_wrist"):
        item = transforms["transforms"][camera_name]
        checks.append(check(f"{camera_name}_camera_orientation", "pass" if _known(item.get("quaternion_xyzw")) else "missing", "Camera optical-frame orientation requires calibration."))
    registration = load_structured(args.registration) if args.registration else None
    scale_result = load_structured(args.scale_result) if args.scale_result else None
    base_fit = load_structured(args.base_fit) if args.base_fit else None
    scale_assessment = load_structured(args.scale_assessment) if args.scale_assessment else None
    table_pose = load_structured(args.table_pose) if args.table_pose else None
    if registration:
        scale = registration.get("scale")
        checks.append(check("reconstruction_scale", "pass" if isinstance(scale, (int,float)) and scale > 0 else "fail", "Positive metric similarity scale.", scale))
        rms = registration.get("rms_error_m")
        provisional_input = "provisional" in str(registration.get("input_correspondence_status", ""))
        checks.append(check("registration_residuals", "warn" if provisional_input else "pass" if rms is not None and rms < 0.01 else "warn", "Overall table-to-base preliminary target is below 10 mm; correlated provisional inputs prevent PASS regardless of the numerical residual.", rms, "<0.01 m with independent control"))
        maximum = registration.get("max_error_m")
        checks.append(check("correspondence_errors", "pass" if maximum is not None and maximum < 0.01 else "warn", "Maximum registration correspondence error.", maximum, "<0.01 m; <0.005 m in manipulation zone where practical"))
    elif scale_result:
        scale = scale_result.get("estimated_scale_m_per_reconstruction_unit")
        acceptance = scale_result.get("metric_acceptance_status", "unspecified")
        scale_status = "warn" if isinstance(scale, (int, float)) and scale > 0 else "fail"
        checks.append(check("reconstruction_scale", scale_status, f"Scale-only result is {acceptance}; it is not a registered T_P_R/T_B_R.", scale))
        agreement = scale_result.get("A3_A4_agreement")
        checks.append(check("scale_source_agreement", "pass" if agreement and agreement.get("status") == "agree" else "warn", "Independent A3/A4 group agreement at the configured estimator threshold.", agreement))
        checks.extend([check("registration_residuals", "missing", "No T_P_R/T_B_R registration result supplied."), check("correspondence_errors", "missing", "No P/B correspondences supplied.")])
    else:
        checks.extend([check("reconstruction_scale", "missing", "No scale/registration result supplied."), check("registration_residuals", "missing", "No T_P_R/T_B_R result supplied."), check("correspondence_errors", "missing", "No correspondences supplied.")])
    if scale_assessment:
        checks.append(check(
            "independent_scale_source_agreement",
            "warn" if str(scale_assessment.get("acceptance_status", "")).startswith("PROVISIONAL") else "pass",
            "Primary source-level scale span compared with the 1-2% guidance.",
            scale_assessment.get("max_primary_source_span_fraction"), "<=0.02",
        ))
    if base_fit:
        diameter_residual = abs(float(base_fit["base_outer_circle"]["diameter_residual_m"]))
        checks.append(check("base_outer_diameter_124mm", "pass" if diameter_residual <= 0.002 else "warn", "ROI/CAD-gated fitted fixed-base diameter residual.", diameter_residual, "<=0.002 m provisional target"))
        pcd = base_fit["mounting_hole_pattern"].get("fitted_PCD_m")
        checks.append(check("mounting_PCD_110mm", "missing" if pcd is None else "pass" if abs(float(pcd) - 0.110) <= 0.002 else "warn", "Four mounting-hole centers are required for a PCD check.", pcd, 0.110))
        spacing = float(base_fit["parallel_rails"]["centerline_spacing_m"])
        candidate = 0.110 / np.sqrt(2.0)
        checks.append(check("rail_centerline_spacing_candidate", "warn", "Sparse rail fit supports the 45-degree mounting-hole hypothesis but only two bolts are visible.", {"fitted_m": spacing, "candidate_m": candidate, "residual_m": spacing - candidate}, "visual four-bolt verification"))
        checks.append(check("aluminium_profile_width_50mm", "missing", "Groove-dominated sparse points do not expose both profile faces reliably.", None, 0.050))
        checks.append(check("mounting_plane_height_50mm", "warn", "P fit uses rather than independently validates the approximate tabletop-to-mounting-plane height.", 0.050, "0.050 m approximate +/-0.002..0.005"))
    if table_pose:
        selection = table_pose.get("selection")
        checks.append(check("P_to_table_edge_constraints", "missing" if selection is None else "warn", "Table pose has four mirror candidates until physical front/right edge signs are labeled.", selection))
        checks.append(check("table_collision_enablement", "pass" if table_pose.get("tabletop", {}).get("collision") is False else "fail", "Collision must remain disabled while the table mirror candidate is unresolved."))
    t_b_p_status = transforms.get("transforms", {}).get("T_B_P", {}).get("status")
    checks.append(check("T_B_P_evidence", "missing" if t_b_p_status == "unresolved" else "warn", "Model mesh supports center/z-plane coincidence, but installed x/y datum evidence is absent.", t_b_p_status))
    checks.append(check("T_B_R_composition", "missing" if not _known(transforms.get("transforms", {}).get("T_B_R", {}).get("translation_m")) else "pass", "T_B_R may be numeric only after T_B_P is verified."))
    visual = workspace.get("visual_reconstruction", {})
    duplicate_status = "pass" if visual.get("path") is None or visual.get("scanned_robot_removed") else "fail"
    checks.append(check("duplicate_scanned_simulated_robot", duplicate_status, "A loaded visual reconstruction must have its scanned robot removed/masked.", visual))
    mandatory_missing = [item["name"] for item in checks if item["status"] == "missing"]
    if args.screenshot:
        try:
            renderer = mujoco.Renderer(model, height=720, width=1280)
            renderer.update_scene(data, camera="digital_twin_validation_camera")
            rgb = renderer.render()
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.screenshot), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            renderer.close()
            checks.append(check("validation_screenshot", "pass", f"Rendered {args.screenshot}."))
        except Exception as exc:
            checks.append(check("validation_screenshot", "warn", f"Offscreen rendering unavailable: {type(exc).__name__}: {exc}"))
    world_poses = {}
    for name in ("jaka_Link_0", "jaka_Link_6", "jaka_dummy_tcp", "rh56_R_hand_base_link"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            world_poses[name] = {"translation_m": data.xpos[body_id].tolist(), "rotation_matrix": np.asarray(data.xmat[body_id]).reshape(3,3).tolist()}
    return {
        "schema_version": 1,
        "status": "incomplete" if mandatory_missing or any(item["status"] == "fail" for item in checks) else "pass_with_warnings",
        "engineering_targets": {"table_to_base_alignment_m": 0.01, "manipulation_zone_alignment_m": 0.005, "camera_reprojection_px_preferred": 1.0, "external_transform_repeatability": "must be measured", "reconstruction_scale_error": "must be reported from multiple references"},
        "checks": [{**item, "status_label": item["status"].upper()} for item in checks],
        "missing_mandatory_calibration_fields": mandatory_missing,
        "world_poses_at_zero_configuration": world_poses,
    }


def markdown(report: dict) -> str:
    counts = {status: sum(item["status"] == status for item in report["checks"]) for status in ("pass", "warn", "fail", "missing", "not_run")}
    rows = "\n".join(f"| {item['name']} | {item['status']} | {item['message']} |" for item in report["checks"])
    return f"""# Digital-Twin Validation Report

Overall status: **{report['status']}**. Counts: {counts}.

This is an initial scaffold validation. Missing measurements/calibrations are reported rather than replaced with plausible values.

| Check | Status | Detail |
|---|---|---|
{rows}

## Preliminary engineering targets

- Overall table-to-base alignment: below 10 mm.
- Key manipulation-zone alignment: below 5 mm where practical.
- Camera reprojection: preferably below 1 px, while reporting actual values.
- External-transform repeatability and reconstruction-scale error: explicitly measured and reported.

No target above is claimed as achieved while the required inputs are missing.
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate frames, units, model scale, gravity, mount, registration and static-scene readiness.")
    parser.add_argument("--model", type=Path, default=Path("models/digital_twin/jaka_inspire_workspace.xml"))
    parser.add_argument("--transforms", type=Path, default=Path("digital_twin/configs/transforms.yaml"))
    parser.add_argument("--workspace", type=Path, default=Path("digital_twin/configs/workspace.yaml"))
    parser.add_argument("--registration", type=Path)
    parser.add_argument("--scale-result", type=Path, help="Optional scale-only result; does not substitute for T_P_R/T_B_R registration.")
    parser.add_argument("--base-fit", type=Path, help="Optional reconstruction-01 base primitive fit.")
    parser.add_argument("--scale-assessment", type=Path, help="Optional independent source-level scale assessment.")
    parser.add_argument("--table-pose", type=Path, help="Optional provisional P-frame table pose configuration.")
    parser.add_argument("--json-output", type=Path, default=Path("artifacts/digital_twin/validation_report.json"))
    parser.add_argument("--markdown-output", type=Path, default=Path("artifacts/digital_twin/validation_report.md"))
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--strict", action="store_true", help="Exit nonzero for fail or missing checks.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        for path in (args.model, args.transforms, args.workspace):
            if not path.is_file():
                raise FileNotFoundError(f"Required input does not exist: {path}")
        report = validate(args)
        if args.dry_run:
            print(json.dumps(report, indent=2)); return
        write_json(args.json_output, report)
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown(report), encoding="utf-8")
        print(f"Validation reports written to: {args.json_output} and {args.markdown_output}")
        if args.strict and report["status"] == "incomplete":
            raise SystemExit(2)
    except (FileNotFoundError, KeyError, TypeError, ValueError, mujoco.FatalError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
