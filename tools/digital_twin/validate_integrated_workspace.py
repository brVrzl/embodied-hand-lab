#!/usr/bin/env python3
"""Validate the clean, operationally oriented P-world MuJoCo workspace."""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
import trimesh

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_json


def item(name: str, status: str, message: str, value=None, target=None) -> dict:
    return {"name": name, "status": status, "message": message, "value": value, "target": target}


def object_id(model: mujoco.MjModel, kind: mujoco.mjtObj, name: str) -> int:
    return mujoco.mj_name2id(model, kind, name)


def angular_error_deg(vector: np.ndarray, target: np.ndarray) -> float:
    vector = vector / np.linalg.norm(vector); target = target / np.linalg.norm(target)
    return math.degrees(math.acos(float(np.clip(np.dot(vector, target), -1.0, 1.0))))


def orientation_status(error_deg: float) -> str:
    if error_deg < 3.0:
        return "PASS"
    if error_deg <= 8.0:
        return "PROVISIONAL"
    return "FAIL"


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate P-world root orientation, clean visual layers, zero reference state, collisions and package readiness.")
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--sparse-debug-scene", type=Path, help="Optional debug XML used only to verify non-colliding sparse markers.")
    parser.add_argument("--static-config", type=Path, required=True)
    parser.add_argument("--operational-config", type=Path, required=True)
    parser.add_argument("--transforms", type=Path, required=True)
    parser.add_argument("--segmentation-manifest", type=Path, required=True)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--visual-mesh", type=Path, required=True, help="Clean parameterized OBJ/GLB layer, not sparse debug geometry.")
    parser.add_argument("--object-layer", type=Path, required=True)
    parser.add_argument("--collision-sweep-summary", type=Path, help="Optional completed offline sweep summary used for Simulation Ready gating.")
    parser.add_argument("--json-output", type=Path, required=True)
    parser.add_argument("--markdown-output", type=Path, required=True)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        required = (args.scene, args.static_config, args.operational_config, args.transforms, args.segmentation_manifest, args.scene_manifest, args.visual_mesh, args.object_layer)
        for path in required:
            if not path.is_file():
                raise FileNotFoundError(path)
        static = load_structured(args.static_config); operational = load_structured(args.operational_config)
        transforms = load_structured(args.transforms); segmentation = load_structured(args.segmentation_manifest)
        scene_manifest = load_structured(args.scene_manifest); object_layer = load_structured(args.object_layer)
        checks: list[dict] = []
        checks.append(item("world_frame_policy", "PASS" if static.get("frame") == "P" and scene_manifest.get("world_frame") == "P" else "FAIL", "Engineering and MuJoCo world must be P."))
        t_b_p = transforms["transforms"]["T_B_P"]
        unresolved = t_b_p.get("translation_m") == [None, None, None] and t_b_p.get("quaternion_xyzw") == [None, None, None, None]
        checks.append(item("calibrated_T_B_P_preserved", "PASS" if unresolved else "FAIL", "Calibrated T_B_P must remain unresolved; operational placement is a separate transform.", t_b_p.get("status")))

        model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
        data = mujoco.MjData(model); data.qpos[:] = model.qpos0; mujoco.mj_forward(model, data)
        checks.append(item("scene_load", "PASS", "Default clean MuJoCo XML compiled successfully.", {"nbody": model.nbody, "ngeom": model.ngeom, "nmesh": model.nmesh, "ncam": model.ncam}))
        base = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
        hand = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "rh56_R_hand_base_link")
        checks.append(item("robot_and_hand_assets", "PASS" if base >= 0 and hand >= 0 else "FAIL", "Existing JAKA and Inspire bodies are present exactly once."))
        expected_q_xyzw = np.asarray(operational["quaternion_xyzw"], float)
        expected_q_wxyz = expected_q_xyzw[[3,0,1,2]]
        root_ok = base >= 0 and np.allclose(model.body_pos[base], operational["translation_m"], atol=1e-12) and np.allclose(np.abs(np.dot(model.body_quat[base], expected_q_wxyz)), 1.0, atol=1e-10)
        checks.append(item("operational_root_transform", "PASS" if root_ok else "FAIL", "T_P_B_operational is explicitly applied to jaka_Link_0.", {"translation_P_m": model.body_pos[base].tolist(), "quaternion_wxyz": model.body_quat[base].tolist(), "yaw_deg": operational["yaw_deg"]} if base >= 0 else None))
        base_center_ok = base >= 0 and np.allclose(data.xpos[base], [0,0,0], atol=1e-10)
        checks.append(item("robot_base_center_and_height", "PASS" if base_center_ok else "FAIL", "Root yaw must not move the base centre or mounting height.", data.xpos[base].tolist() if base >= 0 else None, [0,0,0]))

        jaka_joint_ids = [object_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in operational["reference_state"]["jaka_joint_names"]]
        jaka_qpos = [float(model.qpos0[model.jnt_qposadr[index]]) for index in jaka_joint_ids if index >= 0]
        checks.append(item("jaka_zero_reference", "PASS" if len(jaka_qpos) == 6 and np.allclose(jaka_qpos, 0, atol=0) else "FAIL", "All six JAKA joints remain at canonical zero; root yaw is not hidden in qpos.", jaka_qpos, [0.0]*6))
        rh_joint_ids = [index for index in range(model.njnt) if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index) or "").startswith("rh56_")]
        rh_qpos = [float(model.qpos0[model.jnt_qposadr[index]]) for index in rh_joint_ids]
        checks.append(item("rh56_default_reference", "PASS" if len(rh_qpos) == 12 and np.allclose(rh_qpos, 0, atol=0) else "FAIL", "RH56 retains its existing default-zero mounted state.", rh_qpos, [0.0]*12))

        palm_world = np.asarray(data.xmat[hand]).reshape(3,3) @ np.asarray(operational["direction_constraints"]["palm"]["local_vector"], float)
        cable_world = np.asarray(data.xmat[base]).reshape(3,3) @ np.asarray(operational["direction_constraints"]["communication_cable_side"]["local_vector"], float)
        target = np.asarray([-1.0,0.0,0.0])
        palm_error = angular_error_deg(palm_world, target); cable_error = angular_error_deg(cable_world, target)
        checks.append(item("palm_normal_alignment", orientation_status(palm_error), "RH56 local +y palm normal must face P -x at all-zero state.", {"direction_P": palm_world.tolist(), "angular_error_deg": palm_error}, [-1,0,0]))
        checks.append(item("communication_cable_side_alignment", "PROVISIONAL" if cable_error < 3 else "FAIL", "Link0 body-fixed +x operational cable-side reference must face P -x; connector mesh is absent.", {"direction_P": cable_world.tolist(), "angular_error_deg": cable_error, "evidence": "annotated_P_frame.jpg"}, [-1,0,0]))
        checks.append(item("palm_and_cable_debug_sites", "PASS" if object_id(model, mujoco.mjtObj.mjOBJ_SITE, "rh56_palm_normal") >= 0 and object_id(model, mujoco.mjtObj.mjOBJ_SITE, "jaka_cable_side_direction") >= 0 else "FAIL", "Nonphysical direction arrows exist without modifying collisions."))

        mount_pos = model.body_pos[hand].tolist() if hand >= 0 else None
        mount_ok = hand >= 0 and np.allclose(model.body_pos[hand], [0,0,.009], atol=1e-12) and np.allclose(np.abs(np.dot(model.body_quat[hand], [0, .707106781, .707106781, 0])), 1.0, atol=1e-8)
        checks.append(item("T_F_H_unchanged", "PASS" if mount_ok else "FAIL", "Repository T_F_H remains unchanged; the common yaw error is corrected at the root.", {"translation_m": mount_pos, "quaternion_wxyz": model.body_quat[hand].tolist()} if hand >= 0 else None))

        sparse_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "colmap_sparse_debug")
        layers = static["visual_layers"]
        clean_default = sparse_id < 0 and layers.get("sparse_reconstruction_debug") is False and layers.get("permanent_background") is False
        checks.append(item("clean_default_visual_layers", "PASS" if clean_default else "FAIL", "Default XML contains no sparse cubes or unqualified sparse background.", {"sparse_geom_id": sparse_id, "visual_layers": layers}))
        if args.sparse_debug_scene:
            if not args.sparse_debug_scene.is_file():
                raise FileNotFoundError(args.sparse_debug_scene)
            debug_model = mujoco.MjModel.from_xml_path(str(args.sparse_debug_scene.resolve()))
            debug_id = object_id(debug_model, mujoco.mjtObj.mjOBJ_GEOM, "colmap_sparse_debug")
            debug_safe = debug_id >= 0 and debug_model.geom_contype[debug_id] == 0 and debug_model.geom_conaffinity[debug_id] == 0
            checks.append(item("optional_sparse_debug_layer", "PASS" if debug_safe else "FAIL", "Optional compact sparse markers exist only in the debug scene and have collision disabled."))
        else:
            checks.append(item("optional_sparse_debug_layer", "MISSING", "No debug scene supplied to validator; default scene remains clean."))

        camera_sites = ["camera_external_placeholder", "camera_wrist_placeholder", "iphone_reconstruction_camera_placeholder"]
        camera_ok = all(object_id(model, mujoco.mjtObj.mjOBJ_SITE, name) >= 0 and object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name) < 0 for name in camera_sites)
        checks.append(item("camera_placeholders_noncolliding", "PROVISIONAL" if camera_ok else "FAIL", "Camera placeholders are named sites, not collision geoms; extrinsics remain uncalibrated."))

        table_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_tabletop")
        rail_names = ["workspace_rail_positive_y", "workspace_rail_negative_y", "workspace_front_transverse_member"]
        rail_ids = [object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in rail_names]
        table = static["segmentation"]["tabletop"]
        table_ok = table_id >= 0 and np.allclose(model.geom_pos[table_id], table["center_P_m"], atol=1e-12) and np.allclose(model.geom_size[table_id], np.asarray(table["dimensions_m"])/2, atol=1e-12)
        checks.append(item("table_alignment", "PROVISIONAL" if table_ok else "FAIL", "Operational table primitive is unchanged by root-yaw correction."))
        rails_ok = all(index >= 0 for index in rail_ids)
        checks.append(item("aluminium_frame_alignment", "PROVISIONAL" if rails_ok else "FAIL", "Operational aluminium primitives are unchanged by root-yaw correction."))

        robot_workspace_contacts, self_contacts = [], []
        workspace_ids = {table_id, *rail_ids, object_id(model, mujoco.mjtObj.mjOBJ_GEOM, "workspace_floor")}
        for index in range(data.ncon):
            contact = data.contact[index]
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)), mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))]
            record = {"geoms": names, "distance_m": float(contact.dist)}
            (robot_workspace_contacts if int(contact.geom1) in workspace_ids or int(contact.geom2) in workspace_ids else self_contacts).append(record)
        checks.append(item("robot_workspace_penetration_zero_pose", "PASS" if not robot_workspace_contacts else "FAIL", "No new robot/table/frame/floor contacts at corrected all-zero pose.", robot_workspace_contacts))
        checks.append(item("existing_robot_self_contacts", "WARN" if self_contacts else "PASS", "Canonical self-contacts are reported separately.", self_contacts))

        mesh = trimesh.load_mesh(args.visual_mesh, process=False)
        clean_mesh_ok = len(mesh.faces) > 0 and np.all(np.isfinite(mesh.face_normals)) and int(mesh.body_count) <= 4
        checks.append(item("clean_static_export", "PASS" if clean_mesh_ok else "FAIL", "Default OBJ/GLB contains only clean table/frame primitives, not sparse markers.", {"vertices": len(mesh.vertices), "faces": len(mesh.faces), "components": int(mesh.body_count)}))
        checks.append(item("duplicate_reconstructed_robot", "PASS" if segmentation.get("robot_reconstruction_removed") and not segmentation.get("calibration_boards_included") else "FAIL", "Reconstructed robot and boards are excluded from active/default visual layers."))
        checks.append(item("future_object_layer", "PASS" if object_layer.get("status") == "ready_empty_layer" and object_layer.get("objects") == [] else "FAIL", "Future objects remain insertable in P."))

        package_files = ["workspace_scene.ply","workspace_scene.obj","workspace_scene.glb","workspace_clean_engineering.png","workspace_clean_presentation.png","zero_pose_top_verified.png","orientation_before.png","orientation_after.png","sparse_debug_optional.png"]
        missing_package = [name for name in package_files if not (args.visual_mesh.parent / name).is_file()]
        checks.append(item("scene_package", "PASS" if not missing_package else "FAIL", "Required clean/debug/orientation package files exist.", missing_package))

        collision_sweep = None
        if args.collision_sweep_summary:
            if not args.collision_sweep_summary.is_file():
                raise FileNotFoundError(args.collision_sweep_summary)
            collision_sweep = load_structured(args.collision_sweep_summary)
            completed = int(collision_sweep.get("static_configuration_count", 0)) > 0 and int(collision_sweep.get("dynamic_trajectory_count", 0)) > 0
            checks.append(item("offline_collision_sweep_completed", "PASS" if completed else "FAIL", "Deterministic offline MuJoCo sweep produced static, dynamic, contact and diagnostic evidence.", {"static_configurations": collision_sweep.get("static_configuration_count"), "dynamic_trajectories": collision_sweep.get("dynamic_trajectory_count"), "dynamic_steps": collision_sweep.get("total_dynamic_steps")}))
            ready = collision_sweep.get("digital_twin_maturity") == "Simulation Ready" and collision_sweep.get("status") == "PASS"
            checks.append(item("simulation_ready_collision_gate", "PASS" if ready else "FAIL", "Simulation Ready requires every sweep acceptance gate to pass; this remains simulation-only evidence.", {"sweep_status": collision_sweep.get("status"), "maturity": collision_sweep.get("digital_twin_maturity"), "acceptance": collision_sweep.get("acceptance")}))

        failures = [entry for entry in checks if entry["status"] == "FAIL"]
        status_counts = {status: sum(entry["status"] == status for entry in checks) for status in ("PASS", "PROVISIONAL", "WARN", "FAIL", "MISSING")}
        ready_for_sweep = not failures and palm_error < 3 and cable_error < 3 and not robot_workspace_contacts
        simulation_ready = bool(collision_sweep and collision_sweep.get("status") == "PASS" and not failures)
        report = {
            "schema_version": 2, "status": "PASS_WITH_PROVISIONAL_ITEMS" if not failures else "FAIL",
            "digital_twin_maturity": "Simulation Ready" if simulation_ready else "Integrated Workspace", "world_frame": "P", "T_B_P_required": False,
            "status_counts": status_counts, "checks": checks,
            "orientation": {"selected_operational_yaw_deg": operational["yaw_deg"], "quaternion_xyzw": operational["quaternion_xyzw"], "palm_error_deg": palm_error, "cable_side_error_deg": cable_error},
            "candidate_yaw_evaluation": operational["candidate_yaw_evaluation"],
            "ready_for_offline_joint_space_collision_sweep": ready_for_sweep,
            "offline_joint_space_collision_sweep_completed": bool(collision_sweep),
            "simulation_ready": simulation_ready,
            "collision_sweep_summary": collision_sweep,
            "remaining_manipulation_blockers": ["eye_to_hand_calibration", "wrist_camera_calibration", "final_robot_world_registration", "resolve_collision_sweep_failures"],
        }
        if args.dry_run:
            print(json.dumps(report, indent=2)); return
        write_json(args.json_output, report)
        rows = "\n".join(f"| {entry['name']} | {entry['status']} | {entry['message']} |" for entry in checks)
        candidates = "\n".join(f"| {entry['yaw_deg']:.0f} | {entry['palm_error_deg']:.0f} | {entry['cable_side_error_deg']:.0f} | {entry['accepted']} |" for entry in operational["candidate_yaw_evaluation"])
        markdown = f"""# Clean Integrated Workspace Validation\n\nOverall: **{report['status']}**. Maturity: **{report['digital_twin_maturity']}**. World=P. Operational yaw: **{operational['yaw_deg']:.1f}°**. Palm error: **{palm_error:.6f}°**. Cable-side error: **{cable_error:.6f}°**. Calibrated `T_B_P` remains unresolved.\n\nSummary: {status_counts['PASS']} PASS, {status_counts['PROVISIONAL']} PROVISIONAL, {status_counts['WARN']} WARN, {status_counts['FAIL']} FAIL, {status_counts['MISSING']} MISSING. Offline sweep completed: **{bool(collision_sweep)}**. Simulation Ready: **{simulation_ready}**.\n\nThe sweep is offline MuJoCo characterization, not real-robot safety validation.\n\n## Root-yaw candidates\n\n| Yaw (deg) | Palm error (deg) | Cable-side error (deg) | Accepted |\n|---:|---:|---:|---|\n{candidates}\n\n| Check | Status | Detail |\n|---|---|---|\n{rows}\n"""
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(markdown, encoding="utf-8")
        print(f"Clean integrated scene validation written to: {args.markdown_output}")
        if args.strict and failures:
            raise SystemExit(2)
    except (FileNotFoundError, KeyError, TypeError, ValueError, mujoco.FatalError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
