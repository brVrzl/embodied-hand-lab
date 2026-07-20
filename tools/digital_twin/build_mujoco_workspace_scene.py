#!/usr/bin/env python3
"""Build a P-world MuJoCo scene from the canonical robot MJCF plus workspace layers."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from digital_twin.io import load_structured, write_yaml


def relative_asset(source_parent: Path, file_value: str, output_parent: Path) -> str:
    return Path(os.path.relpath((source_parent / file_value).resolve(), output_parent.resolve())).as_posix()


def append_axis_sites(worldbody: ET.Element, prefix: str, origin: tuple[float, float, float], alpha: float = 1.0) -> None:
    x, y, z = origin
    ET.SubElement(worldbody, "site", {"name": f"{prefix}_origin", "type": "sphere", "pos": f"{x} {y} {z}", "size": ".007", "rgba": f"1 1 1 {alpha}", "group": "4"})
    for suffix, end, color in [("x", (x+.18,y,z), "1 0 0"),("y", (x,y+.18,z), "0 1 0"),("z", (x,y,z+.18), "0 0 1")]:
        ET.SubElement(worldbody, "site", {"name": f"{prefix}_{suffix}", "type": "cylinder", "fromto": f"{x} {y} {z} {end[0]} {end[1]} {end[2]}", "size": ".0025", "rgba": f"{color} {alpha}", "group": "4"})


def xyzw_to_wxyz(values: list[float]) -> list[float]:
    if len(values) != 4:
        raise ValueError("Operational quaternion must contain four xyzw values.")
    return [values[3], values[0], values[1], values[2]]


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate workspace_scene.xml with world=P and an unchanged canonical JAKA/RH56 robot definition.")
    parser.add_argument("--robot-model", type=Path, required=True)
    parser.add_argument("--static-config", type=Path, required=True)
    parser.add_argument("--camera-config", type=Path, required=True)
    parser.add_argument("--operational-config", type=Path, required=True, help="Physically constrained T_P_B_operational configuration.")
    parser.add_argument("--visual-mesh", type=Path, help="Optional compact sparse-debug marker OBJ.")
    parser.add_argument("--show-sparse-debug", action="store_true", help="Include the non-colliding sparse reconstruction debug mesh. Disabled by default.")
    parser.add_argument("--hide-camera-placeholders", action="store_true", help="Omit nonphysical camera placeholder sites.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--alias-output", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        for path in (args.robot_model, args.static_config, args.camera_config, args.operational_config):
            if not path.is_file():
                raise FileNotFoundError(path)
        if args.show_sparse_debug and (args.visual_mesh is None or not args.visual_mesh.is_file()):
            raise FileNotFoundError("--show-sparse-debug requires an existing --visual-mesh OBJ.")
        static, cameras = load_structured(args.static_config), load_structured(args.camera_config)
        operational = load_structured(args.operational_config)
        tree = ET.parse(args.robot_model)
        root = tree.getroot(); root.set("model", "jaka_inspire_integrated_workspace_P")
        root.insert(0, ET.Comment("Generated scene wrapper: canonical robot source remains untouched; derivative applies T_P_B_operational, portable asset paths, and P-world floor/light."))
        compiler = root.find("compiler")
        if compiler is None:
            compiler = ET.Element("compiler"); root.insert(1, compiler)
        compiler.set("angle", "radian"); compiler.set("discardvisual", "false")
        output_parent = args.output.parent
        for mesh in root.findall("./asset/mesh"):
            if mesh.get("file"):
                mesh.set("file", relative_asset(args.robot_model.parent, mesh.get("file"), output_parent))
        asset = root.find("asset")
        if asset is None:
            raise ValueError("Canonical robot model has no asset block.")
        if args.show_sparse_debug:
            ET.SubElement(asset, "mesh", {"name": "colmap_sparse_debug_mesh", "file": Path(os.path.relpath(args.visual_mesh.resolve(), output_parent.resolve())).as_posix()})
        ET.SubElement(asset, "material", {"name": "workspace_table_wood", "rgba": "0.58 0.35 0.16 1", "roughness": ".8"})
        ET.SubElement(asset, "material", {"name": "workspace_aluminium", "rgba": ".72 .76 .80 1", "metallic": ".5", "roughness": ".35"})
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("Canonical robot model has no worldbody.")
        for child in list(worldbody):
            if child.tag == "light" or (child.tag == "geom" and child.get("name") == "floor"):
                worldbody.remove(child)
        robot_root = worldbody.find("./body[@name='jaka_Link_0']")
        if robot_root is None:
            raise ValueError("Canonical robot model does not contain worldbody/jaka_Link_0.")
        translation = operational["translation_m"]
        quaternion_xyzw = operational["quaternion_xyzw"]
        robot_root.set("pos", " ".join(map(str, translation)))
        robot_root.set("quat", " ".join(map(str, xyzw_to_wxyz(quaternion_xyzw))))
        worldbody.insert(0, ET.Comment("World=P. jaka_Link_0/B uses T_P_B_operational; calibrated T_B_P remains unresolved."))
        ET.SubElement(worldbody, "geom", {"name": "workspace_floor", "type": "plane", "pos": "0 0 -0.80", "size": "2.5 2.5 .1", "rgba": ".32 .34 .36 1", "friction": "1 .01 .001", "contype": "1", "conaffinity": "1"})
        ET.SubElement(worldbody, "light", {"name": "workspace_key_light", "pos": "-.4 -.5 1.7", "dir": ".2 .2 -1", "diffuse": ".85 .85 .85", "castshadow": "true"})
        ET.SubElement(worldbody, "light", {"name": "workspace_fill_light", "pos": ".8 -.2 1.0", "dir": "-.6 0 -1", "diffuse": ".45 .48 .52", "castshadow": "false"})
        table = static["segmentation"]["tabletop"]
        table_half = [value / 2 for value in table["dimensions_m"]]
        ET.SubElement(worldbody, "geom", {"name": "workspace_tabletop", "type": "box", "pos": " ".join(map(str, table["center_P_m"])), "size": " ".join(map(str, table_half)), "material": "workspace_table_wood", "friction": "1.0 .01 .001", "contype": "1", "conaffinity": "1", "group": "0"})
        for member in static["aluminium_frame"]["members"]:
            ET.SubElement(worldbody, "geom", {"name": f"workspace_{member['name']}", "type": "box", "pos": " ".join(map(str, member["center_P_m"])), "size": " ".join(str(value/2) for value in member["dimensions_P_xyz_m"]), "material": "workspace_aluminium", "friction": ".8 .01 .001", "contype": "1", "conaffinity": "1", "group": "0"})
        if args.show_sparse_debug:
            ET.SubElement(worldbody, "geom", {"name": "colmap_sparse_debug", "type": "mesh", "mesh": "colmap_sparse_debug_mesh", "rgba": ".72 .72 .72 .72", "contype": "0", "conaffinity": "0", "group": "2"})
        append_axis_sites(worldbody, "P", (0.0,0.0,0.0))
        ET.SubElement(robot_root, "site", {"name": "jaka_cable_side_origin", "type": "sphere", "pos": "0 0 .055", "size": ".006", "rgba": "1 .25 .1 1", "group": "4"})
        ET.SubElement(robot_root, "site", {"name": "jaka_cable_side_direction", "type": "cylinder", "fromto": "0 0 .055 .16 0 .055", "size": ".004", "rgba": "1 .25 .1 1", "group": "4"})
        hand_body = robot_root.find(".//body[@name='rh56_R_hand_base_link']")
        if hand_body is None:
            raise ValueError("Canonical robot model does not contain rh56_R_hand_base_link.")
        ET.SubElement(hand_body, "site", {"name": "rh56_palm_frame", "type": "sphere", "pos": "-.002 0 -.068", "size": ".006", "rgba": ".1 .9 .2 1", "group": "4"})
        ET.SubElement(hand_body, "site", {"name": "rh56_palm_normal", "type": "cylinder", "fromto": "-.002 0 -.068 -.002 .13 -.068", "size": ".004", "rgba": ".1 .9 .2 1", "group": "4"})
        if not args.hide_camera_placeholders:
            ET.SubElement(worldbody, "site", {"name": "camera_external_placeholder", "type": "box", "pos": "-.9 -1.1 .75", "size": ".025 .018 .015", "rgba": ".9 .2 .8 .8", "group": "5"})
            ET.SubElement(worldbody, "site", {"name": "camera_wrist_placeholder", "type": "box", "pos": "0 0 .187", "size": ".018 .012 .010", "rgba": ".2 .8 .9 .8", "group": "5"})
            ET.SubElement(worldbody, "site", {"name": "iphone_reconstruction_camera_placeholder", "type": "sphere", "pos": "-.55 -.85 .65", "size": ".018", "rgba": ".9 .7 .1 .8", "group": "5"})
        ET.SubElement(worldbody, "site", {"name": "future_object_layer_origin", "type": "sphere", "pos": "0 -.35 -.04", "size": ".006", "rgba": ".2 1 .4 .6", "group": "4"})
        ET.SubElement(worldbody, "camera", {"name": "digital_twin_validation_camera", "mode": "fixed", "pos": "-1.35 -1.65 1.25", "xyaxes": ".693641687 -.720320213 0 .339091571 .326532624 .882266032", "fovy": "52"})
        ET.SubElement(worldbody, "camera", {"name": "workspace_oblique_camera", "mode": "fixed", "pos": "-1.25 -1.55 1.05", "xyaxes": ".692531828 -.721387321 0 .317705177 .304996970 .897797454", "fovy": "52"})
        ET.SubElement(worldbody, "camera", {"name": "workspace_top_camera", "mode": "fixed", "pos": "0 -.545 1.55", "xyaxes": "1 0 0 0 1 0", "fovy": "50"})
        ET.SubElement(worldbody, "camera", {"name": "zero_pose_top_camera", "mode": "fixed", "pos": "0 0 1.35", "xyaxes": "1 0 0 0 1 0", "fovy": "30"})
        ET.SubElement(worldbody, "camera", {"name": "external_camera_placeholder", "mode": "fixed", "pos": "-.9 -1.1 .75", "xyaxes": ".773957 -.633238 0 .287439 .351314 .891007", "fovy": "45"})
        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")
        global_visual = visual.find("global")
        if global_visual is None:
            global_visual = ET.SubElement(visual, "global")
        global_visual.set("offwidth", "1280"); global_visual.set("offheight", "720")
        if args.dry_run:
            print(json.dumps({"source_robot": str(args.robot_model), "output": str(args.output), "world": "P", "operational_yaw_deg": operational["yaw_deg"], "sparse_debug": args.show_sparse_debug, "members": len(static["aluminium_frame"]["members"])}, indent=2)); return
        args.output.parent.mkdir(parents=True, exist_ok=True)
        ET.indent(tree, space="  ")
        tree.write(args.output, encoding="utf-8", xml_declaration=False)
        if args.alias_output:
            args.alias_output.parent.mkdir(parents=True, exist_ok=True)
            args.alias_output.write_bytes(args.output.read_bytes())
        source_hash = hashlib.sha256(args.robot_model.read_bytes()).hexdigest()
        manifest = {
            "schema_version": 2, "scene": str(args.output), "world_frame": "P",
            "digital_twin_maturity": "Integrated Workspace",
            "robot_internal_frame": "B", "robot_source": str(args.robot_model), "robot_source_sha256": source_hash,
            "robot_source_modified": False, "robot_scene_copy_policy": "derivative preserves robot joints, meshes, collisions, actuators and T_F_H; changes only root operational pose, relative asset paths, debug sites, and P-world floor/light",
            "operational_robot_placement": {"translation_P_m": translation, "quaternion_xyzw": quaternion_xyzw, "yaw_deg": operational["yaw_deg"], "status": operational["status"], "source": str(args.operational_config)},
            "calibrated_T_B_P_status": "future_robot_calibration_nonblocking_unresolved",
            "default_visual_layers": static["visual_layers"],
            "sparse_debug_included": args.show_sparse_debug,
            "sparse_debug_mesh": str(args.visual_mesh) if args.visual_mesh else None, "sparse_debug_collision": False,
            "table_collision": True, "aluminium_collision": True, "background_collision": False,
            "camera_config": str(args.camera_config), "camera_placeholders_included": not args.hide_camera_placeholders, "camera_placeholder_collision": False,
            "palm_reference": {"body": "rh56_R_hand_base_link", "local_normal": [0,1,0], "site": "rh56_palm_normal"},
            "cable_side_reference": {"body": "jaka_Link_0", "local_direction": [1,0,0], "site": "jaka_cable_side_direction", "model_landmark_status": "fixed_direction_connector_mesh_absent"},
            "reference_qpos": "all model qpos0 values; JAKA six and RH56 twelve are zero",
            "future_object_layer": "digital_twin/configs/object_layer.yaml",
            "segmentation_manifest": "artifacts/digital_twin/static_scene/segmentation_manifest.yaml",
            "validation_report": "artifacts/digital_twin/validation_report.json",
            "assets": {
                "mujoco_scene": str(args.output),
                "scene_glb": "artifacts/digital_twin/static_scene/workspace_scene.glb",
                "scene_obj": "artifacts/digital_twin/static_scene/workspace_scene.obj",
                "scene_ply": "artifacts/digital_twin/static_scene/workspace_scene.ply",
                "preview": "artifacts/digital_twin/static_scene/workspace_scene_preview.png",
                "top": "artifacts/digital_twin/static_scene/workspace_scene_top.png",
                "oblique": "artifacts/digital_twin/static_scene/workspace_scene_oblique.png",
                "wireframe": "artifacts/digital_twin/static_scene/workspace_scene_wireframe.png",
                "axes": "artifacts/digital_twin/static_scene/workspace_scene_axes.png",
                "clean_engineering": "artifacts/digital_twin/static_scene/workspace_clean_engineering.png",
                "clean_presentation": "artifacts/digital_twin/static_scene/workspace_clean_presentation.png",
                "zero_pose_top": "artifacts/digital_twin/static_scene/zero_pose_top_verified.png",
                "orientation_before": "artifacts/digital_twin/static_scene/orientation_before.png",
                "orientation_after": "artifacts/digital_twin/static_scene/orientation_after.png",
                "sparse_debug_optional": "artifacts/digital_twin/static_scene/sparse_debug_optional.png",
            },
        }
        write_yaml(args.manifest, manifest)
        print(f"MuJoCo workspace scene written to: {args.output}")
    except (FileNotFoundError, KeyError, TypeError, ValueError, ET.ParseError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
