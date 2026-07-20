#!/usr/bin/env python3
"""Render clean engineering/presentation and zero-pose orientation diagnostics."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np


def render(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera: str,
    output: Path,
    *,
    show_engineering_sites: bool,
    show_camera_placeholders: bool,
    wireframe: bool = False,
) -> None:
    renderer = mujoco.Renderer(model, height=720, width=1280)
    option = mujoco.MjvOption()
    option.sitegroup[:] = 0
    option.sitegroup[4] = int(show_engineering_sites)
    option.sitegroup[5] = int(show_camera_placeholders)
    option.geomgroup[:] = 1
    renderer.update_scene(data, camera=camera, scene_option=option)
    if wireframe:
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
    rgb = renderer.render()
    renderer.close()
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def annotate_top(path: Path) -> None:
    image = cv2.imread(str(path))
    if image is None:
        raise ValueError(f"Could not read rendered top view: {path}")
    lines = [
        "Zero JAKA qpos / default-zero RH56",
        "green: palm normal -> P -x",
        "orange: fixed cable side -> P -x",
        "red/green/blue at base: P +x/+y/+z",
        "front transverse rail and operator side: P +x",
    ]
    y = 34
    for line in lines:
        cv2.putText(image, line, (22, y), cv2.FONT_HERSHEY_SIMPLEX, .62, (245, 245, 245), 2, cv2.LINE_AA)
        y += 28
    cv2.imwrite(str(path), image)


def main() -> None:
    parser = argparse.ArgumentParser(description="Render clean P-world views and all-zero root-orientation comparisons.")
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sparse-debug-scene", type=Path, help="Optional separately generated scene containing colmap_sparse_debug.")
    parser.add_argument("--show-sparse-debug", action="store_true", help="Render sparse_debug_optional.png from --sparse-debug-scene.")
    parser.add_argument("--hide-camera-placeholders", action="store_true", help="Hide camera sites in the engineering preview.")
    parser.add_argument("--clean-preview", action="store_true", help="Make workspace_scene_preview.png the presentation view instead of engineering view.")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        if not args.scene.is_file():
            raise FileNotFoundError(args.scene)
        if args.show_sparse_debug and (args.sparse_debug_scene is None or not args.sparse_debug_scene.is_file()):
            raise FileNotFoundError("--show-sparse-debug requires --sparse-debug-scene.")
        outputs = [
            "workspace_clean_engineering.png", "workspace_clean_presentation.png",
            "zero_pose_top_verified.png", "orientation_before.png", "orientation_after.png",
            "workspace_scene_preview.png", "workspace_scene_top.png", "workspace_scene_oblique.png",
            "workspace_scene_axes.png", "workspace_scene_wireframe.png",
        ]
        if args.show_sparse_debug:
            outputs.append("sparse_debug_optional.png")
        if args.dry_run:
            print(json.dumps({"outputs": outputs, "clean_preview": args.clean_preview, "camera_placeholders": not args.hide_camera_placeholders}, indent=2)); return

        model = mujoco.MjModel.from_xml_path(str(args.scene.resolve()))
        data = mujoco.MjData(model)
        data.qpos[:] = model.qpos0
        mujoco.mj_forward(model, data)
        show_cameras = not args.hide_camera_placeholders
        render(model, data, "digital_twin_validation_camera", args.output_dir / "workspace_clean_engineering.png", show_engineering_sites=True, show_camera_placeholders=show_cameras)
        render(model, data, "digital_twin_validation_camera", args.output_dir / "workspace_clean_presentation.png", show_engineering_sites=False, show_camera_placeholders=False)
        render(model, data, "zero_pose_top_camera", args.output_dir / "zero_pose_top_verified.png", show_engineering_sites=True, show_camera_placeholders=False)
        annotate_top(args.output_dir / "zero_pose_top_verified.png")
        render(model, data, "workspace_oblique_camera", args.output_dir / "workspace_scene_oblique.png", show_engineering_sites=True, show_camera_placeholders=show_cameras)
        render(model, data, "workspace_top_camera", args.output_dir / "workspace_scene_top.png", show_engineering_sites=True, show_camera_placeholders=show_cameras)
        render(model, data, "digital_twin_validation_camera", args.output_dir / "workspace_scene_axes.png", show_engineering_sites=True, show_camera_placeholders=show_cameras)
        render(model, data, "workspace_oblique_camera", args.output_dir / "workspace_scene_wireframe.png", show_engineering_sites=False, show_camera_placeholders=False, wireframe=True)

        base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "jaka_Link_0")
        if base < 0:
            raise ValueError("Scene does not contain jaka_Link_0.")
        corrected_quaternion = model.body_quat[base].copy()
        model.body_quat[base] = np.asarray([1.0, 0.0, 0.0, 0.0])
        mujoco.mj_forward(model, data)
        render(model, data, "digital_twin_validation_camera", args.output_dir / "orientation_before.png", show_engineering_sites=True, show_camera_placeholders=False)
        model.body_quat[base] = corrected_quaternion
        mujoco.mj_forward(model, data)
        render(model, data, "digital_twin_validation_camera", args.output_dir / "orientation_after.png", show_engineering_sites=True, show_camera_placeholders=False)

        selected = "workspace_clean_presentation.png" if args.clean_preview else "workspace_clean_engineering.png"
        (args.output_dir / "workspace_scene_preview.png").write_bytes((args.output_dir / selected).read_bytes())
        for source, alias in [
            ("workspace_scene_preview.png", "scene_preview.png"), ("workspace_scene_oblique.png", "scene_oblique.png"),
            ("workspace_scene_top.png", "scene_top.png"), ("workspace_scene_axes.png", "scene_axes.png"),
            ("workspace_scene_wireframe.png", "scene_wireframe.png"),
        ]:
            (args.output_dir / alias).write_bytes((args.output_dir / source).read_bytes())

        if args.show_sparse_debug:
            debug_model = mujoco.MjModel.from_xml_path(str(args.sparse_debug_scene.resolve()))
            debug_data = mujoco.MjData(debug_model); debug_data.qpos[:] = debug_model.qpos0; mujoco.mj_forward(debug_model, debug_data)
            render(debug_model, debug_data, "digital_twin_validation_camera", args.output_dir / "sparse_debug_optional.png", show_engineering_sites=False, show_camera_placeholders=False)
        print(f"Clean and diagnostic MuJoCo scene views written to: {args.output_dir}")
    except (FileNotFoundError, KeyError, TypeError, ValueError, mujoco.FatalError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
