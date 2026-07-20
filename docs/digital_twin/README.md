# Real-to-Sim digital twin

## Current state

Maturity remains **Integrated Workspace** after the first deterministic offline collision sweep. The default MuJoCo scene is a clean P-frame engineering view: existing JAKA Mini + Inspire RH56, parameterized table and aluminium frame, floor, lights, optional camera placeholders, coordinate/debug axes, and an empty future-object layer. Sparse COLMAP markers are disabled by default.

This subsystem does not command the physical robot and does not implement grasping, planning, learning, or final camera/robot calibration.

## Frames and operational placement

World = P. P is the centre of the fixed 110 mm mounting PCD on the lowest fixed mounting plane; +z points upward and +x follows the two longitudinal rails toward the front transverse member/operator side. The user-supplied `annotated_P_frame.jpg` confirms that the fixed communication-cable side is P -x.

B remains the internal `jaka_Link_0` frame. A separate scene transform is used:

```text
T_P_B_operational
translation: [0, 0, 0] m
yaw: 180 deg
quaternion xyzw: [0, 0, 1, 0]
status: physically_constrained_provisional
```

This is not calibrated `T_B_P`; `T_B_P` remains null. The distinction is recorded in `digital_twin/configs/transforms.yaml` and `robot_operational_placement.yaml`.

At all-zero JAKA and RH56 state, RH56 local +y is the outward palm normal. The repository `T_F_H` maps it to B +x. The 180° operational root yaw maps it to P -x with 0.000211° numerical error. The body-fixed cable-side reference also maps to P -x. `T_F_H`, joint zeros, robot meshes, collisions, and actuators are unchanged.

## Visual-layer policy

Default:

- robot, table, aluminium frame, floor and lighting: enabled;
- coordinate axes and camera placeholders: enabled in engineering view;
- sparse reconstruction, cables, boards and clutter: disabled;
- unqualified permanent background: disabled because no clean dense/texture representation exists.

The optional `colmap_sparse_debug` layer uses cropped, track-filtered, statistically/radius-filtered, component-filtered and downsampled compact markers. It is stored separately and always has `contype=0`, `conaffinity=0`. It is not a surface and must not become collision geometry.

## Reproduce

These commands reuse accepted reconstruction outputs; they do not rerun COLMAP, ChArUco, or metric scale estimation.

```bash
.venv/bin/python tools/digital_twin/build_workspace_visual.py \
  --points3d artifacts/digital_twin/reconstruction/02/colmap/sparse_text/1/points3D.txt \
  --registration artifacts/digital_twin/calibration/T_P_R.json \
  --config digital_twin/configs/static_environment.yaml \
  --output-dir artifacts/digital_twin/static_scene \
  --max-reprojection-error-px 2.0

.venv/bin/python tools/digital_twin/build_mujoco_workspace_scene.py \
  --robot-model data/sim_assets/jaka_rh56_visual_coacd.xml \
  --static-config digital_twin/configs/static_environment.yaml \
  --camera-config digital_twin/configs/camera_placeholders.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --output models/digital_twin/workspace_scene.xml \
  --alias-output models/digital_twin/scene.xml \
  --manifest artifacts/digital_twin/static_scene/scene_manifest.yaml

.venv/bin/python tools/digital_twin/build_mujoco_workspace_scene.py \
  --robot-model data/sim_assets/jaka_rh56_visual_coacd.xml \
  --static-config digital_twin/configs/static_environment.yaml \
  --camera-config digital_twin/configs/camera_placeholders.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --visual-mesh artifacts/digital_twin/static_scene/sparse_debug.obj \
  --show-sparse-debug \
  --output models/digital_twin/workspace_scene_sparse_debug.xml \
  --manifest artifacts/digital_twin/static_scene/scene_manifest_sparse_debug.yaml

MUJOCO_GL=egl .venv/bin/python tools/digital_twin/render_workspace_scene.py \
  --scene models/digital_twin/workspace_scene.xml \
  --sparse-debug-scene models/digital_twin/workspace_scene_sparse_debug.xml \
  --show-sparse-debug \
  --output-dir artifacts/digital_twin/static_scene

.venv/bin/python tools/digital_twin/validate_integrated_workspace.py \
  --scene models/digital_twin/workspace_scene.xml \
  --sparse-debug-scene models/digital_twin/workspace_scene_sparse_debug.xml \
  --static-config digital_twin/configs/static_environment.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --transforms digital_twin/configs/transforms.yaml \
  --segmentation-manifest artifacts/digital_twin/static_scene/segmentation_manifest.yaml \
  --scene-manifest artifacts/digital_twin/static_scene/scene_manifest.yaml \
  --visual-mesh artifacts/digital_twin/static_scene/workspace_scene.obj \
  --object-layer digital_twin/configs/object_layer.yaml \
  --collision-sweep-summary artifacts/digital_twin/collision_sweep/summary.json \
  --json-output artifacts/digital_twin/validation_report.json \
  --markdown-output artifacts/digital_twin/validation_report.md

MUJOCO_GL=egl .venv/bin/python tools/digital_twin/run_joint_space_collision_sweep.py \
  --scene models/digital_twin/workspace_scene.xml \
  --classification digital_twin/configs/collision_classification.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --output artifacts/digital_twin/collision_sweep
```

Renderer options include `--hide-camera-placeholders`, `--clean-preview`, and explicit `--show-sparse-debug`. The default scene builder never includes sparse markers unless `--show-sparse-debug` is passed.

## Important outputs

- default scene: `models/digital_twin/workspace_scene.xml`;
- optional debug scene: `models/digital_twin/workspace_scene_sparse_debug.xml`;
- clean engineering/presentation views: `workspace_clean_engineering.png`, `workspace_clean_presentation.png`;
- root comparison: `orientation_before.png`, `orientation_after.png`;
- verified top view: `zero_pose_top_verified.png`;
- debug-only sparse view: `sparse_debug_optional.png`;
- validation: `artifacts/digital_twin/validation_report.{json,md}`.
- collision sweep: `artifacts/digital_twin/collision_sweep/summary.{json,md}`, compact WARN/FAIL event JSON, exact sampled qpos, per-step contact timeline and event renders.

## Current boundary and next task

The sweep evaluated 130 static configurations and nine actuator-driven trajectories (31,995 MuJoCo steps). Zero pose remains free of environment contact, no floor contact occurred, the operational yaw stayed 180°, and the simulation remained finite without solver warnings. Three trajectories failed the configured policy: low-table approach (persistent hand/table contact), RH56 open-to-close and close-to-open (non-adjacent thumb/index CoACD self-contact). Forward P -x reach warned on persistent shallow Link5/table contact. Static diagnostic samples reached 81.894 mm environment penetration and 63.370 kN simulated normal constraint force; these are deliberately infeasible sampled states, not hardware-force predictions. Symmetric rail contacts also require rail-spacing/primitive review.

The scene therefore does **not** qualify as Simulation Ready. The next task is to review the rail/table primitive placement and the failing event renders, then decide whether each finding is a scene-geometry error, an overly conservative collision primitive, or a pose that must be excluded. Eye-to-hand calibration, wrist-camera calibration and final robot/world registration remain Manipulation Ready blockers. This sweep is simulation-only characterization, not safety certification.
