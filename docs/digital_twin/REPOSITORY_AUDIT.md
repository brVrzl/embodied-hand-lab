# Digital-Twin Repository Audit

Audit date: 2026-07-15 (Asia/Shanghai)  
Scope: read-only inspection performed before digital-twin implementation. Existing dirty-worktree changes were treated as user work and were not modified.

## Repository structure overview

| Area | Existing role | Digital-twin relevance |
|---|---|---|
| `data/sim_assets/` | Canonical MuJoCo robot assets and meshes | Primary reusable robot layer |
| `src/sim_maniskill/` | Secondary ManiSkill/SAPIEN task and agent integration | Optional downstream simulator integration |
| `src/vision_interface/` | RGB-D interfaces, RealSense adapter, point-cloud transforms and tabletop processing | Reusable calibrated RGB-D geometry utilities |
| `configs/camera/` | Generic camera and two-RealSense configuration | Camera identity/intrinsic source; no base extrinsics |
| `configs/workspace/` | Rough tennis-ball workspace estimate | Provisional only; explicitly unregistered |
| `tools/` and `scripts/` | MuJoCo viewers, collision audits, hardware checks, teleoperation | Existing entry points and examples |
| `docs/` | Rebuild status, collision audit, camera/point-cloud notes | Evidence for asset status and known gaps |
| `IMG_6607.MOV` | Root-level iPhone workspace capture | Real-to-sim reconstruction input |

No existing `digital_twin/`, COLMAP, pycolmap, Nerfstudio, gsplat, Gaussian-Splat, NeRF, or photogrammetry pipeline was found. No JAKA URDF/Xacro/SDF was found in the inspected repository. The active robot model is MJCF.

## Simulation engines and entry points

The primary engine is **MuJoCo**. `pyproject.toml` declares `mujoco>=3.8.0`, and the installed project environment loads MuJoCo 3.9.0. The canonical mounted runtime asset is `data/sim_assets/jaka_rh56_visual_coacd.xml`; `data/sim_assets/jaka_rh56.xml` is a derivation/comparison anchor and must remain available.

Relevant MuJoCo entry points include:

- `tools/debug_mujoco_jaka_rh56_viewer.py`
- `tools/teleop_mujoco_jaka_rh56.py`
- `tools/mujoco_rh56_grasp_benchmark.py`
- `tools/audit_mujoco_rh56_collision_readiness.py`
- `tools/check_mujoco_rh56_collision_modes.py`
- `tools/run_hebi_mujoco_shadow.py`
- `tools/collect_mujoco_tennis_ball_data.py`
- shell wrappers under `scripts/` for viewers and benchmarks

ManiSkill 3/SAPIEN is a secondary simulation route in `src/sim_maniskill/`. It reuses the current mounted JAKA/RH56 model through its agent/task integration, but repository documentation treats this as a software-pipeline anchor rather than a validated physical twin.

ROS2/RViz paths exist for visualization and real-system bridges, but there is no Gazebo world or Isaac Sim scene in the repository.

## Reusable JAKA assets

- `data/sim_assets/meshes/jaka_minicobo_meshes/Link0.STL` through `Link6.STL`
- JAKA body hierarchy, inertials, six hinge joints, ranges and six actuators in both mounted MJCF assets
- model bodies `jaka_Link_0` through `jaka_Link_6`
- `jaka_dummy_tcp`, fixed at the `jaka_Link_6` body origin
- mock/real JAKA configuration and driver adapters under `configs/robot/` and `src/jaka_driver_adapter/`

The JAKA mesh assets are referenced without MJCF `scale` attributes and the JAKA geoms have identity local pose. Trimesh inspection gives meter-scale extents (for example, `Link0.STL` is approximately 0.124 m by 0.124 m by 0.031 m), consistent with MJCF's meter convention. This is evidence of apparent scale, not a physical dimensional calibration.

## Reusable Inspire RH56DFX assets

- Mounted vendor visual meshes in `data/sim_assets/meshes/rh56/`
- Mounted RH56 kinematics, joints, equality couplings and actuators in the JAKA/RH56 MJCF files
- Validated default collision runtime asset `data/sim_assets/jaka_rh56_visual_coacd.xml`
- 148 active `visual_coacd` hulls under `data/sim_assets/meshes/rh56_collision_visual_coacd/`
- Hash/asset manifests in `jaka_rh56_visual_coacd.manifest.json` and the collision directory
- Correll RH56DFX reference assets in `data/sim_assets/correll_rh56dfx/`, used separately for floating-hand FK and fingertip force/torque scene checks

The Correll reference hand is not a drop-in replacement for the mounted hand. Body names, mount frame, actuator order and role differ. The validated CoACD meshes must not be regenerated or retuned in this stage.

## Canonical robot-frame definitions

### JAKA base frame B

For this first implementation, B is the **MJCF body frame of `jaka_Link_0`**, not the visible mesh centroid. In both `jaka_rh56.xml` and `jaka_rh56_visual_coacd.xml`, this root body has `pos="0 0 0"` and identity quaternion under `worldbody`. Therefore the imported-model root and B coincide, and defining W ≡ B requires no additional transform.

- origin in model: `jaka_Link_0` body origin;
- model axes: right-handed MJCF axes; +z is upward/opposite gravity, and joint 1 rotates around +z;
- model/world transform: identity;
- mesh transform: Link0 geom identity; the mesh spans approximately z=0 to z=0.031 m;
- real mounting-surface realization: **unknown**. The model suggests its body origin lies at the bottom plane of the Link0 mesh, but a manufacturer drawing or physical datum is required before claiming this is the real canonical base origin.

Both inspected MuJoCo assets use the same base-body convention. No repository URDF exists for cross-checking a URDF convention.

### JAKA flange frame F

The repository has no explicit body or site named `flange` or `tool0`. The best existing model datum is the `jaka_Link_6` body frame; `jaka_dummy_tcp` is fixed to it with identity pose. Existing IK/debug code often uses the hand-base origin as an end-effector site instead of an independently defined flange site. Accordingly, the digital-twin scaffold will map F to `jaka_Link_6`/`jaka_dummy_tcp` **as repository-defined but physically unverified**. The real JAKA controller config calls its tool frame `jaka_tool0`, but no numerical transform tying controller `tool0` to this MJCF datum was found.

### Inspire base frame H and flange-to-hand mounting

H is the MJCF body frame `rh56_R_hand_base_link`. It is a direct child of `jaka_Link_6`, with repository transform:

- translation in F: `[0, 0, 0.009]` m;
- MuJoCo quaternion wxyz: approximately `[0, 0.707106781, 0.707106781, 0]`;
- equivalent configuration quaternion xyzw: approximately `[0.707106781, 0.707106781, 0, 0]`.

This transform is identical in the derivation and validated CoACD runtime assets. It is therefore reusable as a **repository-defined mount**, but no adapter CAD, manufacturer datum, measurement record, or physical verification was found. It must not be labeled calibrated. The RH56 base visual geom also carries a substantial mesh-level pose (`pos=-0.0781828944 0.248685517 -0.0830575893` and a near-180-degree quaternion), so visual mesh coordinates must not be mistaken for H.

## Units, axes, and transform conventions already present

- MuJoCo compiler angle: radians.
- MuJoCo length: implicit SI meters; model dimensions and meshes are consistent with this but not metrologically validated.
- gravity: `[0, 0, -9.81]` m/s².
- MJCF quaternions: wxyz.
- Existing Python camera geometry: meters.
- Existing RealSense optical convention: +x right, +y down, +z forward.
- Real JAKA SDK pose assumption in config: translation millimetres, rotation radians; adapters must convert explicitly.
- New digital-twin configuration convention: column vectors, `T_A_B` maps B into A, quaternion xyzw.

## Visual and collision mesh references

JAKA visual/collision geoms in the mounted assets use the same link meshes. RH56 vendor visual geoms are disabled for contact in the validated runtime (`contype=0`, `conaffinity=0`), while the 148 CoACD hulls are the active RH56 collision geometry. CoACD hull geoms inherit body frames and use identity geom pose. There are no per-mesh scale attributes in the inspected canonical MJCF. Visual, collision and kinematic origins must remain distinct, particularly for `rh56_R_hand_base_link`, whose visual mesh has a non-identity geom pose.

## Existing camera and scene assets

- `configs/camera/realsense_thor.yaml` identifies side and top RealSense D435 units and optical frame names. Intrinsics are acquired at runtime; no calibrated `T_B_C_ext` is stored.
- `configs/workspace/tennis_ball_lift_current.yaml` contains a rough, photo/RealSense-derived table and camera guess. Its status is explicitly `rough_unregistered_from_photo_and_realsense_view`; values must not be promoted to measured twin geometry.
- Debug MuJoCo tools generate rendering cameras and provisional table geometry. These are visualization/task fixtures, not calibrated external-camera definitions.
- Existing point-cloud code in `src/vision_interface/depth_processing.py` handles deprojection, transforms, cropping, voxel downsampling, plane/table processing and outlier removal. It correctly refuses to relabel camera-frame clouds without an explicit transform.
- No fixed external-camera extrinsic, wrist-camera mount, COLMAP result, reconstruction point cloud, dense mesh, Gaussian Splat or NeRF asset exists.

## Missing components

- Reliable video metadata backend (`ffprobe`/`ffmpeg` are absent on the audited host)
- Frame selection and capture-quality audit
- COLMAP/pycolmap and reconstruction wrappers
- Metric-scale references and scale fit
- Measured correspondences and `T_B_R`
- Physical realization of B and verified flange datum
- Verified adapter CAD or measured `T_F_H`
- Calibrated external/wrist camera intrinsics and extrinsics
- Clean separation of rendering reconstruction and primitive collision scene
- Alignment validation reports and reconstruction/robot duplicate masking
- Parameterized digital-twin workspace scene

## Likely integration risks

1. Treating `Link0` mesh center, mounting plate center or robot pedestal bottom as B without a real datum.
2. Treating the `jaka_Link_6` origin, controller tool frame and mechanical flange datum as interchangeable.
3. Assuming the repository mount transform has been physically verified.
4. Promoting rough workspace values or debug cameras to calibration results.
5. Losing the RH56 visual-mesh local transform when exporting or registering assets.
6. Mixing MuJoCo wxyz quaternions with the new xyzw configuration convention.
7. Duplicating the scanned static robot and articulated simulated robot in the rendering layer.
8. Using dense reconstruction directly for collision.
9. Monocular scale drift and weak registration geometry from coplanar landmarks.
10. iPhone HEVC decoding/rotation/color metadata differences across backends.
11. Existing dirty-worktree camera/vision changes overlapping future integration work.

## Recommended implementation plan

1. Audit and conservatively sample the iPhone video; redact sensitive metadata from derived media.
2. Use COLMAP/pycolmap for camera poses and sparse reconstruction in an isolated environment.
3. Recover scale from a calibrated tag/board or multiple independently measured references.
4. Register R to B with weighted Umeyama alignment and optional RANSAC using at least three non-collinear physical landmarks.
5. Retain sparse/dense/GS/NeRF output as visual evidence only; mask or remove the scanned robot from the final visual layer.
6. Fit the tabletop and fixed obstacles as measured primitives in B for MuJoCo collision.
7. Load the existing mounted runtime robot unchanged in a separate generated/wrapper scene.
8. Validate scale, residuals, axes, gravity, penetrations, camera conventions and duplicate geometry before any Real2Sim claim.

## Files not to modify without further validation

- `data/sim_assets/jaka_rh56_visual_coacd.xml`
- `data/sim_assets/jaka_rh56_visual_coacd.manifest.json`
- `data/sim_assets/meshes/rh56_collision_visual_coacd/`
- `data/sim_assets/jaka_rh56.xml`
- `data/sim_assets/meshes/jaka_minicobo_meshes/`
- `data/sim_assets/meshes/rh56/`
- `data/sim_assets/correll_rh56dfx/`
- JAKA/RH56 driver, teleoperation and real-motion paths protected by `Agents.md`

Any future change to these files requires a concrete file-reference, transform, scale, kinematic or collision defect plus focused regression validation.
