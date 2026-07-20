# Digital-twin handoff — offline collision characterization

Date: 2026-07-15. Current maturity: **Integrated Workspace**.

## Corrected integration

The preview defect was a robot-root yaw error, not a joint-zero or hand-mount error. With identity B placement, RH56 local +y (the palm normal) mapped through the unchanged repository `T_F_H` to P +x. User evidence requires both the all-zero palm and fixed cable outlet to face P -x. Candidate errors were:

| Operational yaw | Palm error | Cable-side error | Result |
|---:|---:|---:|---|
| 0° | 180° | 180° | reject |
| +90° | 90° | 90° | reject |
| -90° | 90° | 90° | reject |
| 180° | 0° | 0° | selected |

`T_P_B_operational` is translation `[0,0,0]` m and xyzw quaternion `[0,0,1,0]`. Provenance is the all-zero reference pose, palm direction, fixed cable-side direction, `annotated_P_frame.jpg`, and repository zero-pose kinematics. It is physically constrained but not metrology-calibrated.

Calibrated `T_B_P` remains null and unresolved. It was not overwritten.

## Preserved robot state

- all six JAKA qpos values: zero;
- all twelve RH56 qpos0 values: zero/default;
- `T_F_H`: unchanged at `[0,0,0.009]` m and xyzw `[0.707106781,0.707106781,0,0]`;
- JAKA/RH56 meshes, collisions, hulls, actuators and joint definitions: unchanged;
- root centre and mounting height: unchanged at P origin/z=0.

The palm direction error after placement is 0.000211°. The cable direction numerical error is 0°; its validation remains PROVISIONAL because the connector itself is absent from the mesh, although `annotated_P_frame.jpg` independently confirms the physical P -x side.

## Clean visual policy

The default `workspace_scene.xml` contains no sparse reconstruction geom. It shows only the articulated robot, parameterized table/frame, floor, lights and engineering sites. The presentation render hides all sites.

Sparse reconstruction survives as:

- original COLMAP/audit data, unchanged;
- seven semantic PLY review segments;
- `sparse_debug.ply/obj/glb` after crop, minimum track length, statistical filtering, radius filtering, small-component removal and downsampling;
- optional `workspace_scene_sparse_debug.xml`, whose `colmap_sparse_debug` geom has collision disabled.

No sparse triangulation was presented as a solid background. A dense textured mesh or Gaussian Splat remains a future rendering layer.

## Key outputs

- `models/digital_twin/workspace_scene.xml` — clean default;
- `models/digital_twin/workspace_scene_sparse_debug.xml` — optional debug;
- `artifacts/digital_twin/static_scene/workspace_clean_engineering.png`;
- `artifacts/digital_twin/static_scene/workspace_clean_presentation.png`;
- `artifacts/digital_twin/static_scene/orientation_before.png` and `orientation_after.png`;
- `artifacts/digital_twin/static_scene/zero_pose_top_verified.png`;
- `artifacts/digital_twin/static_scene/sparse_debug_optional.png`;
- `artifacts/digital_twin/validation_report.{json,md}`;
- `digital_twin/configs/robot_operational_placement.yaml`.

## Validation status

Before collision characterization, orientation/layer validation was **PASS_WITH_PROVISIONAL_ITEMS**: 18 PASS, 4 PROVISIONAL, 1 WARN, 0 FAIL, 0 MISSING.

That result remains the clean-scene baseline. After adding collision characterization, the combined validation is **FAIL for Simulation Ready gating** (19 PASS, 4 PROVISIONAL, 1 WARN, 1 FAIL); the FAIL is the completed sweep’s acceptance result, not a scene compilation failure. The full repository suite now passes **212 tests in 8.74 s** with EGL rendering enabled.

- palm and cable directions satisfy P -x;
- calibrated `T_B_P` remains unresolved;
- default sparse layer is absent;
- optional debug geometry is non-colliding;
- camera placeholders are sites, not collision geoms;
- no new zero-pose table/rail/floor contacts;
- the only warning is the canonical Link0–Link1 self-contact already present before workspace integration.

## Completed offline collision sweep

`run_joint_space_collision_sweep.py` was executed against the clean `workspace_scene.xml`; it does not import the sparse-debug scene or expose any robot-hardware interface. Before sampling it asserts the unchanged operational root translation, 180° yaw/xyzw `[0,0,1,0]`, all-zero reference qpos and palm direction.

- 130 static configurations: repository presets, one-joint-at-a-time limits, adjacent-pair coverage, 48 fixed Halton samples, FK-selected directional aliases and actuator-valid RH56 states;
- nine position-actuator trajectories, 31,995 total `mj_step` calls;
- aggregate throughput: 4,184.9 steps/s;
- zero environment contacts at zero pose; four Link0–Link1 manifold contacts retained as the canonical baseline;
- zero robot/floor, camera-placeholder or sparse/debug contacts;
- no non-finite states or MuJoCo solver warnings;
- three failing trajectories: low tabletop approach, RH56 open-to-close and RH56 close-to-open;
- first warning trajectory: forward P -x reach (persistent shallow arm/table contact);
- maximum sampled environment penetration: 81.894 mm in a diagnostic adjacent-joint configuration;
- peak simulated environment normal constraint force: 63,369.6 N in an infeasible static diagnostic state;
- symmetric rail collision pattern: WARN pending primitive/spacing review.

The force is read with `mj_contactForce` in the MuJoCo contact frame: normal, two tangential components and three torque components. It is simulated constraint output, not a measured or hardware-safe force.

Outputs are under `artifacts/digital_twin/collision_sweep/`. `contact_timeline.csv` is the complete per-active-contact record; `warning_events.json` and `failing_events.json` are compact per-configuration/trajectory reviews with first and maximum-penetration qpos; 88 top/oblique diagnostic images cover those events.

## Handoff status

Maturity remains **Integrated Workspace**, not Simulation Ready and not Manipulation Ready. `T_B_P` is still a future calibration and was not used as a sweep blocker. The immediate blocker is collision interpretation: inspect the table/rail primitive placement and the non-adjacent RH56 thumb/index contacts before changing any geometry. Do not regenerate collision hulls or modify `T_F_H` based solely on this sweep.

Exact next command for a reproducible rerun:

```bash
MUJOCO_GL=egl .venv/bin/python tools/digital_twin/run_joint_space_collision_sweep.py
```
