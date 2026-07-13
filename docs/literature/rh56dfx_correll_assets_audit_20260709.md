# RH56DFX Correll Assets Audit

Date: 2026-07-09

Source repositories checked:

- `correlllab/rh56_controller`, branch `h12`, commit `3db8301`
- `correlllab/h1_mujoco`, commit `331b81c`
- Project page: https://correlllab.github.io/rh56dfx.html
- Paper: https://arxiv.org/abs/2603.08988

## Summary

The useful pieces are not a drop-in replacement for this project. This project already has a JAKA-mounted RH56 MJCF at `data/sim_assets/jaka_rh56.xml` with local JAKA and RH56 meshes, analytic collision proxies, and ManiSkill integration. The Correll assets are better treated as a reference hand model and planner stack:

- Use their `archive/inspire/inspire_grasp_scene.xml` as the source for a floating-hand FK/planning model.
- Use their `archive/inspire/inspire_force_scene.xml` or `archive/inspire/inspire_scene.xml` as references for fingertip sites and MuJoCo force/torque sensors.
- Port selected logic from `rh56_controller/grasp_geometry.py` for width-to-grasp planning.
- Do not directly replace `data/sim_assets/jaka_rh56.xml`; joint names, frame conventions, hand topology, and coupling coefficients differ.

## Simulation Assets Worth Importing

High value:

- `archive/inspire/inspire_grasp_scene.xml`
  - Floating 6-DOF RH56 hand with fingertip sites.
  - Good FK source for line, plane, and cylinder grasp planning.
  - Uses the paper planner's control ranges and coupling assumptions.

- `archive/inspire/inspire_force_scene.xml`
  - Same floating hand plus a configurable object body.
  - Includes fingertip force and torque sensors:
    - `thumb_tip_force`, `index_tip_force`, `middle_tip_force`, `ring_tip_force`, `pinky_tip_force`
    - matching torque sensors.
  - Useful for a MuJoCo-only grasp quality or force-closure viewer.

- `archive/inspire/inspire_scene.xml`
  - Fixed-base hand plus object and fingertip force/torque sensors.
  - Useful as a simpler real2sim force analysis scene.

- `archive/inspire/assets/visual/*` and `archive/inspire/assets/collision/*`
  - The assets are split into visual and collision meshes.
  - This is cleaner than this project's current single vendor STL set for planner-only hand simulation.

Medium value:

- `archive/inspire/ur5_inspire.xml`
  - Useful only as a reference for arm+hand composition and wrist FT site placement.
  - Not directly useful for the JAKA MiniCobo stack.

- `rh56_controller/mujoco_bridge.py`
  - Useful reference for contact extraction, force/torque sensor reading, wrench cones, and Ferrari-Canny-style quality.
  - Should be adapted into project naming conventions rather than imported whole.

- `rh56_controller/grasp_geometry.py`
  - Most valuable code asset.
  - It builds FK tables from MuJoCo, fits interpolators, and solves antipodal grasp widths for line, plane, and cylinder primitives.

Low value / avoid for now:

- H1-2, ROS2, UR5, Magpie, Tkinter UI, and hardware bridge code.
  - These add dependencies and naming assumptions that do not match this project.

## Key Differences From Current Project Assets

Current project:

- Main model: `data/sim_assets/jaka_rh56.xml`
- Mounted on JAKA MiniCobo.
- Uses names like `rh56_R_index_MCP_joint`, `rh56_R_index_DIP_joint`.
- Has 18 qpos, 12 actuators, 0 sites, 0 sensors.
- Uses local analytic capsule/box contact proxies via `src/sim_maniskill/rh56_collision.py`.
- Equality coupling is simple:
  - non-thumb distal = 1.0 * MCP
  - thumb PIP = 0.6 * thumb bend
  - thumb DIP = 0.8 * thumb bend

Correll model:

- Standalone or floating Inspire RH hand.
- Uses names like `index_proximal_joint`, `index_intermediate_joint`, `thumb_proximal_pitch_joint`.
- `inspire_grasp_scene.xml`: 18 qpos, 12 actuators, 6 sites, 0 sensors.
- `inspire_force_scene.xml`: 18 qpos, 12 actuators, 5 sites, 10 sensors.
- Coupling is paper/planner-specific:
  - pinky/ring/middle intermediate = `-0.15 + 1.1169 * proximal`
  - index intermediate = `-0.05 + 1.1169 * proximal`
  - thumb intermediate = `0.15 + 1.33 * pitch`
  - thumb distal = `0.15 + 0.66 * pitch`

## Recommended Integration Order

1. Add Correll assets under a clearly namespaced path, for example:
   - `third_party/correll_rh56dfx/h1_mujoco/archive/inspire/...`
   - include the MIT license from `h1_mujoco`.

2. Add a small loader test that compiles:
   - `inspire_grasp_scene.xml`
   - `inspire_force_scene.xml`
   - the existing `data/sim_assets/jaka_rh56.xml`

3. Port a minimal planner module rather than the UI:
   - start from `rh56_controller/grasp_geometry.py`
   - expose an API that returns this project's canonical order:
     `[index, middle, ring, pinky, thumb_close, thumb_lateral]`

4. Add a mapping layer:
   - Correll actuator order: `[pinky, ring, middle, index, thumb_proximal, thumb_yaw]`
   - Project canonical order: `[index, middle, ring, pinky, thumb_close, thumb_lateral]`
   - Project protocol order: `[pinky, ring, middle, index, thumb_close, thumb_lateral]`

5. Only after planner tests pass, consider adding fingertip sites/sensors to the existing JAKA-mounted model.
   - Directly replacing the JAKA-mounted hand model is risky because the mount transform and naming are already used by ManiSkill and tests.

## Practical Takeaway

The best immediate use is to import the floating-hand planning and force-analysis assets as a reference simulation package, not as the active JAKA+RH56 model. The active model should keep its current names and mount structure; borrow fingertip sites, force sensor definitions, and the planner's coupling-aware FK logic through an adapter.

## Integration Completed

Implemented in this project:

- Imported the minimal validated asset set under `data/sim_assets/correll_rh56dfx/`.
- Added `src/pregrasp/correll_rh56dfx.py`:
  - asset path resolution
  - XML compile/interface validation
  - Correll actuator order to project canonical command mapping
  - FK-backed 2-finger line grasp width planner using `inspire_grasp_scene.xml`
- Updated `GeometryAwarePregraspPredictor` to include a `correll_line_width` candidate when the object width is in the model's useful line-grasp range.
- Added regression tests in `tests/test_correll_rh56dfx_assets.py`.

Validation command:

```bash
.venv/bin/python -m pytest tests/test_correll_rh56dfx_assets.py tests/test_pregrasp_prediction.py tests/test_configs.py
```

Current result: 11 passed.
