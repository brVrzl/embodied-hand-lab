# Integrated MuJoCo digital twin

## Status and scope

The digital-twin subsystem is at **Integrated Workspace** maturity. It combines
the JAKA Mini2 and Inspire RH56 model with a parameterized table, aluminium
frame, floor, lighting, debug axes, camera placeholders, and an empty future
object layer. It is an offline engineering scene, not a calibrated replica of
the physical cell and not a robot-safety authority.

This subsystem does not connect to hardware. It also does not currently provide
grasp planning, policy training, final robot/world registration, or calibrated
external and wrist-camera extrinsics.

## Model authority and contact state

There are two related MuJoCo assets with different roles and current evidence:

| Asset | Role | Initial contact evidence | Current boundary |
|---|---|---|---|
| `data/sim_assets/jaka_rh56_visual_coacd.xml` | Canonical shared robot runtime used by simulation, teleoperation, H0, and benchmark code | Direct load/forward at model `qpos0`: zero contacts. The configured H0 arm start also has zero penetrating contacts. | Current shared robot authority |
| `models/digital_twin/workspace_scene.xml` | Committed derivative that adds the P-frame workspace | Direct load/forward at model `qpos0`: four duplicate contact records for `jaka_Link_0_geom_0` against `jaka_Link_1_geom_0`, each at about -3 mm; no environment contact at that state | Provisional and currently stale relative to the canonical runtime |
| `models/digital_twin/jaka_inspire_workspace.xml` | Compatibility include for `workspace_scene.xml` | Same result as the included scene | Not a separate scene authority |

The current scene generator, when run from the canonical shared runtime asset,
retains its adjacent-link exclusion and produces an integrated scene with zero
initial contacts. That generated result differs from the committed
`workspace_scene.xml`. Therefore, the committed integrated scene must be
regenerated and revalidated before a new collision sweep is treated as current
evidence. Do not use the committed scene's initial Link 0/Link 1 contacts to
describe the shared runtime model.

The shared pre-acceptance target pipeline does not include table or frame
collision from this workspace derivative. Digital-twin collision results are
simulation characterization only; they do not extend the physical control
safety envelope.

## Robot and hand model boundary

The shared model has six JAKA position actuators and six RH56 position
actuators. The RH56 contains 12 modeled joints and six equality couplings:

- thumb PIP and DIP follow the directly actuated thumb-close joint through
  cubic polynomials fitted to the local command/angle table;
- index, middle, ring, and pinky distal joints follow their corresponding
  directly actuated MCP joints.

This is a six-input approximation of an underactuated hand. The equalities
encode deterministic kinematic coupling only. They do not model tendon
compliance, backlash, load-dependent coupling, current or force control,
calibrated force limits, contact sensing, or all passive physical joint
behavior. A finite, collision-free MuJoCo command is not proof of equivalent
physical motion.

## Frames, placement, and calibration status

The scene world is frame `P`:

- origin: center of the fixed 110 mm mounting PCD on the lowest fixed mounting
  plane;
- `+z`: upward;
- `+x`: along the two longitudinal rails toward the front transverse member /
  operator side.

The internal robot base remains frame `B` at `jaka_Link_0`. The scene applies:

```text
T_P_B_operational
translation: [0, 0, 0] m
yaw: 180 deg
quaternion xyzw: [0, 0, 1, 0]
status: physically_constrained_provisional
```

This placement is not a calibrated `T_B_P`. The calibrated transform remains
unresolved in `digital_twin/configs/transforms.yaml`. Table and rail geometry
are provisional, and the external and wrist-camera extrinsics are uncalibrated.
The camera objects in the scene are placeholders, not sensor calibration
results.

The operational placement and transform distinction are defined in:

- `digital_twin/configs/robot_operational_placement.yaml`
- `digital_twin/configs/transforms.yaml`
- `digital_twin/configs/static_environment.yaml`
- `digital_twin/configs/camera_placeholders.yaml`

## Visual-layer policy

The normal engineering view includes the robot, table, aluminium frame, floor,
lighting, axes, and optional camera placeholders. Reconstruction points,
cables, boards, clutter, and an unqualified permanent background are not part
of the default scene.

The builder can optionally include a compact sparse-reconstruction debug mesh.
That layer must remain visual-only with collision disabled; sparse points are
not a physical surface. The reconstruction, registration, segmentation
manifest, and derived visual artifacts used by that workflow are not present
in this source bundle, so the sparse-debug and full integrated validator paths
cannot currently be reproduced from repository files alone.

## Offline checks and regeneration

All commands below are offline and run from the repository root. First verify
that the canonical runtime asset matches its maintained source:

```bash
.venv/bin/python tools/build_rh56_visual_coacd_runtime_asset.py --check
```

Validate the current workspace inputs without writing a scene:

```bash
.venv/bin/python tools/digital_twin/build_mujoco_workspace_scene.py \
  --robot-model data/sim_assets/jaka_rh56_visual_coacd.xml \
  --static-config digital_twin/configs/static_environment.yaml \
  --camera-config digital_twin/configs/camera_placeholders.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --output artifacts/digital_twin/rebuild/workspace_scene.xml \
  --manifest artifacts/digital_twin/rebuild/scene_manifest.yaml \
  --dry-run
```

To inspect a regenerated scene without overwriting the committed model, omit
`--dry-run` and keep the shown artifact output paths. Then load, inspect, and
compare that artifact before replacing any maintained model.

The committed scene can be checked by the renderer and collision-sweep
frontends without executing their expensive stages:

```bash
.venv/bin/python tools/digital_twin/render_workspace_scene.py \
  --scene models/digital_twin/workspace_scene.xml \
  --output-dir artifacts/digital_twin/render \
  --dry-run

.venv/bin/python tools/digital_twin/run_joint_space_collision_sweep.py \
  --scene models/digital_twin/workspace_scene.xml \
  --classification digital_twin/configs/collision_classification.yaml \
  --operational-config digital_twin/configs/robot_operational_placement.yaml \
  --robot-config configs/sim/jaka_collision_sweep_poses.yaml \
  --output artifacts/digital_twin/collision_sweep \
  --dry-run
```

After regenerating the scene and reviewing initial contacts, a short offline
sweep can be run with `--quick --skip-render`; the full sweep should be retained
for a deliberate validation run. The integrated validator additionally needs
the absent reconstruction-derived segmentation manifest, scene manifest, and
visual mesh, so it is blocked until those inputs are restored or regenerated.

## Prior sweep evidence

An earlier repository report described 130 static configurations and nine
actuator trajectories (31,995 MuJoCo steps). It reported no zero-pose
environment or floor contact, but reported policy failures for a low-table
approach and both RH56 open/close directions, plus a shallow Link 5/table
warning. It also reported deep contacts and large simulated constraint forces
at intentionally infeasible sampled states.

The raw sweep summary, sampled states, event renders, and validation artifacts
are absent from this source bundle, and the committed scene has since drifted
from the canonical runtime source. Those numbers are historical claims, not
reproduced current validation and not physical force predictions.

## Readiness blockers

The integrated workspace is not **Simulation Ready**. Before promoting it:

1. regenerate the workspace scene from the current canonical runtime asset;
2. confirm zero-pose and configured-start contact state on the regenerated
   scene;
3. review table/rail primitives and rerun the deterministic collision sweep;
4. preserve the scene, manifest, configuration snapshot, and sweep outputs
   together;
5. restore or repeat robot/world, external-camera, and wrist-camera
   calibration;
6. classify every retained collision finding as geometry error, conservative
   proxy, or excluded infeasible pose.

These steps remain offline validation work. A digital-twin result must never be
reported as physical validation or safety certification.
