# Configuration

This page is the authoritative configuration guide for maintained code.
Configuration does not authorize hardware. Physical approval phrases and
operator acknowledgements are command-line gate inputs, never reusable YAML
defaults.

## Loading and precedence

There is no repository-wide implicit configuration merge and no `.env`
loader. Each entry point owns a documented YAML schema and command-line
surface. For a field supported by that entry point, the intended precedence is:

```text
code schema/default
  < value in the explicitly selected YAML
  < explicit command-line value
```

Environment variables participate only where the code or launcher names them
explicitly. They do not override arbitrary YAML keys.

Examples:

- `embodied-lab sim smoke --config PATH --duration-sec SEC` selects a replay
  YAML and explicitly overrides smoke duration.
- the physical Quest/JAKA wrappers require robot IP, duration, output bounds,
  approvals, and safety acknowledgements on the command line;
- `quest_rh56_hand_test.py --device PATH` owns the actual serial device even
  though the selected hand YAML describes protocol policy;
- `embodied-lab benchmark CONFIG --seed N --output PATH` selects a versioned
  benchmark config and optionally overrides its seed.

Do not assume an older standalone tool implements this hierarchy. Its current
`--help` and loader are authoritative.

## YAML validation

The shared loader requires the YAML root to be a mapping. Subsystem loaders add
their own required fields, units, paths, enumerations, and bounds; some reject
unknown keys while others do not. A syntactically valid YAML file is therefore
not necessarily a valid runtime configuration.

The read-only doctor parses every `configs/**/*.yaml`:

```bash
.venv/bin/embodied-lab doctor --json
```

Exercise the owning loader for semantic validation:

```bash
.venv/bin/embodied-lab sim smoke \
  --config configs/sim/quest_hts_jaka_mini2_offline.yaml

.venv/bin/embodied-lab benchmark \
  configs/benchmark/smoke.yaml \
  --output artifacts/benchmark/config-check.json

.venv/bin/python -m pytest -q tests/test_configs.py
```

The simulation and benchmark commands are offline. Hardware config validation
must use the owning tool's no-connect/preflight mode where one exists; a YAML
parse is not permission to open a device.

## Maintained configuration inventory

### Simulation, motion input, and benchmark

| File | Owner and status |
| --- | --- |
| `configs/sim/quest_hts_jaka_mini2_offline.yaml` | Default headless/offline replay and unified simulation smoke |
| `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | Shared live Quest target-generation, MuJoCo, and native-adapter policy |
| `configs/sim/quest_rh56_retarget.yaml` | Simulation-only Quest-to-RH56 feature calibration |
| `configs/motion_input/quest_hts_right_hand.yaml` | Canonical HTS receiver/operator preparation settings |
| `configs/benchmark/smoke.yaml` | Deterministic offline joint-reach/pre-shape smoke benchmark |
| `configs/sim/jaka_collision_sweep_poses.yaml` | Offline digital-twin collision-sweep pose samples |

The live Quest YAML is shared policy before the output adapter. It contains
input freshness, clutch semantics, frames, provisional calibration, filters,
continuation, IK, singularity checks, output feasibility, MuJoCo settings, and
the thin native adapter contract. Its input-recovery window is capped at
10 seconds and does not alter the native 100 ms producer watchdog. It is not controller state and must not be
used to write payload, TCP, installation, or safety settings.

The offline and live Quest configurations are deliberately different.
The offline file uses a small, uncalibrated simulation-only displacement
envelope and orientation disabled. The live file remains provisional where
marked and is not a claim of full physical calibration.

### RH56

| File | Owner and status |
| --- | --- |
| `configs/hand/rh56_pc_direct_teleop.yaml` | Maintained PC-direct protocol, scheduler, feedback, bounds, channel order, and safety policy |
| `configs/hand/quest_rh56_real_retarget.yaml` | Maintained live Quest feature calibration for hand-only and combined physical RH56, and the live simulation default; does not own protocol travel |

The canonical six-channel order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

The protocol order is:

```text
[pinky, ring, middle, index, thumb_close, thumb_lateral]
```

Do not interpret `ANGLE_ACT` as all coupled passive-joint angles.
`ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are device register
feedback fields. They are not a tactile array, slip sensor, or complete
kinematic state. Nonzero `STATUS` semantics have not been validated and must
remain raw rather than guessed.

`rh56_pc_direct_teleop.yaml` contains a placeholder serial path. The actual
stable device is always selected explicitly with `--device`. Speed/force
values in YAML are software command policy, not permission to write runtime
registers. Runtime configuration writes have a separate approval gate.

### Camera, perception, and collection preparation

| File | Owner and status |
| --- | --- |
| `configs/camera/default_rgbd.yaml` | Small mock RGB-D fixture; not a physical-camera default |
| `configs/camera/realsense_thor.yaml` | Site-specific dual-D435 snapshot with recorded serials; not portable |
| `configs/perception/d435_tabletop.yaml` | Offline tabletop depth/point-cloud processing and uncalibrated target-frame placeholder |
| `configs/data_collection/dual_d435_episode.example.yaml` | Copyable settings consumed by the simulation-backed Quest + dual-D435 episode path; no physical JAKA/RH56 collector consumes it |

Copy site values into the same ignored local path used by the collection
guide rather than editing a versioned example:

```bash
mkdir -p data/local
cp configs/data_collection/dual_d435_episode.example.yaml \
  data/local/dual_d435_episode.yaml
```

Replace both camera serial placeholders and attach calibration snapshot
identity before collection. Camera roles are assigned by serial, never
`/dev/video*` order. The current consumer records real Quest and D435 input
against simulated arm/hand state; a copied config does not make physical
JAKA/RH56 collection implemented or validate the cameras, calibration, or
synchronization on a target host.

The tabletop perception config uses meters. Its
`target_from_camera_npy: null` and `calibration_status: uncalibrated` values
are intentional blockers; do not substitute an identity transform.

### Distributed training

`configs/training/distributed.example.yaml` is a proposed future trainer
contract. It explicitly declares:

```yaml
status:
  consumed_by_current_trainer: false
```

It documents DDP topology, global batch, precision, DataLoader, checkpoint,
logging, and profiling choices. No current ACT, Diffusion Policy, or other
trainer reads it. See
[Distributed training](../training/DISTRIBUTED_TRAINING.md).

### Historical and digital-twin policy

`docs/history/gates/jaka_foundation_20260716/jaka_foundation.yaml` records a
dated foundation-gate policy beside its evidence. It is not the current
production arm configuration.

`digital_twin/configs/` contains calibration evidence, provisional scene
geometry, transform status, collision classification, and example
correspondence files. These are consumed by specific
`tools/digital_twin/*.py` commands, not by a global loader. Many values are
explicitly provisional or unresolved. Follow the
[digital-twin guide](../digital_twin/README.md); do not promote preview camera
poses, sparse debug geometry, or null transforms into physical calibration.

## Units and naming

Use field suffixes as part of the schema:

- `_m`, `_m_s`: meters and meters per second;
- `_rad`, `_rad_s`, `_rad_s2`, `_rad_s3`: radians and derivatives;
- `_deg`: degrees;
- `_sec`, `_s`, `_ms`, `_ns`: explicit time units;
- `_hz`: cycles per second;
- `_bytes`: byte counts;
- `_xyzw`: quaternion order;
- RH56 raw command/register values: dimensionless device counts, normally
  0--1000 where the owning schema says so.

Do not mix a configured MuJoCo actuator radian with RH56 raw counts.
Do not treat `CURRENT` or `FORCE_ACT` as SI torque/force unless a separately
validated calibration supplies that conversion.

Frame names follow the owning architecture document. In transform notation,
`T_A_B` maps coordinates expressed in frame B into frame A. A provisional
operator-to-robot basis, MuJoCo scene placement, camera preview pose, or
COLMAP registration is not automatically a calibrated robot/camera extrinsic.
See [coordinate frames](../architecture/coordinate_frames.md).

## Paths

The installed source tree is found from the `embodiment_core` package.
`EMBODIED_LAB_ROOT` may explicitly select another repository root. Relative
paths in maintained configs are interpreted relative to that repository or by
the owning tool as documented; run operator wrappers from any directory only
when the wrapper itself resolves the root.

Keep these roles separate:

- `data/sim_assets/`: versioned MuJoCo assets;
- `data/local/`: ignored site-specific collection configuration;
- `data/episodes/`: canonical episode data, ignored by default;
- `models/`: model and digital-twin assets;
- `logs/`: runtime evidence, ignored;
- `artifacts/`: generated reports, exports, local configs, checkpoints, and
  temporary experiment outputs, ignored;
- `configs/`: reviewed versioned policy/examples.

Do not place the only copy of calibration, raw data, or a checkpoint in a
temporary directory. Output commands should use unique run/episode paths and
must not overwrite raw evidence.

## Explicit environment variables

Only the following current variables have defined effects:

| Variable | Effect |
| --- | --- |
| `EMBODIED_LAB_ROOT` | Explicit repository-root override used by unified tooling |
| `EMBODIED_LAB_SOURCE_REVISION` | Source-bundle revision/provenance string for episode metadata when project Git metadata is absent |
| `DISPLAY`, `XAUTHORITY` | X11 viewer selection used by the simulation wrapper |
| `MUJOCO_GL` | MuJoCo rendering backend selected by MuJoCo, such as a tested `egl` or `osmesa` setup |
| `PYTHON_BIN` | Interpreter override supported by selected shell wrappers |
| `CUDA_VISIBLE_DEVICES` | GPU visibility/order for PyTorch and diagnostics |
| `LOCAL_RANK`, `RANK`, `WORLD_SIZE` | Complete `torchrun` identity triplet; partial values are rejected |
| `MASTER_ADDR`, `MASTER_PORT` | Distributed rendezvous; must be supplied together for multi-process launch |
| `NCCL_SOCKET_IFNAME`, `NCCL_IB_DISABLE`, `NCCL_P2P_DISABLE`, `TORCH_DISTRIBUTED_DEBUG` | Explicit distributed runtime/debug settings; not general defaults |
| `TRAINING_ACTIVATE`, `GPUS_PER_NODE`, `RUN_ID`, `OUTPUT_ROOT` | Parameters used only by copied Slurm example templates |

The doctor reports safe environment values and only the presence—not the
contents—of known credential variables. Do not store credentials in YAML,
shell history, logs, or examples.

The distributed rank triplet is all-or-nothing. In a normal single-process
shell, leave all three unset. Do not export NCCL debug/tuning variables
permanently; enable them for a diagnosed run only.

## Device-specific values

### JAKA

The robot IP is an explicit command-line gate value. The production shared
YAML owns software limits and frame/transport expectations, not live
controller truth. Before a physical gate the operator must verify payload,
center of mass, installation, active TCP/user frame, collision state, and
safety limits at the controller. Software must not silently apply recorded
values.

### RH56

Select a stable device path explicitly. The YAML owns baud rate, address,
scheduler profile, canonical/protocol mapping, stale thresholds, and command
bounds. An open transport performs no automatic configuration write or
safe-open.

### RealSense

Record camera role, serial, firmware, USB mode, stream profile, depth scale,
alignment policy, timestamp domains, and calibration snapshot. Serial and
calibration identity are data provenance, not optional convenience fields.

### Quest

Bind address, UDP port, project IP, and optional allowed-sender are command-line
network values. Freshness, required hand/head/controller state, clutch
hysteresis, frame mapping, and filters belong to the reviewed YAML/code
contract. Source timestamps and host receipt timestamps have different epochs.

## Configuration change review

Treat changes to any of the following as control or data-contract changes, not
cosmetic tuning:

- coordinate frames, axes, quaternion order, units, or calibration identity;
- joint, workspace, velocity, acceleration, jerk, singularity, collision, or
  stale thresholds;
- clutch and release-before-press behavior;
- RH56 channel order, register semantics, closure/delta/rate/feedback limits;
- camera role, serial, alignment, depth unit, timestamp skew, or extrinsic;
- observation/action schema, camera order, normalization, temporal horizon, or
  action chunk;
- distributed world/global batch, precision, learning rate, split manifest, or
  checkpoint compatibility.

Update the owning behavior test and documentation, parse all YAML, run the
offline smoke/replay that consumes it, and preserve the old configuration hash
with any historical result. Never weaken a robot-control safety boundary to
make a test pass.

For environment installation see
[Installation](../setup/INSTALLATION.md). For failure diagnosis see
[Troubleshooting](../TROUBLESHOOTING.md).
