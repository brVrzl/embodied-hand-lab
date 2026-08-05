# Dataset collection entry

This is the maintained entry point for collecting canonical episodes. Read the
[current status](../status/current_status.md), [real-hardware safety](../safety/REAL_HARDWARE_SAFETY.md),
and [combined teleoperation procedure](../operation/jaka_rh56_combined_teleop.md)
before opening any device. Documentation, `--help`, and offline validation do
not authorize a physical run.

## Prepare the dual-camera configuration

Create the ignored local copy and fill both explicit D435 serial numbers:

```bash
mkdir -p data/local
cp configs/data_collection/dual_d435_episode.example.yaml \
  data/local/dual_d435_episode.yaml
```

The `workspace` and `wrist` serials must be different and must match their
physical viewpoints. Keep the example's raw-first capture settings unless a
measured acquisition experiment justifies a documented change. The complete
schema, staleness semantics, storage estimate, and camera preflight are in the
[collection and quality guide](COLLECTION_GUIDE.md).

The host runtime config is also kept outside Git:

```bash
cp configs/data_collection/physical_collection.example.yaml \
  data/local/physical_collection.yaml
```

Configure host/device identity once in that YAML. Do not export robot, RH56,
Quest, episode, or native-CPU values from `.bashrc`; the collection command
reads them from `data/local/physical_collection.yaml`.

## Maintained combined collection entry

Use the production wrapper below. Host/device values and the verified native
control CPU are stored once in the ignored local runtime config; the command
does not require per-run substitution:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config data/local/physical_collection.yaml \
  --hand-prerequisites-complete \
  --no-auto-retry \
  --estop-accessible \
  --workspace-clear
```

The runtime config sets the bounded 300-second run, operator `01`, camera
episode config, robot/RH56 identity, native CPU, all-J1--J6 1.5 rad/s limits,
and no preview. Add `--episode-preview` only when preview is intentionally needed;
preview is not a required consumer. Normal J1--J6 run velocity is 1.5 rad/s.
This is a project-selected operating value, not a manufacturer maximum, and
all shared IK, collision, singularity, branch-continuity, joint-limit,
acceleration, timing, liveness, native-worker, and controller safety gates
remain active.

Before starting, verify the JAKA controller state, payload/TCP/install state,
E-stop access, clear workspace, Quest boundary, RH56 prerequisites, camera
identity, free storage, and that no other control client is running. The
native-control CPU must be reserved for the native worker; camera, Python
control, RH56, recorder, and preview processes must not use it.

## What is written

Each completed run creates an episode under `data/episodes` with 30 Hz
canonical rows, robot/hand state metadata, and the two camera streams. Camera
frames are produced in independent processes and referenced through the
versioned shared-memory rings; canonical selection, materialization, and
writing remain outside the control tick. References are causal and sequence
checked: an expired reference is marked invalid and is never replaced by a
newer frame or a fabricated timestamp. Queues are bounded and preview cannot
backpressure capture or control.

Recording or camera failure invalidates or stops recording according to the
episode quality policy; it must not be converted into a healthy-robot
emergency stop. JAKA controller, native timing, liveness, collision, tracking,
or RH56 safety faults retain their normal hard-stop path. Do not retry a failed
run automatically; preserve its summary and episode state for diagnosis.

## Validate after collection

Use the episode path reported by the run summary:

```bash
.venv/bin/embodied-lab dataset validate data/episodes/<EPISODE_DIR>
.venv/bin/embodied-lab dataset inspect data/episodes/<EPISODE_DIR>
```

Only episodes that finalize successfully and pass the current validator should
be considered for training. For schema, manifests, statistics, and export, see
the [canonical dataset schema](DATASET_SCHEMA.md) and the repository root's
dataset commands.
