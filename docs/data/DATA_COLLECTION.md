# Physical episode collection entry

This is the one maintained operator entry for collecting physical episodes. It
combines live arm/hand teleoperation with recording; there is no separate
normal physical-teleoperation entry. Read the
[current status](../status/current_status.md), [real-hardware safety](../safety/REAL_HARDWARE_SAFETY.md),
and [combined teleoperation procedure](../operation/jaka_rh56_combined_teleop.md)
before opening any device. Documentation, `--help`, and offline validation do
not authorize a physical run.

## Prepare the unified collection configuration

```bash
${EDITOR:-vi} configs/data_collection/physical_collection.yaml
```

The `workspace` and `wrist` serials must be different and must match their
physical viewpoints. Keep the example's raw-first capture settings unless a
measured acquisition experiment justifies a documented change. The complete
schema, staleness semantics, storage estimate, and camera preflight are in the
[collection and quality guide](COLLECTION_GUIDE.md).

Configure host/device identity once in that YAML. Do not export robot, RH56,
camera, episode, or native-CPU values from `.bashrc`; the collection command
reads them from this file.

## Maintained collection entry

Use the production wrapper below. Host/device values and the verified native
control CPU are stored once in the ignored local runtime config; the command
does not require per-run substitution:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --hand-prerequisites-complete \
  --no-auto-retry \
  --estop-accessible \
  --workspace-clear
```

The runtime config sets the bounded 300-second run, operator `01`, camera
episode config, robot/RH56 identity, native CPU, all-J1--J6 1.5 rad/s limits,
log directory, acceleration-transition recovery, and no preview. Set
`runtime.episode_preview: true` in the YAML only when preview is intentionally
needed; preview is not a required consumer. Normal J1--J6 run velocity is 1.5
rad/s.
This is a project-selected operating value, not a manufacturer maximum, and
all shared IK, collision, singularity, branch-continuity, joint-limit,
acceleration, timing, liveness, native-worker, and controller safety gates
remain active.

`ARM_CLUTCH` is not a separate legacy mode. During combined collection,
releasing the left index clutch places only the arm in bounded hold; pressing
again resumes after the existing fresh-reference rules. The hand grip clutch
controls the hand independently. The arm-only bounded wrapper remains only as
an isolation diagnostic and is not a competing collection entry.

Before starting, verify the JAKA controller state, payload/TCP/install state,
E-stop access, clear workspace, Quest boundary, RH56 prerequisites, camera
identity, free storage, and that no other control client is running. The
native-control CPU must be reserved for the native worker; camera, Python
control, RH56, recorder, and preview processes must not use it.

## What is written by the maintained physical entry

The tracked dual-D435 example selects `lerobot_staging_v1`. Collection writes
reviewable staging data only; it does not import PyArrow or create Parquet in
the live recorder:

```text
raw_episodes/
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes/chunk-000/episode_000000.json
  data/chunk-000/episode_000000.jsonl
  videos/observation.images.workspace/chunk-000/episode_000000.mp4
  videos/observation.images.wrist/chunk-000/episode_000000.mp4
  audit/chunk-000/episode_000000/  # optional JAKA/RH56 records
```

`episode_000000.jsonl` is the aligned 30 Hz robot table. Row `i` and decoded
video frame `i` share the same episode sample. Its columns include
`frame_index`, `timestamp_ns`, `observation.state` (12 values), and `action`
(12 values):

- `observation.state[0:6]`: measured JAKA joint position in radians;
- `observation.state[6:12]`: measured RH56 six-channel normalized state;
- `action[0:6]`: accepted arm joint target sent to the adapter;
- `action[6:12]`: accepted RH56 target in normalized units.

TCP, Quest packets/events, and depth are deliberately not part of this
training view. Quest remains a live control input, but the maintained control
flow has no Quest recording sink and the episode writer rejects Quest raw
streams.
TCP can be derived offline from a reviewed model/calibration if later needed.
The RealSense workers run RGB-only (`capture_depth: false`), so no depth stream
or depth-sized payload is read or persisted.

Camera frames are still produced in independent processes and selected through
the bounded shared-memory rings; selection and video/JSONL writing remain
outside the control tick. An expired reference or dropped slot is represented
in the staging quality JSONL/metadata and never replaced by a newer image or
fabricated timestamp. Queues are bounded and preview cannot backpressure
capture or control.

Recording or camera failure invalidates or stops recording according to the
episode quality policy; it must not be converted into a healthy-robot
emergency stop. JAKA controller, native timing, liveness, collision, tracking,
or RH56 safety faults retain their normal hard-stop path. Do not retry a failed
run automatically; preserve its summary and episode state for diagnosis.

When both valid clutches are released continuously for five seconds, the
recorder finalizes that episode and opens the next numbered episode without
ending robot control. A new clutch press starts the next episode from fresh
state. The staging review page and approval/conversion commands are:

```bash
.venv/bin/embodied-lab dataset review-staging data/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging data/raw_episodes episode_000000 \
  --status approved --notes "reviewed RGB and task outcome"
.venv/bin/embodied-lab dataset convert-staging data/raw_episodes episode_000000 \
  data/lerobot_dataset
```

The old canonical v1/v2 archive and `raw_episode_v1` writer remain available
for simulation and offline compatibility. They are not the default physical
collection format.

## Validate after collection

Use the episode index reported by the run summary and review it offline:

```bash
.venv/bin/embodied-lab dataset review-staging \
  /home/thor/projects/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging \
  /home/thor/projects/raw_episodes episode_000000 --status approved
.venv/bin/embodied-lab dataset convert-staging \
  /home/thor/projects/raw_episodes episode_000000 data/lerobot_dataset
```

Only completed, human-approved episodes should be converted. Check that the
JSONL row count equals both MP4 frame counts and that `timestamp_ns` is strictly
increasing before conversion. The old `dataset validate`/`inspect` commands
remain for canonical and `raw_episode_v1` archives.
