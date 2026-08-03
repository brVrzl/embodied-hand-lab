# Episode collection guide

## Current support boundary

The repository has two producers using the same canonical writer and
dual-D435 runtime. The simulation-backed producer is:

```text
real Quest HTS/CTRL input + real workspace/wrist D435 RGB-D
    -> shared accepted-target pipeline
    -> MuJoCo JAKA Mini2 and RH56 state
    -> one trigger-bounded canonical episode
```

This path is intended to validate trigger boundaries, dual-camera identity and
throughput, causal sampling, archive finalization, and offline export. The arm
and hand data are simulated and metadata records `simulation_only=true` and
`physically_validated=false`.

The separately gated `combined-normal-teleop` path can additionally write v2
episodes from the physical JAKA/RH56 data already present in that loop. It does
not alter the accepted-target pipeline or native worker. The explicitly
started bounded run is the episode boundary; collection starts after fresh
camera/JAKA/RH56 state is ready and remains active across every arm/hand clutch
change. Stable clutch-mode transitions receive automatic segment IDs without
creating multiple episode directories. This integration is offline tested and
has not yet been physically validated.

This guide does not authorize a physical robot, hand, headset, or camera run.
Physical use requires its own session and the exact gate required by the
corresponding operator procedure. No physical device was connected while this
guide was prepared.

## Install the collection environment

From the repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev,realsense,vision-teleop]"
```

Install `dataset-export` only on a machine that will create ACT HDF5 or
LeRobot output:

```bash
.venv/bin/python -m pip install -e ".[dataset-export]"
```

Keep the canonical archive independent of exporter availability. In
particular, collection does not require LeRobot.

Confirm the maintained command surfaces without opening hardware:

```bash
.venv/bin/embodied-lab dataset --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/check_realsense_stream.py --help
```

## Prepare a local dual-camera config

Copy the tracked example to the ignored `data/` area:

```bash
mkdir -p data/local
cp configs/data_collection/dual_d435_episode.example.yaml \
  data/local/dual_d435_episode.yaml
```

Do not edit the example with local serial numbers. `data/` is ignored so the
copy can contain machine-specific camera identities and calibration paths
without becoming repository configuration.

The implemented config fields are:

```yaml
dataset:
  root: data/episodes
  fps: 30
  camera_max_age_ms: 70.0
  control_max_age_ms: 40.0
  hand_start_tolerance_rad: 0.05

cameras:
  workspace:
    serial: <explicit D435 serial>
    width: 640
    height: 480
    fps: 30
    allow_profile_fallback: true
    align_depth_to_color: true
    warmup_frames: 5
    timeout_ms: 5000
    max_timestamp_skew_ms: 33.333334
    sync_retry_frames: 30
    filters: ...
  wrist:
    # Same fields, with a different explicit serial.

calibration:
  snapshot_files: []
```

The two serials are mandatory, must not retain the `REPLACE_...` placeholder,
and must differ. Roles are never inferred from `/dev/video*` order.
`--episode-root` overrides `dataset.root`; it is accepted only when
`--episode-data-config` is also present.

At 30 Hz, keep `camera_max_age_ms: 70.0`. Canonical rows causally select the
latest frame at or before the recorder timestamp, so this permits ordinary
host receive jitter and at most roughly one repeated selection. It never
selects a future frame. A longer camera stall still aborts the episode, while
raw frame numbers and device/host timestamps remain available for the summary.

`control_max_age_ms: 40.0` applies only to the recorder's causal snapshot
selection. It covers the target-host producer's measured 30.6 ms scheduling
jitter. It does not alter Quest/JAKA heartbeat freshness, native worker
watchdogs, ServoJ timing, or any physical control safety threshold.

`calibration.snapshot_files` should contain the exact versioned files needed to
interpret this capture. The current writer resolves those paths from the
process working directory, copies them into the episode, and records SHA-256
values. Snapshot basenames must be unique because the episode stores them in
one calibration directory; a missing file or basename collision rejects the
start without silently overwriting a calibration. Prefer repository-relative
paths when the files are versioned and absolute paths for intentionally
external local calibration.

The current config does not define a formal task/object registry, camera
extrinsic acceptance threshold, or physical RH56 calibration. Those remain
planned.

## Camera preflight

On an explicitly authorized camera host, enumerate identities:

```bash
.venv/bin/python tools/check_realsense_stream.py --list-devices
```

Then test each selected serial independently before running two streams:

```bash
.venv/bin/python tools/check_realsense_stream.py \
  --serial <WORKSPACE_SERIAL> --width 640 --height 480 --fps 30 \
  --duration-sec 5

.venv/bin/python tools/check_realsense_stream.py \
  --serial <WRIST_SERIAL> --width 640 --height 480 --fps 30 \
  --duration-sec 5
```

Check:

- the physical viewpoint matches the declared `workspace` or `wrist` role;
- RGB is not BGR-swapped, rotated unexpectedly, frozen, or severely
  underexposed;
- raw depth is present and the resolved profile includes depth scale,
  intrinsics, distortion, extrinsics, firmware, serial, frame rate, and
  resolution;
- the two cameras do not share a serial and do not exceed USB bandwidth;
- mounting and calibration versions match the intended experiment.

The example stores lossless RGB, raw depth, and aligned depth as NPY. At
640×480×30 Hz with two cameras and aligned depth enabled, payload throughput is
approximately 123 MiB/s, or about 7.2 GiB/minute, before filesystem overhead.
Canonical hard links do not double that payload. Measure sustained write rate
and free space on the actual destination; do not rely on the estimate for a
long collection session.

Bottle Pickup combined capture is raw-first and does not write a visual preset
while starting the two camera workers. A target-host trial with full-rate disparity-domain spatial filtering
made the shared producer loop stall for 131 ms (versus 36 ms maximum in the
unfiltered Phase A run), so online spatial processing is not enabled in this
control-coupled process. Temporal filtering and global hole filling also stay
off: temporal persistence can leave motion trails, while hole filling can
invent surfaces across occlusion edges. Native unaligned Z16 remains in
`depth_raw`; `depth_aligned_to_rgb` remains the lossless aligned Z16 view. The
live preview uses the aligned view with a fixed 0.15–1.5 m scale and renders
invalid/out-of-range pixels black instead of stretching every frame by its
percentiles. Use the independent camera checker with `--filter-profile
spatial` when comparing depth quality without the robot control process. Any
intentional preset comparison belongs in that independent preflight; concurrent
XU preset writes during a combined startup produced a target-host
`Device or resource busy` failure and are therefore excluded here.

Keep the episode root on one local filesystem so the final staging rename is
atomic. Avoid an unstable network mount for live capture. If data must live on
shared storage, record to local SSD/NVMe, validate, then copy with checksums.

## Run the current simulation-backed capture

This command receives live Quest packets and live D435 frames but commands only
MuJoCo:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
  --episode-data-config data/local/dual_d435_episode.yaml \
  --episode-root data/episodes \
  --task-name <TASK_ID> \
  --operator <OPERATOR_ID>
```

The default MuJoCo viewer and dual-camera preview are enabled. Use
`--no-viewer` or `--no-episode-preview` only when the corresponding visual
check is intentionally unnecessary. `--duration-sec` defaults to 180 seconds
and bounds the outer session.

Do not use `unlabeled_task` or `unknown` for a real study. A stable task ID and
non-sensitive operator pseudonym are easier to audit than later filename-based
inference. The writer still sets `success_label=unlabeled`; task success must be
reviewed separately. An unlabeled episode can be structurally valid, but it
cannot enter a training split or be exported.

### Operator sequence

1. Start with the arm trigger released. Verify both preview roles and status.
2. Establish the existing release-before-press Quest reference/clutch sequence.
3. Press and hold the configured arm trigger. The collector enters `ARMING`;
   it does not create a training episode until reference, accepted target,
   measured/estimated state, hand continuity, and fresh post-trigger camera
   prerequisites all pass.
4. Once the preview reports `REC`, perform one coherent demonstration. Avoid
   combining multiple trials in one trigger hold.
5. Release the arm trigger immediately at the intended episode boundary. The
   last already-complete canonical sample is retained, no release tail is
   fabricated, the archive finalizes, and the command exits.
6. Review and label the semantic outcome with `dataset label`. Do not equate
   `completed` with task success.

Closing the preview while recording, pressing Ctrl+C, camera disconnect,
source staleness, timestamp regression, heartbeat loss, hard fault, queue
overflow, or write failure ends the episode as `aborted` or `invalid`.
Pre-recording rejection writes a `rejected-start-*.json` report and no episode
directory.

## Run one physical v2 episode

This is a physical motion gate. Executing the complete real-device command
authorizes the current process. After camera identities, calibration snapshots,
controller state, E-stop,
workspace, device identity, CPU isolation, and RH56 prerequisites are checked:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --robot-ip <ROBOT_IPV4> \
  --rh56-device /dev/serial/by-id/<RH56_ADAPTER> \
  --hand-prerequisites-complete --no-auto-retry \
  --estop-accessible --workspace-clear \
  --native-control-cpu <VERIFIED_CPU> \
  --duration-sec <BOUNDED_SECONDS> \
  --episode-data-config data/local/dual_d435_episode.yaml \
  --episode-root data/episodes \
  --task-name fixed_bottle_pick_lift_10cm_hold_3s_replace \
  --operator <OPERATOR_ID> --episode-preview
```

The first trial should be short. The combined command opens the separately
gated JAKA/RH56 paths even when neither clutch is pressed, so it is not a
read-only preflight. Use the maintained JAKA and RH56 read-only probes for the
static hardware checks before authorizing this command.

### Bottle Pickup A/B/C gate

Use the same wrapper and schema for every phase. One process invocation is one
episode; arm and hand clutch transitions only create `control_segment_id`
boundaries inside it.

Phase A is a 10 second no-new-target record. Keep both clutches released for
the whole run; the canonical action is then explicitly sourced as
`measured_hold_reference`, not presented as a demonstration:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --rh56-device "$RH56_DEVICE" --allow-direct-ch341-device \
  --duration-sec 10 --hand-prerequisites-complete --no-auto-retry \
  --estop-accessible --workspace-clear --native-control-cpu 6 \
  --rh56-scheduler-profile fast40 \
  --episode-data-config data/local/dual_d435_episode.yaml \
  --episode-root data/episodes \
  --task-name fixed_bottle_pick_lift_10cm_hold_3s_replace \
  --operator <OPERATOR_ID> --episode-preview --log-dir logs
```

For Phase B, use the identical command with `--duration-sec 60`. Perform one
approach, grasp, approximately 10 cm lift, approximately 3 second hold,
replace, and retreat. Do not use clutch edges as episode boundaries. Stop the
bounded process after the task, then inspect and label its single result.

For Phase C, repeat the Phase B process exactly three times. Reset the bottle
before each process, preserve each printed `EPISODE_RESULT`, inspect and label
it immediately, and do not proceed if deep validation is not both
`valid=true` and `training_eligible=true`. Export all three individually to
the ACT-layout HDF5 view and run the existing exporter regression before
starting a larger collection.

## Inspect and validate every episode

The command prints an `EPISODE_RESULT=<absolute path>` line. Preserve that path
in the session log, then run deep validation:

```bash
.venv/bin/embodied-lab dataset validate \
  data/episodes/episode-<uuid> \
  --output data/reports/episode-<uuid>.validation.json
```

Validation exit code is nonzero if structural integrity fails. The JSON fields
have separate meanings:

- `valid=true` means the archive satisfies the offline structural contract.
- `training_eligible=true` additionally means the episode completed, is
  non-empty, has no canonical missed slots, and has an explicit `success` or
  `failure` label.
- `physically_validated` remains `false`.

Immediately after collection, `valid=true` with
`training_eligible=false` is expected because the writer deliberately
finalizes `success_label=unlabeled`. Use `dataset label` only after reviewing
both camera views and task outcome, then run deep validation again.

Review warnings instead of discarding them blindly:

- `repeated_camera_selection_count` can be expected when source and canonical
  rates differ, but a large value may indicate a stalled source;
- `identical_rgb_payload_transition_count` can reflect a static scene or a
  frozen camera and needs visual review;
- `maximum_absolute_source_offset_ns` is useful only for sources in comparable
  clock domains;
- `image_shapes`, camera profile, and dropped-frame counters should remain
  stable through the episode.

Also inspect:

- `metadata.json`: correct task/operator, serial-to-role mapping, calibration
  snapshot, control-config hash, `simulation_only`, termination reason, and
  sample count;
- `canonical/samples.jsonl`: causal offsets, action status, provenance, and
  camera frame-number progression;
- representative first/middle/last RGB and raw depth arrays;
- available disk space and a checksum-based copy after transfer.

Deep validation reads every canonical NPY and all raw JSONL. `--fast` is useful
for repeated catalog scans but is not the final acceptance check.

## Build a versioned training selection

After validating the complete collection batch:

```bash
.venv/bin/embodied-lab dataset manifest \
  data/episodes \
  data/manifests/dataset-v1.json \
  --seed embodied-lab-v1 \
  --train-fraction 0.8 \
  --validation-fraction 0.1

.venv/bin/embodied-lab dataset statistics \
  data/manifests/dataset-v1.json \
  data/manifests/dataset-v1.statistics.json
```

Before training:

- inspect `split_counts`; a small set may have no validation or test episodes;
- confirm all intended episodes appear and rejected/invalid data is excluded;
- confirm every selected episode is independently reviewed and labeled
  `success`; `failure` and `unlabeled` episodes remain visible but are excluded
  from behavior-cloning splits;
- define a group-aware split when object, scene, task, operator, or collection
  session leakage would invalidate the experiment;
- freeze the manifest and statistics hashes with the run config;
- never recompute normalization statistics from validation or test data.

The current manifest uses whole-episode UUID hashing. It prevents frame leakage
between splits, and includes only explicitly successful episodes. Reviewed
failures remain in the canonical dataset for diagnosis but are not exported or
used for normalization. The manifest does not
enforce object/session grouping or reviewer identity/provenance. Deep payload
validation is the default; explicit `--fast` creates an
inventory whose episodes are all excluded from splits.

## Create derived training formats

Install the optional exporter dependencies before these commands.

ACT HDF5:

```bash
.venv/bin/embodied-lab dataset export \
  data/episodes/episode-<uuid> \
  act-hdf5 \
  data/exports/act/episode-<uuid>.hdf5
```

LeRobot v3:

```bash
.venv/bin/embodied-lab dataset export \
  data/episodes/episode-<uuid> \
  lerobot-v3 \
  data/exports/lerobot/episode-<uuid> \
  --repo-id <LOCAL_NAMESPACE>/<DATASET_NAME>
```

Both behavior-cloning exports require an explicit `success` label, process one
episode at a time, and refuse an existing destination. The LeRobot exporter stores RGB
and low-dimensional data through the official SDK and keeps raw `uint16` depth
in an explicit sidecar. The repository has no multi-episode merge command or
framework trainer. See
[Training integration](../training/TRAINING_INTEGRATION.md).

## Episode acceptance checklist

An episode intended for supervised imitation should satisfy all applicable
items:

- [ ] Final directory is `episode-<uuid>`, not `.partial`.
- [ ] Deep validation returns `valid=true` and `training_eligible=true`.
- [ ] `completion_status=completed` and termination reason matches the
      operator action.
- [ ] `success_label` is independently reviewed and explicitly `success` or
      `failure`; it was not inferred from completion.
- [ ] `canonical_missed_slot_count=0`.
- [ ] Camera roles, serials, shapes, frame counters, depth scale, and
      calibration snapshot are correct.
- [ ] State/action units and provenance match the producer.
- [ ] No unexpected `held_rejected` interval or an explicit training policy
      exists for it.
- [ ] Task, object/session metadata, and independently reviewed outcome are
      available for the intended experiment.
- [ ] Representative RGB/depth/state/action playback has been inspected.
- [ ] Manifest split does not leak the evaluation unit.
- [ ] The canonical archive and manifest/statistics hashes are preserved.

`completed` is a recording outcome, not a manipulation success label.
`valid` is an archive property, not physical calibration evidence.

Use the offline inspection entry before accepting an episode:

```bash
.venv/bin/embodied-lab dataset inspect data/episodes/episode-<uuid> \
  --output artifacts/inspection/episode-<uuid>.json \
  --plot artifacts/inspection/episode-<uuid>.png
```

Add `--playback` on a host with a local OpenCV display to review workspace and
wrist RGB/raw-depth frames at recorded timing; `--playback-rate` changes only
review speed. The plot compares measured/observed state with commanded action
and shows source offsets. These tools are offline aids: an operator must still
review camera roles, outcome, occlusion, freeze, and failure context.

## Interrupted capture and recovery

- A visible `.episode-<uuid>.partial` means finalization did not complete.
  Quarantine it and preserve the process log.
- Do not add it to a manifest, rename it to look complete, or export it.
- Re-run collection for training data. Manual salvage may be useful only for
  diagnostics and must produce a new versioned archive through a future audited
  recovery tool.
- A `rejected-start-<uuid>.json` is expected evidence of a failed start gate,
  not an empty demonstration.
- Ending the outer session while `ARMING` writes that rejected-start evidence;
  ending it while still `IDLE` closes the unused writer without creating an
  episode or rejection artifact.
- Copy finalized data with a tool that verifies checksums. The manifest
  currently hashes metadata and the canonical index, not every NPY payload.

The current writer's final rename protects readers from observing a
half-finalized namespace, but it is not a full crash-consistent storage system.

## Physical combined collection boundary

The physical v2 producer is now integrated at the existing authoritative
hardware loop after it produces the immutable `AcceptedArmTarget`. It reuses
the same JAKA/RH56 session and does not create a second controller, follow
MuJoCo `qpos`, or call JAKA inverse kinematics in the native joint worker.
Measured JAKA state, accepted/held arm targets, successful final RH56 targets,
and all five required RH56 feedback register groups are retained with their
provenance and timestamps.

This integration is offline tested, but a completed, training-eligible
physical Bottle Pickup episode has not yet been recorded. The remaining gate
is the A/B/C target-host validation above, including sustained dual-camera
write throughput and timing review; implementation alone is not a physical
PASS.

## Data custody

Raw episodes can contain people, rooms, object identities, network endpoints,
and operator metadata. Use pseudonymous operator IDs, restrict access, and
review frames before external sharing. Keep original episodes and formal
checkpoints on durable storage; use local NVMe only for capture/training cache
unless it is also backed up.

Do not commit episodes, exports, videos, calibration secrets, or model weights.
Keep a dataset release record with:

- dataset schema and manifest/statistics hashes;
- episode count and split counts;
- collection host/software revision;
- calibration and control-config versions;
- task/object/session grouping policy;
- labeling policy and reviewer;
- known quality exceptions;
- copy/checksum evidence and storage location.

The exact schema and implemented/planned boundary are defined in
[Canonical episode dataset schema](DATASET_SCHEMA.md).
