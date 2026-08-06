# Episode collection guide

## Current support boundary

The repository has two producers using the same dual-D435 runtime. The
simulation-backed producer uses the older canonical writer:

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

The maintained physical collection path (`combined-normal-teleop` internally)
writes review-first `lerobot_staging_v1` data from the physical JAKA/RH56 loop.
It contains two feature-key RGB MP4s and an aligned
`data/chunk-000/episode_<index>.jsonl` state/action table; the tracked
configuration disables depth. Quest remains a live control input but is not
recorded in the episode. Both valid clutches released for five seconds end
the current episode and rotate to the next numbered episode; robot control
continues and a new press starts from fresh state. Parquet is created only by
the offline conversion command after human review. This integration is
offline tested and has not yet been physically validated.

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
  root: data/raw_episodes
  format: lerobot_staging_v1
  fps: 30
  video_codec: mp4v
  camera_max_age_ms: 100.0
  camera_severe_stale_limit_ms: 500.0
  camera_consecutive_stale_limit: 15
  camera_missing_timeout_ms: 1000.0
  quality_min_valid_ratio: 1.0
  quality_max_invalid_run: 0
  control_max_age_ms: 40.0
  hand_start_tolerance_rad: 0.05
  camera_ring_capacity: 16
  recorder_queue_capacity: 16
  recorder_overflow_policy: drop_newest
  preview_max_fps: 10.0
  writer_batch_size: 8
  writer_flush_interval_s: 1.0
  writer_shutdown_timeout_s: 5.0

cameras:
  workspace:
    serial: <explicit D435 serial>
    width: 640
    height: 480
    fps: 30
    capture_depth: false
    allow_profile_fallback: true
    align_depth_to_color: false
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

At 30 Hz, `camera_max_age_ms: 100.0` is the initial data-quality limit. Canonical rows causally select the
latest frame at or before the recorder timestamp, so this permits ordinary
host receive jitter, including the measured ~83 ms producer stall. It never
selects a future frame. A stale slot writes a `canonical_data_quality` row with
`workspace_valid`/`wrist_valid`, source age, ring sequence, and a literal stale
reason; it does not copy an old image under a new timestamp and does not stop
robot control. Recording degrades only when the configured consecutive window
also reaches the severe/missing criterion. The 500 ms, 15-slot, and 1000 ms
values are conservative initial host settings for a 30 Hz stream, not robot
safety thresholds; tune them only from measured acquisition evidence.

`control_max_age_ms: 40.0` applies only to the recorder's causal snapshot
selection. It covers the target-host producer's measured 30.6 ms scheduling
jitter. It does not alter Quest/JAKA heartbeat freshness, native worker
watchdogs, ServoJ timing, or any physical control safety threshold.

The recorder uses a lossy latest-only ownership model: runtime clamps the
bounded metadata queue to the camera ring capacity, and a reference that is
overwritten is recorded as `metadata_only` with `reason=ring_reference_expired`.
Queue overflow is `reason=recorder_queue_full`; neither event copies a later
image. `workspace_drop_count`/`wrist_drop_count` count camera frames skipped by
latest-only draining, while recorder counters count queue/write events.
Episode metadata reports valid/invalid slot counts, longest invalid runs,
writer/ring expiry counts, and a `quality_state` of `completed_valid`,
`completed_degraded`, `aborted_recording`, `aborted_robot_safety`, or
`partial_writer_failure`. Only the first state is training-eligible by default.

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
- RGB is present and the resolved profile includes firmware, serial, frame
  rate, and resolution; depth is intentionally disabled for the maintained
  collection format;
- the two cameras do not share a serial and do not exceed USB bandwidth;
- mounting and calibration versions match the intended experiment.

The camera workers copy each frameset once into a preallocated 16-slot ring.
Queue messages contain only immutable timestamps and ring sequences, never RGB
or depth ndarrays. Odd/even slot versions and a second version check prevent a
consumer from accepting a half-overwritten frame. A slow writer may lose an
overwritten reference; that event is counted and does not backpressure capture.

The maintained example writes RGB through OpenCV MP4 encoders and does not
allocate or persist depth frames. Video encoding cost and file size depend on
the selected codec and host; measure sustained write rate and free space on
the actual destination before a long collection session.

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

The default MuJoCo viewer and dual-camera preview are enabled for simulation.
For physical capture, omit `--episode-preview` after the camera preflight:
the preview is an optional visual check, and its Qt/OpenCV refresh competes
with the recorder and can cause camera freshness gaps on the target host.
`--duration-sec` defaults to 180 seconds and bounds the outer session.

Do not use `unlabeled_task` or `unknown` for a real study. A stable task ID and
non-sensitive operator pseudonym are easier to audit than later filename-based
inference. The writer still sets `success_label=unlabeled`; task success must be
reviewed separately. An unlabeled episode can be structurally valid, but it
cannot enter a training split or be exported.

### Operator sequence

1. Start with the arm trigger released. Verify both camera roles and status
   during the separate camera preflight, or use a short preview-only check.
2. Establish the existing release-before-press Quest reference/clutch sequence.
3. Press and hold the configured arm trigger. The collector enters `ARMING`;
   it does not create a training episode until reference, accepted target,
   measured/estimated state, hand continuity, and fresh post-trigger camera
   prerequisites all pass.
4. Once the capture is active, perform one coherent demonstration. Avoid
   combining multiple trials in one trigger hold.
5. Release both valid clutches and keep them released for five seconds. The
   last already-complete staging sample is retained, no release tail is
   fabricated, the episode finalizes, and the recorder opens the next numbered
   episode while the control process continues.
6. Generate the local review page, approve/reject the staging episode, and
   convert only approved data with `dataset convert-staging`. Do not equate
   `completed` with task success.

Preview rendering uses the newest ring references at no more than
`preview_max_fps`; it never consumes a historical FIFO. Resize, RGB conversion,
depth colour mapping, and GUI calls run only on the preview thread. Preview
drops are independent of recording drops.

Recorder publication uses a bounded metadata/reference queue and
`put_nowait`. `drop_newest` preserves already accepted writer work when full;
the new item is counted and control continues. The writer drains up to
`writer_batch_size` messages, retains open JSONL handles, flushes them at the
configured interval, and performs final validation/fsync/rename only during
bounded outer-session cleanup. It writes each raw image once and hard-links a
canonical view, retaining the public NPY layout.

Robot controller alarms, communication/liveness loss, RH56 explicit errors,
command-safety violations, and real hard timing faults retain their existing
stop/hold behavior. A camera stale slot, ring overwrite, preview drop, queue
full, or transient write problem is a recording/data-quality event. A camera
worker exit or persistent configured acquisition fault stops recording only;
it does not issue a JAKA emergency stop. Pressing Ctrl+C or an actual
robot/control fault still ends the outer session.
Pre-recording rejection writes a `rejected-start-*.json` report and no episode
directory.

Episode validation/summary includes camera inter-frame and age distributions,
stale/drop/ring counters, canonical compute time, recorder queue high-water and
drop counts, writer batch/write/flush distributions and throughput, and
preview drop/latency. Percentiles are computed from bounded samples outside the
real-time path.

For a future separately authorized device validation, stage evidence in this
order: dual-D435 camera-only capture to confirm serials/USB topology; recording
to the intended local NVMe with the robot disabled; MuJoCo plus both cameras;
read-only robot/hand state plus cameras; then the repository's existing bounded
physical teleoperation gate with operator stop access and unchanged safety
limits. Compare camera intervals, control p99/max, queue high-water, ring
overwrites, sustained bytes/s, and shutdown completeness at every stage. A
camera-only or offline result is not a physical teleoperation PASS.

## Run physical staging collection

This is a physical motion gate. Executing the complete real-device command
authorizes the current process. After camera identities, calibration snapshots,
controller state, E-stop,
workspace, device identity, CPU isolation, and RH56 prerequisites are checked:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config data/local/physical_collection.yaml \
  --hand-prerequisites-complete --no-auto-retry \
  --estop-accessible --workspace-clear \
  --log-dir logs
```

The first trial should be short. The combined command opens the separately
gated JAKA/RH56 paths even when neither clutch is pressed, so it is not a
read-only preflight. Use the maintained JAKA and RH56 read-only probes for the
static hardware checks before authorizing this command. Keep this physical
capture command without `--episode-preview`; use the preview only for a short
camera check before recording if needed.

### Bottle Pickup A/B/C gate

Use the same wrapper and schema for every phase. One process invocation is one
episode; arm and hand clutch transitions only create `control_segment_id`
boundaries inside it.

Phase A is a 10 second no-new-target record. Keep both clutches released for
the whole run; the canonical action is then explicitly sourced as
`measured_hold_reference`, not presented as a demonstration:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config data/local/physical_collection.yaml \
  --duration-sec 10 --hand-prerequisites-complete --no-auto-retry \
  --estop-accessible --workspace-clear --log-dir logs
```

For Phase B, use the identical command with `--duration-sec 60`. Perform one
approach, grasp, approximately 10 cm lift, approximately 3 second hold,
replace, and retreat. Release both valid clutches for at least five seconds to
rotate the completed episode, then press to begin the next one. Do not treat a
Parquet file as available until the staging episode is reviewed and converted.

For Phase C, repeat the Phase B process exactly three times. Reset the bottle
before each process, preserve each printed episode index, review each staging
episode immediately, and convert only approved episodes before starting a
larger collection.

## Inspect and validate every staging episode

Generate a local review page and inspect both synchronized MP4s, the task
metadata, row count, and the quality/audit JSONL files:

```bash
.venv/bin/embodied-lab dataset review-staging \
  data/raw_episodes episode_000000
.venv/bin/embodied-lab dataset approve-staging \
  data/raw_episodes episode_000000 --status approved \
  --notes "reviewed first, middle, and last frames"
```

Before conversion, confirm that the episode metadata says `completed`, the
JSONL is non-empty with strictly increasing `timestamp_ns`, and both MP4 frame
counts equal the JSONL row count. A rejected or interrupted final staging
episode is not converted. Then materialize the approved episode:

```bash
.venv/bin/embodied-lab dataset convert-staging \
  data/raw_episodes episode_000000 data/lerobot_dataset
```

The older `dataset validate`/`inspect` and canonical manifest commands remain
available for canonical and `raw_episode_v1` archives; they are not the live
staging acceptance gate.

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

The physical `lerobot_staging_v1` producer is now integrated at the existing
authoritative hardware loop after it produces the immutable `AcceptedArmTarget`.
It reuses
the same JAKA/RH56 session and does not create a second controller, follow
MuJoCo `qpos`, or call JAKA inverse kinematics in the native joint worker.
The aligned core table retains measured JAKA/RH56 state and accepted arm/hand
targets. Optional JAKA/RH56 audit records retain device provenance and
timestamps. Quest packets, TCP, and depth are intentionally absent from this
training view.

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
