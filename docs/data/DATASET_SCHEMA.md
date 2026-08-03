# Canonical episode dataset schema

## Scope and authority

This page documents the dataset code that currently exists in
`src/episode_dataset/`. It is the contract for
`embodied_lab.single_episode.v1` and the physical-hand extension
`embodied_lab.single_episode.v2`; examples or historical reports do not
override the source schema.

The status words used below are deliberate:

- **Implemented** means the field or behavior exists in the current repository.
- **Implemented, simulation source** means the schema exists but the only
  end-to-end producer currently connected to it uses simulated arm and hand
  state.
- **Planned** means the field or workflow is required before physical
  JAKA/RH56 collection or policy training, but does not yet exist.

Canonical v1 remains the simulation-hand archive. V2 reuses the same writer,
camera payloads, causal clock, vector ordering, and lifecycle while declaring
physical RH56 normalized actuator units and retaining raw register telemetry.
The physical producer is wired only into the separately authorized combined
JAKA/RH56 gate. Implementation and offline tests are not a physical PASS.

## Episode directory

An accepted recording is first written under a hidden staging name and is
renamed after finalization:

```text
DATASET_ROOT/
  rejected-start-<uuid>.json
  .episode-<uuid>.partial/             # exists only while writing
  episode-<uuid>/
    metadata.json
    validation_report.json
    raw/
      <stream>.jsonl
      cameras/
        workspace/
          rgb/*.npy
          depth_raw/*.npy
          depth_aligned_to_rgb/*.npy
        wrist/
          rgb/*.npy
          depth_raw/*.npy
          depth_aligned_to_rgb/*.npy
    canonical/
      samples.jsonl
      frames/
        workspace/{rgb,depth_raw,depth_aligned_to_rgb}/*.npy
        wrist/{rgb,depth_raw,depth_aligned_to_rgb}/*.npy
    calibration/
      <captured calibration files>
    exports/
```

The raw camera filenames contain the host receive timestamp and both device
frame numbers. Canonical camera filenames are the zero-padded canonical frame
index. A canonical frame is a hard link to the selected raw payload, so the raw
and canonical indexes do not duplicate image bytes.

`exports/` is reserved for derived representations. The canonical archive is
the authority; ACT or LeRobot output must be reproducible from it and must not
replace it.

## Lifecycle and finalization

The implemented state machine is:

```text
IDLE -> ARMING -> REC -> FINALIZING -> DONE
```

1. `IDLE` retains preview and latest-source state but creates no episode
   directory.
2. A rising capture-boundary edge enters `ARMING`. The simulation producer
   uses the arm trigger; physical v2 uses the explicitly started bounded run,
   independently of either clutch.
3. Recording begins only after all start prerequisites pass:
   reference established; first accepted arm target available; measured arm
   position available; measured or explicitly estimated arm velocity and TCP
   pose available; hand observation and hold target available; arm trigger
   still held; arm and hand start-continuity deltas inside their configured
   limits; and fresh, causal post-trigger frames available from both cameras.
4. `REC` writes raw sources asynchronously and emits canonical samples on a
   fixed-rate clock. At the current 30 Hz camera profile, causal latest-frame
   selection permits 70 ms of source age (about two frame periods plus host
   jitter); it never selects a future frame and longer stalls abort capture.
5. The configured capture boundary finalizes a `completed` episode with no
   fabricated tail. Physical v2 remains active across arm/hand clutch changes.
   Camera/control staleness, clock regression, hard fault, heartbeat loss, or
   write failure finalizes an `aborted` or `invalid` episode.
6. A start rejected before recording removes any staging directory and writes
   only `rejected-start-<uuid>.json`.

Finalization closes raw JSONL handles, performs a staging integrity check,
writes final metadata and a validation report, calls `fsync` on
`metadata.json`, `validation_report.json`, and `canonical/samples.jsonl`, then
renames `.episode-<uuid>.partial` to `episode-<uuid>`.

The rename is an atomic namespace operation when staging and final paths are on
the same filesystem. It is **not** a claim of complete power-loss durability:
individual NPY payloads and parent directories are not all explicitly
`fsync`ed, there is no per-payload checksum manifest, and no partial-archive
repair command exists. Preserve a `.partial` directory for forensic inspection;
do not rename it manually or train from it.

The maintenance decision is to retain this bounded design rather than add an
untested transaction layer. The asynchronous writer drains before finalization;
reported write failures downgrade the episode; the staging check prevents a
known incomplete `completed` archive; and default manifest construction
deep-reads payloads and excludes a damaged final directory. These checks are
enough for the current offline pipeline while the limits above remain explicit.

## Metadata contract

`metadata.json` is written at start and finalized at stop. The following fields
are implemented.

| Field | Type | Meaning |
|---|---:|---|
| `schema_version` | string | Exactly `embodied_lab.single_episode.v1`. |
| `episode_uuid` | UUID string | Stable episode identity and split key. |
| `task_name` | string | Operator-provided task identifier. It is not currently validated against a registry. |
| `operator` | string | Operator identifier supplied at launch. |
| `start_host_monotonic_ns`, `end_host_monotonic_ns` | integer | Host monotonic recording bounds. These are not UTC wall-clock values. |
| `start_wall_time_utc`, `end_wall_time_utc` | ISO-8601 string | Human/audit wall-clock bounds; canonical alignment never uses them. |
| `finalized_host_monotonic_ns` | integer | Host time at finalization, separate from the last canonical sample time. |
| `trigger_press_host_monotonic_ns`, `trigger_release_host_monotonic_ns` | integer or null | Trigger boundary evidence. |
| `dataset_fps` | positive integer | Canonical sampling rate. Current example is 30 Hz. |
| `sample_count`, `duration_s` | integer, float | Final canonical row count and elapsed monotonic duration. |
| `canonical_missed_slot_count` | integer | Number of nominal canonical slots skipped; summed from `missed_slots_after`. |
| `robot_model`, `hand_model` | string | Declared embodiment names, not proof that physical devices supplied data. |
| `arm_initial_measured_q_rad` | six floats | Initial arm position used by the start gate. |
| `hand_initial_state`, `hand_initial_state_source` | six floats, provenance | Initial six-channel hand view and whether it was measured, commanded, estimated, or unavailable. |
| `camera_serials`, `camera_profiles` | object | Role-to-device identity and resolved stream/profile metadata. |
| `calibration_snapshot` | object | Version plus copied calibration files and SHA-256 values. |
| `control_config` | object | Control config path and SHA-256 at collection time. |
| `raw_streams` | object | Availability/provenance declaration for expected raw streams. |
| `action_order`, `observation_state_order` | string arrays | Exact vector ordering defined below. |
| `units` | object | Arm radians, arm radians/second, TCP metres and XYZW quaternion, hand radians, raw depth device units, host monotonic nanoseconds. |
| `time_alignment` | object | Causal selection and missed-slot policy. |
| `completion_status` | enum | `completed`, `aborted`, or `invalid`. |
| `termination_reason` | string | Literal reason for finalization. |
| `success_label` | enum | `unlabeled`, `success`, or `failure`. The writer finalizes as `unlabeled`; `dataset label` applies the reviewed outcome. |
| `failure_stage`, `notes` | string/null, string | Reviewed task outcome detail set by `dataset label`. |
| `finalized` | bool | Must be `true` for validation. |
| `code` | object | Git revision/dirty state when available, `EMBODIED_LAB_SOURCE_REVISION`, or an explicit no-repository marker. |
| `simulation_only`, `physically_validated` | bool | Producer-supplied validation boundary. The current connected collector writes `true` and `false`, respectively. |

The writer accepts producer-specific metadata extensions. Current validation
does not enforce schemas for `object_id`, initial object state, failure
category, environment conditions, hardware configuration, or a reviewed
label annotation record. Those fields are **planned**, even though a caller
could insert arbitrary extra keys today.

## Canonical observation and action vectors

### Observation state

The canonical state has 25 scalar values in this exact order:

| Index | Field | Unit |
|---:|---|---|
| 0–5 | `arm_q_measured.J1` … `arm_q_measured.J6` | rad |
| 6–11 | `arm_dq_measured.J1` … `arm_dq_measured.J6` | rad/s |
| 12–14 | `tcp_pose.x_m`, `tcp_pose.y_m`, `tcp_pose.z_m` | m |
| 15–18 | `tcp_pose.qx`, `tcp_pose.qy`, `tcp_pose.qz`, `tcp_pose.qw` | unit quaternion, XYZW |
| 19–24 | `hand.H1` … `hand.H6` | rad in canonical v1 |

The field name `arm_dq_measured` is retained by v1, but each row also carries
`arm_dq_source`. A producer that derives velocity must label it `estimated`,
not `measured`. The same rule applies to TCP pose and hand state.

### Action

The canonical action has 12 scalar values:

```text
[J1, J2, J3, J4, J5, J6, H1, H2, H3, H4, H5, H6]
```

Indices 0–5 are the accepted arm joint target in radians. Indices 6–11 are the
hand actuator target in radians for canonical v1. The arm target is the shared
pipeline's accepted target, not MuJoCo `qpos`, not an EDG transport point, and
not a rejected IK candidate.

`action.arm_status` is:

- `accepted`: a new safe target was accepted;
- `held_rejected`: candidate feasibility failed and the last accepted target
  was held while the control heartbeat remained live.

A training adapter must make an explicit, recorded decision about
`held_rejected` rows. The current statistics code includes them and neither
exporter silently removes them.

## Per-frame canonical record

Every line of `canonical/samples.jsonl` is one JSON object.

| JSON path | Type | Semantics |
|---|---:|---|
| `frame_index` | integer | Contiguous stored-row index starting at zero. |
| `timestamp` | float | Seconds since the episode start monotonic timestamp. |
| `timestamp_host_monotonic_ns` | integer | Nominal canonical timestamp. |
| `observation.images.<role>.rgb` | path | HWC `uint8` RGB NPY. |
| `observation.images.<role>.depth_raw` | path | Two-dimensional `uint16` device-unit depth NPY. |
| `observation.images.<role>.depth_aligned_to_rgb` | path or null | Optional aligned `uint16` depth; it never replaces raw depth. |
| `observation.state.arm_q_measured` | six floats | Arm position, radians. |
| `observation.state.arm_dq_measured` | six floats | Arm velocity view, radians/second. |
| `observation.state.tcp_pose` | seven floats | `[x_m,y_m,z_m,qx,qy,qz,qw]`. |
| `observation.state.hand` | six floats | Hand actuator-space observation, radians in v1. |
| `observation.state.*_source` | enum | `measured`, `commanded`, `estimated`, or `unavailable`. |
| `observation.state.arm_trigger`, `hand_grip` | bool | Input clutch state selected at the canonical time. |
| `observation.state.control_segment_id`, `control_segment_mode` | integer, enum | Increments on stable arm/hand clutch-mode transitions: idle, arm-only, hand-only, or both. |
| `action.arm_q_target` | six floats | Accepted/held arm target, radians. |
| `action.hand_target` | six floats | Hand target, radians in v1. |
| `action.arm_status` | enum | `accepted` or `held_rejected`. |
| `action.arm_source` | enum | `accepted_target` or the pre-engagement `measured_hold_reference`. |
| `timing.source_timestamps_ns` | object | Source timestamps keyed by source name. |
| `timing.source_timestamp_domains` | object | Clock domain for every source timestamp. |
| `timing.signed_offsets_ns` | object | `source_timestamp - canonical_timestamp` when clocks are comparable. |
| `timing.synchronization_valid` | bool | Must be `true` for a stored canonical row. |
| `timing.stale_sources`, `dropped_sources` | arrays | Explicit source-quality annotations. |
| `timing.nominal_slot_index` | integer | Fixed-clock slot; may jump when a deadline is missed. |
| `timing.missed_slots_before`, `missed_slots_after` | integer | Gap evidence adjacent to this stored row. |
| `timing.timing_valid` | bool | True only when both adjacent missed-slot counts are zero. |
| `camera.<role>.*_device_timestamp_ms` | float | Device timestamp as reported by the camera. |
| `camera.<role>.*_timestamp_domain` | string | RealSense timestamp domain; never assumed to equal host monotonic time. |
| `camera.<role>.*_frame_number` | integer | Device color/depth frame counter. |

The two camera roles are fixed as `workspace` and `wrist`. A camera must be
selected by serial number; `/dev/video*` enumeration order is not identity.
`depth_raw` remains in device counts. Convert it to metres only with the depth
scale captured in the resolved camera profile.

## Time alignment and missing slots

The canonical clock has period:

```text
period_ns = round(1_000_000_000 / dataset_fps)
timestamp_ns = start_host_monotonic_ns + nominal_slot_index * period_ns
```

For each source, the collector selects the latest sample at or before the
canonical timestamp. Future samples are never selected. A source older than its
configured age limit aborts collection instead of being copied forward.

The writer never emits a catch-up burst. If the process is late, it skips
expired nominal slots:

- `frame_index` remains contiguous because it indexes stored rows.
- `nominal_slot_index` jumps across the missing interval.
- The gap count is written as `missed_slots_after` on the row after which the
  gap was discovered and carried as `missed_slots_before` on the next stored
  row.
- `canonical_missed_slot_count` and validation quality count each missing slot
  once by summing `missed_slots_after`.

An episode with any canonical gap can still be structurally valid, but it is
not training eligible. Repeated selection of the same camera device frame is a
quality warning rather than an automatic error because camera and canonical
rates can differ. Identical RGB bytes across increasing device frame numbers
are also a warning, not proof of a frozen camera.

Source clocks remain explicit. Host monotonic timestamps can be directly
offset from the canonical clock; device and Quest source clocks require a
separate synchronization model before cross-domain latency claims are valid.

## RH56DFX semantics

Canonical v1 and the maintained physical RH56 driver expose different units.
They must not be mixed.

### Canonical v1

| Field | Required unit | Current connected producer |
|---|---|---|
| `observation.state.hand` | six actuator-space radians | simulated measured state |
| `action.hand_target` | six actuator-space radians | simulated command |
| `hand_source` | provenance enum | `measured` in the simulation producer |

The validator rejects a canonical v1 archive whose metadata does not declare
the hand unit as `rad`. Raw RH56 register counts therefore require a different
versioned schema or a separately declared training view.

### Physical v2

V2 declares canonical `observation.state.hand` and `action.hand_target` as
`normalized_closure_0_to_1`. `raw/rh56_feedback.jsonl` retains commanded
normalized/raw targets, measured `ANGLE_ACT`, normalized `ANGLE_ACT`, raw
`CURRENT`, raw `FORCE_ACT`, raw `ERROR`, raw `STATUS`, command/feedback
timestamps, per-register successful-read timestamps, latency, sequences,
disposition, transport state, and fault state. `raw/jaka_state.jsonl` retains
accepted commanded joints, measured joints, finite-difference velocity, the
accepted commanded TCP pose, and JAKA observation/command/record host
timestamps. Canonical TCP provenance is `commanded`; measured joints remain
available for offline FK without adding FK work to the command-critical loop.

### Physical RH56 PC-direct raw record

`src/rh56_driver/pc_direct_control.py::episode_record` currently exposes:

| Raw field | Unit/meaning | Do not reinterpret as |
|---|---|---|
| `action.hand_target` | normalized closure, six values in the configured 0–1 command space | radians or passive finger joints |
| `action.requested_hand_target` | caller request in the same normalized command convention | a confirmed actuator write |
| `action.selected_hand_position_raw` | selected six-channel raw command counts | measured position |
| `observation.hand_position` | six `ANGLE_ACT` raw controller counts | full articulated finger-joint state |
| `observation.hand_position_normalized` | driver-normalized `ANGLE_ACT` view | calibrated SI angle without a calibration contract |
| `observation.hand_current_or_load` | six signed `FORCE_ACT` raw counts | force in newtons, tactile array, or slip signal |
| `observation.hand_current_raw_count` | six `CURRENT` raw counts | calibrated torque or force |
| `hand_error`, `hand_status` | six raw `ERROR`/`STATUS` register values | generic booleans with undocumented semantics |
| command/feedback timestamps and sequences | host monotonic scheduling and freshness evidence | synchronized device time |

`ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` are raw controller
feedback fields. They are not the complete passive-joint configuration, a
tactile sensor array, or direct slip sensing. Defaults or missing values from a
schema helper must never be promoted to measured zero.

Physical v2 retains the raw values, normalized views, calibration/version
identifiers, provenance, register freshness, read latency, command disposition,
and validity evidence. It does not convert counts to canonical-v1 radians.

## Validation

Run deep validation by default:

```bash
.venv/bin/embodied-lab dataset validate \
  data/episodes/episode-<uuid> \
  --output data/reports/episode-<uuid>.validation.json
```

Use `--fast` only for an index-time check that intentionally skips NPY payload
loading and raw JSONL parsing.

`valid` and `training_eligible` are different:

- `valid` requires a finalized supported schema, matching vector orders, the
  matching hand-unit declaration (`rad` for v1 or normalized closure for v2),
  readable metadata/indexes,
  contiguous stored frame indices, monotonic canonical time and nominal slots,
  correct fixed-slot timestamps, valid finite vector shapes, JAKA arm
  state/target inside the manufacturer boundaries, valid action status, safe
  in-episode paths, present camera payloads, and matching recorded calibration
  checksums. Deep mode also checks NPY type/dimensions/shape consistency and raw
  JSONL syntax. The validator does not yet exhaustively compare every other
  unit key with a complete metadata schema.
- `training_eligible` additionally requires `completion_status=completed`,
  at least one sample, zero canonical missed slots, and an explicit
  `success_label` of `success` or `failure`.

This validator flag means a labeled trajectory is structurally usable for a
downstream task that intentionally models either outcome. The maintained
behavior-cloning manifest and ACT/LeRobot exporters are stricter: they include
only `success`; reviewed failures stay archived and are excluded from splits,
normalization, and export.

Validation is offline evidence only. Its report always sets
`physically_validated=false`. It does not establish task success, calibration
accuracy, sensor semantics, or physical safety.

An episode finalized by the current collector remains structurally valid with
`success_label=unlabeled`, but it is not training eligible. An audited
post-collection review must explicitly set `success` or `failure` before
manifest construction or export:

```bash
.venv/bin/embodied-lab dataset label data/episodes/episode-<uuid> \
  --success success --notes "reviewed bottle task"

.venv/bin/embodied-lab dataset label data/episodes/episode-<uuid> \
  --success failure --failure-stage grasp --notes "bottle slipped"
```

## Dataset manifest, split, and statistics

Build a deterministic manifest:

```bash
.venv/bin/embodied-lab dataset manifest \
  data/episodes \
  data/manifests/dataset-v1.json
```

The implemented manifest schema is `embodied_lab.dataset_manifest.v1`.
Eligible episodes are assigned by SHA-256 of
`<seed>:<episode_uuid>`, with default fractions 0.8/0.1/0.1 for
train/validation/test. The split unit is the complete episode, so frames from
one demonstration cannot leak across splits. Missing, malformed, or duplicate
UUIDs (all occurrences), invalid/aborted episodes, unlabeled episodes, and
timing-gapped episodes receive `excluded`. Hidden `.partial` directories are
not indexed.

Manifest creation performs deep payload validation by default. Therefore a
final directory with a malformed NPY payload remains visible in the manifest
with validation errors and `split=excluded`. Explicit `--fast` creates an
inventory only; it assigns every episode to `excluded` because payload
integrity has not been established.

The manifest records task/object/operator/success metadata when present,
simulation status, sample count, timing-gap count, calibration version, control
config hash, metadata hash, canonical-index hash, and validation findings.
Those two hashes do not cover every image payload. Small datasets can
legitimately receive an empty validation or test split; inspect
`split_counts`.

The current hash split does not group related episodes by object instance,
scene, operator, or session. A benchmark that tests generalization must define
a group-aware split outside the current manifest implementation and must avoid
choosing it after inspecting test results.

Compute normalization statistics from only manifest `train` episodes:

```bash
.venv/bin/embodied-lab dataset statistics \
  data/manifests/dataset-v1.json \
  data/manifests/dataset-v1.statistics.json
```

`embodied_lab.normalization_statistics.v1` stores sample count, population
mean, population standard deviation, minimum, and maximum for the 25-D state
and 12-D action, plus the source manifest SHA-256 and episode UUIDs. Image and
depth statistics are not computed. A training adapter must replace a zero
standard deviation with 1, record affected fields, and preserve the statistics
file/hash with every checkpoint. Statistics currently include
`held_rejected` rows. Statistics require a deep-validation manifest and reject
train entries whose path escapes the dataset root or whose UUID, metadata
hash, or canonical-index hash has changed since manifest construction. The
current manifest does not checksum every image payload.

## Exported training views

Only training-eligible episodes can be exported. Both exporters refuse to
overwrite an existing destination and stage output before rename.

| View | Implemented content | Important boundary |
|---|---|---|
| ACT-style HDF5 | `/observations/qpos` = arm q + hand (12); `/observations/qvel` = arm dq (6); workspace/wrist RGB; lossless raw depth extension; `/action` (12); timestamps; timing validity; arm action status; ordering/unit attributes | One episode per file. It is an exporter, not an ACT trainer. The upstream ACT reference code assumes a 14-D ALOHA state/action in multiple paths, so this 12-D view requires a reviewed adapter. Depth is a project extension, not an upstream ACT requirement. |
| LeRobot v3 | Official SDK features for two RGB videos, state (25), action (12), trigger/grip, provenance, action status, source timing, synchronization flag, and task | One episode per dataset output. No repository merge or training loader exists. |
| LeRobot depth sidecar | Lossless canonical `uint16` raw/aligned depth plus copied canonical index and code tables | Explicitly `official_lerobot_feature=false`; it is not silently quantized into video. |

The LeRobot timing exporter uses the minimum signed 64-bit integer as the
sentinel for an unavailable timestamp/offset. Its metadata records the sentinel
and code tables.

See [Training integration](../training/TRAINING_INTEGRATION.md) for the
framework-specific boundary.

## Planned schema work

The following are requirements, not current capabilities:

- require structured task, object, initial-state, success, failure-category,
  hardware-configuration, environment, and session metadata;
- define device-to-host clock models and synchronization uncertainty rather
  than comparing unrelated clock domains;
- add crash discovery/quarantine and, if justified, repair for `.partial`
  archives;
- add complete payload checksums and directory durability where required;
- add group-aware split policies and dataset-version identity;
- add scalable multi-episode packing/cache/shard tooling only after measured
  filesystem and loader bottlenecks justify it.

Any incompatible unit, vector order, sensor meaning, or required-field change
must create a new schema version and an explicit converter. Do not silently
reinterpret v1 data.
