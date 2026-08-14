# Strong ACT canonical rollout qualification — 2026-08-14

## Scope and frozen experiment

This report covers the Strong ACT checkpoint only; no model was retrained and
no raw/master dataset was changed.

| Item | Value |
|---|---|
| Base commit | `f89d3a5` (`Add absolute-time ACT temporal executor`) |
| Final code/report commit | `Finalize canonical Strong ACT rollout path` (see `git log -1`) |
| Branch | `research/thread-b-force-interaction` |
| Checkpoint | `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model` |
| Checkpoint contract | full action chunk `[60, 12]`; two RGB images; state `[12]`; absolute action `[12]` |
| Strong config SHA-256 | `932b95e0b778d27519c200c6e54db96565bd0256d0da0791698aa69e39b1440a` |
| Nominal52 split SHA-256 | `a5abaf80523a065ffdf84d611c44c218d1d6cf27b64c7a19a85c349cbbb28741` |
| Temporal coefficient | `m=0.01` |
| Safety layers | unchanged JAKA native checks, RH56 projection/delta/contact checks |

The checkpoint declares `n_action_steps=2`, but canonical execution does not
use that value to truncate a chunk: it predicts the complete chunk, maps every
horizon to an absolute control tick, and emits one ensembled action for the
current tick.

## Phase 1: command-disabled 30/30 shadow

Artifact: `outputs/act_physical_rollouts/20260814_strong_canonical_shadow_002/`

The shadow used the same cameras, read-only JAKA diagnostic, RH56 read-only
feedback, checkpoint, model container, preprocessing, and 30 Hz cadence as the
intended rollout. It had no command API.

| Check | Result |
|---|---:|
| Queries | 300, all finite `[60,12]` |
| Achieved query rate | 30.000005 Hz |
| Query interval p95 / max | 33.354 / 33.456 ms |
| Model roundtrip p50 / p95 / p99 / max | 20.45 / 23.22 / 24.36 / 25.51 ms |
| End-to-end observation-to-prediction p50 / p95 / p99 / max | 22.31 / 26.31 / 26.90 / 27.32 ms |
| Workspace / wrist FPS | 30.34 / 30.14 Hz |
| Camera frame gaps | 0 / 0 |
| Stale source counts | workspace 0, wrist 0, JAKA 0, RH56 0 |
| RH56 register writes | 0 |
| Hardware-limit violations | 0 |
| Deterministic repeat check | exact, max difference 0 |

The shadow gate passed for model throughput, finite outputs, freshness, camera
integrity, and read-only behavior.

## Phase 1: physical qualification artifact

The first attempted output directory had an overlong Unix socket path and
failed before hardware initialization (`command_count=0`). It is not a
qualification attempt. Phase 2 now places the model IPC socket in a short
temporary `/tmp` endpoint, independently of the artifact label. The actual
single qualification artifact is:

`outputs/act_physical_rollouts/canonical001/`

It was launched with:

```text
control/command rate: 30/30 Hz
ACT execution: canonical_temporal_ensemble
chunk: dynamically read [60,12]
temporal coefficient: 0.01
consume-K: not used
duration limit: 60 s
```

The run stopped safely after the freshness gate detected that the physical
producer loop was not sustaining the intended cadence:

| Measurement | Result |
|---|---:|
| Full chunks queried | 6 |
| Commands written by producer | 5, ticks 0–4 |
| Query interval | 70.14, 70.90, 69.58, 71.75, 72.14 ms |
| Effective producer/query cadence | approximately 14.1 Hz |
| Abort | `ACT temporal-ensemble result exceeded policy freshness limit` |
| Projection events | 0 |
| RH56 contact detections | 0 |
| RH56 feedback timeouts | 0 |
| RH56 serial writes | 8 |
| RH56 command deadline lateness | 48.35 ms (last diagnostic) |
| RH56 submit-to-write | 46.39 ms (last diagnostic) |
| JAKA missed deadlines / hard misses | 0 / 0 |
| JAKA tracking hard crossings | 0 |
| Controller alarms/collision/e-stop | 0 / false / false |

The first five selections demonstrate the required absolute-time provenance:

```text
tick 0: q1/h0
tick 1: q1/h1, q2/h0
tick 2: q1/h2, q2/h1, q3/h0
tick 3: q1/h3, q2/h2, q3/h1, q4/h0
tick 4: q1/h4, q2/h3, q3/h2, q4/h1, q5/h0
```

Thus the new executor did not discard overlapping future horizons. The run
ended before a bottle manipulation stage was reached. Grasp, lift, transport,
place, and release are therefore **not evaluated** by this artifact; it must
not be labeled a physical task success or failure. The immediate issue exposed
by the qualification is the difference between the command-disabled model
benchmark and the complete hardware-writing producer loop. The existing
freshness gate correctly stopped rather than allowing stale ensemble output to
continue. The cause of the approximately 70 ms loop period needs a separate
timing investigation before another physical qualification.

Native diagnostics also recorded ordinary output-acceleration boundary
rejections on several joints (no hard-boundary rejection), but no native timing
fault, controller alarm, tracking hard fault, collision, or emergency stop.

## Phase 2: production interface cleanup

`tools/act_physical_rollout.py` now resolves canonical Strong ACT options as
follows:

1. A checkpoint with chunk size at least 60 defaults to
   `canonical_temporal_ensemble`.
2. Query rate is derived from command rate; canonical mode performs one query
   per control tick.
3. The full checkpoint-derived chunk enters the absolute-time ensembler and
   exactly one current action exits each tick.
4. `m=0.01` is the canonical default.
5. Maximum source horizon is derived internally as `chunk_size - 1`.
6. `--consume-actions`, an independent `--query-rate-hz`, explicit source
   horizon, prediction-age, or temporal-buffer overrides are rejected in
   canonical mode.
7. `consume_k`, `temporal_ensemble`, and `async_temporal_ensemble` remain
   explicit diagnostic/backward-compatible modes only. Their legacy options
   are hidden from ordinary `--help` usage and remain available for historical
   replay.
8. Projection, delta limiting, contact safety, and the native JAKA write path
   remain downstream of temporal aggregation and unchanged in semantics.

The normal Strong ACT command is therefore:

```bash
PYTHONPATH=src .venv/bin/python tools/act_physical_rollout.py \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --checkpoint outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model \
  --model-container jaka-lerobot-dev:snapshot-before-raw-mount \
  --output outputs/act_physical_rollouts/<run-label> \
  --duration-sec 60
```

The command intentionally does not require consume-K, query frequency, source
horizon, temporal buffer capacity, or ensemble coefficient parameters.

## Validation

Focused tests after the interface cleanup:

```text
33 passed
```

They cover absolute-time horizon preservation, canonical weighting,
procrastination regression, dynamic chunk handling, projection fail-closed
behavior, canonical default option resolution, rejection of legacy overrides,
and isolation of explicit consume-K mode. `git diff --check` passed.

## Qualification decision

**NO-GO for another physical rollout at this point.** The canonical temporal
semantics and command-disabled inference gate passed, and the one physical
attempt terminated safely, but the complete producer did not sustain 30 Hz.
No task-stage or release conclusion can be drawn from five commands. Resolve
the approximately 70 ms producer-loop timing discrepancy and rerun a fresh
shadow/qualification procedure before evaluating policy behavior.

Generated rollout/shadow outputs are local artifacts and are not part of the
source commit. Raw/master data is unchanged.
