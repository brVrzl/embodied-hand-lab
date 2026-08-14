# Strong ACT physical qualification — 2026-08-14

## Scope and frozen configuration

This report covers the one authorized Strong ACT qualification rollout after
the command-disabled startup sanity check. No model, raw/master episode, ACT
semantics, safety threshold, gain, or speed-limit change was made for this
run.

- Commit: `043cf0e` (`Select stable ACT runtime query rate`)
- Branch: `research/thread-b-force-interaction`
- Checkpoint: `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model`
- Checkpoint contract: dynamically validated chunk `[60, 12]`
- Policy query: 25 Hz
- Robot command/control timeline: 30 Hz
- Executor: absolute-time asynchronous temporal ensemble
- Ensemble coefficient: `m = 0.01`
- Source horizon: `0..59`, mapped to the 30 Hz command ticks
- Legacy `consume-K`: not used
- Output: `outputs/act_physical_rollouts/20260814_strong_act_qualification_001/`

The rollout artifact remains outside Git; its synchronized videos and JSON/NPZ
provenance are the evidence for this report.

## Startup sanity gate

The exact 25/30 asynchronous configuration was run command-disabled for 15 s
before motion.

| metric | result |
|---|---:|
| policy queries / command ticks | 376 / 451 |
| policy/control rate | 25 Hz / 30 Hz |
| ACT freshness abort | none |
| temporal-ensemble fallbacks | 2 |
| temporal-buffer drops | 0 |
| writer queue high watermark / drops | 2 / 0 |
| workspace / wrist FPS | 30.242 / 30.114 |
| camera frame gaps | 0 / 0 |
| critical-path p50 / p95 / p99 / max | 1.609 / 2.886 / 3.051 / 4.020 ms |

The gate passed without changing freshness or safety limits.

## Physical qualification result

The one 60 s rollout completed normally (`abort_reason = None`). It wrote
1,801 accepted command targets and queried 1,499 full ACT chunks. The video
shows the complete task: approach, bottle enclosure, lift, transport over the
box, placement, and hand opening/retraction. The bottle remained upright on
the box after release.

| stage | result | synchronized evidence |
|---|---|---|
| APPROACH | PASS | Workspace video: approach from start through approximately 0–10 s |
| GRASP | PASS | First selected closure near 9.97 s; hand encloses bottle and lift follows at approximately 10–13 s |
| LIFT | PASS | Bottle leaves the table in the approximately 11–14 s interval |
| TRANSPORT | PASS | Bottle is carried toward/above the box, approximately 14–17 s |
| PLACE | PASS | Bottle is placed upright on the box, approximately 17–18 s |
| RELEASE | PASS | Hand opens/retracts after placement, approximately 18–20 s; bottle remains on box |

This is a successful single qualification trial, not evidence of general
policy success over a statistically meaningful set of scenes.

## Grasp provenance

The coordinated-closure diagnostic uses the existing RH56 action ordering
(`index`, `middle`, `ring`, `pinky`, `thumb_close`, `thumb_lateral`) and a
diagnostic threshold on the first five channels. It is not a contact label.

- First raw coordinated closure: query sequence 178, source horizon `h=59`,
  target command tick 273 (about 9.10 s).
- First ensembled selected coordinated closure: tick 299 (about 9.97 s),
  first-five minimum `0.1028`.
- First written coordinated closure: tick 300 (about 10.00 s), with the same
  selected hand target reaching the RH56 write path.
- First measured coordinated RH56 response: tick 304 (about 10.13 s),
  first-five minimum approximately `0.124`.
- Video evidence: the hand encloses the bottle and the bottle leaves the
  table shortly afterward.

At selected tick 299 there were 50 aligned contributors, with source
horizons spanning the older prediction at `h=59` through the newest available
prediction. The newest contributor age was approximately 18.8 ms. This is
the expected preservation of future predictions; the `h=59` prediction was
not discarded by a consume-K window.

## Release provenance

The release/opening proxy was present in the raw chunks and survived the
absolute-time ensemble:

- First raw opening proxy: query sequence 388, source horizon `h=59`, target
  tick 525 (about 17.50 s).
- First selected opening after grasp: tick 539 (about 17.97 s). It reduced
  the first five finger targets and brought `thumb_close` to the legal lower
  boundary through the normal projection path.
- First written opening target: tick 540 (about 18.00 s).
- Measured RH56 opening response: visible in the video after placement and
  reflected by the falling measured finger values by approximately tick 544
  (18.13 s).

The raw chunk, ensemble contributors/weights, requested target, projected
target, post-delta target, contact-selected target, actually-written target,
and measured state are present in `commands.jsonl`. There is no evidence of a
valid release request being rejected by contact protection:

- contact detector detections: 5;
- final contact latches: all false;
- relief pending: false;
- contact-relief writes: 0;
- serial writes: successful;
- controller alarms/faults: 0.

Thus release is classified as `POLICY_PREDICTED_AND_WRITTEN`, not
`POLICY_RELEASE_PREDICTED_BUT_NOT_EXECUTED` or
`RELEASE_REQUEST_BLOCKED_BY_CONTROLLER`.

## Timing, camera, and safety health

| metric | result |
|---|---:|
| accepted command rate | 29.962 Hz |
| policy deadline skips | 1 |
| workspace / wrist received FPS | 30.042 / 30.008 |
| workspace / wrist frame gaps | 0 / 0 |
| workspace / wrist late interval count | 1 / 0 |
| temporal-buffer capacity/expiry drops | 0 / 0 |
| writer queue high watermark / drops | 2 / 0 |
| native hard timing misses | 0 |
| native timing warning events | 1 |
| JAKA tracking hard crossings | 0 |
| JAKA target rejections | 0 |
| controller alarm events | 0 |
| controller emergency stop | false |
| RH56 serial write p50 / p95 / max | 5.32 / 6.90 / 10.70 ms |

The native metrics contain nonzero JAKA output-acceleration boundary counters,
but all corresponding hard-boundary counters are zero; no command was
rejected and no safety fault occurred. RH56 projection occurred 15 times,
all on `thumb_close`, with maximum correction `0.002547`, well below the
existing `0.02` projection allowance. These were small legal-boundary
corrections, not a domain failure.

The recorded control critical path was p50/p95/p99/max =
1.794/2.901/5.951/6.864 ms. The rollout therefore did not reproduce the
earlier freshness abort.

## Transition classification

No task transition failed in this qualification. Consequently there are no
instances to classify as `POLICY_NOT_PREDICTED`,
`POLICY_PREDICTED_NOT_SELECTED`, `SELECTED_NOT_WRITTEN`,
`WRITTEN_NOT_TRACKED`, `TASK_PHYSICS_FAILURE`, or `UNKNOWN`.

The rollout continued until its configured 60 s normal completion after the
task was already complete; this post-success tail is not treated as another
task attempt.

## Evidence files

Relative to the repository root:

- [rollout summary](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/rollout_summary.json)
- [full command/provenance stream](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/commands.jsonl)
- [policy chunks](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/act_predictions.npz)
- [workspace video](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/workspace.mp4)
- [wrist video](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/wrist.mp4)
- [native safety metrics](../outputs/act_physical_rollouts/20260814_strong_act_qualification_001/native_metrics.json)
