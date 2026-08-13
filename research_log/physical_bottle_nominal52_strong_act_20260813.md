# Physical bottle nominal52 strong ACT baseline

Date: 2026-08-13

Repository commit at start: `a3fcf4ac4bcf87fc2a9bc0d334bf00d01ce68667`

Validation level: offline dataset audit and training smoke passed; 100k training
is in progress; no physical rollout was performed.

## Dataset and fixed split

The source is the immutable, task-trimmed
`data/training/physical_bottle_v4_nominal52` master. The existing validation
report passes finite schema, timestamp causality, both-camera decode/alignment,
ACT/ACT+Force identity, source provenance, raw hash preservation, and logical
episode boundaries. The generated LeRobot 0.6.2 ACT view independently loaded
all 33,111 rows and 52 episodes with a `[60,12]` action chunk; every episode's
last row had exactly one valid action followed by repeat-last padding. No chunk
crosses a logical trajectory or a 172/174 reset gap.

The split is locked in
`configs/training/physical_bottle_v4_nominal52_split.yaml`. The LeRobot view is
ordered train first and validation last because pinned LeRobot 0.6.2 holds out
the final `ceil(52 * eval_split) = 15` episodes. The configured episode ratio
is therefore `15/52`, not the row ratio.

| split | trajectories | rows | duration | acquisition sessions |
|---|---:|---:|---:|---|
| train | 37 | 23,802 | 792.300 s | 13 |
| validation | 15 | 9,309 | 309.800 s | 4 |
| total | 52 | 33,111 | 1,102.100 s | 17 |

Validation sessions are `session_87_88`, `session_108_109`,
`session_138_142`, and `session_168_174`. All segments from source 172 and all
retained segments from 174 stay together in `session_168_174`; the same is
true for source 99 in training. There is no source/session leakage. Exact
trajectory identities and frame/timestamp ranges are recorded in
`outputs/training/physical_bottle_v4_nominal52/analysis/horizon_coverage.json`.

## Episode 161 audit

Raw episode 161 has 535 rows and ends with metadata
`completion_status=invalid`, `quality_state=aborted_robot_safety`, and
`termination_reason=control_source_timestamp_regression:jaka_command`. The
nominal view keeps source frames 1--519 (17.267 s). The warning is retained; it
was not erased or relabeled.

The retained rows are finite, source timestamps are monotonic/causal, and the
workspace/wrist videos decode. Representative synchronized evidence is:

| source frame | task time | visual/motion evidence | RH56 target summary |
|---:|---:|---|---|
| 1 | 0.00 s | task start; bottle ungrasped | open |
| 80--140 | 2.63--4.63 s | arm approach | open |
| 193--220 | 6.40--7.30 s | grasp closes around bottle | closure rises to `[.474,.577,.605,.598,.314]` on the five closing channels |
| 260--380 | 8.63--12.63 s | bottle lift/transport/placement | held closed |
| 440--480 | 14.63--15.97 s | bottle released on target, arm withdraws | returns near open |
| 506 | 16.83 s | grip clutch release | near open |
| 519 | 17.27 s | both clutches released; retained crop ends | held target |
| 520--534 | 17.30--17.77 s | stationary post-task tail | no target change |

The metadata fault occurs at finalization after the retained task. The sparse
event record at monotonic timestamp `2502602107862058` reports a normal
`DISENGAGED` / `duration_complete` boundary and no active arm or hand fault.
The retained crop ends at `2502602135394974`, while the raw termination follows
about 0.5 s later. This supports keeping episode 161 as operator-audited usable:
there is no observed abnormal motion, incomplete task, non-finite target/state,
or safety intervention inside frames 1--519. This does not invalidate the raw
warning; a future audit should revisit 161 if more detailed fault telemetry
contradicts the available evidence.

## Strong ACT contract

The new config is
`configs/training/lerobot/act_physical_bottle_v4_nominal52_strong.json`; it does
not overwrite any pilot config.

| component | value |
|---|---|
| input | workspace RGB, wrist RGB, 12-D measured state |
| target | native absolute 12-D action |
| backbone | ImageNet `ResNet18_Weights.IMAGENET1K_V1`, pinned cached SHA-256 `f37072fd...e07ec` |
| chunk | 60 actions (about 2.0 s at 30 Hz) |
| transformer | model 512, 8 heads, FFN 3200, encoder 4, decoder 1, dropout 0.1 |
| VAE | enabled, latent 32, encoder 4, KL weight 10 |
| optimizer | AdamW, policy/backbone LR `1e-5`, weight decay `1e-4` |
| batch / steps | 16 / 100,000 |
| checkpoints/eval | every 10,000 steps, including 10/20/40/60/80/100k |
| image augmentation | disabled, controlled first baseline |

The historical 1k/2k runs used only 16k/32k sampled examples. On nominal52,
2k steps at batch 16 are only 1.34 train-dataset-equivalent sampled passes
(`32,000 / 23,802`), so they remain pilot diagnostics rather than evidence
against ACT.

Pinned LeRobot 0.6.2 makes `chunk_size` define the action delta timestamps and
therefore the training target and decoder output shape. `n_action_steps` does
not affect target construction or architecture: `ACTPolicy.select_action`
queues only `actions[:, :n_action_steps]`. The custom physical rollout calls
`predict_action_chunk` and has its own `--consume-actions`; therefore the
strong config records `n_action_steps=2` as a safe inference default, while
the 60-step prediction horizon cannot silently make the robot execute 60
open-loop commands. Physical execution remains separately reviewed and was not
run here.

## Temporal supervision coverage

Grasp onset is an objective action proxy: the first sustained five-closing-
channel RH56 action displacement above `max(0.10, 25% of trajectory peak)`
relative to the lowest-closure 15-row baseline. Release is the final persisted
`hand_grip` true-to-false edge after grasp. These are reproducible proxies, not
semantic contact labels.

All 52 trajectories have detected grasp and release proxies. There are 13,712
approach rows and 19,399 rows at/after grasp. Grasp onset is 5.30--22.80 s
(median 7.92 s); release is 6.83--32.17 s (median 17.53 s), demonstrating real
timing variation.

| chunk | future grasp queries | fraction of rows | grasp after h0/1 | future release queries | padded action slots |
|---:|---:|---:|---:|---:|---:|
| 16 | 780 | 2.36% | 728 | 780 | 1.18% |
| 32 | 1,612 | 4.87% | 1,560 | 1,612 | 2.43% |
| 60 | 3,068 | 9.27% | 3,016 | 3,068 | 4.63% |

Chunk60 supplies about four times as many pre-grasp transition-supervised
observations as chunk16 while keeping boundary padding modest. This supports
training the requested chunk60 model first. A 16/32/60 training ablation is not
yet justified; reconsider it only after checkpoint behavior is known.

## Training and checkpoint gate

The pinned Docker image and cached backbone passed an offline two-step backward
smoke with batch 16, finite loss, 52M trainable parameters, and about 2.08 GB
peak allocated GPU memory. Eight data workers were selected after a controlled
200-step replay: with identical seed/model/data, the step-200 metrics exactly
matched the two-worker run while decoder wait fell from 1.043 s to 0.207 s and
throughput rose from 13 to 46 samples/s. This is a loader-only throughput
change.

The detached 100k training was started with:

```bash
scripts/train_physical_bottle_lerobot.sh strong-act
```

Runtime PID and logs are under
`outputs/training/physical_bottle_v4_nominal52/logs/`; checkpoints and generated
views are ignored and must not be committed. After training, evaluate every
saved checkpoint with:

```bash
scripts/evaluate_physical_bottle_nominal52_checkpoints.sh
```

Checkpoint selection will use fixed-validation aggregate loss, JAKA/RH56
errors, transition recall/timing, per-session recall, RH56 within-chunk range,
phase-conditioned errors, and copy-current-state comparison. A checkpoint is
not a physical candidate if transition recall is concentrated in one held-out
session, chunks collapse, outputs are non-finite/out of deployment contract,
or deployment freshness checks fail.

No final checkpoint is selected yet, and there is not yet evidence that this
model learns the full approach--grasp--lift--place--release sequence offline.
No physical command is prepared until the checkpoint gate passes.
