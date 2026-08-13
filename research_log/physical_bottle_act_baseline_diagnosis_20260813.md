# Physical bottle ACT baseline diagnosis

> Superseded dataset interpretation (2026-08-13): the operator's subsequent
> human audit established that this checkpoint was trained on a mixed-quality,
> malformed-segmentation view, not 25 clean expert demonstrations. The 0/56
> transition result below remains an observed property of that checkpoint, but
> the old `89,98,114,116` val4 split is retired for clean-model conclusions.
> See `research_log/physical_bottle_nominal16_training_audit_20260813.md` for
> the immutable-source nominal16 curation, corrected 99/102 boundaries, clean
> split, and controlled scratch/pretrained comparison.

Date: 2026-08-13 (Asia/Shanghai)

Audit base: `dev` at `c50a1c5f318ce54b92af591fb4943d014357e312`.
Scope: repository-owned offline analysis and deployment-adapter hardening only.
No device was opened and no command-enabled motion was run during this audit.

## Decision

The saved evidence does **not** support the hypothesis that grasp is repeatedly
predicted at chunk horizons 2--15 and discarded by `K=2`. The current val4
checkpoint instead emits almost time-constant 16-step chunks. On held-out
recorded grasp observations it predicts a closed hand immediately, but on all
56 held-out observations whose recorded action changes from approach/open to
closure within the next 2--15 steps it predicts that transition zero times.

The operator-observed later `approach -> no hand closure` attempts are
post-freshness-fix artifacts. Their command rows match the JAKA observation
timestamp attached to their policy query in 100% of non-startup commands. The
stale shared-socket consumer bug invalidates the first ACT and ACT+Force runs,
but cannot explain the later val4 behavior.

Current classification: **Case 3 / INVESTIGATE checkpoint training**, with a
real but already-fixed deployment freshness defect as a separate earlier
issue. Keep `consume_actions=2` for the next bounded post-fix validation; a
larger prefix has no grasp transition to expose in this checkpoint.

## Artifacts audited

All command-enabled artifacts under `outputs/act_physical_rollouts` were
preserved unchanged. Query/command rates below are computed from monotonic
timestamps, not requested configuration.

| rollout | q / cmd | q Hz / cmd Hz | unique / repeated query JAKA timestamps | JAKA age p50/p95/max ms | camera age max workspace/wrist ms | projection events | native result | freshness version |
|---|---:|---:|---:|---:|---:|---:|---|---|
| `20260812_act` | 451 / 901 | 15.000 / 30.000 | 449 / 2 | 16.13 / 31.77 / 93.15 | 34.08 / 34.06 | 76 | zero hard timing/tracking/alarm; cleanup timeout | pre-fix |
| `20260812_act_force` | 451 / 901 | 15.000 / 30.000 | 447 / 4 | 17.60 / 31.49 / 92.76 | 34.90 / 34.67 | 42 | one timing warning; no hard timing/tracking/alarm; cleanup timeout | pre-fix |
| `20260812_val4_act_60s` | 901 / 1801 | 15.000 / 30.000 | 901 / 0 | 16.85 / 32.36 / 32.97 | 34.60 / 35.15 | 1410 | one timing warning; completed | post-fix |
| `20260812_val4_act_multi_01` | 901 / 1801 | 15.000 / 30.000 | 901 / 0 | 17.25 / 30.87 / 33.48 | 34.58 / 34.33 | 2095 | completed | post-fix |
| `20260812_val4_act_multi_02` | 901 / 1801 | 15.000 / 30.000 | 901 / 0 | 16.44 / 31.54 / 31.94 | 35.67 / 35.77 | 1806 | completed | post-fix |
| `20260812_val4_act_multi_03` | 387 / 769 | 15.000 / 30.000 | 386 / 1 | 17.38 / 31.27 / 113.22 | 32.90 / 36.66 | 799 | stopped by existing J5 tracking-lag fault | post-fix |

The version attribution is behavioral, not based only on file modification
time. For each post-fix val4 run, every non-startup command's logged JAKA
timestamp exactly equals the status attached to its referenced policy query
(1800/1800, 1800/1800, 1800/1800, and 768/768). The two earlier runs match only
1/900 each because the command loop still consumed the shared status socket.

Workspace/wrist capture stayed near 30 Hz. Workspace frame-gap counts were
0, 0, 0, 1, 1, 1 respectively; wrist gaps were zero. All stored JAKA, RH56
ANGLE/FORCE, and camera host timestamps used by policy queries were no later
than `observation_ready_ns`. Policy deadline skips were zero. These checks do
not relabel any pre-fix physical trajectory as a task success or failure.

The command-disabled preflight/shadow directories were also inspected:
`20260812_preflight_act`, `20260812_preflight_act_force`, their result
directories, and `20260812_postfix_shadow_act_force`. The latest shadow has
300 finite `[16,12]` queries at 15 Hz, zero RH56 writes and zero stale-source
counts. It validates inference/input plumbing, but cannot physically validate
the command-worker socket fix.

## Action domains and RH56 projection

The physical path is:

```text
ACT network normalized tensor
  -> checkpoint postprocessor / inverse normalization
  -> saved native absolute [16,12] chunk
  -> selected native absolute [12] command
  -> final RH56 legal-domain gate in normalized closure [0,1]
  -> existing RH56 contact/delta safety
  -> calibration denormalization to RH56 raw ANGLE_SET register counts
```

`act_predictions.npz` starts after checkpoint postprocessing; the normalized
network tensor was not persisted in the old rollouts. The first six values are
absolute JAKA joint targets in radians. The last six are canonical normalized
RH56 closure targets ordered `index, middle, ring, pinky, thumb_close,
thumb_lateral`. The dataset, checkpoint processor, rollout adapter,
`RH56PcDirectWorker`, and `RH56PcDirectControl` use that same order. There is no
second inverse-normalization and no action-delta transform.

The five-demo source view consists of episodes 65, 66, 67, 69 and 70. Its
stored RH56 action minima/maxima are:

| channel | training min | training max | checkpoint prediction min | prediction max | below legal | above legal | maximum correction |
|---|---:|---:|---:|---:|---:|---:|---:|
| index | 0.0000 | 0.40986 | -0.01344 | 0.41454 | 19.06% | 0% | 0.01344 |
| middle | 0.0000 | 0.44750 | -0.01105 | 0.46626 | 16.42% | 0% | 0.01105 |
| ring | 0.0000 | 0.48678 | -0.01692 | 0.50465 | 5.10% | 0% | 0.01692 |
| pinky | 0.0000 | 0.40250 | -0.00738 | 0.40794 | 5.03% | 0% | 0.00738 |
| thumb_close | 0.0000 | 0.31250 | -0.01932 | 0.24271 | 7.28% | 0% | 0.01932 |
| thumb_lateral | 0.42320 | 0.94405 | 0.40068 | 0.94447 | 0% | 0% | 0 |

Fractions use every valid horizon from checkpoint inference over all 3,268
five-demo rows. These are common but small negative boundary regressions, not
large hidden domain changes. Across all 7,974 old physical commands, 6,228
channel projections occurred: index 113 (correction p50/p95/max
0.00160/0.01226/0.01372), middle 278
(0.00241/0.00595/0.00739), and thumb-close 5,837
(0.00396/0.00605/0.01451). No old physical correction exceeds 0.02 and no
channel exceeds the upper legal endpoint.

Git history contains the small-boundary projection itself but no persisted
approved numerical tolerance. The adapter now states a fail-closed maximum
correction of 0.02 in normalized RH56 command units. It preserves raw and
projected values, logs signed and absolute correction per channel, and rejects
NaN/Inf or any larger correction before either actuator command is published.
The existing RH56 legal, delta and contact limits remain authoritative. This
is classified as **small bounded regression extrapolation**, not a transform
bug and not evidence that the RH56 controller ignored a sent closing command.
The later rollout chunks themselves request an open hand.

## Chunk procrastination audit

`tools/analyze_act_bottle_rollout.py` loads the saved NPZ/JSONL/summary files
and generates per-channel horizon and execution plots under the ignored local
directory `outputs/act_bottle_diagnosis_20260813/`.

Answer to the requested question: **NO**. None of the six rollout artifacts
contains a query that is open at horizons 0--1 and grasp-like at a later
horizon under the documented diagnostic heuristic. More importantly, visual
inspection of all six native RH56 channels shows that h=0,1,2,4,8,15 nearly
overlap. The current 25-demo val4 checkpoint's held-out predicted mean
peak-to-peak range within a chunk is only 0.000123--0.000160 for RH56 and
0.000325--0.000418 rad for JAKA.

The heuristic uses all first five RH56 closure channels `>=0.1`; it is not a
semantic grasp label. Video contact sheets were reviewed as supporting context,
but no durable approach/grasp/lift overlay was added because operator
intervention and the lack of authoritative phase labels make those boundaries
ambiguous.

Offline latest-query replay gives the following worst p95 re-query jump across
the four post-fix val4 artifacts:

| K | nominal open-loop span ms | oldest used chunk index age ms | within existing 250 ms freshness gate | JAKA jump p95 max rad | RH56 jump p95 max |
|---:|---:|---:|---|---:|---:|
| 2 | 66.7 | 33.3 | yes | 0.00618 | 0.00113 |
| 4 | 133.3 | 100.0 | yes | 0.01245 | 0.00264 |
| 8 | 266.7 | 233.3 | yes, with little margin | 0.02507 | 0.00594 |
| 16 | 533.3 | 500.0 | **no** | 0.05005 | 0.01215 |

Larger K increases open-loop exposure and boundary discontinuity without
reaching a missing grasp transition. Recommendation: **K=2**. `K=16` is
incompatible with the unchanged policy freshness gate.

## Teacher-forced held-out diagnosis

The current val4 checkpoint was rerun through the actual pinned LeRobot 0.6.2
loader and checkpoint processors on source episodes 89, 98, 114 and 116
(2,680 rows). First-action MAE is 0.04776 rad for JAKA and 0.01628 for RH56.

| source episode | recorded closure rows | immediate predicted closure on those rows | recorded transition enters h2--15 | model predicts that later transition | RH56 chunk mean peak-to-peak |
|---:|---:|---:|---:|---:|---:|
| 89 | 307 | 100% | 14 | 0 | 0.000157 |
| 98 | 260 | 99.62% | 14 | 0 | 0.000123 |
| 114 | 219 | 100% | 14 | 0 | 0.000160 |
| 116 | 224 | 100% | 14 | 0 | 0.000147 |

Thus the model recognizes the already-closed/grasp observation and emits a
closed target immediately, but does not predict the impending transition from
recorded approach observations. Its RH56 range is not globally collapsed;
instead its temporal chunk is effectively the current phase/action repeated.
This creates a plausible closed-loop fixed point: at approach, the policy waits
for grasp-state evidence that its own command must create. Scalar train/eval
loss alone did not reveal this failure.

The original five-demo checkpoint is somewhat better but still weak: it
predicts only 13 of 70 recorded approach-to-closure transitions in h2--15. That
offline result also rules out a universal deployment mapping defect.

## Deployment changes

- `--consume-actions K` now accepts 1--16 and defaults to 2. A bounded consumer
  finishes the configured prefix before adopting the newest available chunk.
- Every rollout summary records `consume_actions`, consumed indices, skipped
  predictions, runtime config and model-container selection.
- New rollouts persist every query chunk with sequence numbers; query rows now
  include the exact 12-D state and command rows include measured RH56 position.
- The RH56 final projection is bounded and fail-closed as described above.
- `evaluate-checkpoint` provides reproducible held-out inference through the
  real LeRobot loader/processors.

No teleoperation, collection, camera, native JAKA, RH56 safety, raw/master
dataset or checkpoint code was changed.

## Training configuration review

The B0 pilot uses uninitialized ResNet18, 2,000 optimizer steps, batch 16,
chunk 16, model width 256, feed-forward width 1024, KL weight 1, base/backbone
LR `1e-4`, and no augmentation. A step is one optimizer update, not an epoch.
With 18,064 training rows and batch 16, 2,000 steps expose about 1.77 nominal
passes over the row count (sampling/chunk details mean this is only an
approximation).

Pinned LeRobot 0.6.2 defaults differ materially: ImageNet ResNet18 weights,
chunk/action steps 100, model width 512, feed-forward width 3200, KL weight 10,
and base/backbone LR `1e-5`. The encoder/decoder depth (4/1), heads (8), VAE
latent (32), dropout (0.1), and disabled temporal ensemble match.

`B0-strong-pretrained` is deliberately controlled: it keeps the val4 data,
both cameras, native 12-D state/action, chunk 16 and 256/1024 transformer, but
uses cached ImageNet ResNet18 initialization with backbone LR `1e-5`. The
cached file `resnet18-f37072fd.pth` has pinned SHA-256
`f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec`;
training remains in the pinned container with network disabled. Stages are
2k, reviewed resume to 5k, then reviewed resume to 10k. Continue only while
validation and grasp-transition diagnostics improve. The larger 512/3200
architecture remains deferred.

## Ranked causes and gates

1. **Checkpoint phase-transition failure / weak visual-temporal learning —
   strongest current cause.** Proven by 0/56 held-out transition predictions
   and nearly constant chunks. `B0-strong-pretrained` stage 2k: **GO**.
2. **JAKA status socket-consumer defect — confirmed historical cause.** It
   invalidates the two first-rollout artifacts, but the later observed stall is
   post-fix. One clean bounded command-enabled K=2 validation: **GO, operator
   only**.
3. **K=2 receding-horizon procrastination — refuted for this checkpoint.** No
   hidden later grasp action. K>2 physical test: **KILL for current B0**.
4. **RH56 projection/mapping — not the observed cause.** Small extrapolation is
   now bounded; recorded grasp observations produce valid closed targets.
   Mapping investigation is warranted only if a future rollout logs a closing
   projected target while measured RH56 position does not follow.
5. **Large model-domain excursion — not observed.** Maximum audited correction
   is 0.01932 offline and 0.01451 in physical commands. Any future correction
   above 0.02 fails closed.

`B0-pilot`: **INVESTIGATE / not a validated physical baseline**. Existing
pre-fix rollouts are not task evidence; post-fix runs show operational timing
but checkpoint behavior is inadequate at the grasp transition. Do not claim
the system or policy physically fixed until the operator-run validation passes.

## Exact next operator-run validation

After the operator independently verifies the controller payload/COM,
installation/TCP, unchanged controller safety settings, scene reset, emergency
stop access, and the configured camera/RH56 identities, run exactly one bounded
post-fix B0 pilot with the unchanged `K=2` schedule:

```bash
timestamp="$(date +%Y%m%d_%H%M%S)"
PYTHONPATH=src .venv/bin/python tools/act_physical_rollout.py \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --checkpoint outputs/training/physical_bottle_v2/act_val4_run/checkpoints/002000/pretrained_model \
  --model-container jaka-lerobot-dev:snapshot-before-raw-mount \
  --output "outputs/act_physical_rollouts/${timestamp}_postfix_val4_act_k2" \
  --duration-sec 60 \
  --query-rate-hz 15 \
  --command-rate-hz 30 \
  --consume-actions 2 \
  --label UNLABELED \
  --notes "single operator-run post-freshness-fix K=2 validation"
```

This command is documented for the operator; it was not executed in this
audit. Stop on any unexpected motion, hardware-limit rejection, watchdog,
controller alarm, tracking fault, or projection-tolerance abort.

Offline reproduction commands:

```bash
.venv/bin/python tools/analyze_act_bottle_rollout.py rollout \
  --rollout outputs/act_physical_rollouts/20260812_val4_act_multi_01 \
  --output outputs/act_bottle_diagnosis_20260813/example_rollout

scripts/train_physical_bottle_lerobot.sh strong-pretrained 2000
```

The training command is prepared and validated but was not launched by this
audit.

## Validation performed

- New and directly related ACT/LeRobot tests: **30 passed, 4 skipped**. The
  skips are optional installed-LeRobot integration cases in the host venv; the
  pinned container checkpoint replay above exercised the real LeRobot 0.6.2
  path.
- An earlier focused run that also included native JAKA fake-worker tests:
  **57 passed, 4 skipped**.
- Full repository run: **670 passed, 4 skipped, 14 failed**. No failure touched
  changed code. All failures were native fake-worker hard timing outcomes while
  an unrelated, externally-owned 100k-step LeRobot training process was active
  and host load average reached about 29. An isolated retry under the same load
  produced 58 passes and 21 timing failures. These are not claimed as passing
  and that external process was not stopped or modified.
- `compileall`, shell syntax checks, CLI help checks, pinned-container strong
  config decoding, cached-weight checksum, and `git diff --check`: passed.
- Independent counterexample review confirmed that a stalled query cannot use
  more than K configured actions indefinitely without the unchanged 250 ms
  freshness gate aborting; `K=16` necessarily violates that gate. Exact legal
  endpoints and exact 0.02 corrections pass, while corrections of 0.020001,
  NaN and Inf fail closed. No new telemetry path bypasses existing JAKA/RH56
  safety.

No new live command-disabled check was started because this audit did not open
hardware. The strongest available no-motion evidence is the preserved 300-row
post-fix shadow plus the new full checkpoint/data replay.
