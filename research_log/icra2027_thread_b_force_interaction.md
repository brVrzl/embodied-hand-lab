# ICRA 2027 Thread-B: FORCE / interaction / underactuation continuation

Date: 2026-08-13 (Asia/Shanghai)

Branch: `research/thread-b-force-interaction`

Continuation base: `0dc8bf989e610c616517691a79097b516c5775dc`

Validation level: repository-owned offline analysis only; no physical operation

## PREVIOUSLY COMPLETED

- The prior usage-limited session had audited the mixed-quality
  `physical_bottle_v2` master, recovered physical collection-session groups,
  established the canonical RH56 channel order, and run a first
  leave-one-session-out (LOSO) raw-load probe.
- It had established that the 30 Hz dataset rows carry approximately 10 Hz
  zero-order-held `FORCE_ACT` sources. Histories therefore must count unique
  source timestamps, not repeated rows.
- It had prepared a signal-blind RGB annotation schema/protocol and most of an
  offline ACT force-intervention tool. Those files were uncommitted; there was
  no earlier Thread-B commit or isolated Thread-B worktree.
- The ignored prior result was recovered from
  `outputs/research/exploration/force_information_20260813/`. Its conclusions
  remain mixed-quality diagnostic evidence only.

## PARTIALLY COMPLETED

- The earlier force-information probe had 3- and 5-update histories but no
  reusable derived-history adapter, no 0.5/1.0 s candidates, and incomplete
  classifier reporting.
- The ACT intervention core used row-count lags and dropouts. That implicitly
  treated cached 30 Hz rows as force timing and had no tests.
- Policy-use analysis could not yet run scientifically: the only force-aware
  checkpoint was trained on the old mixed-quality, session-leaky val4 cohort.
- Semantic interaction, stable/unstable, failure, and slip labels had not been
  produced. Therefore the earlier numeric load-change probe could not answer
  those questions.

## NEWLY COMPLETED

- Continued on the isolated branch above without merging into `dev`. The
  linked broad-research worktree was inspected and left untouched.
- Consumed Thread A's canonical nominal16 master read-only once it appeared;
  no competing segmentation or materialization code was created. The exact
  materialization fingerprint is
  `9b5792f37d9c513d86854038e52051bdde9d1322e988853ce6b1086c2f6a6c72`.
- Added a causal `ForceHistory` adapter that deduplicates actual source
  timestamps before constructing oldest-to-newest histories, and exposes raw
  load, delta, source age, present/valid masks, initial-offset-corrected load,
  and actuator-load magnitude share.
- Added deterministic tests for no-future leakage, cached-row deduplication,
  episode boundaries, channel order, missing/invalid samples, session split
  integrity, and policy-intervention timing.
- Reran the low-complexity probe on (a) canonical nominal16, (b) the old full
  mixed cohort, and (c) ten source trajectories labeled only as generic
  `MANUAL_AUDIT_NON_NOMINAL`. Results use acquisition-session LOSO.
- Completed command-disabled policy-intervention infrastructure for factual,
  training mean, raw zero, causal time shift, same-session episode shuffle,
  q-conditioned load, channel permutation, and timed load dropout. It has not
  been run against a checkpoint in this continuation.
- Prepared, but did not execute, a sequential controlled physical
  observability protocol with explicit positive/negative evidence and stopping
  rules.

## BLOCKED

- A clean nominal16 `ACT + Instantaneous Actuator Load` checkpoint on the same
  split as standard ACT does not yet exist. The old ACT+load checkpoint is only
  a mixed-quality pilot/diagnostic checkpoint.
- No locked RGB interaction/stability/failure annotations exist. Lift/place
  alignment and semantic classifiers are therefore blocked; action-derived
  closure heuristics are not substituted for physical labels.
- No retained, defensibly labeled slip/drop or impending-instability payload
  exists. Stability, slip-warning, causal lead-time, and correction claims are
  blocked on targeted labels/trials.
- Generic non-nominal labels do not identify collision, stall, grasp failure,
  or any other mechanism. Individual excluded trajectories remain
  `UNKNOWN / NEEDS MANUAL LABEL` beyond the generic audit label.
- Thread A's nominal16 files were still concurrent worktree changes during
  this continuation. The fingerprint and hashes below must be revalidated
  after Thread A's canonical commit before a final paper experiment is frozen.

## NEXT

1. Lock RGB-only labels on nominal16 using the existing annotation protocol,
   then run simple q/load/history probes with acquisition-session grouping.
2. With separate physical authorization, execute Gate 0 and Gate 1 of the
   controlled sidecar protocol; stop immediately if q-conditioned interaction
   observability fails.
3. Train matched standard ACT and `ACT + Instantaneous Actuator Load` on the
   same canonical nominal16 split, then run the prepared offline interventions.
4. Try the 0.2 s causal history only if a semantic probe or clean policy
   baseline justifies it; do not prioritize 0.5/1.0 s history now.
5. Do not build a corrective supervisor until an independently labeled state
   passes grouped validation.

## 1. What `FORCE_ACT` actually measures

The maintained PC-direct RH56 driver reads six signed 16-bit values from raw
register `FORCE_ACT` at address 1582. The driver converts protocol order

```text
pinky, ring, middle, index, thumb_close, thumb_lateral
```

to the repository's canonical order

```text
index, middle, ring, pinky, thumb_close, thumb_lateral.
```

The dataset stores these as float32 containers with units
`rh56_force_act_raw_count`. In this work they are described only as native
actuator/push-rod load feedback. They are not calibrated newtons, joint torque,
fingertip force, tactile-array measurements, full passive-joint state, contact
location, or slip ground truth.

The six measured RH56 positions are `ANGLE_ACT` feedback for the six commanded
actuator axes. They are not all passive underactuated joint angles. Thus
position-conditioned load can be scientifically useful, but residual load is
not automatically proof of object contact or underactuated contact geometry.

## 2. Timestamp, polling, validity, and synchronization audit

The canonical nominal16 audit contains 12,177 rows and 4,069 episode-local
unique load-source updates:

| Property | Canonical nominal16 result |
| --- | ---: |
| Dataset row rate | median 30.0000003 Hz |
| Native recorded force-source rate | median 9.9994 Hz |
| Rows per unique force source | 2.993 |
| Repeated cached rows | 8,108 |
| Force interval p50 / p95 / p99 / max | 100.02 / 103.33 / 107.10 / 119.47 ms |
| Force age p50 / p95 / p99 / max | 59.24 / 110.44 / 117.75 / 125.59 ms |
| Valid / invalid force rows | 12,177 / 0 |
| Future-source rows | 0 |
| Accepted / held-rejected rows | 11,159 / 1,018 |

The old mixed materialization independently gives the same conclusion: 20,744
rows, 6,927 unique source updates, 2.995 rows/source, 10 Hz source rate, force
age p50/p95/max 58.65/111.36/130.07 ms, and zero invalid or future-source
rows. Its 2,104 held-rejected rows (10.14%) are a potential analysis confound;
nominal16 has 8.36% and the generic non-nominal cohort 12.38%.

`ForceHistory` now counts actual unique sources. Candidate widths of 3, 6, and
11 sources have measured median spans of 199.98, 499.97, and 999.94 ms in
nominal16. The corresponding 5th--95th percentile spans are
196.04--203.89, 495.06--505.29, and 993.81--1006.43 ms. No 30 Hz row is
misrepresented as a fresh force observation.

Verdict: **GO** for the timestamp/causality contract and offline history
construction. Native physical sensor bandwidth remains unresolved; the
production 10 Hz polling decision is unchanged.

## 3. Existing-data observability

### Canonical nominal16 descriptive structure

The 16 nominal trajectories span seven physical acquisition sessions. The
initial ten-source median is used only to remove each trajectory's offset; it
is not called a no-contact baseline.

- Baseline-corrected six-channel magnitude has median 17.72 and p95 674.73 raw
  counts. Per-source absolute delta medians are 0, 0, 0, 0, 0, and 1 count;
  p95 values are 21, 11, 19, 3, 35.6, and 33 counts.
- Raw centered-load PCA explains 72.5% in PC1 and 88.2% in the top two PCs.
  Delta PCA explains 54.6% in PC1 and 74.6% in the top two. The signal is
  low-dimensional, but PCA does not assign physical meaning to its axes.
- Cross-session normalized-progress trace correlations are uneven by channel:
  `[0.624, 0.293, 0.024, -0.033, 0.725, 0.427]`. Index and thumb-close are
  most repeatable under this crude, non-phase-aligned comparison; ring and
  pinky are not.
- Linear reconstruction of one centered channel from the other five gives
  LOSO mean R2 `[0.631, 0.035, 0.125, -2.307, 0.472, 0.281]`. Some channels
  cannot be dropped merely because six channels appear correlated, but this
  instability also argues against a premature one-channel sensor claim.
- At similar measured-position deciles, q-increasing and q-decreasing updates
  show nonzero load differences. Direction, time, task phase, and interaction
  are confounded, so this is only a hysteresis candidate.

An older ignored RH56-only active-grasp artifact provides a controlled-command
pilot: `force_reads.jsonl` has 1,376 reads and ten complete
baseline/closing/grasp-hold/opening/open-hold cycles (3 each at 10/15/20 Hz and
1 at 30 Hz before a guard abort). Across cycle medians, baseline-corrected load
L2 is 0.0 in baseline, 32.7 during closing, 33.4 in grasp hold, 3.7 during
opening, and 3.3 in open hold. Nearest-q closing/opening comparisons have a
median load-vector difference of 8.0 counts across cycles (cycle range
3.4--15.7) at median normalized q distance 0.0153. Because rate blocks were
sequential, drift occurred, and there is no matched no-contact control, this
cannot establish polling-rate benefit, contact sensing, or physical
hysteresis. It only justifies the controlled sidecar.

Verdict: **INVESTIGATE** repeatable residual/redistribution structure with
controlled conditions. **KILL** any interpretation of PCA or channel
separability as contact localization.

## 4. Does load add information beyond measured hand position?

### Current load from q

A quadratic q-only ridge model predicting current centered load under
acquisition-session LOSO is unstable on canonical nominal16:

- mean pooled R2 `0.017 +/- 1.213`;
- median pooled R2 `0.503`;
- six of seven held-session folds range from 0.215 to 0.728, while the singleton
  episode-102 session is -2.926;
- adding target, q/target error, and direction does not help (mean -0.074,
  median 0.501).

This supports two narrow conclusions. Hand configuration often explains a
substantial portion of load within familiar sessions, but the relationship is
not session-invariant. Large cross-session residual structure remains. The
top two PCs explain 81.4% of cross-fitted q/target/direction residual variance.
That residual may include object interaction, underactuation, direction,
baseline drift, setup, and other unmodeled factors; it has no semantic label.

### Future numeric load dynamics

The probe predicts load two actual source updates ahead and classifies whether
the future load-vector change exceeds a threshold fit on training sessions
only. The binary target is a top-quantile raw-count redistribution event, not
contact, stability, failure, or slip.

Canonical nominal16 accepted-query LOSO results are:

| Features | Future-load R2 | numeric-change AUROC | balanced acc. | precision | recall | false-positive rate |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| q now | 0.269 | 0.747 | 0.716 | 0.374 | 0.761 | 0.329 |
| load now | 0.874 | 0.780 | 0.722 | 0.602 | 0.570 | 0.126 |
| q + load now | 0.875 | 0.845 | 0.748 | 0.526 | 0.706 | 0.211 |
| q + 0.2 s history | 0.892 | 0.851 | 0.754 | 0.533 | 0.728 | 0.221 |
| q + 0.5 s history | 0.884 | 0.847 | 0.759 | 0.574 | 0.699 | 0.181 |
| q + 1.0 s history | 0.857 | 0.841 | 0.771 | 0.598 | 0.688 | 0.147 |

The pooled confusion matrix for q + instantaneous load is TN 2,351, FP 517,
FN 221, TP 490. The improvement from q-only AUROC 0.747 to q+load 0.845 is
evidence that the latest causal actuator load carries short-term load-dynamics
information beyond measured q. Much of the future-load R2 is expected temporal
persistence, so it is not by itself a manipulation contribution.

Verdict: **GO** for the scientific question "does raw actuator load contain
short-term dynamics not captured by q?" **INVESTIGATE** semantic interaction
observability. No contact or stability claim follows from this target.

## 5. Instantaneous load versus history

History is not yet the winner. Relative to q + instantaneous load, q + 0.2 s
history changes numeric-change AUROC by only +0.006 and future-load R2 by
+0.017. The 0.5 s and 1.0 s histories do not improve AUROC, and 1.0 s degrades
future-load R2. Load-only history shows a similarly modest peak at 0.2 s.

This result kills the assumption that "more history must be better." It does
not kill history for a semantic transition label, where derivatives or
hysteresis may matter differently.

- 0.2 s history: **INVESTIGATE** as the only justified first candidate.
- 0.5 s and 1.0 s history: **HOLD**; do not train large temporal models now.
- Large temporal architecture: **KILL** until a simple grouped semantic probe
  saturates.

The adapter can also expose source age, validity/presence, delta,
initial-offset-corrected load, and normalized actuator-load magnitude share.
The last term is not a fingertip-force distribution.

## 6. Nominal versus generic non-nominal observations

Thread A's audit supplies ten full-source trajectories labeled only
`MANUAL_AUDIT_NON_NOMINAL` that can be read from the old mixed master without
inventing a failure mechanism. Episode 102_a was omitted from this comparison
because the old materialization's boundary differs from Thread A's canonical
boundary.

The pooled generic non-nominal cohort has baseline-corrected load L2 p95 976.6
counts versus 674.7 in canonical nominal16. That is a hypothesis-generating
tail difference, not a classifier result. It is not consistent even in the
only two acquisition sessions containing both labels:

- session 87/88: nominal episode p95 values 752/465; generic non-nominal
  530/623/453/411;
- session 108/109: nominal 752/561; generic non-nominal 948/629.

The sign therefore changes by session, and broad labels are also confounded by
duration, task phase, collection setup, and held-rejected frequency. No
nominal/non-nominal classifier was reported. No excluded episode was assigned
a specific failure label.

Verdict: **HOLD** abnormal-interaction classification. First apply the blinded
annotation protocol and use `UNKNOWN / NEEDS MANUAL LABEL` where direct RGB is
insufficient. The current pooled magnitude difference is not paper evidence.

## 7. Physical observability experiments still required

`research/exploration/CONTROLLED_OBSERVABILITY_PROTOCOL.md` specifies a
sequential, sidecar-only experiment. It was not executed.

- Gate 0: three independent setup blocks, two no-contact matched hand cycles
  per block, to measure drift and q/direction hysteresis.
- Gate 1: in each block, two randomized repetitions of the same bounded hand
  trajectory under `FREE_MOTION`, `FINGERTIP_REGION_CONTACT`,
  `MIDDLE_DISTAL_PHALANX_CONTACT`, `SIDE_CONTACT`, and `BLOCKED_CLOSURE`.
- Gate 2, only after Gate 1 passes: stable bottle hold and bounded controlled
  pull-tendency trials with independent event timing and a safety catch.
- Gate 3, only after Gate 2 passes: paired object-mass and optional controlled
  friction transfer.

All comparisons hold target trajectory fixed, group by setup block, and begin
with logistic/linear models and random forest. A controlled-condition
classifier is not called contact localization. If interaction versus free
motion does not improve over q on held-out blocks, kill the supervisor and
contact-condition directions.

Verdict: **GO** to Gate 0/1 only after a separately authorized physical
session. No JAKA motion or RH56 command was issued here.

## 8. Direct policy fusion versus a supervisor

No architecture winner is supported.

Direct fusion is the cheaper next baseline because the repository already has
an instantaneous six-dimensional ACT input path. However, its old 0.1513 eval
loss versus standard ACT's 0.1527 came from a mixed-quality, session-leaky
four-episode validation split and is scientifically indeterminate. Thread A's
0/56 approach-to-closure diagnostic and collapsed RH56 chunk dynamics also
come from the old mixed-quality checkpoint. Neither result is final clean-data
force evidence.

The completed intervention tool can test whether a force-aware checkpoint
actually uses load, rather than merely accepting the input. It verifies exact
paired rows/videos, force order, checkpoint hashes, and split identity before
running in a command-disabled, network-disabled container. Time shift and
dropout are specified in seconds/source timestamps rather than 30 Hz force
frames. Execution is **HOLD** until matched nominal16 standard and
instantaneous-load checkpoints exist.

A lightweight supervisor is potentially more interpretable, but it currently
has no reliably observable semantic state. It must not be built from the
numeric raw-load-change target. Interaction confirmation is **INVESTIGATE**
after annotation/controlled trials; grasp confirmation and instability warning
are **HOLD**; bounded hand correction is **KILL for now** until a supervisor has
causal lead time and acceptable false-positive behavior.

Current decision: prioritize the clean instantaneous direct-fusion baseline
as a controlled baseline, while using simple supervisor-style probes as the
scientific observability gate. Do not commit to either as the paper's final
method yet.

## 9. Supported force claims

The current evidence supports only these statements:

1. `FORCE_ACT` is six-channel native RH56 actuator/push-rod load feedback in
   canonical order, stored as signed raw counts.
2. The bottle masters have 30 Hz canonical rows but approximately 10 Hz causal
   load sources; most force values are cached across about three rows.
3. In canonical nominal16, load trajectories contain repeatable structure in
   some channels and low-dimensional covariance/redistribution structure.
4. Measured q explains load inconsistently across sessions; large
   position-conditioned residual structure remains.
5. Instantaneous load improves prediction/classification of a future numeric
   raw-load change beyond q under acquisition-session LOSO.
6. On that numeric target, 0.2--1.0 s history provides no compelling advantage
   over instantaneous load.
7. Generic non-nominal trajectories merit further annotation, but pooled load
   magnitude is not a repeatable abnormality detector.

## 10. Claims prohibited by current evidence

Do not claim any of the following:

- fingertip force, calibrated force/torque, dense tactile sensing, or contact
  localization;
- observed contact state, stable grasp, unstable grasp, slip, impending slip,
  or causal instability lead time;
- that nominal16 load signatures identify task phase, lift, place, or release
  without locked labels;
- that ForceHistory improves manipulation or is superior to instantaneous
  load;
- that the old ACT+load pilot improves policy performance or uses load
  causally;
- that generic non-nominal episodes are collision, stall, weak-grasp, or other
  specific failures;
- that higher polling exposes higher native sensor bandwidth; or
- that sparse intrinsic physical feedback is sufficient relative to tactile
  sensing.

## Ranked directions

| Rank | Direction | Hypothesis | Existing evidence | Missing / cheapest decisive experiment | Implementation | Paper value | Risk | Verdict |
| ---: | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | Blinded semantic observability on nominal16 | Load/residual load improves visible interaction or closure-transition state beyond q | Strong raw-dynamics gain beyond q; annotation contract ready | RGB-only lock, then q/load/0.2 s history LOSO; kill if no gain or labels unobservable | Ready for annotation and simple probes | High: establishes what the sensor tells us | Visual contact may be occluded | **GO** |
| 2 | Controlled matched-trajectory sidecar | Physical interaction conditions create residual signatures beyond q/direction | Low-dimensional residuals and hysteresis candidates | Gate 0/1, 3 blocks x 2 repeats; kill if effect is drift/q | Protocol/config ready, not executed | Very high: cleanest observability evidence | Requires a small authorized physical session | **GO** after authorization |
| 3 | Clean instantaneous-load ACT plus causal-use ablation | A matched clean policy uses load near interaction transitions | Only mixed pilot exists; no scientific policy evidence | Train on identical nominal16 split; run prepared interventions | Intervention infrastructure ready; checkpoint blocked | High if action sensitivity aligns with labels and task benefit | Policy may ignore load; baseline may remain weak | **INVESTIGATE** |
| 4 | Short causal history / underactuation residual | About 0.2 s of residual redistribution helps semantic transitions | Numeric target: +0.006 AUROC over instantaneous; residual top-2 PCA 81.4% | Test only after Rank 1/2 label gate; compare q, instant, 0.2 s | Causal adapter/tests ready | Medium to high if a simple, interpretable gain appears | Current history advantage is negligible | **INVESTIGATE** (0.2 s); **HOLD** longer |
| 5 | Failure-aware interaction supervisor | Load warns of an independently labeled unstable/failure transition with lead time | No defensible slip/drop payload; broad non-nominal tail is inconsistent | Targeted stable/disturbance trials; report AUROC, lead time, false alarms | Annotation schema ready; model/controller absent by design | Potentially high but downstream | Labels and rare events may kill it | **HOLD** |

Other decisions:

- one/few-channel or sparse-rate claim: **HOLD** until a semantic task exists;
- large temporal architecture: **KILL** now;
- contact-region localization claim: **KILL** (controlled-condition
  discrimination, if found, must remain narrowly named);
- data-efficiency curves: **HOLD** until matched clean policy baselines exist.

## Experiment dependency graph

```text
Thread A canonical nominal16 + session split
  +---> RGB-only locked annotations
  |       +---> q / instant-load / 0.2 s-history semantic probes
  |               +-- pass ---> interaction-state supervisor probe
  |               +-- fail ---> KILL supervisor/history novelty
  |
  +---> matched ACT and ACT+Instantaneous-Load checkpoints
          +---> offline force interventions
                  +-- meaningful, label-aligned sensitivity ---> rollout ablation
                  +-- negligible sensitivity -----------------> KILL fusion claim

Timestamp/source audit ---> causal ForceHistory adapter ---> every history probe/view

Controlled Gate 0 (free-motion drift/hysteresis)
  +-- pass ---> Gate 1 matched interaction conditions
                 +-- pass ---> underactuation/sparse-channel/rate ablations
                 +-- fail ---> KILL interaction-observability claim

Independent stable/disturbance labels
  +---> causal lead-time + false-alarm test
          +-- pass ---> bounded supervisor design
          +-- fail ---> KILL instability-warning/correction direction

Generic non-nominal sources ---> blinded manual event labels
  +-- no labels ---> remain UNKNOWN / descriptive only
```

## What can be tested when

| Availability | Experiments |
| --- | --- |
| Existing data, immediate | Nominal16 timestamp/signal/residual probes (done); blinded RGB annotation; q/load/history semantic probes; exact paired-view identity audit |
| Existing data after clean checkpoint | Zero/mean/shuffle/time-shift/channel-permutation/q-conditioned/dropout policy interventions |
| Few targeted physical trials | Free versus controlled contact conditions, blocked closure, stable hold, bounded pull tendency, mass/friction transfer |
| New full demonstrations | Not currently justified for Thread-B; first use controlled sidecar trials and existing nominal16 |
| Currently unjustified | Slip prediction, stability correction, contact localization, tactile equivalence, large temporal models, sparse-feedback paper claim |

## Reproducibility and implementation status

Tracked, repository-owned Thread-B artifacts:

- `research/exploration/force_history.py` and `tests/test_force_history.py`;
- `research/exploration/force_information.py`,
  `tools/analyze_force_information.py`, configs, and tests;
- `research/exploration/act_force_interventions.py`,
  `tools/analyze_act_force_use.py`, config, and tests;
- blinded annotation schema/protocol/template and tests; and
- controlled observability protocol/config.

Ignored result identities:

| Result | Dataset identity | Result SHA-256 |
| --- | --- | --- |
| canonical nominal16 | fingerprint `9b5792f3...a6c72`; materialization SHA `3f3fd382...be02` | `0dfc5a80...3f25` |
| generic non-nominal descriptive | old mixed fingerprint `1b836162...0572` | `7afa84f6...1bf6` |
| full mixed diagnostic | old mixed fingerprint `1b836162...0572` | `5c4d6e7e...f9f0` |
| prior active-grasp trace | RH56-only ignored artifact | `2a634f18...da54` |

The old ACT and ACT+load master Parquets are byte-identical as paired sample
sources (aggregate SHA `6dac3b9bbfe41887a130ff9fbce421ab8e84a94846d52a4e8919c95ff3f4ef4d`),
with shared video assets. Final clean policy comparisons must repeat this exact
identity check on Thread A's canonical views and use the same train/validation
split.

The intervention loader's host-only identity dry run passed on that historical
pair: 20,744 rows / 25 logical episodes, shared-row digest
`1f8259357c4722191d618b8f7aeb60f621604a54718c761cd0bfbeb4f8973fe8`,
raw force float32 digest
`5ec843e16f96ab3d963819c49c34be3ea94ef399f101219830db2c88bcac30dc`,
and three explicitly detected split-leaky sessions. Checkpoint inference was
not run.

No robot, hand, camera, or Quest device was opened. No raw/master dataset or
paper file was modified by Thread-B.
