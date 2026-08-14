# Strong ACT transition/execution audit — 2026-08-14

## Decision

**Offline gate: INVESTIGATE / NO-GO for hardware.** No command-enabled rollout was
performed. The Strong ACT checkpoint is not a collapsed policy: it predicts the
T1 grasp hand transition across the held-out validation sessions and its
predicted horizon generally moves toward the front of the chunk. The executor
still has a real horizon-truncation contract, and T2/T3 object-level events are
not sufficiently annotated to claim autonomous five-stage competence.

The next physical run must wait until the contract/logging checks below are
reviewed and the requested→selected→written RH56 trace is exercised in a
command-disabled replay or explicitly bounded validation procedure.

## Frozen experiment state

| Item | Value |
|---|---|
| branch / commit | `research/thread-b-force-interaction` / `b6d377559090320797ac2ea2d173ae36a8dcc8a2` |
| dataset | `data/training/physical_bottle_v4_nominal52/` |
| split | 37 train trajectories / 23,802 rows; 15 validation trajectories / 9,309 rows; deterministic session-grouped |
| validation logical segments | `ep087`, `ep088`, `ep108`, `ep109`, `ep138`–`ep142`, `ep172_a`–`ep172_c`, `ep173`, `ep174_a`, `ep174_b` |
| strong checkpoint | `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model` |
| checkpoint contract | two RGB inputs `[3,240,320]`, state `[12]`, action `[12]`, `chunk_size=60`, `n_action_steps=2`, no force input |
| ACT config SHA256 | `167a362f8440a65fd78eb43dd3330bc7dd684612295906784d8ca8475d4ca638` |
| split config SHA256 | `44228dedfa769724c7b68b80757c5bf1cc60869d2aa1c3095840625c4a23e638` |
| LeRobot | pinned container `0.6.2`, official commit `f66e5128ecb2456e8c54a63d15404fa59c16aebf` |
| training/inference hardware | Strong checkpoint evaluation ran in the pinned CUDA container; no hardware was opened in this audit |

The dataset/master, raw recordings, and checkpoint were not modified. The
complete machine-readable results and figures are in
[`research_log/strong_act_transition_execution_audit_20260814/audit.json`](strong_act_transition_execution_audit_20260814/audit.json)
and its `figures/` directory.

## Checkpoint contract finding

The repository rollout path had a genuine integration defect: it hard-coded
`[16,12]` and fixed workspace/wrist key names. The completed Strong checkpoint
declares `[60,12]` with:

```text
observation.images.workspace: [3, 240, 320]
observation.images.wrist:     [3, 240, 320]
observation.state:             [12]
action:                         [12]
chunk_size:                    60
n_action_steps:                 2
```

The rollout and command-disabled model worker now read this contract from the
checkpoint, validate shapes before inference, and record it in the rollout
summary. `consume_actions` is bounded by the checkpoint chunk size rather than
by the old value 16. This is a fail-fast compatibility fix; it does not change
the action representation or safety limits.

## Transition definitions and limits

T1 uses the existing reproducible RH56 closure proxy: the curated per-segment
baseline and closure threshold from `analyze_act_horizon_coverage.py`. T4 is
anchored to the persisted final `hand_grip` release edge and uses a separately
reported hand-opening proxy. It is not a force/contact label. T2 (grasp→lift)
and T3 (transport→place) do not have authoritative object/lift/place event
annotations in this materialized view, so they are marked **uncertain** rather
than inferred from arbitrary arm thresholds.

## Strong ACT teacher-forced results

The full `[9309,60,12]` prediction chunks were retained. All values were finite.
Aggregate first-action MAE from the existing validation replay was JAKA
`0.0451` and RH56 `0.0195`. Phase-conditioned first-action errors were:

| phase proxy | JAKA MAE | RH56 MAE |
|---|---:|---:|
| approach | 0.0416 | 0.0147 |
| grasp transition | 0.0513 | 0.0179 |
| post-grasp | 0.0497 | 0.0234 |
| release transition | 0.0479 | 0.0273 |
| post-release | 0.0401 | 0.0178 |

| transition proxy | eligible queries | recall | mean absolute timing error |
|---|---:|---:|---:|
| T1 grasp | 885 | 865 / 885 = **97.7%** | 9.54 frames / 0.318 s |
| T4 release hand proxy | 885 | 771 / 885 = **87.1%** | 22.50 frames / 0.750 s |
| T2 lift | — | **uncertain** | no objective event annotation |
| T3 place | — | **uncertain** | no objective event annotation |

T1 recall was present in all four held-out session groups, not only one
episode. The per-session T1 recall was 1.000 for `session_87_88` and
`session_108_109`, 1.000 for the `session_138_142` group under this audit's
eligible-query definition, and 0.944 for `session_168_174`. The T4 proxy was
weakest in `session_138_142` (about 0.702) and should not be called robust.

The copy-current-state baseline recalled only 3 T1 transition queries in the
earlier validation analysis (`0.33%`), while Strong ACT's RH56 chunk peak-to-
peak was approximately `0.091` on average. This is evidence against a pure
state-persistence policy, although it is not a physical success claim.

## Horizon convergence

The aggregate teacher-forced horizon plots are:

* [Figure A — T1 grasp horizon](strong_act_transition_execution_audit_20260814/figures/figure_a_grasp_horizon.png)
* [Figure B — T4 release horizon](strong_act_transition_execution_audit_20260814/figures/figure_b_release_horizon.png)

Across 15 validation segments, the mean slope of predicted transition horizon
against time-to-event was `0.77` frames/frame for T1 and `0.51` for the T4
proxy. Most segments had a positive correlation and a non-increasing horizon
as the event approached. This is materially different from a repeated
`14,13,15,14,...` procrastination pattern. Some segments have poor/flat
convergence, especially for release, so the result is mixed rather than a
five-stage guarantee.

## Executor semantics

The custom rollout calls the full-chunk prediction API and then applies its
own `ActionChunkConsumer`. For `query=15 Hz`, `command=30 Hz`:

* `K=2` executes only h0/h1 before adopting a newer query;
* `K=8` and `K=16` execute a longer prefix but retain an older chunk until that
  prefix is exhausted;
* a transition that remains at h>1 can still be postponed with K=2;
* K=2 is therefore not a proof of correct chunk execution, although it is the
  smallest setting that matches a two-command receding horizon at 15/30 Hz.

The offline replay is teacher-forced, so it measures executor semantics and
not closed-loop physical behavior. Selected key results:

| strategy | grasp proxy consumed | release proxy consumed | repeated-action fraction | mean selected/query age |
|---|---:|---:|---:|---:|
| consume K=2, 15 Hz | 15/15 | 15/15 | 0.000 | 0.50 frames |
| consume K=8, 15 Hz | 15/15 | 15/15 | 0.000 | 3.49 frames |
| consume K=16, 15 Hz | 15/15 | 15/15 | 0.000 | 7.46 frames |
| consume K=8, 3 Hz | 15/15 | 15/15 | 0.199 | 4.48 frames |
| consume K=1, 30 Hz | 15/15 | 15/15 | 0.004 | 0 frames |
| canonical ensemble, 15 Hz, coeff 0.01 | 15/15 | 15/15 | 0.500 | 0.50 frames |
| canonical ensemble, 30 Hz, coeff 0.01 | 15/15 | 15/15 | 0.000 | 0 frames |

The negative mean event delays in this table mean the teacher-forced policy
predicted the hand proxy before the curated event anchor; they do **not** mean
that the robot physically completed the task. The release proxy is especially
sensitive to the persisted clutch edge and is not sufficient to choose a
physical policy by itself.

LeRobot 0.6.2 was inspected in the pinned container. `n_action_steps` controls
the number of actions queued by `select_action`; it does not change ACT's
prediction horizon or architecture. With a non-null temporal-ensemble
coefficient, LeRobot instead updates an `ACTTemporalEnsembler` using overlapping
chunks and returns one blended action. Our custom path currently uses
`predict_action_chunk` plus `ActionChunkConsumer`, so it is not the canonical
LeRobot queue/ensemble path.

## Historical external rollout audit

These runs used `/home/thor/LeRobot/models/jaka_mini2_act_320x240`, not the
nominal52 Strong checkpoint. They are executor evidence only. All three used
`chunk=30`, query 3 Hz, command 30 Hz, K=8; only h0…h7 were ever selected and
h7 could repeat until the next query. Figure C shows predicted versus executed
horizons.

| run | prediction classification | evidence |
|---|---|---|
| `act_nodup_test_002` | 175/181 queries predicted opening only outside executed prefix; 6/181 no opening | predicted release horizons were beyond h7; no selected/written release trace existed |
| `act_nodup_test_005` | 91/91 `POLICY_NO_RELEASE` | no opening transition found in any full chunk under the audit proxy |
| `act_nodup_test_006` | 72/181 predicted opening outside prefix; 109/181 no opening | same K=8 truncation; no release request was visible in executed h0…h7 |

There is no evidence in these logs of a valid release request being rejected by
contact protection. More precisely, the old logs do not contain the selected-
versus-written fields needed to prove controller blocking; because the release
requests were absent from the executed prefix, the observed failures are
classified as policy-no-release or executor-truncation, not contact blocking.

## Filtering and safety traceability

No configurable low-pass filter was found in the current ACT policy path.
RH56 has safety mechanisms, not a policy smoother:

* ordinary hand delta limit: `0.05` normalized command units;
* contact-stop closing step: `0.0125` under the configured fresh-force/contact
  path;
* force freshness, contact latch, serial fault, and command watchdogs remain
  authoritative.

Teacher-forced counterfactuals for the recommended K=2/q15 stream gave mean
hand-step attenuation of approximately `1.75%` for the 0.05 delta guard and
`8.20%` for the 0.0125 contact-closing guard. This is not a contact-stop
simulation because measured feedback and force latching are unavailable.
Therefore the operator observation “filter off helps” cannot yet be separated
from query frequency using these data alone.

The rollout row plus RH56 controller now expose a bounded in-memory command
trace containing:

`policy_requested_target` (rollout-side) / `controller_requested_target` (RH56
input) → post_projection_target → post_filter_target →
post_contact_safety_selected_target → post_delta_limit_target →
actually_written_target/raw`, with request/write timestamps, sequence, and
disposition. The rollout row records the policy/projection fields and the
latest RH56 trace without doing serialization in the serial worker. This is a
logging/observability change only; no safety equation or contact-unlatch rule
was changed.

## Timing and freshness

Strong teacher-forced arrays preserve the fixed causal validation rows. The
historical executor logs show causal source timestamps for camera, JAKA,
ANGLE_ACT, and FORCE_ACT. Their JAKA ages were approximately p95 31–32 ms,
ANGLE p95 46–62 ms, FORCE p95 79–91 ms; native deadline hard misses were zero
in test002/test005 and one soft warning/miss in test006, with no hard tracking
fault. These records do not support timing/freshness as the dominant cause of
the old release failure, but the Strong checkpoint has not been physically
validated.

## Answers to the requested gates

1. **All five local stages?** Not established. Strong ACT shows local approach,
   hand closure, post-grasp and opening behavior; lift/place object events are
   not objectively annotated here.
2. **Each phase transition?** T1 is learned under the RH56 proxy; T4 is mixed
   under a clutch-anchored hand-opening proxy; T2/T3 remain uncertain.
3. **Horizon convergence?** Generally yes for T1, mixed but not perpetually
   postponed for T4.
4. **Are historical release predictions discarded?** Yes for test002/test006
   when they occur beyond h7; test005 mostly has no release prediction.
5. **Does consume_actions truncate future actions?** Yes, by construction.
6. **Would K=2 solve it?** No general proof; it can still discard h2…h59.
7. **Does canonical ensemble perform better?** It reduces discontinuity in
   teacher-forced replay (q30 max hand/action step about 0.038 versus K2 about
   0.309), but q15 introduces held outputs and physical closed-loop benefit is
   not proven. Do not enable it on hardware from this audit alone.
8. **Filtering delay?** No low-pass filter exists; only safety delta guards
   were counterfactually quantified above.
9. **Query-frequency effect?** Higher q reduces query age; q3 adds repeated
   actions. It does not by itself prove better task completion.
10. **Contact blocked valid release?** Not demonstrated. Historical logs lack
    the trace, and no executed valid release request is visible.
11. **Timing causal?** Not supported by the available historical timing data.
12. **Next rollout configuration?** **NO-GO for now.** After contract/logging
    review, the smallest evidence-based shadow candidate is Strong ACT,
    `chunk_size=60`, query 15 Hz, command 30 Hz, K=2, filtering off (there is
    no policy filter), force logging only, no temporal ensemble. It must first
    pass command-disabled replay with full stage annotations and trace fields.
13. **Dominant problem?** Mixed: the old external runs had a clear executor
    truncation problem, while Strong ACT still has uncertain T2/T3 and a weak
    release proxy. It is not justified to attribute the remaining risk to one
    mechanism.

## Files changed

* `src/embodiment_core/act_contract.py` — dependency-free dynamic checkpoint
  contract validator.
* `tools/act_shadow_model_worker.py` — dynamic input/output contract use.
* `tools/act_physical_rollout.py` — dynamic chunk/key handling, contract in
  summary, and staged RH56 trace fields; no hardware execution performed.
* `tools/act_transition_execution_audit.py` — command-disabled transition,
  horizon, executor, filter-counterfactual, and historical-log audit.
* `tests/test_act_checkpoint_contract.py` — contract regression tests.
* `tests/test_act_transition_execution_audit.py` — horizon, replay, and
  temporal-ensemble regression tests.
* `tests/test_act_physical_rollout_adapter.py` — 60-step consumer and contract
  coverage.
* `tests/test_rh56_pc_direct_control.py` and
  `src/rh56_driver/pc_direct_control.py` — bounded requested/selected/written
  trace coverage. The RH56 files already contained unrelated uncommitted user
  changes; those were preserved.
* `research_log/strong_act_transition_execution_audit_20260814/` — generated
  machine-readable audit and Figures A–D.

## Validation run

Focused tests passed:

```text
49 passed
```

The pinned Strong checkpoint teacher-forced replay completed with output
`[9309,60,12]`, finite outputs, and aggregate validation L1 `0.2360`. No
camera, serial, JAKA, RH56, or command-enabled process was started.

Full repository tests passed:

```text
732 passed, 4 skipped, 2 warnings
```

`git diff --check` also passed. The warnings are the existing Python
multi-threaded `fork()` deprecation warnings from episode-process isolation.
