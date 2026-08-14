# Strong ACT temporal executor fix — 2026-08-14

## Decision

The fixed-horizon `consume_k` path was the immediate cause of the Strong ACT
pilot not exposing its predicted grasp.  The new command-disabled absolute-time
temporal ensemble retains those predictions and selects them at their intended
30 Hz command ticks.  No robot was moved in this audit.

The implementation passes the focused regression suite and replay gates.  The
next physical qualification is **not executed automatically**: it still needs
the operator's explicit rollout authorization and a short command-disabled
end-to-end camera shadow check.

## Frozen experiment state

| Item | Value |
|---|---|
| repository commit at audit start | `4015591543e1b663d0edc8659c5d4758b3406663` |
| branch | `research/thread-b-force-interaction` |
| Strong checkpoint | `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model` |
| checkpoint contract | workspace/wrist RGB `[3,240,320]`, state `[12]`, action `[12]`, chunk `[60,12]`, `n_action_steps=2` |
| ACT config SHA256 | `932b95e0b778d27519c200c6e54db96565bd0256d0da0791698aa69e39b1440a` |
| split config SHA256 | `44228dedfa769724c7b68b80757c5bf1cc60869d2aa1c3095840625c4a23e638` |
| split manifest SHA256 | `a5abaf80523a065ffdf84d611c44c218d1d6cf27b64c7a19a85c349cbbb28741` |
| pinned LeRobot | 0.6.2, external checkout commit `f66e5128ecb2456e8c54a63d15404fa59c16aebc` |
| normalization | loaded from checkpoint; absolute/native action representation unchanged |
| physical input rates | policy query ≈15 Hz, command 30 Hz |

The raw/master dataset and checkpoint were not modified.

## Canonical temporal aggregation audit

The pinned LeRobot implementation is in
`/home/thor/LeRobot/src/src/lerobot/policies/act/modeling_act.py`.
`ACTTemporalEnsembler` maps a horizon to the next absolute sequence position,
keeps overlapping predictions, and computes

```text
w_i = exp(-m * i),  i=0 is the oldest overlapping prediction
```

Thus positive `m` gives the oldest contributor the largest weight.  The
original ACT implementation uses the same absolute-time table and `m=0.01`.
`n_action_steps` is an inference queue/consumption setting; it does not alter
the trained chunk size or target construction.  A non-null temporal ensemble
coefficient in LeRobot returns one blended action per control step and requires
`n_action_steps=1` in that API.

The repository executor now has three explicit modes:

* `consume_k` — legacy prefix consumption, retained for diagnostics;
* `temporal_ensemble` — absolute-tick overlap with canonical weighting;
* `async_temporal_ensemble` — the same absolute-tick buffer for a lower-rate
  policy producer and 30 Hz command consumer.  It is repository-local and is
  not claimed to be the stock LeRobot queue implementation.

For Strong ACT, omitting `--act-execution-mode` resolves to
`temporal_ensemble`; `--consume-actions 2` is no longer the Strong default.
Short historical chunks still resolve to the legacy mode unless explicitly
changed.

## Implementation contract

Each completed query records `query_id`, completion timestamp, command tick,
and chunk size.  Every horizon `h` is stored as:

```text
(query_id, horizon=h, target_command_tick=query_command_tick+h, action[h])
```

At command tick `t`, only entries with `target_command_tick == t` are eligible.
Predictions are not deleted merely because another query completed.  A bounded
buffer, maximum source horizon, optional maximum prediction age, and fallback
counter keep the mechanism finite.  A command tick with no valid time-aligned
prediction may use the last selected action; startup with no selected action
uses the existing measured-target hold behavior.

Temporal aggregation runs before the existing RH56 projection, delta limit,
contact protection, serial write, JAKA publication, and watchdog paths.  No
safety equation or RH56 polling behavior changed.

Every temporal command row now carries the full contributor list, including
query id/timestamp, query command tick, source horizon, target tick, predicted
action, and normalized weight, followed by:

```text
ensemble_raw_target
post_projection_target
post_filter_target
post_delta_limit_target
post_contact_safety_selected_target
actually_written_target
measured state / timestamps
```

`act_predictions.npz`, analysis tools, and shadow validation now derive `H`
and action dimension from the checkpoint/output shape.  No Strong ACT path
requires `[16,12]`.

## Real Strong pilot replay

Input: `outputs/act_physical_rollouts/20260814_strong_act_pilot_001`, shape
`[301,60,12]`, 301 queries and 601 recorded command ticks.  This is a
command-disabled replay of saved data; it does not rerun the robot.

The coordinated closure threshold below is the existing diagnostic heuristic
(`min(index,middle,ring,pinky,thumb_close) >= 0.1`), not a contact label.

| replay mode | closure command-tick fraction | first closure | selected/source horizon | fallback | mean abs step | p95 abs step | max abs step |
|---|---:|---:|---:|---:|---:|---:|---:|
| consume K=2 | 0% | none | h0–h1 | 0.17% | 0.00433 | 0.00918 | 4.503 |
| consume K=8 | 0% | none | h0–h7 | 0.17% | 0.00707 | 0.02342 | 4.503 |
| consume K=16 | 0% | none | h0–h15 | 0.17% | 0.01040 | 0.02355 | 4.503 |
| temporal ensemble `m=.01` | **60.07%** | tick 240 / 8.00 s | contributors through h59; newest selected h0–h2 | 0.17% | 0.00538 | 0.00754 | 4.503 |

The 4.503 maximum is the initial measured-target-to-first-policy-action
startup transition present in all raw replay streams, not a query-boundary
ensemble jump.  Existing hardware delta/safety layers remain authoritative.
Excluding that startup transition, temporal ensembling has a lower p95 step
than K=2 in this replay.

Temporal mode added 300 queries before the saved command horizon ended and
selected 17,130 overlapping prediction points.  Of 18,000 available points,
930 were naturally outside the recorded command interval; they were not
superseded at query arrival.  The temporal contributors had age p50 29 ticks,
p95 56, max 59.  This is the required evidence that future grasp predictions
were retained rather than discarded.

The real pilot's full chunks contained coordinated grasp predictions in 208 of
301 queries, with first grasp horizon median h21 and p90 h31.  Under the
simple historical opening proxy (first-five RH56 mean at least 0.10 below h0),
**0/301** pilot chunks contained a release prediction.  Therefore this replay
establishes the executor effect for grasp, but does not establish release
capability for this physical pilot window.

Machine-readable result:
[`real_pilot_executor_replay.json`](strong_act_transition_execution_audit_20260814/real_pilot_executor_replay.json).

## Teacher-forced nominal52 replay

The fixed held-out teacher-forced arrays are `[9309,60,12]`.  The current
absolute-time replay was rerun for the 15 nominal validation segments:

| transition proxy | eligible queries | recalled | recall | mean absolute timing error |
|---|---:|---:|---:|---:|
| T1 approach → grasp RH56 proxy | 885 | 865 | 97.74% | 9.54 frames / 0.318 s |
| T4 place → release hand proxy | 885 | 771 | 87.12% | 22.50 frames / 0.750 s |

These are synchronized hand/action proxies.  T2 lift and T3 place remain
uncertain because the current materialized metadata does not provide
authoritative object/lift/place labels.

In teacher-forced replay, all tested modes can consume the proxy on some
recorded rows because the recorded observation is reset to the demonstration
trajectory at every row.  That result must not be confused with closed-loop
physical success.  The real-pilot replay above is the relevant executor test:
K2/K8/K16 selected no coordinated grasp, while temporal overlap selected it.

Updated figures and the session-level teacher-forced tables remain in
[`strong_act_transition_execution_audit_20260814/`](strong_act_transition_execution_audit_20260814/):

* [Figure A — grasp horizon](strong_act_transition_execution_audit_20260814/figures/figure_a_grasp_horizon.png)
* [Figure B — release horizon](strong_act_transition_execution_audit_20260814/figures/figure_b_release_horizon.png)
* [Figure C — historical predicted/executed horizons](strong_act_transition_execution_audit_20260814/figures/figure_c_historical_horizons.png)
* [Figure D — teacher-forced executor comparison](strong_act_transition_execution_audit_20260814/figures/figure_d_executor_comparison.png)

## 30 Hz inference feasibility

Command-disabled model-worker benchmark in the pinned CUDA container, Strong
100k checkpoint, zero-valued contract-shaped observations, 60 serial socket
queries:

| measurement | p50 | p95 | p99 | max | over 33.33 ms |
|---|---:|---:|---:|---:|---:|
| socket round trip | 19.03 ms | 20.05 ms | 20.25 ms | 20.25 ms | 0/60 |
| worker total | 15.92 ms | 16.80 ms | 16.94 ms | 17.00 ms | 0/60 |

The serialized model-worker rate was approximately 53.4 Hz.  This supports
30 Hz model inference in this isolated benchmark, but it does not measure
camera acquisition, Python snapshot work, or a live JAKA/RH56 session.  An
end-to-end command-disabled camera shadow remains a pre-rollout gate.

## Why K=2/8/16 and not K=32/60

With q15/cmd30, K2 exposes only 66.7 ms of the 2-second prediction horizon;
K8 exposes 267 ms and K16 533 ms.  The observed coordinated grasp begins
around h21–h31, so all three systematically throw it away when a new query is
adopted.  K32 would expose some predictions but creates about 1.07 s of open
loop control; K60 creates about 2 s.  Those settings reduce replanning and
reactivity and are not a principled replacement for absolute-time overlap.

Temporal ensembling differs by preserving a prediction's intended absolute
time and combining all overlapping predictions for that time.  It does not
select “the grasp horizon now” and does not choose the maximum closure.

## Safety and remaining uncertainty

No evidence in the Strong pilot indicates an ordinary filter or contact-stop
removed the grasp prediction.  The earlier external rollouts lacked complete
selected-versus-written traces; the new physical path records those stages.
Timing at q15/cmd30 was healthy in the prior physical pilot, and this change
does not alter JAKA safety, RH56 force polling/freshness, contact-stop logic,
watchdogs, or serial scheduling.

The remaining uncertainty is closed-loop visual/state divergence and whether
the Strong policy's T2/T3 object behavior and release behavior are adequate.
Temporal replay is evidence for executor correctness, not physical task
success.

## Recommended qualification configuration — do not execute automatically

After the camera/state command-disabled shadow passes, the single proposed
qualification is:

```text
checkpoint: outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model
query rate: 15 Hz
command rate: 30 Hz
chunk size: 60 (read from checkpoint)
executor: async_temporal_ensemble
temporal ensemble coefficient: 0.01
max source horizon: 59
consume_actions: ignored in temporal mode (retain value 2 only for legacy logs)
filter: no policy low-pass filter; existing safety delta/contact guards ON
force: production FORCE_ACT logging only; no force conditioning
```

The rollout must stop on any existing JAKA/RH56/camera/watchdog fault, stale
source violation, projection tolerance violation, recorder failure, or native
hard timing fault.  Every command must retain contributor provenance and the
requested → selected → written RH56 trace.  A 30/30 canonical ensemble is a
later alternative after the end-to-end shadow establishes that the measured
model margin survives camera and snapshot overhead.

## Files changed for this fix

* `src/embodiment_core/act_temporal_executor.py` — bounded absolute-tick
  prediction buffer and canonical weighting.
* `tools/act_physical_rollout.py` — explicit execution modes, pending chunk
  forwarding, temporal command selection, and per-command provenance.
* `tools/act_transition_execution_audit.py` — teacher-forced replay now uses
  the same absolute-time executor for temporal mode.
* `tools/act_temporal_executor_replay.py` — command-disabled replay of saved
  physical rollouts.
* `tools/analyze_act_bottle_rollout.py` and `tools/act_live_shadow.py` —
  dynamic `[N,H,A]`/checkpoint output handling.
* `tests/test_act_temporal_executor.py`,
  `tests/test_act_transition_execution_audit.py`, and
  `tests/test_act_bottle_rollout_analysis.py` — temporal, procrastination,
  weighting, no-query, reset/expiry, and H=60 regressions.
* `research_log/strong_act_transition_execution_audit_20260814/` — updated
  machine-readable replay/figures.

The pre-existing uncommitted RH56 files were not changed by this fix.

## Validation

Focused suite after the fix:

```text
29 passed
```

Full repository suite after the fix:

```text
738 passed, 4 skipped, 2 warnings
```

`git diff --check` passed.
No physical command was issued.
