# Physical bottle hardware-write timing audit — 2026-08-14

## Scope and safety boundary

This audit followed commit `d696bba` (`Finalize canonical Strong ACT rollout path`).
It did not start another bottle task, retrain ACT, modify raw/master data, or
change a freshness, watchdog, JAKA, RH56 contact, velocity, acceleration, or
jerk limit.

The isolation runs were 5-second stationary tests. The command path held the
currently measured JAKA and RH56 state rather than forwarding the model target
to the hardware. The normal entry point still started the existing native JAKA
worker, RH56 feedback worker, cameras, watchdogs, and safety layers. In the
`none`/`rh56_only` cases the native JAKA worker still received the required
startup alignment target and heartbeat/hold traffic; “JAKA writes disabled”
means that per-tick policy target publication was disabled, not that the
native ServoJ process was removed.

The model was:

```text
outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model
```

with `jaka-lerobot-dev:snapshot-before-raw-mount`, the production collection
YAML, canonical 30 Hz ACT temporal ensembling, chunk size 60, and coefficient
`m=0.01`. Generated run artifacts are under
`outputs/act_physical_rollouts/write_timing_*_003/` and are not training data.

## Architecture and call graph

### JAKA

```text
ACT selected [12]
  -> JakaAcceptedJointTargetAdapter.apply_joint_position
  -> joint_position_target_packet
  -> ArmOnlyRuntime.dispatch_packet
  -> LatestTargetPublisher.publish
  -> nonblocking AF_UNIX datagram
  -> native worker latest target mailbox
  -> bounded ServoJ resampler
  -> vendor edg_servo_j on the native 125 Hz worker
```

The Python producer call is nonblocking. `LatestTargetPublisher` uses a
nonblocking Unix datagram socket and a bounded kernel buffer; it does not wait
for a JAKA ACK. The vendor `edg_servo_j` call occurs on the isolated native
worker, not on the ACT policy/control Python thread.

Native metrics from the corrected D run measured the actual SDK/write side:

| native metric | result |
|---|---:|
| SDK call p95 | 0.269 ms |
| SDK call max | 0.478 ms |
| ServoJ write p95 | 0.168 ms |
| ServoJ write max | 0.338 ms |
| native hard timing misses | 0 |
| native deadline misses | 0 |

The native worker did report target-age warning cycles because the producer
was only providing targets at roughly 14 Hz after the policy loop overran;
this is a freshness consequence, not a blocking SDK call.

### RH56

```text
ACT selected hand target
  -> RH56PcDirectWorker.submit_target
  -> one-slot locked latest-target mailbox
  -> RH56 worker scheduler
  -> RH56PcDirectControl.command
  -> projection/delta/contact safety
  -> RH56SerialBackend.write_register
  -> serial write + 5 ms protocol delay + bounded response polling
```

`submit_target` only updates a bounded mailbox under a short lock. It does not
perform serial I/O. The blocking operation is in the dedicated RH56 worker:
`write_register()` calls `_exchange()`, which performs the serial transaction,
the configured 5 ms inter-transaction delay, and response polling. Feedback
polling also runs in that worker. No RH56 serial write was placed on the ACT
producer thread by this audit.

### Logging

The producer uses `AsyncRolloutWriter.submit()` with a bounded queue. The
measured enqueue stage was below 0.1 ms at p95 in all corrected runs. Video and
JSONL persistence therefore did not account for the observed loop delay.

## Instrumentation

`tools/act_physical_rollout.py` now records bounded per-stage online
statistics for camera/status/feedback acquisition, observation assembly,
preprocessing, policy socket legs, temporal aggregation, action split,
projection, JAKA/RH56 submission, recorder enqueue, and the complete critical
path. Each command row also carries the stage durations and the model-selected
versus pre-safety target fields.

`tools/act_shadow_model_worker.py` separates:

* `policy_worker_receive_wait`: time waiting for the next client request;
* `policy_worker_receive_decode`: actual read/decode after the socket is
  readable.

The earlier measurement incorrectly attributed the inter-query wait to
receive/decode; that instrumentation error was corrected before the final A-D
comparison.

## Controlled stationary isolation results

Mapping:

* A — `write-path-mode none`: no per-tick JAKA target publication and no RH56
  target submission; feedback/watchdogs remain active.
* B — `jaka_only`: JAKA target publication enabled, RH56 target submission
  disabled.
* C — `rh56_only`: JAKA producer target publication disabled, RH56 target
  submission enabled.
* D — `both`: both per-tick target submissions enabled.

The tests were deliberately short. They all terminated through the existing
policy freshness gate rather than by a robot fault. This is itself a NO-GO for
another qualification, and the freshness threshold was not increased.

| mode | commands / queries | critical p50 | critical p95 / max | model forward p50 | JAKA submit p50 | RH56 submit p50 | result |
|---|---:|---:|---:|---:|---:|---:|---|
| A none | 8 / 9 | 30.85 ms | 35.58 / 37.49 ms | 14.99 ms | 0.360 ms | 0.001 ms | freshness abort |
| B JAKA | 4 / 5 | 38.58 ms | 41.11 / 41.37 ms | 24.53 ms | 0.258 ms | 0.000 ms | freshness abort |
| C RH56 | 4 / 5 | 38.69 ms | 40.73 / 41.04 ms | 24.74 ms | 0.215 ms | 0.051 ms | freshness abort |
| D both | 4 / 5 | 38.58 ms | 39.56 / 39.56 ms | 23.47 ms | 0.090 ms | 0.047 ms | freshness abort |

The exact command-start intervals were:

| mode | command intervals after startup |
|---|---|
| A | 70.96 ms, then 33.32–33.36 ms |
| B | 69.26, 73.08, 74.81 ms |
| C | 70.83, 72.37, 74.49 ms |
| D | 71.08, 73.01, 73.01 ms |

The A run is important: after its first startup interval, it sustained
approximately 30 Hz while the model worker happened to run at about 16 ms.
The B-D runs had approximately 24–28 ms worker inference and therefore crossed
the 33.33 ms control budget once the host-side processing and ensemble work
were included. The model runtime varied materially between sequential runs;
the hardware mode did not produce a matching increase in the producer’s
JAKA/RH56 call stage.

### RH56 serial-side measurements

| mode | serial writes | serial write last/max | submit→write p50 / p95 | RH56 worker cycle p95 / max |
|---|---:|---:|---:|---:|
| A | 0 | — | — | 8.28 / 115.81 ms |
| B | 0 | — | — | 6.81 / 136.72 ms |
| C | 3 | 5.20 / 5.57 ms | 45.64 / 64.63 ms | 6.14 / 125.43 ms |
| D | 3 | 5.23 / 7.64 ms | 44.18 / 61.50 ms | 6.88 / 71.99 ms |

`submit→write` is not a blocking duration of `submit_target`; it is the age
of the target when the separate RH56 worker eventually writes it. The actual
serial transaction is approximately 5–8 ms in these short runs. The 40–60 ms
number is therefore stale-command age caused by the producer’s late command
ticks and worker scheduling, not a 40 ms synchronous producer call.

### Safety and device health

Across A-D:

* JAKA native hard misses: 0.
* JAKA tracking hard faults: 0.
* controller alarm/collision/e-stop: none.
* RH56 serial timeouts/faults: none.
* RH56 contact latch activations: 0.
* recorder queue failures: none observed.
* the only termination was the existing ACT temporal-ensemble freshness
  exception.

## Root cause

### Answer to “where does the extra approximately 40 ms come from?”

The controlled test does **not** support the claim that hardware write calls
add approximately 40 ms to the ACT producer tick. The direct calls are:

* JAKA Python target publication: approximately 0.09–0.36 ms p50 in A-D;
* RH56 mailbox submission: approximately 0.001 ms with no write and
  0.047–0.051 ms with a write-enabled path;
* native JAKA SDK/write: sub-millisecond p95;
* RH56 actual serial transaction: approximately 5–8 ms, isolated in its own
  worker.

The dominant critical-path cost is the policy query: live preprocessing,
socket transfer, and the variable 23–28 ms model forward in B-D. Including
ensemble and bookkeeping, this produces approximately 38–41 ms ticks. A’s
lower 15 ms model-forward sample shows that inference variance, rather than
the device call itself, is the main remaining timing uncertainty.

Hardware writes do have an **indirect observable effect**: when RH56 writes
are enabled, the serial worker reports target age around 44–46 ms because the
producer is already late. The test cannot prove that this worker activity is
the cause of the model-runtime variation between sequential runs. It does
prove that the producer is not synchronously waiting for those writes.

### Is it JAKA, RH56, logging, locking, or combined scheduling?

* Direct JAKA blocking: disproven for the Python producer; the SDK call is in
  the isolated native worker and sub-millisecond in the captured metrics.
* Direct RH56 blocking: disproven for `submit_target`; serial I/O is in the
  RH56 worker. The worker can accumulate command age when the producer misses
  deadlines.
* Logging: not causal in this test; bounded enqueue is below 0.1 ms p95.
* Locking: no recurrent long producer lock section was observed; the RH56
  submit lock is short.
* Combined scheduling/inference: the remaining cause of the producer miss is
  the end-to-end ACT inference path under the physical runtime, with possible
  but unproven CPU/GPU/serial-worker contention contributing to its run-to-run
  variance.

### Are current command APIs blocking?

* JAKA producer API: nonblocking latest-only AF_UNIX publication. The vendor
  ServoJ API is called by the native worker.
* RH56 producer API: nonblocking bounded mailbox submission. RH56 serial
  request/ACK is blocking, but only inside the dedicated serial worker.

### Is 30 Hz physical streaming supported by the existing stack?

The robot-side streaming mechanisms are present and safety-tested: JAKA uses
the native 125 Hz ServoJ loop and RH56 uses a dedicated scheduled serial
worker. The current full ACT producer did not sustain 30 Hz in the B-D
stationary tests because the inference critical path exceeded the 33.33 ms
budget. Thus the device layers support streaming, but this checkpoint/runtime
combination has not passed the end-to-end 30 Hz physical gate.

## Code change and architectural decision

The exact justified change in this audit is instrumentation plus a bounded
stationary isolation mode. No device-layer architecture change is justified
yet: moving already-nonblocking producer submissions to more threads would
not remove the measured bottleneck and could obscure selected→dispatched→written
provenance.

The normal path remains unchanged (`write-path-mode=both`, no stationary
override, canonical ACT temporal ensemble). The new A-D switches are bounded
diagnostic options and reject non-stationary use for isolated modes.

The next engineering action should be an offline/command-disabled inference
runtime optimization or stabilization study, with the same freshness gate,
before considering a producer/device dispatcher change. No model, action
chunk, temporal aggregation, or safety threshold change is part of this
audit.

## Files changed

Repository-owned changes from this audit:

* `tools/act_physical_rollout.py` — bounded per-stage timing, stage fields in
  command/query records, safe A-D isolation options, canonical summary
  compatibility fix, and newest-contributor freshness gating.
* `tests/test_act_physical_rollout_adapter.py` — regression coverage for
  bounded timing, canonical/legacy option separation, and the temporal
  ensemble freshness rule.
* `tools/act_live_shadow.py` and `tools/act_shadow_model_worker.py` —
  protocol-level timing fields for the shadow/physical differential.
* `research_log/physical_bottle_hardware_write_timing_audit_20260814.md` —
  this rate-selection addendum and differential timing tables.

## Stable real-runtime policy query rate (addendum)

This follow-up used the same Strong ACT checkpoint, production YAML, Docker
worker, cameras, JAKA status/native worker, RH56 serial worker, bounded writer,
and normal logging services.  It was command-disabled and stationary: the
runtime continued to publish the required native heartbeat and held the
measured target, but no task action was sent to either actuator.  No raw or
master data was touched.

The key correction made before these rate tests was to apply the existing
250 ms policy freshness gate to the **newest** contributing policy query, not
to the oldest historical contributor retained by temporal ensembling.  The
oldest age remains logged.  Rejecting on the oldest contributor incorrectly
aborted a healthy overlapping ensemble after several command ticks; this did
not weaken the threshold or change ensemble weights.

### Tested rates

| policy query | executor | run | queries / 30 Hz command ticks | query interval p50/p95/p99 (ms) | query E2E p50/p95/p99 (ms) | command interval p50/p95/p99 (ms) | freshness abort | writer drops / high-watermark | result |
|---:|---|---|---:|---:|---:|---:|---|---:|---|
| 30 Hz | canonical temporal ensemble | `query_rate_30_canonical_002` / 30 s | 853 / 853 | 33.33 / 37.63 / 74.12 | 26.14 / 28.83 / 37.66 | 33.33 / 39.89 / 75.03 | no | 0 / 2 | **not stable at 30 Hz**: only 28.41 Hz realized; recurrent over-budget intervals |
| 25 Hz | async absolute-time temporal ensemble | `query_rate_25_async_60s` / 60 s | 1,499 / 1,801 | 40.00 / 40.77 / 40.86 | 27.57 / 30.46 / 32.90 | 33.33 / 35.03 / 36.56 | no | 0 / 2 | **stable** |
| 20 Hz | async absolute-time temporal ensemble | `query_rate_20_async` / 30 s | 599 / 901 | 50.00 / 50.47 / 50.58 | 28.86 / 36.72 / 42.15 | 33.33 / 33.35 / 34.10 | no | 0 / 2 | stable |
| 15 Hz | async absolute-time temporal ensemble | `query_rate_15_async` / 30 s | 451 / 901 | 66.67 / 66.93 / 67.10 | 26.05 / 38.04 / 46.64 | 33.33 / 33.37 / 33.78 | no | 0 / 2 | stable |

The 25 Hz run is the longest tested run and is therefore the selected
production rate.  One query deadline skip occurred over 60 s; it did not
produce queue growth, freshness failure, or a command-loop interruption.  In
all async runs, every full `[60,12]` chunk was inserted into the absolute
30 Hz command timeline.  The temporal buffer reported zero capacity drops and
zero age-expired prediction points.  The 25 Hz run selected 1,799 command
ticks with only two startup fallbacks; the 30 Hz command interval remained
near 33.33 ms.

The 30 Hz result is not called stable merely because it completed without an
abort.  Its realized query/command rate was 28.41 Hz and its p99 command
interval was 75.03 ms.  This fails the stated stable-rate requirement.  The
25 Hz configuration preserves the trained 30 Hz horizon semantics: if a
query starts at command tick `t`, horizon `h` is still assigned to
`t + h`, not rescaled to the 25 Hz query period.

### Shadow versus physical-runtime differential

The table below compares the known-good 30 Hz command-disabled shadow
(`20260814_strong_canonical_shadow_002`) with the corrected physical-runtime
no-target-write stationary run (`write_timing_A_none_003`).  Values are
`p50 / p95 / p99` in milliseconds.  A dash means that the older artifact did
not record that sub-stage separately.  The worker receive-wait value is an
idle inter-request wait, not critical compute and must not be added to the
request latency.

| stage | shadow | physical A | delta (physical - shadow) |
|---|---:|---:|---:|
| control/query interval | 33.33 / 33.35 / 33.41 | 33.33 / 33.36 / 33.36* | approximately 0 / 0 / 0* |
| camera acquisition | 0.176 / 1.374 / 1.696 | 0.152 / 0.832 / 1.083 | -0.024 / -0.542 / -0.613 |
| JAKA + RH56 state acquisition | 0.040 / 0.051 / 0.054** | 0.183 / 0.261 / 0.267 | +0.143 / +0.210 / +0.213 |
| observation assembly | included above** | 0.101 / 0.134 / 0.137 | — |
| image preprocessing | 1.317 / 2.281 / 2.352 | 3.076 / 3.420 / 3.487 | +1.760 / +1.139 / +1.135 |
| request serialization/socket send | — | 3.585 / 4.301 / 4.459 | not separately measured in old shadow |
| worker receive/decode | — | 1.783 / 2.805 / 3.059 | not separately measured in old shadow |
| checkpoint preprocessing | 0.900 / 1.117 / 1.392 | 0.833 / 1.121 / 1.205 | -0.067 / +0.004 / -0.188 |
| pure model forward | 14.947 / 15.937 / 16.220 | 14.993 / 19.186 / 20.874 | +0.046 / +3.249 / +4.654 |
| checkpoint postprocessing | 0.217 / 0.253 / 0.267 | 0.215 / 0.246 / 0.253 | -0.002 / -0.007 / -0.014 |
| response receive/decode | part of 20.447 / 23.218 / 24.363*** | 17.109 / 21.642 / 23.336 | not an identical boundary |
| temporal ensemble | not run | 0.508 / 1.871 / 2.396 | physical-only |
| projection + action/device submission | not run | <0.51 / <0.18 / <0.18 | physical-only |
| provenance enqueue | not run | 0.070 / 0.086 / 0.088 | physical-only |
| complete request/control critical path | 22.310 / 26.313 / 26.896 | 30.850 / 35.577 / 37.103 | +8.540 / +9.264 / +10.207 |

\* Physical A includes the initial startup interval in its raw query stream;
the steady-state command intervals are 33.33–33.34 ms.  \*\* The shadow
called this combined read/assembly stage `state_assembly`; physical A exposed
the JAKA, RH56, and assembly sub-stages separately.  \*\*\* Shadow
`model_roundtrip` includes the complete host request/response leg, whereas
physical `policy_socket_receive` is the post-send response leg; they are not
claimed as an exact transport subtraction.

This comparison does **not** show a 40 ms synchronous hardware-write cost.
The direct producer calls remain sub-millisecond.  It shows that the stable
physical no-write critical path is roughly 8–10 ms slower than the prior
shadow, while model-forward tail latency varies between runs.  In the
corrected 30 Hz run, model-forward p50 was 15.25 ms and p95 16.72 ms, but
runtime p99 critical path was 41.12 ms.  In the 60 s 25 Hz run, model-forward
was 15.63 / 17.63 / 19.84 ms (p50/p95/p99), and the asynchronous command
critical path was 1.74 / 3.15 / 5.66 ms because inference no longer blocks
the 30 Hz command loop.

### Same-code protocol-level differential

After the first comparison, the model protocol was instrumented with a small
sideband timing frame. It measures request pickle/serialization, host socket
send, raw response receive, response deserialization, worker batch wrapping,
worker output-to-host copy, worker response serialization, and worker socket
send separately. The frame is consumed before the next request and does not
alter the prediction or safety path. The new shadow is
`20260814_strong_canonical_shadow_006`; the matching physical-runtime sample
is `query_rate_25_async_003` with command path `none` and the stationary hold.
The physical run used 25 Hz queries while the shadow used 30 Hz, so
inter-query wait is shown for diagnosis but is not treated as compute delta.
Values are p50/p95/p99 in milliseconds; deltas are physical minus shadow.

| stage | shadow 30 Hz | physical runtime 25 Hz | delta |
|---|---:|---:|---:|
| camera acquisition | 0.157 / 0.506 / 0.835 | 1.065 / 1.406 / 1.881 | +0.908 / +0.900 / +1.046 |
| JAKA + RH56 state acquisition | 0.037 / 0.048 / 0.056 | 0.060 / 0.082 / 0.241 | +0.023 / +0.034 / +0.185 |
| observation assembly | — | 0.052 / 0.067 / 0.078 | physical-only subdivision |
| image preprocessing | 0.618 / 1.352 / 1.636 | 2.835 / 3.892 / 4.206 | +2.217 / +2.540 / +2.570 |
| request serialization | 0.643 / 0.969 / 1.232 | 1.956 / 2.454 / 2.804 | +1.313 / +1.485 / +1.572 |
| host socket send | 1.087 / 1.522 / 3.139 | 2.836 / 4.259 / 4.896 | +1.749 / +2.737 / +1.757 |
| worker receive/decode | 1.386 / 1.570 / 1.620 | 1.037 / 1.758 / 2.327 | -0.349 / +0.188 / +0.707 |
| worker scheduling wait* | 14.340 / 16.404 / 18.030 | 21.983 / 29.600 / 30.758 | +7.643 / +13.196 / +12.728 |
| host-to-device/batch wrap | 0.027 / 0.032 / 0.041 | 0.028 / 0.042 / 0.045 | +0.001 / +0.010 / +0.004 |
| checkpoint preprocessing | 0.960 / 1.317 / 1.458 | 1.006 / 1.444 / 1.737 | +0.046 / +0.127 / +0.279 |
| pure model forward | 15.806 / 16.483 / 16.714 | 15.103 / 17.927 / 20.965 | -0.703 / +1.444 / +4.251 |
| device-to-host/output copy | 0.024 / 0.027 / 0.028 | 0.023 / 0.027 / 0.030 | -0.001 / 0.000 / +0.002 |
| checkpoint postprocessing | 0.228 / 0.246 / 0.252 | 0.213 / 0.437 / 0.684 | -0.015 / +0.191 / +0.432 |
| worker response serialization | 0.097 / 0.108 / 0.115 | 0.084 / 0.100 / 0.145 | -0.013 / -0.008 / +0.030 |
| worker socket send | 0.036 / 0.039 / 0.041 | 0.033 / 0.038 / 0.045 | -0.003 / -0.001 / +0.004 |
| raw response receive | 18.056 / 18.863 / 19.387 | 17.330 / 20.530 / 24.579 | -0.726 / +1.667 / +5.192 |
| response deserialization | 0.048 / 0.062 / 0.070 | 0.053 / 0.069 / 0.086 | +0.005 / +0.007 / +0.016 |
| temporal ensemble | not in shadow | 0.936 / 1.237 / 1.573 | physical-only |
| projection/action split | not in shadow | <0.21 / <0.22 / <0.23 | physical-only |
| logging/provenance enqueue | not in shadow | 0.162 / 0.199 / 0.228 | physical-only |

\* `worker scheduling wait` is the worker's wait for the next request. It is
not on the inference compute critical path and must not be summed with model
forward. It is larger at 25 Hz because the worker is intentionally idle
between queries. The physical query end-to-end latency was 26.75/30.17/33.83
ms (p50/p95/p99); the command-loop critical path was 1.71/3.13/3.80 ms.

The new measurements separate the likely source of the shadow-to-physical
difference: physical camera access and image preprocessing add roughly 1–3
ms, request serialization and send add roughly 3–7 ms combined at the tail,
and the model-forward tail is a further variable 4.25 ms at p99 in this
sample. Worker batch wrapping, device-to-host copy, worker serialization and
worker send are all sub-millisecond. Thus pure model forward does not
systematically double; the observed instability is host/runtime scheduling
plus data movement variance, not a blocking JAKA/RH56 write.

### Resource and persistence audit

The active topology during the rate runs was: two camera reader threads, one
RH56 feedback/serial worker, the native isolated JAKA worker, one ACT
inference thread, one persistent CUDA model worker process, and one bounded
rollout writer thread.  The model worker summary confirms LeRobot 0.6.2,
CUDA on NVIDIA Thor, zero inference failures, and checkpoint normalization
loaded from the checkpoint.  The normal writer remained active: over 60 s it
accepted 3,300 rows, reached queue depth 2, and dropped zero rows.  Camera
rates were 30.05/30.01 Hz with zero frame gaps in the 60 s run.  Therefore
there is no evidence of recorder backpressure, camera loss, serial timeout,
or JAKA deadline failure causing the rate selection.

The exact change needed for the operational goal was not a speculative model
or hardware rewrite.  It was to use the already implemented asynchronous
absolute-time temporal executor at the highest measured stable policy rate,
and to correct the false freshness rejection of old-but-valid ensemble
contributors.  This decouples inference from the 30 Hz command loop without
discarding future horizons.  No latency threshold was increased, no device
safety behavior was changed, and no new filesystem path was added to the
control loop.

### Offline/online semantic gate

The 25 Hz run retained full future chunks and mapped each horizon to the 30 Hz
timeline.  It did not use `consume_actions`; the summary's legacy
`consume_actions: 2` field is metadata inherited from the compatibility
schema and is not consulted by `async_temporal_ensemble`.  `prediction_points_capacity_dropped=0`,
`prediction_points_expired=0`, and `pending_prediction_drops=0` confirm that
the delayed h18–h32 predictions were not discarded by queue replacement.
This is the required distinction from legacy K=2/K=8/K=16 execution.

### Selected configuration and qualification gate

The highest stable command-disabled real-runtime configuration is:

```text
checkpoint: outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model
policy query: 25 Hz
command/control: 30 Hz
chunk: checkpoint-derived 60 steps
executor: async absolute-time temporal ensemble
coefficient: m=0.01
filtering/contact/JAKA/RH56 safety: unchanged
```

This is a **shadow-only** result.  A bottle rollout remains NO-GO until the
operator authorizes it in a later session and the selected-to-written
provenance and stop conditions are reviewed.  The exact next physical
qualification is not run here.
* `tools/act_shadow_model_worker.py` — separate worker receive wait from
  actual receive/decode timing.
* `tests/test_act_physical_rollout_adapter.py` — bounded timing-statistics
  regression test.
* this report.

The pre-existing user changes in
`src/rh56_driver/pc_direct_control.py` and
`tests/test_rh56_pc_direct_control.py` were not modified or staged by this
audit.

## Gate decision

**Next Strong ACT qualification: NO-GO.**

The current stationary B-D test does not demonstrate a sustained near-30 Hz
hardware-enabled loop, and every short run terminated on the existing policy
freshness gate. No bottle task was attempted. A valid next qualification
requires a command-disabled/inference-runtime change or a runtime condition
that can sustain the existing checkpoint’s full canonical 30/30 path while
preserving the current freshness and safety gates. Only then should one
operator-authorized bottle qualification be prepared.

## Validation

Focused ACT rollout adapter tests after the instrumentation changes:

```text
21 passed
```

The full repository suite after this audit was `742 passed, 4 skipped, 2
warnings`. The physical artifacts above are deliberately not part of the
default test suite.
