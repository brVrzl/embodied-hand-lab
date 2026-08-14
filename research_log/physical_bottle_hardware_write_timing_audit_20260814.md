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
  command/query records, safe A-D isolation options, and canonical summary
  compatibility fix.
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
