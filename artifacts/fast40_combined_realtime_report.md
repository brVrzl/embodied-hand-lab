# Fast40 combined real-time diagnosis

Date: 2026-07-30 (Asia/Shanghai)

Status: **fast40 combined remains physically unverified at five minutes**. A
60.105 s live-Quest episode passed after CPU isolation and timing-path fixes.
A later live-Quest episode exposed observer-induced timing load and failed at
60.416 s. After correcting that instrumentation, a 200.943 s episode had zero
native hard timing faults but correctly stopped on Quest controller liveness
loss. No episode has completed the full 300 s gate.

## 2026-07-31 recurrence and trigger correlation

Eight combined native captures were recorded on 2026-07-31. Four ended in a
native cycle-start hard timing fault:

| Start | Elapsed | Native outcome | Period | Wake late | Terminal CPU |
|---|---:|---|---:|---:|---:|
| 14:04:45 | 14.165 s | consecutive start warnings | 11.520 ms | 3.520 ms | 8 |
| 14:07:28 | 158.796 s | consecutive start warnings | 10.024 ms | 2.024 ms | 8 |
| 14:10:13 | 6.884 s | consecutive start warnings | 11.568 ms | 3.568 ms | 3 |
| 17:18:45 | 14.221 s | single full-period wake miss | 15.982 ms | 8.042 ms | 6 |

All eight runs, including all four failures, recorded
`configured_control_cpu=-1`: the formal wrapper had not been given
`--native-control-cpu`. The native control thread therefore remained
`SCHED_OTHER`, priority 0, with affinity to all 14 allowed CPUs. Two terminal
cycles migrated CPUs exactly at the fault, one run migrated 28 ms before the
fault, and one had not migrated for more than 10 seconds. Migration is thus a
load/jitter amplifier but not a necessary cause; the direct terminating event
is an OS scheduler wake delay.

The trigger hypothesis is not supported as a direct cause. In the final
250 ms of all four failures, index was continuously `1.0` and grip
continuously `0.0`. Three runs had no clutch transition in the preceding
second. The remaining run captured its arm reference about 0.5 s before the
fault, far earlier than the terminal two-cycle warning pair. Three of the four
fault runs had zero hand-retarget events. The earlier Quest-absent unpinned
fault and an offline fake-worker test that transiently produced
`hard_start_period_miss` under test-suite load also require no trigger input.
Holding index engages the arm producer and can add host load, but pressing or
switching a trigger is neither necessary nor temporally sufficient.

The recurring operational defect was that CPU isolation remained optional in
the maintained wrapper and was absent from the combined-operation command
template, despite the previous physical evidence selecting CPU6. The gate now
requires an explicit verified `--native-control-cpu` before opening either
hardware path. No timing threshold, heartbeat timeout, fault classification,
or cleanup behavior was weakened.

## Fault chain and direct finding

The previously established primary/secondary ordering remains authoritative:

```text
native consecutive_start_timing_misses
  -> native hard_timing_fault
  -> native endpoint stops draining
  -> producer transport symptom (historical behavior)
```

The new terminal ordering closes/unlinks the target datagram endpoint before
SDK cleanup and large JSONL serialization, and persists native metrics before
cycle telemetry. In the new failing episode the wrapper reported
`abort_reason=hard_timing_fault` and `transport_symptom_reason=null`; the old
secondary heartbeat transport symptom did not obscure the native fault.

The isolated 17:23 episode reproduced the fault with:

- zero Quest datagrams, arm targets, native heartbeats, IK work, RH56 targets,
  and RH56 writes;
- `consecutive_start_timing_misses` at 23.805 s;
- terminal start period 10.680 ms and wake lateness 2.680 ms on CPU0;
- the preceding recorded cycle starting 9.755 ms after its predecessor with
  1.826 ms wake lateness;
- native SDK call p99 0.203 ms and maximum 0.533 ms;
- no migration within 526 ms of the terminal fault;
- no controller, cleanup, RH56 serial, protocol, checksum, ACK, queue, or
  logging fault.

This proves RH56 command traffic, Quest ingest, IK, and combined event logging
are not necessary causes. The direct failure is OS scheduling wake delay of a
normal `SCHED_OTHER`, priority-0 native thread with an unrestricted 14-CPU
affinity mask. CPU migration is a source of variability but not a sufficient
cause: only 3 of 20 warnings in the earlier complete episode coincided with a
migration, and its worst wake delay occurred 16.3 s after the last migration.

CPU0 was a poor control placement in the failing envelope. Between the first
warning and terminal fault it averaged about 1,516 IRQ/s and 567 softirq/s;
CPU6 averaged about 84 IRQ/s, 44 softirq/s, and 1.7% utilization. CPU frequency
was governed by `schedutil`; the terminal CPU was observed at 972 MHz. Pressure
files are unavailable on this kernel. The evidence does not justify changing
the hard-timing threshold or claiming one individual IRQ as the unique cause.

The 19:56 live-Quest episode identified an additional run-local cause. A
recoverable warning (9.205 ms period, 1.327 ms wake lateness) requested a full
OS snapshot even though it was not the first warning of the run. The observer
snapshot took 8.85 ms and overlapped the next absolute deadline. The following
cycle had a 10.097 ms period and 2.097 ms wake lateness, completing the
unchanged two-consecutive-warning hard-stop condition. The worker stayed on
CPU6 with zero migrations and SDK-call p99 was 0.141 ms. Repeated warning
snapshots were therefore an instrumentation-induced load amplifier.

## Implemented corrections

- `f83e3d8`: early target-endpoint shutdown, metrics-before-large-telemetry
  persistence, native placement/migration evidence, and bounded non-real-time
  warning/fault OS snapshots.
- `6805ee5`: combined Python, Quest receiver, RH56 serial/logging, and native
  process placement records.
- `4ee2316`: removed references to non-existent compute-budget counters that
  crashed summary construction after otherwise safe hardware cleanup.
- `49cefb6`: added start/warning/fault/shutdown load snapshots so per-CPU deltas
  can be computed.
- `7c01f27`: configurable `--native-control-cpu`; all existing Python/BLAS and
  later Quest/RH56 threads are moved to the remaining allowed CPUs, while the
  native control thread must verify a single-CPU affinity or fail explicitly.
  The observer remains on non-real-time CPUs. No scheduling priority, timing
  threshold, heartbeat, IK, feasibility, clutch, or safety policy changed.
- `56ae313`: applies native affinity after JAKA SDK setup so SDK helper threads
  do not inherit the isolated control CPU.
- `2baacf5`: bounds measured RH56 activation commands to the configured command
  envelope while preserving the measurement as the delta reference.
- `10b56ef`: re-arms a completion miss from completion time, preventing an
  immediate catch-up ServoJ command.
- `549df74`: requests the bounded `/proc` snapshot only for the first timing
  warning. Terminal and shutdown snapshots, timing thresholds, and fail-closed
  behavior remain unchanged.

The repair validation will explicitly use CPU6 on the observed 14-CPU host.
The CPU number is not hard-coded and isolation is disabled when the option is
omitted.

## Physical episodes so far

### 17:18:50, live Quest, unpinned

Log prefix:
`logs/quest_jaka_rh56_combined_20260730_171850_3760846`

- 10,514 Quest datagrams and normal arm/hand activity;
- native elapsed 60.103 s, `operator_stop_command`, 7,498 cycles;
- 1,812 accepted native target packets, including 1,696 heartbeats;
- hard timing faults 0; timing warnings 20; migrations 25;
- period p95/p99/p99.9/max 8.004/8.060/9.797/14.323 ms;
- wake p95/p99/p99.9/max 0.063/0.085/1.860/6.382 ms;
- controller and cleanup faults 0;
- RH56 successful writes 283 while hand targets were active, successful rate
  18.23 Hz over that active interval, unique target rate 24.24 Hz;
- feedback achieved rates ANGLE 15.00 Hz and CURRENT/FORCE/STATUS/ERROR
  approximately 10.00 Hz, all warning/failure counts zero;
- summary creation failed after hardware cleanup because three report fields
  referenced counters absent from the production session.

This is useful physical timing evidence, but not an overall gate PASS because
the required summary failed and it is only one 60 s segment.

### 17:23:18, Quest absent, unpinned

Log prefix:
`logs/quest_jaka_rh56_combined_20260730_172317_3762933`

- Quest datagrams and arm/RH56 targets 0;
- native elapsed 23.805 s and failed at the retained hard-timing gate;
- primary classification `hard_timing_fault`, no transport symptom;
- RH56 feedback remained bounded and fault-free; RH56 writes 0;
- this is an isolation/failure reproduction, not combined teleoperation.

### 19:40:01, live Quest, CPU6 isolated

Log prefix:
`logs/quest_jaka_rh56_combined_20260730_194001_3792423`

- elapsed 60.105 s, `duration_complete`, valid combined summary;
- 7,495 native cycles, zero hard timing misses and zero CPU migrations;
- 26 recoverable warnings/realignments; period p99 8.064 ms and maximum
  12.691 ms; wake p99 0.084 ms and maximum 4.750 ms;
- controller, cleanup, transport, RH56 worker, and RH56 protocol faults zero;
- RH56 completed 190/190 writes; feedback was approximately 15 Hz for ANGLE
  and 10 Hz for CURRENT/FORCE/STATUS/ERROR with no age warning.

This is a bounded 60-second physical PASS, not five-minute validation.

### 19:56:37, live Quest, CPU6 isolated

Log prefix:
`logs/quest_jaka_rh56_combined_20260730_195637_3799451`

- elapsed 60.416 s; primary `consecutive_start_timing_misses` / native
  `hard_timing_fault`; `control_heartbeat_transport_failure` remained a
  secondary transport symptom;
- terminal period 10.097 ms and wake lateness 2.097 ms; the preceding warning
  period was 9.205 ms with 1.327 ms wake lateness;
- CPU migrations, controller faults, cleanup faults, and RH56 faults zero;
- RH56 completed 301 writes at 29.83 Hz, with unique targets at 37.24 Hz;
- the repeated warning snapshot immediately before the terminal cycle took
  8.85 ms and overlapped the next deadline.

This is a physical FAIL and motivated `549df74`. No automatic retry occurred.

### 20:04:22, live Quest, CPU6 isolated, post-observer fix

Log prefix:
`logs/quest_jaka_rh56_combined_20260730_200422_3802773`

- elapsed 200.943 s; native hard timing misses zero, migrations zero, accepted
  target rate 40.05 Hz, SDK-call p99 0.163 ms;
- period p99 8.014 ms and wake p99 0.070 ms; no terminal timing object;
- controller alarms, native cleanup faults, transport symptoms, RH56 serial,
  protocol, logging, and worker faults zero;
- hand frames remained live at about 70.07 Hz;
- immediately before the stop, fresh CTRL datagrams explicitly reported
  `connected=1, active=0, tracked=1, index=0, grip=0`. The producer correctly
  stopped heartbeats for this protocol-invalid controller state, and the native
  100 ms liveness gate stopped with `command_stream_timeout` /
  `producer_liveness_loss`;
- this was not ordinary clutch release: current protocol validity requires
  `connected && active && tracked && fresh`.

This is a physical FAIL due to actual controller liveness loss. Weakening that
gate is not an acceptable repair.

## Offline validation

The CPU-isolation and evidence batch passed:

```text
150 passed in 22.92s
```

The suite covered the shared Quest/JAKA pipeline, output feasibility,
singularity/liveness, EDG resampling, native worker, bounded/combined entry,
RH56 serial backend, and RH56 PC-direct scheduler. The native affinity test
verifies the requested CPU is the sole actual affinity and migration count is
zero. Compileall, shell syntax, build, and `git diff --check` also passed.

## Remaining physical validation

Run the fast40 combined wrapper's 300 s default with Quest HTS and CTRL active
for the complete segment and explicit CPU6 isolation. Confirm non-zero hand
frames, arm targets, RH56 targets, and command writes before counting it. Stop
at the first hard fault or actual input liveness loss. Required evidence is:

- native actual affinity `{6}`, migration count 0, and observer off CPU6;
- zero native hard timing/controller/cleanup faults;
- zero RH56 serial/protocol/worker faults and bounded STATUS/ERROR age;
- bounded producer heartbeat and queue/drop counts;
- stable arm accepted-target/heartbeat path and non-zero normal hand activity;
- timing and command-age comparison against the unpinned episodes.

Until one full 300 s episode completes, do not claim five-minute physical
stability.
