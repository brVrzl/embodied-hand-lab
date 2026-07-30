# Fast40 combined real-time diagnosis

Date: 2026-07-30 (Asia/Shanghai)

Status: **fast40 combined remains physically unverified**. One 60.10 s hardware
control episode completed without a native/controller/RH56 fault, but its
Python summary construction failed after cleanup. A second episode reproduced
the native timing fault with Quest streaming absent and therefore was not a
normal combined teleoperation run. The CPU-isolation correction is offline
validated but still requires physical validation with live Quest input.

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

With Quest HTS/CTRL streaming active, run fast40 combined using explicit CPU6
isolation. Confirm non-zero Quest hand frames, arm targets, RH56 targets, and
command writes before counting a segment. Complete consecutive independent
60 s bounded episodes equivalent to the requested 2–3, 5, and 10 minute
envelopes, stopping at the first hard fault. Required acceptance evidence is:

- native actual affinity `{6}`, migration count 0, and observer off CPU6;
- zero native hard timing/controller/cleanup faults;
- zero RH56 serial/protocol/worker faults and bounded STATUS/ERROR age;
- bounded producer heartbeat and queue/drop counts;
- stable arm accepted-target/heartbeat path and non-zero normal hand activity;
- timing and command-age comparison against the unpinned episodes.

Until those episodes complete, do not claim physical stability.
