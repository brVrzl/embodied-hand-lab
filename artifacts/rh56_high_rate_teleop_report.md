# RH56 high-rate teleoperation result

Validation date: 2026-07-30. Branch: `dev/rh56-command-priority`.

## Implementation

RH56 serial access remains single-threaded and strictly serialized. The worker now
chooses one due operation at a time: forced/safety command, new latest target,
over-age STATUS/ERROR, ANGLE, then CURRENT/FORCE. Command and each feedback
register have independent monotonic deadlines. The existing 5 ms protocol delay
is retained per transaction, but five feedback reads no longer form a mandatory
blocking block before every command.

The latest-only mailbox, exact normalized duplicate suppression, forced/safety
bypass, structured worker failure, stale/error safety policy, and bounded
diagnostic logging remain active. Profiles `baseline`, `fast30`, `fast40`, and
`fast50` request 15/30/40/50 Hz command rates. `fast40` is selected as the default.

## Hand-only physical results

All four valid runs received nonzero Quest hand frames, produced nonzero retarget
targets, and completed nonzero serial writes. Each run lasted about 60 seconds.

| Profile | Quest frames | Retargets | Requested Hz | Successful write Hz | Unique submitted target Hz | Writes |
|---|---:|---:|---:|---:|---:|---:|
| baseline | 4,298 | 624 | 15 | 15.01 | 54.12 | 157 |
| fast30 | 4,298 | 2,805 | 30 | 29.14 | 53.04 | 1,363 |
| fast40 | 4,299 | 3,166 | 40 | 37.66 | 53.88 | 1,987 |
| fast50 | 4,291 | 2,819 | 50 | 38.23 | 48.04 | 2,016 |

`Unique submitted target Hz` measures producer-side exact target changes, not
successful serial writes. Every serial write attempt in these valid runs
succeeded.

| Profile | Command age p50/p95/p99/max ms | Submit-to-write p50/p95/p99/max ms | Deadline lateness p50/p95/p99/max ms | Serial utilization |
|---|---|---|---|---:|
| baseline | 19.25 / 22.09 / 22.74 / 23.72 | 25.93 / 29.15 / 30.74 / 31.27 | 1.35 / 6.03 / 7.06 / 7.50 | 44.24% |
| fast30 | 7.76 / 25.49 / 26.81 / 29.72 | 13.57 / 31.95 / 34.27 / 35.62 | 4.00 / 6.66 / 8.26 / 108.16 | 45.52% |
| fast40 | 12.64 / 21.65 / 26.41 / 27.97 | 18.81 / 28.40 / 32.50 / 35.65 | 2.06 / 11.44 / 13.47 / 15.15 | 50.77% |
| fast50 | 10.51 / 27.05 / 29.41 / 109.59 | 16.27 / 34.13 / 36.27 / 121.05 | 1.97 / 16.42 / 17.94 / 20.43 | 50.45% |

Fast30 materially reduced median command age from 19.25 ms to 7.76 ms. Fast40
improved p95 command age and submit-to-write over fast30 while increasing actual
writes to 37.66 Hz. Fast50 added only 0.57 successful writes/s over fast40 and
worsened p95/tail latency, so its higher requested value brought no practical
throughput benefit.

Feedback remained scheduled and bounded:

| Profile | ANGLE achieved Hz / max observed age ms | CURRENT | FORCE | STATUS | ERROR |
|---|---:|---:|---:|---:|---:|
| baseline | 15.00 / 60.55 | 14.99 / 65.98 | 14.99 / 66.53 | 14.99 / 70.16 | 14.99 / 65.89 |
| fast30 | 14.96 / 93.84 | 10.00 / 102.59 | 9.99 / 96.56 | 10.00 / 91.38 | 10.00 / 94.62 |
| fast40 | 14.96 / 91.10 | 10.00 / 96.26 | 10.00 / 96.14 | 10.00 / 92.00 | 10.00 / 96.48 |
| fast50 | 14.96 / 92.36 | 9.98 / 95.87 | 9.98 / 101.51 | 9.98 / 95.63 | 9.98 / 95.78 |

Fast30 and fast40 each recorded one isolated ANGLE warning interval, but no
sustained starvation; STATUS and ERROR remained well below their 250 ms warning
age. Valid runs recorded zero timeout, checksum, protocol, acknowledgement, or
worker failures. Duplicate suppression/coalescing remained active, and no stale
command was dropped.

The operator did not provide a separate text rating of physical feel during the
runs, so no subjective claim is invented. Timing and throughput show a clear
15-to-30 improvement, a smaller but real 30-to-40 improvement, and no useful
40-to-50 gain.

## Combined physical result

The combined run used `fast40` and stopped after about 21.02 seconds with
`control_heartbeat_transport_failure`. Native classified the stop as
`hard_timing_fault` / `consecutive_start_timing_misses`; therefore the run is a
physical **FAIL**, was not retried, and is not evidence that the arm path is
stable.

The native worker received 229 accepted targets (10.89 Hz), while the Python
adapter dispatched 35 new accepted targets. Producer heartbeat age p95/max was
17.30/30.05 ms, 200 shared control ticks exhausted the existing compute budget,
and shared-session control tick max was 29.75 ms. No controller alarm,
collision, E-stop, SDK error, or tracking hard crossing was reported. The
failure occurred in the existing arm timing/heartbeat path, which this task did
not modify.

Before the arm terminal stop, RH56 received 106 hand retargets and completed 89
writes at 32.29 Hz. Its live observed feedback ages remained bounded (ANGLE max
92.99 ms, STATUS 91.44 ms, ERROR 96.95 ms) with zero hand serial/protocol/worker
failure. The larger final snapshot ages were accumulated during coordinated arm
shutdown and are not live-operation starvation. The combined hand rate was below
the 37.66 Hz hand-only fast40 result because producer submissions fell to
38.19 Hz and the episode ended on the arm fault.

## Decision

`fast40` is the recommended/default profile: it achieves 37.66 successful writes/s,
keeps feedback fresh, and has better p95 command age than fast30. Fast30 remains a
lower-utilization fallback. Fast50 should not be used by default because actual
throughput saturated near 38 Hz while latency tails worsened.

The fixed full-five-register blocking cycle is removed. The remaining RH56
bottleneck is the 5 ms per-transaction protocol delay plus device/serial response
time, visible as saturation near 38 Hz when feedback remains enabled. The
combined arm path remains physically failed at its existing heartbeat/timing
gate and requires separate diagnosis before combined operation can be called
stable.

Evidence summaries:

- `logs/rh56_high_rate_baseline_20260730_155948.summary.json`
- `logs/rh56_high_rate_fast30_20260730_160245.summary.json`
- `logs/rh56_high_rate_fast40_20260730_160357.summary.json`
- `logs/rh56_high_rate_fast50_20260730_160514.summary.json`
- `logs/quest_jaka_rh56_combined_20260730_160632_3727787.summary.json`
- `logs/quest_jaka_rh56_combined_20260730_160632_3727787.native_metrics.json`
