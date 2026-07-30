# 2026-07-30 combined teleoperation failure analysis

Validation level: offline analysis of a failed physical capture. No hardware was
opened or commanded during this analysis.

Evidence prefix:
`logs/quest_jaka_rh56_combined_20260730_160632_3727787`.

## Finding

The primary recorded stop was the native worker's
`consecutive_start_timing_misses`, classified as `hard_timing_fault`. The
summary's `control_heartbeat_transport_failure` was a later transport symptom,
not the initiating fault.

The native loop stopped draining its bounded Unix datagram target socket after
the timing fault, then performed controller cleanup and serialized 2,615 stored
cycle rows (8.48 MB) before writing metrics and exiting. The Python producer
continued to generate valid HOLD_REJECTED heartbeats during that teardown
window. Several sends still entered the finite socket buffer; it then filled and
one non-blocking send returned false. At that instant the process had not yet
been reaped and its metrics file was not yet observable, so the wrapper recorded
the secondary heartbeat transport symptom.

This explains the stop classification race, but it does not prove why the host
scheduler delivered the two consecutive late native starts. The capture lacks
the terminal cycle telemetry row and does not record CPU number, scheduling
class changes, migration, IRQ load, or per-process contention. The native hard
timing policy is therefore retained unchanged.

## Monotonic timeline

The time origin below is the first recorded native cycle,
`1289281311440658 ns` (`CLOCK_MONOTONIC`).

| Relative time | Monotonic ns | Event |
|---:|---:|---|
| 1.592961 s | 1289282904401213 | Native start warning; arm and RH56 command paths inactive. |
| 2.818061 s | 1289284129502142 | Native start warning; arm and RH56 command paths inactive. |
| 5.596641 s | 1289286908081988 | Native start warning; arm and RH56 command paths inactive. |
| 15.889614 s | 1289297201054650 | First accepted arm target. |
| 16.058954 s | 1289297370394740 | First compute-budget-exhausted HOLD_REJECTED heartbeat. |
| 16.293745 s | 1289297605185960 | Native start warning. |
| 16.912620 s | 1289298224060360 | Native start warning. |
| 18.055798 s | 1289299367238335 | First RH56 scheduled command record. |
| 20.732599 s | 1289302044039545 | Native start warning: 11.920 ms period, 3.979 ms wake lateness. |
| 20.780226 s | 1289302091666174 | Last RH56 scheduled command record. |
| 20.926337 s | 1289302237778156 | Last recorded native cycle and first terminal warning: 9.129 ms period, 1.739 ms wake lateness. |
| 20.927739 s | 1289302239180125 | Last recorded native cycle ends after 1.402 ms work; SDK command portion was 0.116 ms. |
| 21.036621 s | 1289302348061972 | Last producer tick whose heartbeat send still returned true while native no longer drained the socket. |
| 21.058190 s | 1289302369630735 | Producer starts the tick whose heartbeat send returned false. |
| 21.061088 s | 1289302372529035 | Last independent RH56 telemetry record. |
| <=21.078654 s | <=1289302390094453 | Failed heartbeat detected by the end of the 20.464 ms producer tick. |

The terminal native start row is included in aggregate metrics count (2,616)
but not in the 2,615-row cycle JSONL because the loop breaks during start-time
validation before allocating cycle telemetry. Aggregate maxima remained
11.920 ms start period and 3.979 ms wake lateness, below the separate 16 ms/full
period hard-start boundaries. The hard stop came from the retained policy that
escalates two consecutive start warnings.

## Load, queue, and timing evidence

- Native command-write p95/p99/max was 0.115/0.176/4.253 ms. The last recorded
  cycle used 1.402 ms total and 0.116 ms in command write, so its late start was
  wake scheduling, not same-cycle SDK execution.
- Native worker CPU was 58.64% of one CPU over 21.02 s. The host exposes 14
  CPUs and the code requests neither real-time scheduling nor affinity. The
  historical run did not record CPU placement, so priority/affinity starvation
  remains unproven.
- Producer heartbeat age at native was p95 17.30 ms, p99 21.46 ms, max
  30.05 ms. Heartbeat dispatch cost remained below 0.136 ms until the terminal
  false return. Production and transport were therefore healthy before the
  native timing stop.
- The arm producer entered its 20 ms compute-budget path at 16.06 s and then
  ran mostly at about 45 Hz. It generated 194 heartbeats and exhausted the
  budget 200 times. This is substantial Python load, but it did not create a
  native liveness timeout; the native outcome was a start-scheduling fault.
- Native start warnings also occurred at 1.59, 2.82, and 5.60 s, before arm
  engagement and before RH56 commands. Command-priority serial scheduling is
  therefore not a necessary cause of the timing jitter.
- RH56 commands were active from 18.06 to 20.78 s, so they overlap the final
  timing pair but do not establish causality. RH56 had no worker/serial fault,
  no worker overrun, a one-slot latest-only mailbox, and 51 coalesced targets.
- Quest input drops, RH56 telemetry drops, and RH56 logging failures were all
  zero. No pre-fault target transport drop was observed; all earlier target or
  heartbeat dispatches reported success.
- Combined event logging was synchronous. JSON serialization max was 1.109 ms
  and write max was 5.325 ms. RH56 logging was bounded and buffered. Native
  cycle JSONL was accumulated in memory during control and written only after
  the fault, so that 8.48 MB write explains the classification window but could
  not cause the initiating native late wake.
- The 32.29 Hz RH56 diagnostic rate covers the active hand-command window, not
  the full 21 s episode. Hand commands began after earlier native warnings and
  ceased before the native stop; the reduced combined rate is not evidence of
  heartbeat backpressure.

## Offline correction

The combined wrapper now reconciles a provisional
`control_heartbeat_transport_failure` or `IPC_failure` after native cleanup and
metrics loading. A nonzero native error with a typed fault classification becomes
the authoritative `abort_reason`; the original transport symptom is retained as
`transport_symptom_reason`. Future summaries also record bounded arm transport
sent/drop counters.

Native metrics now also preserve a terminal hard-timing object even when the
loop exits before allocating its ordinary cycle row. It records the phase,
exact `CLOCK_MONOTONIC` timestamp, actual start period, wake/completion
lateness, consecutive warning count, and CPU number. This is additive
observability; it does not change wake scheduling or fault policy.

The deterministic regression uses the captured native outcome
`consecutive_start_timing_misses`, `error_code=1`, and
`stop_classification=hard_timing_fault`. It verifies that the final reason is
`hard_timing_fault` while the heartbeat symptom remains visible. A normal native
completion is explicitly not relabelled.

No timing threshold, heartbeat timeout, IK budget, clutch behavior, controller
gate, or native fail-closed action changed.

## Remaining risk and required validation

The OS-level cause of the consecutive native wake delays remains unresolved.
Before combined operation can be called stable, a separately authorized
physical run must retain the current hard timing policy and capture:

- the new native terminal timing object and enough system scheduling evidence
  to interpret its CPU placement;
- producer sent/drop counters and the reconciled primary/symptom reasons;
- the same per-stage producer timing, native cycle telemetry, RH56 diagnostics,
  controller status, and exact cleanup ordering;
- at least one full bounded combined duration without native hard timing,
  controller, liveness, or RH56 fault.

Hand-only fast40 remains physically validated in its recorded envelope. This
offline correction and the failed combined capture do not validate combined
physical stability.
