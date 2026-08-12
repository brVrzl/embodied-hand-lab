# Physical collection root-cause report

Date: 2026-08-11 (Asia/Shanghai)
Evidence: source inspection, session `quest_jaka_combined-normal-teleop_20260811_100941_2700851`,
and offline analysis of physical episodes 59--64. No hardware was accessed.

## Online versus offline 30 Hz reconstruction

The fixed period is `round(1e9/30) = 33,333,333 ns`. `expected` is the number of nominal slots from
the first through last online canonical timestamp. `offline` is the existing causal synchronizer's
timeline length over the same interval. Camera validity uses the configured 100 ms tolerance.

| episode | online rows | expected slots | offline slots | missing online | offline valid JAKA / ANGLE / FORCE | offline valid workspace / wrist |
|---:|---:|---:|---:|---:|---:|---:|
| 59 | 725 | 849 | 849 | 124 | 849 / 849 / 849 | 744 / 745 |
| 60 | 673 | 811 | 811 | 138 | 811 / 811 / 811 | 801 / 801 |
| 61 | 289 | 390 | 390 | 101 | 390 / 390 / 390 | 370 / 370 |
| 62 | 1465 | 2218 | 2218 | 753 | 2218 / 2218 / 2218 | 1872 / 1872 |
| 63 | 441 | 479 | 479 | 38 | 479 / 479 / 479 | 471 / 471 |
| 64 | 422 | 469 | 469 | 47 | 469 / 469 / 469 | 454 / 454 |

This does not prove that every missing slot has a unique acquired camera image: the offline utility
can reuse a previous causal source and marks cameras invalid outside tolerance. It does prove that
the low-dimensional source observations retained on either side of the live gaps are sufficient to
construct a complete fixed timeline without future FORCE.

At online timestamps, the offline latest-causal source often differs from the live row: JAKA in
42--396 rows per episode, ANGLE in 14--238, and FORCE in 11--188. A later row can expose an
observation whose source timestamp was already before an earlier canonical deadline, while the live
30 Hz snapshot had not yet published it. This is acquisition/publication phase loss, not merely a
different display frame index.

## Proven issues

### P0: nested source values can be from the future of the canonical deadline

**Evidence.** Episodes 60, 62, 63 and 64 contain respectively 7, 58, 7 and 0 future ANGLE rows;
episodes 62, 63 and 64 contain 13, 1 and 4 future FORCE rows. Episode 62 also has one future JAKA
command timestamp. Maximum positive offsets are 7.99 ms for ANGLE, 9.16 ms for FORCE and 0.71 ms
for JAKA command. JAKA measured observation is never future in these episodes.

For representative episode 62 frame 576:

- canonical deadline: `2304806606724491`;
- stored producer control timestamp: `2304806606713600`;
- FORCE register timestamp: `2304806609035441` (2.311 ms after canonical and 2.322 ms after the
  producer timestamp);
- source arrival/row-construction timestamps are not persisted separately, so they cannot be
  reconstructed exactly from this episode.

The source code supplies the missing ordering evidence: `now_ns` is captured at the beginning of a
60 Hz tick. Status polling, the complete session control tick and cache reads happen afterward.
`ControlSample.host_monotonic_ns` is nevertheless set to that old `now_ns`. RH56 feedback values and
per-register timestamps are also fetched through separate, unsynchronized accesses. A serial cache
update between those operations can therefore be packaged as if it existed at tick start. The
collector causally selects only the outer `ControlSample` timestamp; it does not causally select the
nested modality timestamps.

The staging validity fields detect the positive offset for current v2 rows, but the future ANGLE
value remains inside the 12-D `observation.state` and the future FORCE value remains inside
`observation.force`. A false mask does not make an unconditional ACT input causal.

**Root cause.** An early outer timestamp plus a non-atomic composite RH56 snapshot and bundled
control-history selection.

**Minimal fix.** Take one coherent lightweight feedback snapshot, timestamp the completed producer
snapshot after the cache reads, validate domains, and have the recorder retain small per-modality
histories so each canonical deadline selects the newest source timestamp `<= t`. If none exists,
retain shape but mark that source invalid. Do not interpolate FORCE.

**Regression risk.** Accidentally changing hand safety feedback, command timing, or replacing an
action with an observation. The snapshot must be read-only and must not add a serial transaction.

**Tests.** Delayed producer with future ANGLE and FORCE; domain mismatch; independently selected
10 Hz FORCE, 15 Hz ANGLE and ~31 Hz JAKA histories; no future value after delayed iteration.

### P1: the canonical clock is driven by a same-rate, conditionally suppressed source

**Evidence.** The clock is fixed-deadline and correct, but the hardware producer calls
`ingest_control` only when its own 30 Hz deadline is due. A 30 Hz consumer driven by a 30 Hz source
has no phase margin. In 59--64 there are 671 gap events and 1,201 skipped nominal slots. Many are
single-slot gaps while a modality remains active: for example arm->arm accounts for 100 slots in
episode 60, 70 in 61, 304 in 62, 22 in 63 and 16 in 64. The retained producer window shows
`complete_outer_tick` p50 10.96 ms, p95 12.30 ms, p99 13.14 ms and max 17.75 ms. The native process
remained healthy (124.96 Hz, 3 isolated warning/missed deadlines, zero hard misses).

In addition, the producer stops all dataset publication as soon as both clutches are released even
though the episode is not rotated until release lasts five seconds. Re-engagement inside that
window resumes the same episode with the original canonical origin. Endpoint evidence includes 51
idle->arm skipped slots in episode 59 and, in episode 62, 128 arm->idle + 114 idle->idle + 55
idle->arm skipped slots. Endpoint state cannot identify every interior tick, so these are lower-bound
attributions, not invented per-tick traces.

**Root cause.** Canonical decision-making is coupled to an outer snapshot published at exactly the
dataset rate and to a clutch/boundary UI gate. Lower-rate source reuse is supported inside the
collector but cannot run when no compact control sample arrives.

**Minimal fix.** Publish the already-constructed compact control snapshot on each 60 Hz producer
tick while an episode is active, including the boundary debounce interval. Keep actual
`arm_trigger`/`hand_grip` fields unchanged. Keep lifecycle start and five-second external rotation
semantics separate from the row's clutch state. The recorder remains the sole fixed 30 Hz selector;
it still emits no catch-up bursts.

**Regression risk.** A false capture-active value could prematurely finish or incorrectly start an
episode; doubling compact queue traffic could expose insufficient queue headroom. Tests must cover
start, idle-within-active-episode, release boundary, and bounded queue behavior. Physical validation
must confirm zero recorder drops.

**Tests.** 60 Hz source driving fixed 30 Hz deadlines; multiple expired wall deadlines without
bursts; 30 Hz timeline with slower FORCE/ANGLE and reusable camera frame; idle control rows do not
end an externally bounded active episode.

### P1: episode 62's 4.3 s FORCE gap is a recording-observability gap

**Evidence.** The largest unique FORCE timestamp interval is 4298.693 ms, between source timestamps
`2304810198963681` and `2304814497656187`. It spans canonical rows 670 to 672, whose canonical
timestamps differ by 4333.333 ms. The largest canonical deadline gap is 4300.000 ms (128 skipped
slots) over the same interval. The complete session summary reports achieved FORCE rate 9.9993 Hz,
zero FORCE failures, zero serial feedback timeouts, and complete ANGLE feedback at 14.996 Hz.

**Root cause.** No canonical/control row observed the intermediate FORCE cache generations. Current
production diagnostics intentionally do not persist per-register transaction rows, so individual
serial operations inside that interval cannot be replayed, but all available scheduler evidence
contradicts a 4.3 s FORCE scheduler stop.

**Minimal fix.** The same continuous compact publication and modality-history fix above. Do not
change FORCE rate or add a serial read/log stream.

**Regression risk.** Misreporting repeated 30 Hz force values as new measurements. Preserve the
original FORCE timestamp, raw counts and zero-order-hold age.

**Tests.** 10 Hz FORCE over a 30 Hz timeline and no future FORCE after delayed producer wake.

## Investigated hypotheses not supported as root cause

- **Recorder/video backpressure:** P2 monitoring only. Session queue HWM was 2 for recorder ingress
  and 4 for the async writer; all drops, ring expiry, writer failures and process errors were zero.
  Online MP4 encoding is therefore not changed.
- **Camera acquisition loss inferred from canonical gaps:** rejected. Canonical frame-number jumps
  include frames skipped by sparse canonical selection. Camera-process frame-gap counters and
  canonical repeated-frame counters remain separate.
- **New-status/new-feedback requirement:** rejected. Existing code reuses cached JAKA and RH56
  state. It merely fails to invoke the selector without an outer snapshot.
- **Cross-clock comparison:** no observed P0 in 59--64. Host robot/control timestamps share
  `CLOCK_MONOTONIC`; RealSense device time is mapped explicitly offline. Domain validation should be
  enforced at the source-history boundary as defense against future configuration changes.
- **RH56 serial starvation/timeout:** not supported for the 4.3 s interval; see above.
- **Quest latest-only modality merge:** fixed in current source. Right, left and head are independent;
  CTRL remains ordered. Existing interleaving tests cover the former regression.
- **Unbounded diagnostic memory or synchronous hot-path filesystem I/O:** not found in production.

## Lower-priority findings

| Priority | Issue | Evidence / consequence | Minimal action |
|---|---|---|---|
| P2 | Async-writer percentile summary uses unsorted samples | Session summary can report p50 greater than `max`; drop/error counts remain trustworthy | Fix separately; do not mix into causal P0/P1 patch |
| P2 | Live rows do not persist producer wake, source availability and row-build completion as three distinct timestamps | Exact scheduling-stage attribution is impossible for old episodes | Do not add hot logging; if needed, add bounded production counters or compact timestamps only in a separately reviewed change |
| P2 | Per-episode camera acquisition loss is not separable from shared-session final counters | Canonical frame jumps cannot prove RealSense loss | Preserve current process counters; validate next run at session and episode boundaries |
| P3 | Offline camera clock fit is estimated from selected canonical pairs | Sparse online rows reduce fit evidence but do not create future FORCE | Keep explicit residual/domain reporting |

## Phase 2 gate

Phase 1 establishes two changes as necessary: causal nested-source selection (P0), and decoupling
fixed 30 Hz canonical decisions from same-rate/idle-suppressed publication (P1). It does not justify
changes to teleoperation mapping, IK, native ServoJ, RH56 scheduling/safety, camera architecture,
online encoding, collection schema dimensions, or realtime placement.

## Phase 2 implementation and offline validation

The minimal patch implements the Phase 1 gate without changing control or safety behavior:

- the RH56 serial worker atomically publishes one immutable dataset view containing the already-read
  feedback plus matching ANGLE_ACT/FORCE_ACT register timestamps; no serial operation was added;
- the producer stamps the compact dataset snapshot after cache reads and publishes it at its existing
  60 Hz outer cadence while an externally bounded episode is active, including a sub-five-second
  both-clutch release interval;
- the recorder's bounded outer control history is searched independently for JAKA observation,
  JAKA command provenance, RH56 ANGLE and RH56 FORCE. A selection requires both source timestamp
  `<= canonical` and compact-snapshot availability `<= canonical`;
- a known low-dimensional source in a non-host-monotonic domain is rejected at the recorder boundary;
- the canonical clock, expired-deadline skip accounting and no-catch-up behavior are unchanged.

The focused causal/profile/RH56/dataset set passed 57 tests. The safety-critical Quest/JAKA,
native, process-isolation, nonblocking recorder, dual-clutch and RH56 set passed 168 tests. The full
offline suite passed 653 tests with two pre-existing multiprocessing/fork deprecation warnings.
`compileall` and `git diff --check` also passed.

This is not a physical PASS. The next hardware run must confirm that doubling compact control
snapshot publication does not raise recorder queue drops or native deadline misses, and must measure
the episode tail produced when the five-second release debounce actually rotates an episode.
