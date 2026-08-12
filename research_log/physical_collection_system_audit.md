# Physical collection system audit

Date: 2026-08-11 (Asia/Shanghai)
Scope: current `combined-normal-teleop` + `lerobot_staging_v1` production path and physical episodes 59--64.
Validation level: source audit and offline log replay only. No hardware was opened or commanded.

## Architecture and data flow

```text
Quest UDP
  CTRL -------- ordered bounded FIFO ---------+
  right HTS --- independent latest slot ------|
  left HTS ---- independent latest slot ------+--> Python 60 Hz producer
  head HTS ---- independent latest slot ------|      clutch -> mapping/filter -> IK
                                                 |      -> feasibility/safety
                                                 |      -> AcceptedArmTarget
                                                 |             |          |
                                                 |             |          +--> RH56 latest target mailbox
                                                 |             +-------------> JAKA target Unix datagram
                                                 |
                                                 +--> compact ControlSample queue
                                                         |
RealSense workspace process -> shared ring -> descriptor -+--> recorder process
RealSense wrist process -----> shared ring -> descriptor -+      fixed 30 Hz selector
                                                                -> staging JSONL + MP4

JAKA native 125 Hz ServoJ worker -- compact ~31.25 Hz status --> Python cache
RH56 serial worker -- ANGLE 15 Hz / FORCE 10 Hz / other 10 Hz -> Python cache

staging JSONL + camera provenance --> offline causal synchronizer/materializer
```

The Quest modality split currently has the required semantics. `CTRL` packets are retained in
arrival order. Right hand, left hand, and head each have an independent latest slot, so a newer
right-hand datagram cannot supersede a pending head or left-hand datagram. Receive timestamps and
payload bytes are passed through unchanged. The previous single combined hand/head latest slot is
not present in the audited code.

## Boundary inventory

`CLOCK_MONOTONIC` below means the Linux host monotonic clock shared by Python and the native
worker on this machine. `global_time` is a RealSense device-reported domain and is not directly
compared with host monotonic time online.

| Boundary | Producer -> consumer / execution context | Nominal rate | Timestamp and domain | Buffer/cache and capacity | Semantics and overflow | Serialization / filesystem | May block producer? |
|---|---|---:|---|---|---|---|---|
| Quest UDP receive | Quest -> `quest-hts-receive` thread | device-driven | receive `time.monotonic_ns`; original payload timestamps retained | kernel UDP socket, then channels below | malformed input still reaches parser/fault boundary | none unless optional datagram recorder supplied | socket receive blocks only its dedicated thread, with 20 ms timeout |
| CTRL ingress | receive thread -> Python producer | event-driven; producer drains at <=60 Hz | original receive monotonic timestamp | `queue.Queue`, default capacity 256; producer drains at most 32 per pass | ordered event FIFO; full drops newest and counts it | none | producer uses `get_nowait`, never blocks |
| right-hand HTS | receive thread -> Python producer | device-driven | original receive monotonic timestamp | one independent latest slot | latest state; superseded right samples counted | none | lock is short; no I/O |
| left-hand HTS | receive thread -> Python producer | device-driven | original receive monotonic timestamp | one independent latest slot | latest state; superseded left samples counted | none | lock is short; no I/O |
| head HTS | receive thread -> Python producer | device-driven | original receive monotonic timestamp | one independent latest slot | latest state; superseded head samples counted | none | lock is short; no I/O |
| HTS canonical state | producer/router -> `HtsCanonicalAssembler` and `PoseSampleBuffer` in producer process | consumed <=60 Hz | original host receive monotonic timestamps | canonical modality state plus pose buffer capacity 16 | continuous state; assembler retains modalities independently | none | bounded local work |
| CTRL canonical state | ordered CTRL packets -> controller provider / clutch machines | event-driven | packet source timestamp plus host receive monotonic | receiver FIFO above; provider retains latest validated state after processing every packet | press/release order is processed, not coalesced | none | bounded by 32 packets per outer iteration |
| Python control tick | producer thread -> mapping/filter/IK/safety | configured 60 Hz | tick `time.monotonic_ns` | no work queue; current canonical state | latest continuous state plus ordered clutch state | no production filesystem I/O; sparse event object only when due | IK can consume the configured 20 ms compute budget; fixed scheduler skips expired producer ticks |
| Accepted JAKA target | Python adapter -> native worker | accepted target about 60 Hz; native 125 Hz | Python host monotonic generation/receive/dispatch | nonblocking Unix datagram; finite kernel send buffer (publisher requests about 8 packets) | native drains newest valid sequence; latest command semantics; drops counted | none | nonblocking send |
| JAKA compact status | native ServoJ thread -> Python cache | every 4 cycles, about 31.25 Hz in production | observation is native read-end `CLOCK_MONOTONIC`; command and worker timestamps same domain | nonblocking Unix datagram / latest runtime status cache; kernel capacity | compact latest state; a missed packet does not block ServoJ | none; detailed cycle telemetry disabled in production | native `MSG_DONTWAIT`; Python poll nonblocking |
| Native ServoJ loop | native worker -> JAKA EDG API | 125 Hz | absolute `CLOCK_MONOTONIC` deadlines | current target and bounded native state | latest target with all velocity/acceleration/jerk, tracking, health and watchdog checks retained | final metrics only after realtime scheduler is restored; no cycle rows in production | SDK calls occur in dedicated realtime process, not Python producer |
| RH56 target | Python producer -> RH56 serial worker | submitted <=60 Hz; command scheduler 40 Hz | host monotonic submission timestamp | one latest pending target slot | latest command state; unobserved targets coalesced and counted | none | lock + wake event only |
| RH56 serial operations | serial worker -> RH56 device/cache | command 40 Hz; ANGLE 15 Hz; FORCE/CURRENT/STATUS/ERROR 10 Hz | host monotonic transaction timestamp; all on same host clock | one worker, one selected due operation per cycle; immutable latest feedback object plus register timestamp cache | command can preempt a due feedback operation; fixed per-operation deadlines skip expired occurrences; safety freshness remains authoritative | no production telemetry serialization/I/O; diagnostic async sink only in diagnostic profile | serial I/O blocks only serial worker up to configured timeout; producer reads cache |
| Camera acquisition | each D435 -> its own process/shared ring | configured 30 Hz | host timestamp immediately after `wait_for_frames`; device ms/domain/frame number retained | per-camera shared ring capacity 16 and descriptor queue capacity 16 | latest-oriented descriptors; oldest descriptor replaced when full and counted; seqlock consistency checked | none in camera process | may block only camera process in librealsense |
| Camera forwarding | parent-side `episode-camera-forwarder` thread -> recorder camera queues | as frames arrive, about 30 Hz per role | descriptor timestamps unchanged | parent reference deque 16; separate workspace/wrist recorder queues, each 16 | latest-oriented per role; oldest camera command dropped when full and counted | none | independent non-realtime thread; teleop producer is not the pump |
| Low-dimensional recording input | Python producer -> recorder process | currently gated to 30 Hz and only while `episode_capture_active` | `ControlSample.host_monotonic_ns` currently taken at control-tick start | dedicated control queue capacity 16 | ordered snapshots; full is a recorder error, not shared with camera queues | compact object pickling by multiprocessing transport; no filesystem I/O in producer | `put_nowait`; nonblocking unless failure handling runs |
| Recorder command scheduling | camera/control queues -> recorder child | event-driven | source timestamps preserved | three queues above; recorder also uses bounded writer queue 16 | camera commands are drained before a pending control command; no evidence of overflow in session 59--64 | no direct file work in command producer | recorder child can be delayed by camera/writer work, but cannot change a `ControlSample` timestamp |
| Canonical clock | recorder `SingleEpisodeCollector` -> canonical writer | fixed 30 Hz | host monotonic `t0 + k*round(1e9/30)` | `CanonicalClock`; control history 4096, camera histories 16 | emits at most one due slot per input; expired deadlines are counted/skipped, no catch-up burst | canonical enqueue only; asynchronous writer does JSON/MP4 | recorder process only; does not block robot control |
| Async staging writer | recorder collector -> JSONL/MP4 files | selected 30 Hz rows | all provenance retained | queue 16, batch 8, bounded diagnostic windows 4096 | drop/error/ring-expiry counters retained | JSON serialization, OpenCV conversion/MP4 and flush here | dedicated writer thread, never the teleop producer |
| Sparse event log | Python producer -> event-log worker | faults/edges/boundaries and about 1 Hz health | host monotonic | queue 256 | drop newest and count | worker serializes and flushes | producer only copies/enqueues a due record |
| Offline synchronization | staging files -> offline utility | requested 30 Hz | maps RealSense device clock to host from paired timestamps; robot sources already host monotonic | in-memory episode observations | latest causal JAKA/ANGLE/FORCE; camera within tolerance; no interpolation | offline reads and report generation | offline only |

## Scheduler and coupling findings

The canonical clock itself is mathematically fixed. `start(t0)` emits frame zero at `t0`, sets
`next=t0+period`, and every subsequent deadline is obtained by integer period advancement. A late
call emits at most the oldest due slot and advances over all additional expired deadlines. It does
not re-anchor to `now` and does not emit catch-up bursts.

The live source feeding this clock is incorrectly coupled in two ways:

1. The producer publishes `ControlSample` at the same nominal 30 Hz as the consumer clock. Normal
   60 Hz control computation takes roughly 9--13 ms in the retained 512-sample timing window. A
   small phase error or delayed iteration therefore leaves no spare source sample before the next
   30 Hz deadline, and `CanonicalClock.due()` skips a slot. Camera, JAKA status and RH56 feedback do
   not have to be new; their old causal values could be reused, but no canonical decision is made
   without a new outer `ControlSample`.
2. During the five-second episode-boundary debounce, both-clutch release immediately sets
   `episode_capture_active=False`. If the operator re-engages before rotation, the same episode
   resumes but no control snapshots were published during the idle interval. The canonical clock
   keeps its original origin and skips the entire interval on the next snapshot.

No canonical sample requires a new camera frame, a new compact JAKA status, or a new RH56 register
read. The observed coupling is to production `ControlSample` publication and its
`episode_capture_active` gate.

## Clock-domain audit

- Python control, Quest receive, RH56 scheduling and camera host timestamps use
  `time.monotonic_ns()`.
- Native status and native absolute deadlines use Linux `CLOCK_MONOTONIC`. The compact observation
  timestamp is the JAKA read completion (`read_end`), not Python receipt time.
- Python `perf_counter_ns()` is used only for duration statistics, not persisted source ordering.
- RealSense device timestamps in episodes 59--64 declare `global_time`; they are retained with
  domain and frame number. Online selection uses the paired host monotonic timestamp. Offline code
  explicitly fits device-ms to host-ns before comparison.
- Wall/Unix timestamps are metadata only. No audited online causal comparison mixes them with
  monotonic timestamps.

The staging writer already marks a source invalid when its declared domain is not
`host_monotonic_ns`, but the live collector does not prevent the corresponding values from entering
the fixed-shape row. This is a latent boundary weakness, not an observed domain change in 59--64.

## Queue, backpressure, allocation and I/O audit

The shared session (episodes 59--64) recorded control queue HWM 2, camera queue HWM 2 per role,
writer queue HWM 4, and zero queue drops, recorder drops, writer errors, ring reference expiry or
ring inconsistency. Writer p95 was about 11.8 ms and max about 19.0 ms per operation, with a maximum
batch around 32.3 ms. These measurements do not support recorder/video backpressure as the cause of
the missing source timestamps.

Normal production has no standalone RH56 JSONL, no native cycle telemetry, no event extract, and no
duplicated JAKA/RH56 audit streams. RH56 serialization is disabled on the serial worker. Event JSON
and filesystem flush run on a bounded worker. Producer timing and measured-joint evidence are
bounded deques (512 and 256); shared-session diagnostic histories are bounded and persisted
`event_records` are cleared on the hardware path. No production diagnostic container found in this
path grows linearly with session duration.

One diagnostics-only defect exists: `episode_dataset.async_writer._summary()` indexes unsorted
duration samples, so its reported p50/p95/p99 and `max` can be internally inconsistent. Raw drop and
error counters are unaffected. This is P2 and is outside the minimal P0/P1 behavior fix.

## Latest-only semantic audit

The current independent Quest slots, JAKA target latest-state transport, RH56 target mailbox,
camera descriptor channels, preview update slot and compact status cache each match continuous-state
semantics. CTRL remains ordered. No second case was found where semantically independent modalities
were merged into one latest slot. The unclassified/malformed HTS slot is intentionally separate so
the parser can retain existing fault behavior.
