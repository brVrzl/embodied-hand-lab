# JAKA control foundation — Migration Gates 1–2

> **Status: historical snapshot, 2026-07-16.** This document records the exact
> foundation gate at that date. It is not the current operating guide. See
> [`docs/status/current_status.md`](../../../status/current_status.md) and
> [`docs/operation/jaka_arm_teleoperation.md`](../../../operation/jaka_arm_teleoperation.md).

Date: 2026-07-16  
Status: implemented and validated without robot hardware

## Scope and hard boundary

This foundation is arm-only and clean-slate. `src/teleoperation` and
`native/jaka_servo_worker` do not import or link the legacy HEBI teleoperation
path. The historical `src/teleop_tools` package remains intact and is explicitly
classified as a prototype in `src/teleop_tools/LEGACY.md`.

TeleDex ingestion, live pose following, IK-driven motion, filtering, Ruckig,
clutching, recentering, scaling, Quest input, and hand control are not in this
runtime. The arm composition constructs no hand object, SDK, stub, or contract.
Automated dependency tests enforce both boundaries.

## Architecture and ownership

```text
synthetic/device-neutral source (Python, non-real-time)
  -> typed 6-DoF PoseTarget with clock-separated timestamps
  -> optional future filter / shaper / safety / trajectory boundaries
  -> bounded Unix datagram, latest-target semantics
  -> native jaka_servo_worker (only JAKA SDK/EDG owner)
       -> JAKA SDK
       -> fixed-size timing store
  <- bounded native status datagrams
  <- Python supervision and post-loop reporting
```

The Python layer performs non-real-time supervision and never calls the JAKA
SDK. The native process has one control thread and is the exclusive owner of
login, state acquisition, EDG entry, cyclic commands, safe exit, and logout.
Signal handlers only set an atomic stop flag. Cleanup occurs synchronously after
the loop through explicit cleanup plus the backend's RAII destructor; it never
depends on Python garbage collection.

The real-time loop contains no console output, file access, configuration load,
Python callback, growing queue, or memory allocation. Metrics use preallocated,
bounded arrays allocated before the loop and are formatted/written afterward.
The optional status send is a non-blocking local datagram at about 9.6 Hz.

## Device-neutral contracts and units

The schema is `arm_teleoperation.v1`. Contracts cover input samples, pipeline
timestamps, pose targets, robot state, controller state, health, safety, command
acknowledgements, and timing statistics.

- Position is metres in Python contracts.
- Orientation is a unit quaternion in `x,y,z,w` order.
- Joint position is radians; velocity is radians/second.
- Local timing is integer nanoseconds from the host monotonic clock.
- `source_capture_ns` remains in the source clock domain and is never subtracted
  from local monotonic timestamps without an explicit clock-synchronization
  model.
- `local_receive_ns`, `processing_ns`, `dispatch_ns`, `robot_command_ns`, and
  `robot_state_observation_ns` identify each local pipeline stage separately.
- Missing optional timestamps are `None` in Python and zero only at the fixed
  binary wire boundary.
- The current command frame identifier is `robot_base`. Frame transforms are not
  implemented at these gates. A future adapter must output a declared source
  frame and a later centralized transform layer must produce `robot_base`.
- The vendor SDK uses millimetres for Cartesian translation and radians for RPY.
  No live Cartesian conversion is active in Gates 1–2.

## Python-to-native transport

The selected transport is a versioned Unix-domain datagram protocol on one
host. A target datagram is exactly 124 bytes and a status datagram is exactly
108 bytes. Both use little-endian fixed fields, magic, version, and CRC32.
Commands include kind, flags, frame ID, sequence, source/local timestamps, and a
fixed eight-double payload.

There is no Python FIFO. The non-blocking producer has a finite kernel send
buffer and reports sends/drops. Each 8 ms worker cycle drains every queued
datagram and retains only the highest new valid sequence. Earlier datagrams in
the same drain are superseded; duplicates, reordered packets, bad CRC, unknown
kinds, non-finite payloads, and invalid/future dispatch timestamps are rejected.
The worker therefore cannot replay an accumulated pose backlog.

Copy path: Python serialization copies once into the socket; the kernel copies
the datagram; the worker copies each fixed packet into a stack object and the
newest valid packet into one fixed slot. Expected local transport latency is
below one worker cycle when the producer dispatch precedes the next wake, but
this is an expectation, not a hardware result. The synthetic pipeline exposes
actual transport and command age.

Failure modes are bounded:

- absent or full consumer: producer drops and increments a counter;
- stalled producer after prior traffic: age thresholds drive hold/stop/fault;
- stalled consumer: finite kernel buffer drops rather than growing;
- supervisor crash: no new packets, so the worker applies stale policy;
- status receiver loss: non-blocking status sends are discarded and do not
  delay control;
- malformed traffic: rejected and forces controlled loop exit;
- socket unlink/process signal: worker cleanup owns EDG exit.

Shared memory was rejected at this stage because a small fixed datagram is
already bounded, observable, process-failure tolerant, and simpler to inspect.
No distributed middleware is required on one host.

## Target semantics

The defaults are independently configurable:

| Condition | Default behavior |
|---|---|
| No target has ever arrived | Hold the invariant startup state; do not infer motion |
| Valid increasing target | Accept newest; dry-run state may report RUNNING |
| Age ≥ 40 ms | Health warning boundary; command action remains otherwise eligible |
| Age ≥ 100 ms | HOLDING; no extrapolation |
| Age ≥ 500 ms | CONTROLLED_STOP and leave the loop |
| Age ≥ 2000 ms | FAULT if reached before an earlier configured stop |
| Negative or >5 ms future dispatch time | Reject as invalid and controlled stop |
| Duplicate/backward sequence | Reject; retained target is unchanged |
| Python disconnect after traffic | Hold then controlled stop by age |
| Robot/SDK communication failure | FAULT; deterministic EDG cleanup |

The zero-motion probe intentionally needs no input stream: its sole target is
the verified startup joint state. No live-input hardware mode exists yet.

## Lifecycle and safety state machine

| State | Entry / allowed activity | Exit and timeout behavior | Cleanup and logging |
|---|---|---|---|
| DISCONNECTED | Process start; no SDK calls or motion | Explicit start -> CONNECTING; shutdown -> SHUTDOWN | No cleanup; configuration errors print before loop |
| CONNECTING | Login is the only operation | Success -> CONNECTED; SDK error -> FAULT | Partial owned resources cleaned by RAII; no loop logging |
| CONNECTED | Read simple status; verify fault-free, powered, enabled, E-stop clear, collision clear, exact tool/user IDs | Checks pass -> ARMED; failure -> FAULT | Post-loop fault reason in metrics |
| ARMED | Read finite initial joints; require all within ±2π rad; preflight optional minimal probe | State-read -> HOLDING; EDG entry -> EDG_READY | No automatic robot enable or recovery commands |
| EDG_READY | EDG enabled, servo mode enabled and confirmed; reread joints and require ≤0.0001 rad initial delta | Verification -> HOLDING; timeout/SDK error -> FAULT | Status datagram; cleanup disables servo then EDG |
| HOLDING | Command startup invariant only in zero/minimal probe; state-read commands nothing | Fresh dry-run target -> RUNNING; stop age/operator stop -> CONTROLLED_STOP; SDK error -> FAULT | Bounded status only |
| RUNNING | Gates 1–2: communication/state machinery only on fake backend; no real input-driven motion | Stale -> HOLDING; stop request/age -> CONTROLLED_STOP; error -> FAULT | Latest sequence acknowledges acceptance |
| CONTROLLED_STOP | Exit cyclic loop; no new target accepted | Cleanup -> SHUTDOWN; cleanup fault remains reported | Disable servo, exit EDG, logout; metrics afterward |
| FAULT | Latch SDK failure, invalid lifecycle, fatal timeout, or consecutive overrun | Cleanup -> SHUTDOWN | Fault reason and code exported after loop |
| SHUTDOWN | Terminal, non-moving state | None | Final status datagram and metrics |

An operator SIGINT/SIGTERM/SIGHUP requests controlled loop exit. A command
packet of kind STOP requests the same path. Fifty consecutive cycles that
cannot complete before the following scheduled release are treated as a loop
overrun fault. The SDK call durations remain explicit metrics.

## Execution modes

1. `dry-run` (default): native loop plus fake lifecycle/backend. It cannot
   connect to hardware.
2. `state-read`: real SDK connection and state reads; no EDG or cyclic writes.
3. `zero-motion`: verifies controller state, frames, units, finite state, EDG
   servo state, and ≤0.0001 rad initial delta, then repeatedly writes the exact
   captured joint target.
4. `minimal-motion`: disabled unless explicitly selected. It limits one joint
   to ≤0.002 rad, uses a quintic out-and-return profile of at least one second
   per leg, checks both endpoints with SDK forward kinematics against
   operator-supplied Cartesian workspace bounds, limits predicted TCP endpoint
   displacement to 5 mm, enforces analytic peak limits of 0.005 rad/s and 0.02
   rad/s², and automatically returns to the initial target.

All connected modes require `--hardware`, a robot IP, exact tool/user frame IDs,
and the exact acknowledgement `I_ACKNOWLEDGE_JAKA_HARDWARE_RISK`. The minimal
launcher adds an interactive E-stop/workspace confirmation. No connected mode
was run during implementation.

There is no physics simulation mode in this foundation. Dry-run is a lifecycle
and timing stub, not a robot simulation. Metrics separately report the maximum
intentional command delta and maximum observed joint delta; zero-motion is
expected to report an intentional delta of exactly zero.

Commanding a measured current pose can still move a physical robot due to stale
state, controller dynamics, calibration, payload, or frame errors. Zero-motion
is a conservative capability probe, not proof of zero physical displacement.

## IK placement review

No IK solver was implemented. The earlier proposal that Python permanently own
IK is not accepted as an untested constraint.

| Placement | Advantages | Risks / dependencies |
|---|---|---|
| Cartesian target to native worker and `edg_servo_p` | Small Python load; SDK/robot owns kinematics; low command-path jitter | Limited visibility into singularity policy, branch selection, joint limits, and continuity; harder deterministic tests |
| Python IK to native joint target | Testable branch/limit policy; explicit joint continuity and safety integration | Python compute jitter; depends on freshest state; serialization adds latency; branch switching and singularities become project responsibility |
| Native-side IK with SDK `kine_inverse` or a native solver | Same-owner state access; bounded scheduling; potential lower jitter | SDK inverse behavior still opaque; native solver raises maintenance cost; test fixtures and branch policy still required |

The Gate 2 decision is to retain both Cartesian and joint target kinds in the
wire contract while executing neither from external input on hardware. Gate 4
should benchmark each viable path with identical state snapshots and quantify
compute jitter, branch continuity, limit behavior, singularity behavior, and
end-to-end age. Python IK remains a candidate, not a permanent constraint.

## Filtering, shaping, safety, and trajectory boundary

`motion_boundaries.py` defines separate protocols for measurement filtering,
target shaping, safety limiting, and trajectory generation. They are not
wired to the worker. One Euro and Ruckig are deliberately absent from live
motion. Later trajectory work must use interruptible state-to-state local
updates; it may not consume a stale FIFO or conceal missing timestamps.

## Tests and measured timing

Automated tests cover contract serialization, timestamp order, quaternion and
sequence validation, stale behavior, fixed wire/CRC, bounded producer behavior,
latest-target drains, fake lifecycle/failures/thread ownership, repeated
initialization, zero-motion delta checks, state transitions, timing statistics,
hardware CLI gates, signal cleanup, native failure injection, producer
disconnect, and forbidden legacy/hand dependencies.

Three independent 5-second `dry-run` measurements on this development host:

| Run | Samples | mean ms | median ms | stddev ms | min ms | max ms | p95 ms | p99 ms | completion misses | CPU % |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 624 | 8.000093 | 7.999899 | 0.019910 | 7.729455 | 8.284834 | 8.006078 | 8.050664 | 0 | 0.0567 |
| 2 | 624 | 8.000091 | 7.999959 | 0.004157 | 7.971657 | 8.061166 | 8.004994 | 8.009265 | 0 | 0.0456 |
| 3 | 624 | 8.000090 | 7.999996 | 0.088712 | 6.532532 | 9.473013 | 8.005317 | 8.050307 | 0 | 0.0530 |

The completion-miss definition is loop work completing after the next scheduled
release, not every wake-period sample over 8 ms. P99.9 is omitted because each
run has fewer than 1000 samples. These results demonstrate a capable native
timer with the fake backend only; they do **not** demonstrate stable connected
125 Hz JAKA operation.

A 2-second 60 Hz synthetic sinusoidal translation run with duplicate, reordered,
and burst injection accepted 110 newest targets, rejected 11 duplicate/reordered
packets, recorded zero completion misses, and completed cleanly. Its cycle mean
was 8.000203 ms and p99 was 8.012462 ms. The post-producer 250 ms tail is
included in transport-age statistics, so that run is a failure-state exercise,
not an input latency headline.

No connected state-read, zero-motion EDG, minimal-motion, end-to-end physical
latency, robot tracking error, or hardware stability measurement was performed.

## Commands

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_servo_worker -j2

PYTHONPATH=src pytest -q \
  tests/test_teleoperation_contracts.py \
  tests/test_teleoperation_state_and_safety.py \
  tests/test_teleoperation_wire.py \
  tests/test_teleoperation_fake_jaka.py \
  tests/test_teleoperation_synthetic.py \
  tests/test_teleoperation_timing.py \
  tests/test_teleoperation_isolation.py \
  tests/test_native_jaka_servo_worker.py

PYTHONPATH=src python tools/teleoperation/benchmark_native_worker.py --duration-s 10 --runs 3
PYTHONPATH=src python tools/teleoperation/run_synthetic_pipeline.py \
  --duration-s 10 --rate-hz 60 --pattern sine_translation --duplicate-every 17 \
  --reorder-every 29 --burst-every 23
```

Connected commands are intentionally not copy-paste defaults. Review
`tools/teleoperation/run_jaka_hardware_probe.py --help`, the precautions above,
the selected frame IDs, a robot-specific workspace, reduced controller limits,
and E-stop access before supplying the explicit hardware acknowledgement.

## Unresolved risks and next evidence gate

- Connected `edg_get_stat` and `edg_servo_j` blocking distributions are unknown.
- The vendor library's scheduling, socket behavior, reconnect behavior, and
  internal command buffering remain opaque until instrumented on hardware.
- This process does not request OS real-time policy or CPU affinity; those
  should be evaluated after connected timing shows a need and system policy is
  reviewed.
- Datagram status is intentionally lossy. Critical faults remain in final
  metrics, but an external supervisor crash can lose live observability.
- No robot-specific workspace, joint-limit, singularity, calibration, payload,
  or collision-envelope validation exists for continuous motion.
- No live target mode exists, by design.
- P99.9 and long-duration behavior need ≥1000-cycle and soak measurements.

The next approval should be limited to staged hardware capability validation:
read-only connected timing first, then separately approved zero-motion EDG.
Minimal motion should remain unapproved until those measurements show cleanup,
SDK timing, frame verification, and cycle behavior are acceptable.
