# JAKA clutch-recovery transport contract (offline research)

This page defines a future transport contract. It is not an operator procedure,
does not authorize controller access, and has not been physically validated.
The implementation in this worktree is an SDK-free fake only.

## Current implementation audit

Evidence is separated so an implementation detail or SDK declaration is not
mistaken for controller behavior.

### Verified in current source

- `RealBackend` in `native/jaka_servo_worker/main.cpp` is the sole owner of its
  `JAKAZuRobot` object and SDK session. It logs in once, enables EDG and servo,
  sends absolute `edg_servo_j(..., step_num=1)`, and reads status on that same
  object.
- While EDG is active, `read()` obtains `EDGState` but currently copies only
  `jointVal`; the available `jointVel` is discarded. Outside EDG it reads only
  joint position through `get_actual_joint_position`.
- Any joint-teleop stop packet changes the worker state to controlled stop and
  exits the loop. The common exit path always disables servo, disables EDG,
  logs out, and terminates the worker. `NativeWorkerProcess` is likewise a
  process-lifetime wrapper, and `JakaAcceptedJointTargetAdapter.stop()` is a
  permanent local latch.
- Consequently, current production transport does **not** support a
  session-held clutch pause or re-engagement. Its cleanup order is
  `servo_move_enable(false) -> edg_init(false) -> login_out()`.

### Verified in the local JAKA SDK 2.2.7 headers

- `EDGState` contains measured joint position in radians and joint velocity in
  rad/s. It has no joint-acceleration field.
- `get_actual_joint_position` provides position only.
- `get_robot_status_simple` provides controller error, power, and enable
  status; the worker separately classifies E-stop and collision after an
  unhealthy status.
- `servo_j` and `edg_servo_j` document `step_num * 8 ms`; the ServoJ text asks
  clients to send the next point immediately to avoid delay.

### Inferred design, not yet controller evidence

One process should continue to own the sole SDK session. After the final
controlled-braking command it may retain that session only if a separate gate
establishes that the controller permits a stopped interval without commands.
On re-engagement, that same owner reads measured state, verifies normalized
health, arms a newer safety epoch, and consumes only already-shaped commands.

### Unknown until separately authorized validation

- Whether EDG and servo mode remain valid during a command-free stopped pause.
- Whether the controller instead requires repeated identical-position
  commands while stopped, and what watchdog/deadline applies.
- Whether `edg_init(true)` or servo enable must be repeated before resumed
  streaming, and whether that creates a discontinuity.
- Exact freshness/timestamp behavior of EDG feedback across a pause.
- Cleanup and restart behavior after each controller alarm class.

The thin skeleton therefore defaults both `PauseCommandPolicy` and
`ResumePreparationPolicy` to `Unverified` and refuses servo preparation until
both are selected explicitly. Its fake-only tests cover `NoCommandRequired`
and `RepeatStoppedPositionRequired`; the latter repeats only the already-shaped
final stopped q and is valid only with `KeepPrepared`. Supporting both offline
does not select either policy for a real controller.

## Contract

The future transport owns only the SDK session, measured-state/status reads,
already-shaped joint-command sends, sequence/epoch/freshness validation,
normalized fault classification, and cleanup. It must not perform IK,
collision or singularity planning, shaping, braking, target interpolation,
Quest mapping, or reference capture.

Normal release is:

```text
Streaming(epoch N)
  -> send bounded ControlledBraking commands
  -> send final Stopped command
  -> stop accepting epoch-N motion output
  -> keep health/state observation and, only under a verified pause policy,
     retain the sole SDK session
  -> collect trustworthy measured state
  -> upstream captures current robot and current Quest references
  -> arm epoch N+1 and reinitialize the shaper from measured q/dq/ddq
  -> first epoch-N+1 output equals measured q within the continuity gate
  -> resume Streaming
```

The paused input side is latest-only. It accumulates no target and the
transport sees no Quest observations. Thus movement during release cannot be
replayed after re-engagement. A delayed epoch-N command is a hard epoch fault,
not a replacement target.

Alarm, E-stop, collision, SDK failure, power/servo loss, stale or untrusted
measurement, hard timing fault, and explicit hard stop latch `Faulted`.
Pressing clutch cannot clear it. Cleanup leaves `reset_required=true`; only an
explicit reset after external fault resolution may reconnect, re-verify
health, acquire a new measured state, and arm a new epoch.

## Measured-state recovery

The recovery priority is:

1. q/dq/ddq: use one fresh, finite, ordered measured sample directly.
2. q/dq: require a configurable run of stationary, fresh, ordered samples;
   keep measured q/dq and initialize ddq to zero with an explicit quality tag.
3. q only: estimate dq from multiple monotonic samples, require the same
   stability gate, and initialize ddq to zero with a lower quality tag.

The SDK-free reference defaults to three samples, a 32 ms maximum sample age
and interval, and 0.002 rad/s stationary velocity. These are research defaults,
not approved hardware thresholds. Non-finite, stale, duplicate, out-of-order,
wrong-DOF, or smuggled absent fields fail closed. Last sent q is never used as
measured q.

The current header and worker evidence makes option 2 the likely EDG source,
but the production worker must be changed to retain `EDGState.jointVel` before
that design can be implemented.

## SDK-free reference tests

`clutch_recovery_transport.hpp` defines an `IFakeJakaSdkInterface` and a pure
in-memory implementation. They contain no SDK include, handle, symbol, network,
or controller behavior. Tests cover a retained-session release/brake/stop/
re-engage cycle, new-epoch acceptance, old-epoch rejection, first-command q
continuity, q/dq/ddq and q/dq and q-only measurement paths, send failure,
stale state, servo disabled, alarm, collision, E-stop, cleanup, and explicit
reset. The existing fixed telemetry ring continues to preserve the terminal
fault record.

## Thin adapter skeleton

`ThinJakaTransportAdapter` is a standalone C++ transport state machine backed
only by `JakaSdkFunctionTable`. The table has fixed callbacks for login,
EDG/servo enable, shaped-q send, q/dq read, normalized status read, hard stop,
and logout. It contains a caller-owned opaque context but no JAKA header,
handle, return type, library, network, IK, shaping, braking, Quest mapping, or
reference capture.

Its recoverable path is:

```text
Streaming -> ControlledStopping -> StoppedReady
          -> MeasuredStateRefresh -> ServoReady -> Streaming
```

Its terminal path is:

```text
Faulted -> Cleanup -> ResetRequired -> explicit reset -> Disconnected
```

The adapter keeps a depth-one shaped-command mailbox, validates ABI epoch,
sequence, freshness and first-command q continuity, preserves q and dq from
feedback, polls normalized health, and records fixed counters for tick timing,
command age, replacement, status, pause keepalive, and clutch cycles. It never
creates a joint target. The `RestartEdg` and `RestartEdgAndServo` policies make
the disable/re-enable sequence explicit; unexpected servo/EDG loss remains a
hard fault.

The deterministic fake load executes 1,000 release/refresh/resume cycles over
7,000 exact 8 ms ticks with one login, no logout, 7,000 health polls, 1,000
latest-target replacements, zero deadline misses, zero first-frame q delta,
and zero observed allocations in the measured loop. A separate matrix covers
15 hard/recovery faults, including send timeout, status/measurement staleness,
session loss, pause alarm, cleanup failure, old epoch and discontinuous resume.
These are executable state-machine results, not SDK or controller evidence.

## Minimum gates before first physical recovery attempt

1. Obtain vendor-confirmed pause/watchdog and EDG restart semantics for SDK
   2.2.7/controller firmware, including whether no command or repeated stopped
   q is required. Keep recovery disabled if this remains unknown.
2. Add a separately reviewed vendor translation unit behind the existing thin
   function table, retaining sole-session ownership. Keep the no-SDK build and
   tests as a mandatory isolated target before any SDK-linked test is run.
3. Establish trustworthy EDG q/dq timing and freshness. Validate multi-sample
   stability and the ddq initialization rule without substituting commanded q.
4. Add a restart-continuity gate that compares the first shaped q/dq/ddq with
   the just-read measured state and rejects old epochs and stale commands.
5. Verify cleanup and explicit-reset sequencing for alarm, E-stop, collision,
   SDK failure, power/servo loss, timing fault, and measurement loss.
6. Prove 125 Hz scheduling, command freshness, latest-only consumption, health
   polling, and telemetry under Thor load without hardware motion.
7. In a new explicitly authorized session, perform a bounded no-motion
   lifecycle gate with operator stop access before any motion test. Confirm
   pause duration, session/EDG state, status freshness, and cleanup evidence.
8. Only after that passes, define a separately authorized minimal-motion
   release/re-engage gate with strict displacement, duration, workspace,
   continuity, alarm, and RH56-isolation constraints.

The unresolved J4 collision, current production acceleration-gate physical
validation, TCP/payload/controller-state verification, and real transport
implementation remain blockers. Nothing here is a physical guarantee.
