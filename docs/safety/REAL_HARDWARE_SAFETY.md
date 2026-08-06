# Real hardware safety

## 中文摘要

维护、源码审查、测试、--help、仿真、回放和 fake worker 都不会打开、连接或控制任何真实设备。

任何真机运行都必须由操作者主动执行维护中的真机入口，并满足对应运行模式的全部安全前置条件，包括设备身份、工作区检查、停止装置可达、控制器状态、运行时长限制、命令边界以及确定性的退出和清理。

碰撞、报警、急停、看门狗、SDK、时序、活性或清理不确定性故障仍必须立即停止，不得绕过。

## Runtime safety boundary

Repository maintenance, source review, tests, --help, simulation, replay, and fake-worker execution do not open, connect to, or command a JAKA, RH56DFX, Quest headset, camera, or any other actuator.

Real-device operation must always be started explicitly by the operator through one of the maintained hardware entry points. Each entry remains responsible for selecting the intended operation mode, validating the target device, enforcing bounded execution duration, verifying controller state, checking workspace conditions, applying command limits, and performing deterministic cleanup.

Runtime configuration writes, fault reset, and force-sensor calibration remain separate operation modes with their own safety prerequisites. These modes are selected explicitly by the operator and are never entered automatically.

Hardware entry points must reject missing runtime prerequisites before opening any physical device. Automatic retry remains prohibited.

Use [physical hardware prerequisites](../operation/hardware_prerequisites.md)
and [physical test gates](physical_test_gates.md) before proposing any run.

## Controller-owned facts

The latest operator record states:

| Item | Recorded value |
|---|---|
| Payload | 0.8 kg |
| Center of mass | `[9.289, 12.427, 36.961]` mm |
| Installation | upright/floor, X=0°, Z=0° |
| TCP1--TCP10 | zero |
| Controller safety limits | unchanged |

These values are evidence, not software-owned truth. The software must not
identify, apply, or alter payload, center of mass, installation, TCP, collision
settings, or controller safety limits. An operator must verify them on the
controller before each future physical operation.

TCP calibration is not complete. The prior J4 servo collision cause is
unresolved. Do not repeat the earlier approximately 128 mm multi-axis motion
with large wrist rotation or expand the physical envelope from simulation
evidence.

## Arm command authority

The only current Quest-to-JAKA motion authority is:

```text
validated Quest input
  -> clutch/reference capture
  -> mapping and filters
  -> shared continuation IK and feasibility
  -> immutable AcceptedArmTarget
  -> JAKA accepted-joint adapter
  -> native 8 ms ServoJ/EDG worker
```

The accepted arm target contains absolute J1--J6 radians. The physical adapter
must not map frames, filter, recompute IK, select another branch, follow MuJoCo
`qpos`, or modify the target. Native `joint-teleop` must make zero JAKA
`kine_inverse` calls.

Python is a candidate prefilter, not the final output authority. It retains
the six-finite-joint, branch, hard-range, and obvious velocity/acceleration
checks and rejects an infeasible candidate into bounded hold. It does not
predict jerk or reproduce the native active segment. The native worker still
performs the real 8 ms velocity/acceleration/jerk shaping and final hard
checks; the JAKA controller remains an additional hardware protection layer.
No output or watchdog limit was increased for this simplification.

The `0.22 rad` candidate-jump setting is not a J1/J4/J6 travel or 360-degree
limit. Periodic joints are allowed to progress through multiple candidates;
their safety boundary is the selected legal absolute branch, the hard joint
range, accumulated winding guard, Python output prefilter, and native 8 ms
velocity/acceleration/jerk checks. The setting remains applicable to obvious
non-periodic joint discontinuities. A candidate with zero equivalent-branch
offset is never promoted to `JOINT_BRANCH_DISCONTINUITY` solely for exceeding
that value.

The measured post-EDG state is the startup authority. The first target must be
continuous with that state. A newly captured Quest reference cannot legalize a
joint jump.

Normal combined operation currently requests the project-selected run limits
of 1.5 rad/s for J1--J6. These are not manufacturer
maximum-speed claims. Conservative position bounds, the software margin, shared
output feasibility, native final boundaries, controller safety, and cleanup all
remain active.

## Recoverable hold versus hard stop

`HOLD_REJECTED` is not a fault bypass:

- An infeasible candidate is discarded before acceptance.
- No new motion target is sent.
- A fresh heartbeat keeps producer liveness distinct from candidate validity.
- The last safe destination remains authoritative.
- Recovery is allowed only when a later candidate passes every retained check.

Left-index release requests a bounded arm pause. The native worker brakes to a
hold, reports `STOPPED_READY`, and permits a fresh release-before-press
reference capture. Invalid or stale Quest clutch/wrist input also pauses
immediately. A transient outage has a bounded 10-second recovery window; after
that deadline the Python session enters a persistent disengaged hold and keeps
emitting no-motion heartbeats. It does not end the process. Returning input
still requires a valid released sample, release-before-press, and a fresh
measured-joint/reference capture; the stale reference is never resumed.

The four runtime levels are intentionally separate:

- `HARD_STOP`: measured or indeterminate robot/actuator danger, native
  watchdog or hard timing/liveness fault, actual joint/output violation,
  branch/winding fault, RH56 nonzero `ERROR`/fatal protocol/worker fault, or
  cleanup uncertainty.
- `BOUNDED_HOLD`: transient Quest loss, input recovery timeout, candidate
  infeasibility, compute-budget exhaustion, RH56 ordinary rate/delta limiting,
  or contact clamp. The last authoritative safe target is retained and
  heartbeat stays fresh.
- `RECORDING_DEGRADED`: camera stale/drop/ring expiry, a missing canonical
  field, recorder queue drop, writer failure, or persistent camera acquisition
  failure. The episode may be invalid/aborted, but healthy control is not
  promoted to a robot hard stop.
- `DIAGNOSTIC_ONLY`: preview failure, event-log drop, placement/`/proc` read
  failure, latency/queue watermark, incomplete summary, or non-critical
  provenance telemetry failure.

The following are terminal hard-stop conditions:

- controller collision or servo alarm;
- emergency stop;
- loss of robot power or enable;
- SDK or command transport error;
- final command illegality or tracking hard crossing;
- hard command-loop timing failure;
- actual producer heartbeat, IPC, or worker liveness loss;
- native watchdog expiry or Python/IPC process death;
- actual joint hard-limit, output velocity/acceleration/jerk, branch, or
  winding violation;
- RH56 nonzero `ERROR`, fatal serial/checksum/protocol fault, serial worker
  death, or indeterminate actuator command state;
- deterministic cleanup cannot be guaranteed;
- operator stop or process interruption.

A hard stop terminates new output and runs cleanup. It must never be converted
to `HOLD_REJECTED`, retried automatically, or hidden by a later secondary
transport symptom.

The 60 Hz Python producer preserves the remote maximum of five continuation
backtracks after its initial solve. A rejected trial is not committed; the last authoritative
target remains in force and the producer publishes a bounded hold heartbeat.
The per-tick timing ring stores retry and IK-call counts without constructing
full attempt dictionaries on normal accepted ticks. Detailed diagnostics are
sampled or emitted only for rejects, faults, clutch/reacquisition events, and
bounded samples.

The 10-second input-recovery window does not change the native command-stream
watchdog. A live Python session emits bounded-hold heartbeats after the window;
Python/IPC death emits none and the native watchdog still stops in 100 ms. The
window is capped by configuration validation and cannot be extended above 10
seconds.

## Native worker rules

The physical JAKA worker:

- owns the sole JAKA SDK session;
- does not automatically power or enable the robot;
- verifies the expected tool/user-frame identity and healthy controller state;
- enters EDG/servo only inside the exact authorized gate;
- uses an absolute 8 ms schedule without catch-up command bursts;
- applies final position, velocity, acceleration, jerk, tracking, timing, and
  liveness checks before or around SDK dispatch;
- performs lightweight controller polling in the command worker;
- disables servo, exits EDG, and logs out during cleanup.

Do not restore a second monitoring login. A prior physical no-motion attempt
showed that the second SDK session prevented the command worker from reaching
`CONNECTED`.

## Provisional workspace geometry

The table and mounting members injected into the live MuJoCo viewer are
provisional scene geometry. They are not loaded by the shared target generator
and are not a pre-acceptance physical collision authority. Viewer clearance,
simulation contact, or offline FK does not replace a clear physical workspace,
operator line of sight, fixture inspection, or accessible E-stop.

## RH56DFX safety semantics

The maintained physical hand path is PC-direct USB/RS485. Prefer an
operator-confirmed `/dev/serial/by-id/...` device. The explicit CH341 fallback
is permitted only by the current wrapper after VID:PID/driver identity and tty
path checks.

Opening the serial transport performs zero register writes. It does not clear
errors, write speed/force, send a target, or open the hand. The first active
target is based on fresh measured `ANGLE_ACT`, remains inside the configured
command envelope, and is subject to the command-rate and per-command delta
limits.

Feedback semantics are:

| Field | Maintained meaning |
|---|---|
| `ANGLE_ACT` | measured six-axis actuator feedback |
| `CURRENT` | raw current-register telemetry |
| `FORCE_ACT` | raw load/force-register telemetry |
| `ERROR` | raw error; nonzero faults |
| `STATUS` | raw status; code meanings are not guessed |

These values do not expose every passive finger joint, tactile contact, or
slip. `CURRENT` and `FORCE_ACT` are not calibrated force limits in the current
PC-direct controller.

Grip release normally holds the last target by sending no new position writes;
it does not automatically open the hand and is not a torque-off or vendor
emergency stop. If feedback-qualified contact detection observes a loaded
channel while that held target remains active, the controller may issue one
bounded opening relief target; this is the contact-safety exception and is
recorded separately from ordinary grip commands. A single typed serial read
timeout while the previous complete feedback snapshot is inside the freshness
window holds the next hand command and retries. A repeated timeout or stale
snapshot enters `HAND_FAULT`; checksum/protocol failure, nonzero `ERROR`, write
failure, worker death, or unknown actuator command state remains terminal.
Ordinary rate limiting and contact clamping are hand-local holds/diagnostics
and do not become an arm hard stop.

In combined operation, a terminal arm fault stops new hand commands. A terminal
hand fault invalidates the episode and requests the arm's safe terminal path.
Neither subsystem may continue silently after the other reports a terminal
fault.

## Required operator controls

Before an authorized physical gate:

1. Confirm the intended code/config/executable identity and that no other
   controller client or stale process exists.
2. Inspect the robot, hand, adapter, cables, fixtures, workspace, and expected
   motion envelope.
3. Verify controller power/enable, alarms, collision state, E-stop, payload,
   installation, TCP, tool/user frames, and safety limits.
4. Keep manual stop and E-stop continuously accessible.
5. Use the exact bounded duration and acknowledgements; keep automatic retry
   disabled.
6. Preserve raw logs, native metrics, and the first terminal reason.
7. Review evidence before any new or larger gate.

Do not perform automatic payload identification, TCP calibration, collision
experiments, or controller-configuration writes through this repository.

## Current physical evidence boundary

- A combined `fast40` session completed 60.105 seconds with no hard timing
  miss, controller alarm, arm/RH56 worker fault, serial/protocol fault, or
  transport symptom. This is a bounded 60-second physical PASS only.
- A later run reached 200.943 seconds with zero hard timing faults, then fresh
  CTRL packets reported `active=0`; the retained liveness policy correctly
  stopped with `producer_liveness_loss`.
- No 300-second combined gate has a PASS.
- The latest shared output-acceleration correction is offline tested but has
  not completed a bounded post-fix physical validation.
- The earlier J4 collision cause remains unresolved.
- TCP calibration and complete Quest-driven physical RH56 teleoperation remain
  incomplete.

Offline tests, replay, and simulation must retain those labels. They cannot be
promoted to a physical PASS. See [current status](../status/current_status.md)
for the concise project state and
[combined teleoperation](../operation/jaka_rh56_combined_teleop.md) for the
maintained entry's prerequisites.
