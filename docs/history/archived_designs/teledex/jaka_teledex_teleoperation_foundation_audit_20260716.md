# JAKA + TeleDex teleoperation foundation: repository audit and architecture

Status: **design gate; implementation not approved or started**  
Date: 2026-07-16  
Scope: JAKA arm 6-DoF end-effector teleoperation only. RH56DFX command and hand
retargeting are explicitly out of scope and must remain disabled.

## 1. Executive decision

The repository has useful hardware assets and experimental evidence, but the
current teleoperation path is not a suitable foundation for the requested
system.

The main conclusions are:

1. Do not extend `relative_pose_lag_follow.py`, `JakaPalmTargetJogController`, or
   the combined arm/hand ROS2 bridge. They are legacy experimental control code.
2. Treat the current untracked TeleDex path as a useful input and calibration
   prototype only. It directly imports HEBI types and the HEBI follower, so it
   violates the clean-redesign boundary.
3. Refactor the JAKA SDK boundary before implementing the new controller. The
   present wrapper is synchronous, un-timestamped, has no reconnect or ownership
   model, and exposes real-time calls through an unrestricted escape hatch.
4. Keep the RH56 subsystem disconnected and unconstructed in the arm-only
   runtime. Its current blocking behavior must not share a process or executor
   with arm servoing.
5. Build a new `teleoperation` package around device-neutral, timestamped pose
   samples and a dedicated 125 Hz JAKA EDG worker. The local JAKA SDK says EDG
   commands should be sent on an approximately 8 ms cycle and recommends a
   fixed, elevated-priority CPU thread.
6. Use a speed-adaptive One Euro pose filter for measurement noise and a
   separate jerk-limited online trajectory generator for physical command
   continuity. Filtering and trajectory generation are not interchangeable.
7. Stream bounded joint-position setpoints through EDG after branch-continuous
   IK. This gives the project explicit control over joint limits, singularity
   handling, velocity, acceleration, jerk, and tracking-error supervision.
8. The production servo worker should be native C++ (JAKA C++ SDK + local
   state-to-state Ruckig), isolated from Python/ROS/logging. A Python-only worker
   may be used only if a capability benchmark meets the same timing gates; it is
   not the assumed production design.

No existing source file needs to be deleted. Legacy entrypoints can remain
available under their existing names while the new stack is developed and
validated.

## 2. Audit basis and current repository state

The audit covered the full repository inventory and traced the control-critical
paths through source, configuration, tools, scripts, tests, local vendor SDK
headers, documentation, and existing teleoperation logs.

Current branch at audit time:

```text
integration/rh56-visual-coacd-default
HEAD 6faa64b
```

The worktree was already dirty. It contains user-owned camera, digital-twin,
TeleDex, JAKA controller, test, configuration, and documentation changes. This
audit does not revert or alter them. In particular, the TeleDex files are
currently untracked, while changes to `servo_jog.py` and the legacy follower are
uncommitted.

Control-focused regression result at audit time:

```text
78 passed in 2.20s
```

The run covered JAKA mock/servo/bridge tests, RH56 schema/mock/serial/tool/bridge
tests, TeleDex input/calibration tests, the relative follower, and RViz shadow
synchronization. This demonstrates functional unit coverage, not real-time or
hardware acceptance.

## 3. Repository structure

| Path | Current responsibility | Relevance to new arm stack |
|---|---|---|
| `src/embodiment_core` | Basic dataclasses, YAML loading, logging | Keep utilities selectively; replace control data contracts |
| `src/jaka_driver_adapter` | Generic JAKA adapter, SDK backend, mock, MuJoCo IK, EDG servo jog | Keep SDK/vendor knowledge and robot model; rewrite runtime interface and controller |
| `src/rh56_driver` | RH56 schema, serial and JAKA-tool backends, ROS2 bridge | Preserve but never load in arm-only runtime |
| `src/robot_bringup` | Combined JAKA/RH56 ROS2 bridge and orchestration | Keep legacy; do not use for new servo runtime |
| `src/teleop_tools` | Xbox, HEBI, iPhone hand, shadows, experimental TeleDex | Isolate as legacy/experimental; new package must not import its control pipeline |
| `src/data_recorder` | Episode recording | Keep outside real-time loop; adapt later to new telemetry schema |
| `src/vision_interface` | RGB-D interfaces and processing | Out of current arm-only control scope |
| `src/pregrasp` | RH56 geometry and pregrasp planning | Out of scope |
| `src/sim_maniskill` | Simulation, task, and collision work | Keep as offline/shadow validation support |
| `digital_twin`, `models`, `data/sim_assets` | Workspace and robot/hand models | Keep; use robot kinematics/collision assets through explicit adapters |
| `configs` | Robot, hand, teleop, simulation, camera configuration | Keep convention; add schema-validated new-stack configs |
| `scripts` | Human-facing launchers | Keep legacy scripts; add one explicit arm-only launcher later |
| `tools` | Python entrypoints and hardware diagnostics | Keep diagnostics; new runtime entrypoint should be thin |
| `tests` | Unit and simulation regression tests | Keep; add deterministic clock, replay, fault-injection, and timing suites |
| `third_party/jaka_sdk/v2.2.7` | Official local JAKA binaries, headers, release notes | Keep and use as the authoritative SDK source |
| `third_party/inspire_hand` | Local RH56 protocol reference | Keep for future hand stage |
| `launch` | Minimal ROS2 process launch | Legacy; new servo must not depend on ROS timer determinism |
| `logs/teleop` | Real/experimental telemetry | Preserve as evidence; convert with offline analysis tools only |

The active project is Python 3.10 with NumPy, MuJoCo, PySerial, and PyYAML. ROS2
is an external system dependency. There is no current CMake/native application
layer, schema-validation library, static type gate, linter, benchmark harness,
or real-time scheduling helper.

## 4. Current robot SDK wrappers

### 4.1 JAKA interface as implemented

`JakaBackend` exposes `connect`, `get_joint_state`, point-to-point joint/pose
moves, `stop`, and speed scale. `JakaSDKBackend` loads `jkrc`, logs in, optionally
powers/enables the robot, and provides synchronous SDK calls. The real-time EDG
surface is not part of the interface; `servo_jog.py` reaches it through
`call_sdk_method(method_name, *args)`.

The current real-time path is:

```text
ROS2 String JSON target
  -> parse_palm_target_jog_command
  -> JakaPalmTargetJogController
  -> MuJoCo damped least-squares IK against RH56 palm body
  -> joint velocity/acceleration clipping
  -> JakaServoJogController
  -> edg_servo_j
```

This path runs inside the same single-threaded ROS2 node that publishes state,
polls flags, handles hand commands, publishes markers, serializes JSON, and
flushes a log file on every servo tick.

### 4.2 JAKA SDK review

| Concern | Finding | Required action |
|---|---|---|
| Latency | No SDK call-duration instrumentation. Existing logs reveal scheduler/executor delay but not transport latency. | Measure every SDK call with monotonic nanoseconds and expose histograms. |
| Communication frequency | EDG documentation requires about 8 ms; current bridge requests 50 Hz and achieves about 28-32 Hz logging, with enabled servo tick means of 38-45 ms in two trials. | Dedicated 125 Hz EDG owner; reject startup if capability gate fails. |
| Blocking behavior | All SDK calls are synchronous. No per-call timeout is enforced by the wrapper. Multiple getters run in timers and in each servo tick. | Isolate SDK ownership; benchmark each call; never place slow status calls in the send deadline. |
| Thread safety | No lock, thread-affinity assertion, or documented SDK guarantee. Multiple ROS callbacks call the same object. | Single-owner command session. Use a separate monitor session/process only after a multi-client test. |
| Socket management | Hidden inside `jkrc`; configured `port` is unused. EDG UDP lifecycle is manually invoked by controller code. | Encapsulate login, EDG init, servo enable, servo disable, EDG shutdown, logout in one lifecycle object. |
| Reconnect | None. A failed call becomes a generic exception. | Bounded backoff only in disconnected/standby. Never reconnect and resume active motion automatically. Require a new deadman edge and anchor. |
| Watchdog | Exists in legacy controller, not transport. It uses command receipt time but cannot guarantee a timely SDK send or stop. | Native watchdog in EDG owner based on input, target, feedback, and loop deadlines. |
| Exceptions | SDK error codes become generic `RuntimeError`; getters can extract payloads without first checking the returned error code. | Structured `JakaError` with operation, code, call time, lifecycle state, and retryability. Check every result before payload use. |
| Timestamps | `JointState` and `Pose` carry no sample timestamp, sequence, or receive timestamp. ROS publishes the callback time, not controller sample time. | Immutable feedback packet with controller/EDG detail timestamp, host monotonic receive time, sequence, and age. |
| Command buffering | Opaque controller buffering; ROS depth 10 queues stale targets. No explicit latest-only policy. | Depth-one/latest-value mailbox. EDG setpoint sequence and age must be checked every cycle. |
| State synchronization | Joint position, TCP pose, and flags are fetched separately and can describe different instants. | Prefer one EDG state packet for joint/TCP feedback. Publish a coherent snapshot; slower status is separately timestamped. |
| Shutdown | `disconnect` does not own or guarantee `servo_move_enable(false)` and `edg_init(false)`. | Idempotent fail-safe shutdown sequence with an emergency fallback. |
| API integrity | Arbitrary `call_sdk_method` bypasses types, units, state, and safety. | Remove the escape hatch from production control code; expose only typed operations. |
| Units/frames | Pose conversion is inferred; quaternion/RPY order varies across modules. | One SI-unit pose type, one quaternion convention, explicit frame IDs, validated conversion at SDK edge. |

Additional JAKA issues:

- `move_pose` falls back on `TypeError` text, which can mask binding/version
  problems.
- Speed and acceleration scales are local guesses rather than controller
  capability data.
- Joint limits are duplicated in Python from the model and are not checked
  against live controller configuration.
- The current IK target is the RH56 palm body in a combined MuJoCo asset. The
  new arm controller needs an explicitly configured controlled end-effector and
  tool transform even when the physical hand remains mounted but disabled.
- The controller's NLF filter is useful as a secondary hardware-side envelope,
  but its presence does not replace upstream target validation or online
  trajectory generation.

### 4.3 RH56 interface as implemented

`HandBackend` exposes connect, execute, read state, and stop. Implementations are
mock, direct RS485 serial, an unfinished vendor ROS2 service shim, and JAKA tool
RS485 passthrough. The hand schema has a useful explicit canonical ordering.

### 4.4 RH56 SDK review

| Concern | Finding | Decision for current stage |
|---|---|---|
| Latency/blocking | Direct serial exchange sleeps and can wait up to `max(timeout*4, 0.1)` per register. `read_state` performs three sequential reads. Tool passthrough opens a fresh TCP connection per command and has retry sleeps. | Do not construct, connect, poll, or command RH56 in arm-only runtime. |
| Frequency | Configurations claim 5-20 Hz, but there is no measured effective rate or bounded worst case. | Out of scope; benchmark before future hand integration. |
| Thread safety | No ownership or locking model. | Future hand worker must be isolated from arm servo. |
| Socket/serial management | Direct serial is persistent; JAKA TIO JSON TCP is recreated per request. | Future refactor should use an owned persistent transport where supported. |
| Reconnect | None. | Add explicit disconnected/fault lifecycle in future hand stage. |
| Watchdog/stop | Direct stop is a no-op; JAKA-tool stop only warns. | Never rely on this as an arm safety mechanism. |
| Exceptions | Basic checksum and timeout errors exist; no structured retry/error taxonomy. | Future refactor. |
| Timestamps/state sync | No feedback timestamps; force, angle, and current are separate transactions. Tool mode often has angles only. | Future refactor. |

The RH56 canonical order and calibration/schema utilities are worth keeping.
The transports need redesign before closed-loop hand teleoperation, but that
work is not a prerequisite for the arm because the hand will be hard-disabled.

## 5. Existing teleoperation modules

The repository contains four families:

1. HEBI Mobile I/O AR pose, relative follower, real arm publisher, and shadows.
2. Xbox velocity intent, palm-target IK, real bridge, and shadows.
3. iPhone camera/MediaPipe RH56 hand tracking and safety experiments.
4. The current TeleDex adapter, calibration, real publisher, and RViz shadow.

The TeleDex adapter itself has several good prototype behaviors: it validates a
proper rotation matrix, uses a lock around callback data, rejects stale or
disconnected input, preserves the second button missing from the upstream
normalizer, checks port ownership, and has focused unit tests.

It is nevertheless coupled to the legacy stack:

- `TeleDexPhoneClient.read()` returns `HebiMobileIOSnapshot`.
- The real publisher imports `RelativePoseLagFollower` and HEBI quaternion
  helpers.
- The configuration section is named `relative_pose_lag_follow`.
- The real bridge log source is still `xbox_ros2_bridge_arm_teleop`.
- A ROS JSON message passes through a controller designed for Xbox/HEBI.
- The documented design explicitly says it reuses relative anchoring, lag
  pause, JAKA IK, and EDG behavior, contrary to the clean-redesign requirement.

The calibration file is also not ready for 6-DoF production use. It is a signed
axis translation mapping, has `real_motion_confirmed: false`, records a maximum
fit error of about 10.15 degrees, and does not establish the complete set of
world, robot base, tool, end-effector, and mounted-device orientation
transforms.

## 6. Keep, isolate, and rewrite decisions

### Keep

- Official JAKA SDK binaries, C/C++ headers, and release notes.
- JAKA mini2 and mounted-tool MuJoCo assets, after controlled-frame verification.
- JAKA/RH56 network and protocol diagnostics as hardware bring-up tools.
- RH56 canonical schema and configuration, inactive in this stage.
- YAML loading and conventional `configs`, `scripts`, `tools`, `tests`, `docs`
  layout.
- Existing logs as replay and timing evidence.
- TeleDex transport knowledge: callback fields, port guard, disconnect and stale
  detection, and button semantics. Reimplement behind the new generic contract;
  do not inherit HEBI types.
- Calibration capture evidence as an audit artifact, not as confirmed 6-DoF
  production calibration.
- Simulation and digital-twin assets for offline workspace/collision checks.

### Isolate as legacy/experimental

- All HEBI input, follower, filtering, mapping, timing, and real publisher code.
- Xbox mapping and controller paths.
- `relative_pose_lag_follow.py` and its state/filters.
- `JakaServoJogController` and `JakaPalmTargetJogController`.
- Combined `run_real_arm_hand_ros2_node` control runtime.
- Current TeleDex real publisher and shadow until replaced.
- MediaPipe/RH56 teleoperation.

Legacy scripts should remain runnable, because repository guidance explicitly
protects them, but they must not be imported by the new package.

### Rewrite

- Timestamped device-neutral teleoperation contracts.
- JAKA typed SDK/EDG transport and lifecycle.
- Input-to-target clutch and frame mapping.
- Pose validation and discontinuity handling.
- SO(3)-correct adaptive filtering.
- Branch-continuous IK and singularity monitoring.
- Online jerk-limited trajectory tracking and stopping.
- Safety supervisor and fault state machine.
- Deterministic runtime/thread/process orchestration.
- Non-blocking metrics and logging.
- Arm-only launcher and configuration schema.
- Calibration protocol for translation and orientation frames.

## 7. Technical debt and real-time risks

Priority is P0 (blocks safe foundation), P1 (blocks hardware acceptance), or P2
(maintainability/scale).

| Priority | Debt/risk | Consequence |
|---|---|---|
| P0 | Nominal 50 Hz ROS servo path conflicts with JAKA's 8 ms EDG requirement | Irregular motion, controller starvation, unpredictable safety response |
| P0 | Blocking feedback, flags, JSON, marker publication, hand polling, and log flush share one executor | Deadline misses and cross-subsystem interference |
| P0 | No coherent timestamped input/robot/command data model | End-to-end latency and stale-state safety cannot be proven |
| P0 | TeleDex path reuses forbidden HEBI controller and follower | Clean architecture objective is not met |
| P0 | RH56 is constructed even when a mock hand is selected in combined arm runtime | Unnecessary coupling; future real backend could block arm control |
| P0 | SDK lifecycle does not guarantee EDG shutdown on every failure path | Servo mode can be left ambiguous |
| P1 | Calibration is translation-axis-only and unconfirmed | Unsafe orientation and frame mapping |
| P1 | IK targets a combined-model RH56 palm with no explicit tool/EE contract | Frame errors and model/controller mismatch |
| P1 | No singularity metric, self/environment collision gate, or verified dynamic workspace envelope | Large joint motion can result from small Cartesian targets |
| P1 | ROS queue depth 10 can retain stale commands | Added lag and obsolete targets |
| P1 | `time.time()` is used for control/log freshness in several paths | Wall-clock adjustment can corrupt ages and durations |
| P1 | Current two-level stale path can take roughly 0.45 s before bridge timeout | Slow communication-loss response |
| P1 | Watchdog status also means ordinary target deadband hold | Health telemetry is semantically ambiguous |
| P1 | No CPU, memory, call latency, deadline, or true end-to-end measurement | Production claims cannot be supported |
| P2 | Quaternion ordering varies (`xyzw` and `wxyz`) | Easy frame/orientation defects |
| P2 | Runtime parameters are mostly unvalidated dictionaries | Invalid configurations fail late |
| P2 | Logging payloads are very large and duplicated | Hundreds of MB over short trials; I/O pressure |
| P2 | No explicit architecture dependency test | Legacy imports can silently return |

## 8. Evidence from existing TeleDex/JAKA logs

Three TeleDex publisher logs and three bridge logs dated 2026-07-15 were found.
Only the first two pairs contain enabled servo periods. Offline measurements:

| Trial | Input records/rate | Valid input | Input receive age | Bridge effective log rate | Enabled ticks | Enabled mean/max tick dt | Max bridge TCP target error |
|---|---:|---:|---:|---:|---:|---:|---:|
| 18:10 | 6,500 / 30.00 Hz | 6,448 | mean 8.8 ms, max 46.7 ms | 32.35 Hz | 526 | 44.5 / 130.4 ms | 7.6 mm |
| 18:21 | 11,307 / 30.00 Hz | 11,142 | mean 10.2 ms, max 35.7 ms | 32.48 Hz | 1,647 | 37.7 / 127.8 ms | 18.4 mm |
| 18:29 visible | 398,061 / 30.00 Hz | 0 | no pose | 28.27 Hz with a 250 s gap | 0 | n/a | n/a |

In both motion trials, all but one enabled tick exceeded 16 ms, even though one
EDG interpolation cycle is 8 ms. The active JAKA SDK filter was `joint_nlf`
during enabled ticks. No fault was latched, but the current `watchdog_active`
field was set on most records because it also represents `target_deadband`, not
only failures.

These logs prove that the current architecture misses its nominal loop timing.
They do **not** prove true phone-to-robot latency because there is no common
source-capture timestamp and no physical motion event measurement. CPU
utilization and memory were not recorded. Documentation claiming no real JAKA
motion is also inconsistent with logs that show enabled EDG periods; the exact
physical outcome must be confirmed by the operator rather than inferred.

## 9. Proposed architecture

### 9.1 Architectural rules

- TeleDex is one `PoseInput` implementation, never a framework dependency.
- All control-domain values use SI units, explicit frame IDs, sequence numbers,
  and monotonic timestamps.
- Canonical quaternion storage is `xyzw`; SDK/TeleDex conversions occur only at
  adapters.
- Convention `T_A_B` means a transform mapping coordinates in frame B into
  frame A.
- Latest target wins. No unbounded or FIFO target queue is permitted.
- Logical layers stay independently testable. Real-time layers may be fused in
  one native worker where a thread boundary would harm determinism.
- ROS2 is an observability/integration edge, not the real-time transport.
- Logging is lossy under load by design; motion deadlines are not.
- No RH56 import, backend, topic subscription, or command exists in the arm-only
  executable.

### 9.2 Data flow diagram

```mermaid
flowchart TD
    TD[TeleDex Session] --> IA[TeleDex input adapter]
    FUT[Future pose source] --> PI[PoseInput interface]
    IA --> PI
    PI --> M1[Latest pose mailbox]
    M1 --> PV[Pose validation and source health]
    PV --> CL[Clutch / relative anchor]
    CL --> TF[Central frame transform graph]
    TF --> PF[Adaptive SE(3) pose filter]
    PF --> TT[Target tracking and bounded prediction]
    TT --> WS[Workspace and Cartesian safety]
    WS --> IK[Branch-continuous IK and singularity checks]
    IK --> M2[Latest joint target mailbox]

    M2 --> RT[125 Hz JAKA servo worker]
    RF[JAKA EDG feedback] --> RT
    SS[Slow JAKA safety status] --> RT
    RT --> OTG[Jerk-limited online trajectory]
    OTG --> JL[Joint limits and tracking watchdog]
    JL --> EDG[JAKA EDG joint command]
    EDG --> ROBOT[JAKA mini2]
    ROBOT --> RF

    PV --> SUP[Safety supervisor/state machine]
    WS --> SUP
    IK --> SUP
    RF --> SUP
    SS --> SUP
    SUP --> RT

    PI -. sampled metrics .-> LOG[Bounded telemetry ring]
    SUP -. events .-> LOG
    RT -. timing and state .-> LOG
    LOG --> LW[Low-priority logger/ROS publisher]
```

### 9.3 Runtime/process model

| Execution context | Rate | Owns | Must not do |
|---|---:|---|---|
| TeleDex network thread | Source-driven, observed about 60 Hz upstream | Session callbacks and conversion to `PoseSample` | Robot calls, transforms, filtering, logging I/O |
| Python input/control process | New sample-driven plus supervisor tick | Validation, clutch, transforms, pose filter, Cartesian bounds, IK, high-level state | Direct SDK calls, blocking file writes |
| Native JAKA servo worker, CPU-affined | 125 Hz / 8 ms | Command SDK session, EDG lifecycle, latest target, OTG, joint safety, stop trajectory, deadline watchdog | ROS callbacks, JSON, markers, hand work |
| JAKA status monitor | 10-20 Hz after multi-client validation | Slow controller safety/status calls on separate session | EDG command ownership |
| Logger/metrics worker | Best effort | Binary/compact telemetry drain, aggregation, JSON/ROS export | Backpressure into control |

The native-worker recommendation follows the local SDK's real-time guidance and
avoids relying on the Python GIL, ROS executor scheduling, or garbage collection
for an 8 ms deadline. Local IPC should use a non-blocking latest-value Unix
datagram or fixed shared-memory SPSC mailbox with sequence and monotonic age.
The servo worker remains safe if the Python process or ROS graph disappears.

If the Python `jkrc` binding is retained for an early hardware probe, it must
pass the same 125 Hz timing and fault-injection gates. Failure promotes the
native worker from recommendation to hard requirement without changing any
upstream interface.

### 9.4 Lifecycle/state machine

The new supervisor states are independent of all legacy state machines:

```text
DISCONNECTED -> STANDBY -> ARMED -> ACTIVE
                      ^       |       |
                      |       v       v
                      +--- STOPPING <- DEGRADED
                              |
                              v
                           STANDBY

Any state -- severe transport/robot/safety error --> FAULT_LATCHED
FAULT_LATCHED -- explicit operator reset while safe --> STANDBY
```

- `ARMED` requires valid calibration, fresh input, fresh coherent robot state,
  safe controller status, valid model, and released deadman.
- The deadman rising edge captures both source and robot anchors.
- Releasing deadman immediately requests a jerk-bounded stop; EDG is disabled
  after zero-motion confirmation or a bounded stop timeout.
- Short input delay enters `DEGRADED`: bounded constant-velocity prediction is
  allowed for at most 40 ms, then target velocity decays toward zero.
- Stale input, target IPC, robot feedback, monitor status, or repeated loop
  deadline miss enters `STOPPING` or `FAULT_LATCHED` according to severity.
- Reconnection never resumes motion. It requires deadman release and a new
  rising edge/anchor.
- Collision, estop, protective stop, joint-limit violation, invalid OTG/IK, or
  loss of command transport is latched and may invoke `motion_abort` when a
  controlled stop cannot be trusted.

Initial stale thresholds should be replay- and hardware-tuned, with proposed
bring-up values of 75 ms fresh, 75-150 ms degraded, and greater than 150 ms
stale. The present approximately 450 ms combined timeout is not retained.

## 10. Data contracts

The contracts should be small frozen dataclasses (or matching C structs) with no
device-specific fields in the control core.

| Type | Required fields |
|---|---|
| `PoseSample` | source ID, sequence, source timestamp optional, host receive monotonic ns, frame ID, position m, quaternion xyzw, tracking quality |
| `RunGateSample` | source ID, sequence, monotonic ns, pressed, valid, reason |
| `InputHealth` | connected, pose age, sequence gap, update rate, invalid count, last reason |
| `ArmState` | sequence, controller timestamp optional, host receive monotonic ns, joint q/dq/torque, TCP pose, servo state |
| `RobotSafetyState` | timestamp, powered, enabled, estop, protective stop, collision, soft limit, connection, error code |
| `CartesianTarget` | sequence, generated monotonic ns, base-to-EE pose, target twist, source age, anchor ID |
| `JointTarget` | sequence, generated monotonic ns, q target, optional dq target, model/anchor ID |
| `JointSetpoint` | cycle sequence, scheduled/send timestamps, q/dq/ddq, OTG status |
| `SafetyDecision` | state, allow/stop/abort action, latched flag, machine-readable reasons |
| `TimingSample` | scheduled/start/finish/send timestamps and per-stage durations |

Run-gate state is separate from pose so a future device can provide motion while
an independent safety device provides hold-to-run. Hand data will later use a
separate interface and mailbox; it will not enlarge `PoseSample` or the arm
deadline path.

## 11. Coordinate and calibration design

All transforms live in one `FrameGraph`; no axis swap or quaternion inversion is
allowed in the controller, filter, IK, or runner.

Required named transforms:

```text
T_world_teledex          dynamic TeleDex device pose
T_robot_base_world       calibrated source-world to robot-base mapping
T_operator_control_td    static mounted-device/control-frame extrinsic
T_robot_base_tool        robot feedback/commanded tool pose
T_tool_ee                configured tool-to-controlled-EE transform
T_robot_base_ee          derived controlled end-effector pose
```

Relative clutch mapping is computed from a source anchor and robot anchor, then
expressed through the graph. Translation and rotation use the same rigid-frame
convention. Orientation filtering happens after mapping, on SO(3), not by
averaging quaternion components.

The calibration format must include:

- schema version and transform convention;
- parent/child frame IDs;
- translation and quaternion for every static transform;
- source device/app/version and robot/tool identity;
- capture time and monotonic-independent metadata;
- residual translation/rotation error and capture coverage;
- explicit operator confirmation state;
- content hash so runtime can log the exact calibration used.

The current signed-permutation file may seed translation shadow tests. It cannot
authorize full 6-DoF real motion. The new calibration sequence must validate
six translations and positive/negative rotations about all three axes, then run
a robot-disabled and shadow replay before low-speed hardware use.

## 12. Motion processing and filtering

### 12.1 Pose validation

Reject rather than repair:

- non-finite data;
- non-unit or discontinuous orientation beyond small numeric normalization;
- non-monotonic sequence/time;
- impossible source linear/angular velocity or acceleration;
- a reset/relocalization jump;
- stale tracking or poor source quality;
- an unknown frame or calibration hash.

Quaternion sign is made continuous against the prior sample before computing
angular velocity. A relocalization invalidates the anchor and requires deadman
release/re-arm; it must never be smoothed into real motion.

### 12.2 Selected measurement filter

Use a One Euro-style speed-adaptive first-order low-pass filter:

- translation: one vector filter using vector speed for a common cutoff;
- orientation: geodesic SO(3) filtering using log-map angular speed and slerp;
- derivative filter cutoff independent from pose cutoff;
- actual sample `dt`, clamped only for numeric protection;
- reset on clutch, relocalization, invalid/stale input, or source change.

This method raises cutoff during purposeful fast movement and lowers it while
nearly still, directly addressing the responsiveness/noise tradeoff described
by the original [One Euro Filter paper](https://hal.science/hal-00670496).
A moving average is rejected because it adds a fixed window delay and handles
dropouts poorly.

Initial replay-tuning values, not final hardware constants:

| Channel | `min_cutoff` | `beta` | derivative cutoff | Rationale |
|---|---:|---:|---:|---|
| Translation | 2.0 Hz | 30 s/m | 1.0 Hz | About 2 Hz near still; rises to about 6.5 Hz at 0.15 m/s |
| Orientation | 2.0 Hz | 3 s/rad | 1.0 Hz | About 2 Hz near still; rises to about 5 Hz at 1 rad/s |

These are selected as conservative starting points for recorded ARKit-like
input. Final values must come from a Pareto sweep of jitter RMS versus phase
delay on TeleDex replay and hardware tracking tasks. Separate parameters are
needed because metres/second and radians/second have different scale.

### 12.3 Target tracking and delay handling

- Consume each new source sample once; do not repeatedly re-filter the same pose
  at 125 Hz.
- Estimate bounded target twist from filtered poses.
- For a brief delay, predict with constant velocity for no more than 40 ms and
  cap prediction displacement/rotation.
- Decay prediction velocity to zero as age approaches stale threshold.
- Never jump to a reappearing absolute target. Require a new clutch anchor after
  stale/reconnect/relocalization.

## 13. IK, trajectory generation, and Cartesian controller

### 13.1 Selected command strategy

Use branch-continuous, feedback-seeded IK to convert each safe Cartesian target
to a joint target, then use a six-DoF joint-space online trajectory generator in
the EDG worker.

Reasons:

- joint limits are explicit and testable;
- singularity/conditioning can be supervised before sending;
- command continuity can be guaranteed in the actual controlled coordinates;
- the newest target can replace the prior target without resetting velocity or
  acceleration;
- the controller can generate a safe stop even if the input process dies;
- JAKA's joint NLF can remain a secondary wider envelope.

IK requirements:

- seed from latest measured/commanded joints;
- prefer the prior solution branch;
- damp near singularities and expose condition/manipulability metrics;
- enforce hard and soft joint margins;
- bound per-update solution displacement;
- verify FK target error and reject non-convergence;
- use an explicit controlled frame (`tool0`, configured TCP, or EE), never an
  implicit RH56 body name.

### 13.2 Selected trajectory generator

Use local state-to-state Ruckig with current position, velocity, acceleration,
new target state, and per-joint velocity/acceleration/jerk bounds. Its official
[online trajectory documentation](https://docs.ruckig.com/tutorial.html)
describes cycle-by-cycle replanning from the current kinematic state and bounded
velocity, acceleration, and jerk. Intermediate waypoints are neither needed nor
allowed in this use, avoiding the community edition's non-local waypoint path.

The worker retains the last commanded `q`, `dq`, and `ddq`, advances one 8 ms
step, sends `q`, and feeds the output state into the next update. A new IK target
changes the target state, not the current kinematic state, so the trajectory
continues without stop/restart discontinuities. Measured joints supervise the
commanded state; excessive tracking error requests stop rather than forcibly
snapping the trajectory state to feedback.

Initial low-speed commissioning envelopes, all subject to robot/model
verification:

| Limit | Initial value |
|---|---:|
| TCP linear speed | 0.06 m/s |
| TCP linear acceleration | 0.40 m/s² |
| TCP linear jerk | 2.0 m/s³ |
| TCP angular speed | 0.35 rad/s |
| TCP angular acceleration | 1.0 rad/s² |
| TCP angular jerk | 5.0 rad/s³ |
| Joint speed | 0.25 rad/s |
| Joint acceleration | 0.80 rad/s² |
| Joint jerk | 4.0 rad/s³ |

The effective limit is always the minimum of configured commissioning limit,
verified JAKA/controller limit, model limit, workspace-specific limit, and any
active safety scaling. Tool payload and physical mounted-hand inertia must be
configured even though RH56 actuation is disabled.

## 14. Safety layers

Safety is layered and independently observable:

1. Startup interlocks: explicit real-hardware flag, calibration confirmation,
   robot identity, tool identity, model hash, workspace config, network, power,
   enable, estop, and protective-stop checks.
2. Operator gate: hold-to-run, release-to-stop, rising-edge anchor.
3. Input safety: tracking quality, age, sequence, jump/relocalization rejection.
4. Cartesian safety: workspace polytope/boxes, maximum anchor excursion, target
   lead, speed/acceleration/jerk envelopes.
5. Model safety: IK convergence, joint soft/hard limits, singularity threshold,
   optional self/environment collision check outside the 8 ms critical path.
6. Robot safety: coherent feedback age, command tracking error, controller
   flags, torque/current where available.
7. Runtime safety: loop deadline and consecutive-miss watchdog, target IPC age,
   process heartbeat, SDK call timeout/error policy.
8. Stop escalation: jerk-bounded stop, servo disable/EDG shutdown, then
   `motion_abort` for severe or untrusted conditions.

Workspace definitions must be centralized and expressed in robot base frame.
The current TeleDex box is useful only as a candidate; its origin and tool
semantics must be confirmed before reuse.

## 15. Proposed module decomposition

```text
src/teleoperation/
  __init__.py
  contracts.py              # immutable device-neutral data contracts
  clocks.py                 # monotonic clock abstraction for deterministic tests
  mailbox.py                # latest-value/sequence semantics
  config.py                 # typed schema and cross-field validation

  input/
    interface.py             # PoseInput protocol
    teledex.py               # TeleDex Session adapter only
    replay.py                # timestamp-faithful log replay
    mock.py

  transforms/
    se3.py                   # canonical pose/quaternion operations
    frame_graph.py           # T_A_B ownership and composition
    calibration.py           # loading, validation, hashes

  processing/
    pose_validator.py
    clutch.py
    one_euro_se3.py
    target_tracker.py        # bounded prediction and delay degradation

  motion/
    kinematics.py            # robot-model adapter and controlled frame
    ik.py                    # branch-continuous damped IK
    workspace.py
    limits.py

  safety/
    supervisor.py
    state_machine.py
    watchdogs.py
    decisions.py

  jaka/
    interface.py             # typed arm/servo contract
    python_probe.py          # non-production capability/diagnostic backend
    status_monitor.py
    errors.py

  runtime/
    arm_app.py               # composition root, arm only
    input_worker.py
    control_worker.py
    ipc.py
    metrics.py
    log_worker.py
    ros_observer.py          # optional, one-way observability/integration

native/jaka_servo_worker/
  CMakeLists.txt
  main.cpp
  jaka_edg_session.*         # typed lifecycle and SDK error mapping
  servo_loop.*               # 125 Hz schedule/affinity/watchdog
  trajectory.*               # local Ruckig wrapper
  ipc.*
  telemetry.*

configs/teleoperation/
  teledex_jaka_arm.yaml
  schemas/

tools/teleoperation/
  check_teledex_input.py
  calibrate_frames.py
  benchmark_jaka_edg.py
  replay_session.py
  analyze_session.py

scripts/
  run_teledex_jaka_arm.sh     # future explicit arm-only entrypoint
```

The decomposition has interfaces only at actual replacement boundaries: input
device, robot transport, model/IK, and telemetry sink. Filter, transform,
trajectory, and safety layers are concrete components with single ownership,
not plugin frameworks.

Future hand integration adds a separate hand input/retarget/command worker and
joins only at the top-level supervisor/recorder. It does not change `PoseInput`,
the arm transform path, the JAKA servo worker, or arm timing.

## 16. Test and measurement plan

### 16.1 Offline and automated

- Contract/config validation and serialization round trips.
- SE(3) composition/inversion, quaternion continuity, and frame-graph property
  tests.
- One Euro deterministic step, noise, constant-velocity, variable-rate, dropout,
  and reset tests.
- OTG tests proving continuous position/velocity/acceleration and bounded jerk
  during target changes and stop requests.
- IK branch continuity, singularity, unreachable target, and joint margin tests.
- Safety state-machine transition table and fault-latching tests.
- Fake SDK fault injection: delayed call, error code, disconnect, stale feedback,
  partial startup, failed shutdown.
- Replay every existing TeleDex log through the new pipeline and produce
  latency/noise/smoothness comparison plots.
- Dependency test forbidding imports from HEBI/Xbox/legacy follower/controller
  modules under `src/teleoperation` and `native/jaka_servo_worker`.
- RH56 hard-disable test: arm-only composition must not import or instantiate
  `rh56_driver`.

### 16.2 JAKA capability gates before motion

1. SDK/controller version and EDG method presence.
2. `edg_init`, feedback packet schema, controller timestamp, and clean shutdown.
3. 125 Hz zero-motion hold with command and feedback timestamps.
4. SDK call-duration distribution and impact of a separate status session.
5. CPU affinity/priority behavior on the target machine.
6. Controlled injected target-process loss and native stop response.
7. Reconnect while standby; verify no automatic motion resume.

### 16.3 Proposed acceptance metrics

These are initial engineering gates, not current results:

| Metric | Gate |
|---|---|
| EDG command rate | 125 Hz target; no sustained drift |
| Servo period | p99 <= 9 ms; max <= 16 ms in normal load; zero consecutive >16 ms |
| Deadline misses | <0.1% above 12 ms during two-hour run |
| Input host receive age | p95 <40 ms, p99 <80 ms on validated LAN |
| Target-to-send software latency | p95 <16 ms, p99 <24 ms |
| Deadman/stale detection to deceleration start | <=24 ms after local detection |
| Kinematic bounds | No command exceeds configured q/dq/ddq/jerk or Cartesian envelope (5% numeric tolerance for derived estimates) |
| Slow tracking task | Translation RMS <10 mm, peak <25 mm; rotation RMS <3°, peak <8° |
| Static target stability | No growing oscillation; report RMS/peak TCP and joint jitter |
| Long operation | Two hours, no unhandled exception, no faultless deadline burst, bounded memory, no reconnect/resume |
| CPU | Report per-process/core mean, p95, and peak; control core must retain >=50% idle headroom |

True end-to-end phone-motion-to-robot-motion latency cannot be derived only
from host receive timestamps. Measure it with a shared-clock source timestamp if
TeleDex adds one, and independently with a high-speed camera/LED or physical
motion event method. Report software and physical latency separately.

Every hardware report must include configuration/calibration/model hashes,
controller/SDK/app versions, network topology, payload/tool, rate distributions,
p50/p95/p99/max timing, tracking errors, smoothness bounds, CPU/memory, stop
tests, and run duration. Mock/shadow results must never be labeled hardware
results.

## 17. Migration plan

### Gate 0: approval

Approve or revise this architecture. Make no controller implementation before
this gate closes.

### Gate 1: preserve and isolate

- Preserve current dirty work and logs.
- Add architecture dependency tests.
- Mark existing HEBI/Xbox/combined bridge and current TeleDex runner as legacy or
  experimental without deleting them.
- Define one arm-only configuration namespace; RH56 defaults to absent, not mock.

### Gate 2: contracts and JAKA transport

- Implement timestamped contracts, monotonic clock, latest-value mailbox, and
  typed configuration.
- Implement structured JAKA SDK result handling and lifecycle.
- Build EDG capability/timing probe and clean zero-motion shutdown.
- Decide Python probe versus native worker solely by timing evidence; production
  target remains native unless Python meets every gate with margin.

### Gate 3: device-neutral input and frames

- Implement `PoseInput` and TeleDex adapter without a legacy import.
- Implement central SE(3)/frame graph.
- Replace signed-axis-only authorization with versioned 6-DoF calibration and
  explicit confirmation.

### Gate 4: pure motion and safety core

- Implement validator, clutch, adaptive SE(3) filter, bounded predictor,
  workspace, branch-continuous IK, safety supervisor, and stop semantics.
- Integrate local state-to-state Ruckig in the servo worker.
- Complete deterministic, replay, fault-injection, and dependency tests.

### Gate 5: shadow and soak

- Run TeleDex and recorded replay against robot model only.
- Validate frame directions, orientation, workspace, joint limits,
  singularities, target changes, dropout, and deadman behavior.
- Run multi-hour 125 Hz no-hardware soak with injected load and failures.

### Gate 6: staged hardware validation

1. Read-only feedback and timing.
2. Zero-motion EDG hold.
3. Single-joint milliradian motion with native stop tests.
4. Translation-only low-speed Cartesian motion.
5. One rotation axis at a time.
6. Full 6-DoF within a small verified workspace.
7. Long-operation and communication-degradation tests.

Each substage requires a written metric report and explicit approval before the
next. The RH56 remains powered/command-disabled throughout.

### Gate 7: promote and retain fallback

- Make the new arm-only launcher the documented TeleDex default only after all
  acceptance gates pass.
- Keep legacy scripts clearly named and available for comparison.
- Do not add hand control until a separate RH56 architecture review is approved.

## 18. Approval choices

The recommended approval is:

1. Accept the new device-neutral `teleoperation` package boundary.
2. Accept a dedicated 125 Hz JAKA EDG worker with native C++ as the production
   target and Python only as a measured capability probe.
3. Accept branch-continuous IK plus joint-space jerk-limited Ruckig tracking.
4. Accept adaptive One Euro SE(3) filtering with replay-tuned parameters.
5. Accept the explicit arm-only process with no RH56 construction.
6. Authorize Gate 1 and Gate 2 only for the next implementation stage.

Implementation should stop again after the JAKA zero-motion capability report
if the target controller, SDK binding, native build, EDG feedback, or 125 Hz
timing differs materially from the assumptions documented here.

---

## 19. Implementation status appended 2026-07-16

Migration Gates 1–2 were approved and implemented as a clean-slate arm-only
foundation. The implementation does not add TeleDex ingestion or any live
input-driven robot motion. It isolates the historical HEBI-dependent path,
defines device-neutral timestamped contracts, adds a bounded latest-target Unix
datagram transport, and adds a native single-owner JAKA SDK/EDG worker with fake,
read-only, zero-motion, and explicitly disabled minimal-motion modes.

No hardware mode was executed. Native fake-backend timing met the nominal 8 ms
timer target in three five-second trials, but connected 125 Hz stability remains
unproven. See `docs/jaka_control_foundation_gates_1_2.md` and
`docs/jaka_control_foundation_gates_1_2_implementation_report_20260716.md` for
the design, tests, quantitative results, hardware precautions, and next-gate
recommendation.
