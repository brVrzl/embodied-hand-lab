# Quest-to-JAKA parity follow-up after `14b2909`

Date: 2026-07-22

Scope: offline audit and synchronization only. No JAKA connection, EDG entry,
servo enable, or physical command was performed.

## 2026-07-23 post-payload collision diagnostic instrumentation

The JAKA App reported a J4 servo-collision event during an otherwise normally
terminated run. The previous worker checked controller faults only at preflight,
and the Python log compared measured joints with a future AcceptedArmTarget
endpoint instead of the joint point emitted on that 8 ms tick.

The bounded diagnostic path now polls the SDK 2.2.7 interfaces present in the
committed headers (`get_robot_status_simple`, `is_in_estop`, and
`is_in_collision`) from a separate read-only SDK session. The monitor publishes
a bounded latest snapshot approximately every 16 ms; the 8 ms command thread
only copies that snapshot and never performs these non-deterministic queries.
A query failure, controller error, collision, E-stop, power loss, or enable loss
stops before another ServoJ point. Bounded native telemetry records the current emitted command, same-cycle
measurement, shortest-angle tracking difference, active endpoint, velocities,
acceleration, sequence, heartbeat age, and host monotonic/wall clocks. An
offline extractor selects event windows around alarms, J4/J6 peak lag, peak
wrist acceleration, and clutch release.

The installed SDK exposes no verified per-joint servo-alarm history API, so the
metrics explicitly mark that detail unavailable while retaining the controller
error code/message and live collision state. No alarm is cleared automatically.

For the single post-payload diagnostic, the launcher can lower the shared output
contract to 1.0 rad/s for all joints. Existing continuation/HOLD_REJECTED remains
the only candidate-reduction authority. The worker can also abort before SDK
dispatch above the existing 4π rad/s² acceleration diagnostic; this is a
gate-only rejection, not clipping or a new filter. The 0.35 rad tracking hard
limit remains, and a wrist lag increasing for three cycles at half that hard
limit stops this diagnostic run.

Recorded operator configuration: payload 0.8 kg, COM
[9.289, 12.427, 36.961] mm, upright installation X=0°/Z=0°, TCP zero. The
process records but never writes these settings.

The first post-payload attempt at instrumentation synchronously issued all
three status calls on every 8 ms command tick. It safely stopped before Quest
engagement with `hard_completion_timing_miss` after 66 identical hold commands;
there was no controller alarm and no commanded motion. This demonstrated that
fault polling itself must remain outside the EDG timing-critical path. The
separate monitor above is the offline correction; it was not physically retried
in the same task.

## Repository state at start

- Worktree: `/home/thor/projects/embodied_lab`
- Branch: `feature/jaka-teledex-control-foundation`
- Starting HEAD: `b032055228ff6dd8796ba722e4d0c93065ab0fd6`
- Commit after the prior parity checkpoint: `b032055 stabilize Quest-to-JAKA
  simulation teleoperation checkpoint`
- Concurrent user-owned worktree change:
  `tools/teleop_mujoco_jaka_rh56.py`; preserved and excluded from this work.
- No untracked files existed at task start.

## Current successful simulation entry

Recommended command:

```bash
./scripts/run_quest_jaka_sim_demo.sh --viewer
```

Exact Python entry:

```bash
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof \
  --config configs/sim/quest_hts_jaka_mini2_live_demo.yaml --viewer
```

The wrapper has no hardware imports or fallback. The retired `live` command and
keyboard/SPACE clutch callback were removed in `b032055`.

## Full simulation-side change audit after `14b2909`

The single commit `b032055` changed 14 files. Its behaviorally relevant and
non-behavioral changes are classified below.

| Change | Classification | Physical synchronization |
|---|---|---|
| `se3.bounded_pose_step`: coupled XYZ + quaternion-SLERP progress fraction | Shared target behavior | Moved to shared/default policy |
| Up to five half-step feasibility backtracks after a rejected trial | Shared target behavior | Moved to shared/default policy |
| Rejected feasibility trials hold the last accepted target while the operator retreats | Shared clutch/reference behavior | Moved to shared/default policy |
| Jacobian condition/minimum singular-value gate no longer depends on candidate velocity | Shared acceptance behavior | Already in shared evaluator; retained for both |
| Previous accepted IK seed, target and joint-delta diagnostics | Shared IK/diagnostics | Included in common result/accepted diagnostics |
| Swing/twist wrist-roll diagnostics and safe-limit/branch metrics | Shared diagnostics | Logged for both; never alters adapter output |
| Reference wrist logging | Shared diagnostics | Logged for both |
| Dedicated bounded UDP receive thread and skip-expired-target-ticks behavior | Input/timing behavior | One shared receive worker now used by both entries |
| Removal of keyboard clutch and retired translation-only live entry | Input authorization behavior | Physical path already controller-only; verified no fallback |
| `--viewer/--no-viewer`, X11 discovery, camera, status and `--ik-debug` text | MuJoCo UI/debug only | Deliberately excluded |
| Blue/green marker rendering and viewer synchronization | MuJoCo visualization only | Deliberately excluded |
| 500 Hz `mj_step`, actuator gains, jerk-limited simulated reference and hand slew | MuJoCo plant only | Deliberately excluded |
| Simulation tracking error, viewer skipped-frame and physics-overrun counters | MuJoCo telemetry only | Deliberately excluded |
| Documentation and regression tests, including wrist roll | Documentation/tests | Updated parity coverage; no target-side copy |

No changes were found in `b032055` to HTS syntax parsing, canonical handedness,
latched head-yaw transform, translation/orientation bases, transform order,
One Euro filter coefficients, 20 ms input interpolation, 1:1 gains, controller
thresholds, 60 Hz shared target rate, IK gain/damping/iterations/tolerances, TCP
tool body, or joint order/units. These remain authoritative in the same shared
YAML and modules.

## Parity defect found

`b032055` exposed `simulation_only_recovery=True` only from the MuJoCo launcher.
The physical runner constructed the same session with its default `False`, so
large but valid full-pose input and feasibility retreat behavior could produce a
different filtered TCP, acceptance sequence, and clutch state.

The receive timing was also duplicated: MuJoCo timestamped UDP arrival on a
bounded receiver thread, while the physical Python loop polled UDP inline. A
slow target tick could therefore change physical sample receipt/interpolation.

Finally, the physical launcher instantiated `JakaMujocoSimulation` and a
MuJoCo adapter even though it never stepped physics. That unnecessarily placed
simulator plant state in the physical process and obscured the required plant
boundary.

## Synchronized architecture

```text
shared bounded Quest UDP receipt
-> shared HTS/CTRL validation, timestamps and interpolation
-> shared controller clutch/reference generation
-> shared relative SE(3), head-yaw basis and One Euro filters
-> shared requested TCP
-> shared coupled SE(3) continuation/backtracking
-> SharedJakaTargetGenerator continuation IK and hard acceptance
-> immutable AcceptedArmTarget
   |-> MujocoArmTargetAdapter -> independent MuJoCo plant/viewer
   `-> JakaAcceptedJointTargetAdapter -> bounded datagram -> 125 Hz worker
```

`SharedJakaTargetGenerator` uses the committed MJCF only as a deterministic
kinematic/collision model for FK, Jacobians and continuation IK. It owns no
`MjData` plant control, calls no `mj_step`, and has no viewer. The physical
launcher no longer constructs `JakaMujocoSimulation`, a MuJoCo output adapter,
or a composite fanout, and never reads simulated qpos.

The simulator and the physical robot are separate plants. Both adapters receive
the accepted joint tuple; neither plant's measured state becomes the other's
command. Physical measured joints are synchronized into the plant-free
kinematic reference only while the arm clutch is disengaged.

## Authoritative configuration and accepted-target contract

`configs/sim/quest_hts_jaka_mini2_live_demo.yaml` remains the only live arm
configuration. New `shared_target_generation` values apply to both outputs:

| Parameter | Value |
|---|---:|
| continuation enabled | `true` |
| maximum feasibility backtracks | `5` |
| minimum continuation fraction | `0.03125` |
| rejection policy | hold last accepted and allow operator retreat |

Existing mapping, filter, IK and gate values are not copied into the physical
launcher. The JAKA adapter still contains only representation-contract values:
J1 through J6, radians, absolute EDG servo-j.

The unselected `hardware_conservative` One Euro profile was removed from the
live configuration. It was not active, but retaining a hardware-named tuning
copy created a second apparent authority that could be selected later and
silently break parity.

### Motion-processing and limit inventory

| Stage or limit | Shared target | MuJoCo only | JAKA only | Result of this audit |
|---|---|---|---|---|
| HTS/CTRL parsing, receive timestamp, freshness and 20 ms interpolation | yes | no | no | one receiver/session behavior |
| release-before-first-press, capture, release, dropout and recapture | yes | no | no | one clutch/reference state machine |
| latched gravity-aligned head yaw | yes | no | no | transform order/signs unchanged |
| XYZ basis, 1:1 per-axis translation and proper rotation basis | yes | no | no | no hardware remap or 0.05 scale |
| unrestricted mapped roll/pitch/yaw at 1:1 scale | yes | no | no | no hardware orientation envelope or reduced gain |
| selected position/orientation One Euro filters | yes | no | no | obsolete hardware profile removed |
| 0.20 m target envelope, Cartesian jump/velocity and IK continuity gates | yes | no | no | current simulation gates retained identically; hard reject/continuation, not hardware-only clipping |
| coupled SE(3) bounded step and up to five feasibility backtracks | yes | no | no | latest simulation behavior made authoritative for both |
| previous-accepted seed, DLS IK, collision/singularity/joint-limit checks | yes | no | no | single kinematic evaluator and continuation state |
| accepted J1-J6 tuple | yes | no | no | immutable adapter boundary |
| joint velocity/acceleration/jerk reference model | no | yes | no | deliberately remains after MuJoCo adapter; never reaches JAKA |
| MuJoCo actuator gains, 500 Hz stepping, hand slew and renderer | no | yes | no | deliberately excluded from command-critical path |
| time-resample adjacent accepted tuples onto 8 ms points | no | no | yes | representation/transport-only PWL; accepted endpoints are unchanged |
| finite/CRC/sequence/manufacturer-limit rejection, stale stop, tracking-error abort | no | no | yes | retained pass-through fault containment; no scaling or smoothing |
| old TeleDex workspace/slew/low-pass/tracking-error shaping | no | no | no | not imported or called by either Quest entry |

The 75 degree relative-rotation and 0.30 m operator values in the precision
mapper are warning thresholds, not clippers. The 0.20 m target envelope and
the feasibility jump/velocity limits are part of the current successful shared
simulation acceptance policy and therefore remain shared; no additional
physical workspace box, speed limiter, acceleration limiter, jerk limiter, or
low-pass stage exists before the JAKA adapter. The later EDG transport-only
resampler documented below is after this immutable adapter boundary.

### Retained physical fault containment

These layers are after or beside the accepted-target boundary and are inactive
during healthy operation:

1. Manufacturer joint-position ranges reject the whole invalid J1-J6 packet;
   they never clip it.
2. Startup cannot move: P1/P2 never enter EDG, P4 remains calibration- and
   approval-gated, and its first live tuple must match measured joints within
   0.001 rad.
3. Operator E-stop confirmation, Ctrl-C, clutch release and every exit path
   stop publication and run servo/EDG cleanup where applicable.
4. The 100 ms command-stale threshold stops the stream; it does not change a
   fresh 60 Hz target. Longer controlled-stop and communication-failure
   thresholds handle a failed link.
5. Wire length/version/kind/CRC/sequence/timestamps and every numeric value are
   validated; a malformed or non-finite packet is rejected whole.
6. Quest tracking loss, stale wrist/controller data and explicit clutch release
   disengage the shared state machine, so no new target reaches the adapter.
7. IK, collision, singularity, joint-limit and target-envelope failures retain
   the previous accepted seed/target and publish no rejected candidate.
8. Excessive measured joint error (`0.35` rad for two cycles) aborts; it never
   scales a target as the threshold is approached.
9. SDK read/write failures and a dead native worker terminate the run and invoke
   cleanup rather than producing a substitute target.

No hardware-only continuous workspace, joint/TCP speed, acceleration, jerk,
tracking-error scaling, interpolation, low-pass or trajectory-generation stage
was retained.

`AcceptedArmTarget` now contains:

- unique accepted-target sequence;
- host and source Quest sequence metadata;
- source timestamp, host receipt timestamp and accepted-target timestamp;
- reference and clutch generation identifiers;
- requested and continuation-filtered TCP targets;
- accepted J1 through J6 radians;
- immutable final/attempted acceptance reasons, continuation fraction and
  backtrack count, IK residuals, Jacobian metrics and joint-limit margin.

Rejected trials never create an accepted target or reach either adapter.

## Timing and handoff boundaries

| Domain | Rate/boundary | Coupling |
|---|---|---|
| Quest UDP receipt | dedicated thread, bounded 256-datagram FIFO | shared by both entry points; timestamps at receipt |
| Shared target generation/IK | 60 Hz; expired ticks skipped rather than replayed | no physics/viewer call |
| JAKA transport | separate native process, 8 ms / 125 Hz continuous PWL resampling | Unix datagram, finite kernel buffer, never waits for Python/MuJoCo |
| MuJoCo physics | 500 Hz in simulation entry only | after MuJoCo adapter |
| Viewer | 60 Hz best effort | simulation entry only |

A simulator stall cannot hold a lock needed by the JAKA process: there is no
shared lock, callback, queue join, simulation object, or viewer in the physical
launcher. If shared target generation itself stops publishing, the native
worker's 100 ms stale-command stop remains the fault-containment boundary.

## Target, model and dynamic parity are different

1. **Target parity** means equal validated input, references, mapped/filtered
   TCP, IK decision and immutable accepted J1-J6 sequence before adapters.
2. **Kinematic-model parity** compares JAKA SDK TCP with shared/MuJoCo FK at the
   same measured physical joints.
3. **Dynamic tracking parity** compares time-varying accepted joints with actual
   physical or simulated plant response.

Only target parity is established offline here. Equal targets do not prove
physical TCP accuracy or dynamic tracking.

Remaining physical error sources include robot-model dimensions, joint-zero
offsets, joint sign/order, base registration, active JAKA user frame, installed
TCP/tool frame, payload/tooling, encoder/mechanical calibration, EDG/network
latency, true servo lag, backlash, compliance, friction/load, and MuJoCo
actuator/contact assumptions. No arbitrary compensating offset was added.

## Prepared P1/P2 model validation (not executed)

P1 read-only state/TCP capture:

```bash
.venv/bin/python tools/teleoperation/run_jaka_hardware_probe.py state-read \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-s 30 --expected-tool-id 0 --expected-user-frame-id 0 \
  --acknowledgement I_ACKNOWLEDGE_JAKA_HARDWARE_RISK \
  --metrics-file logs/quest_jaka_p1_state_read.json

.venv/bin/python tools/quest_jaka_model_parity.py \
  --worker-metrics logs/quest_jaka_p1_state_read.json \
  --output logs/quest_jaka_p1_model_parity.json
```

P2 command shadow, still no EDG or command API:

```bash
.venv/bin/python tools/quest_jaka_hardware.py p2-shadow \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-sec 60 --approval I_AUTHORIZE_P2_QUEST_JAKA_COMMAND_SHADOW \
  --log logs/quest_jaka_p2_shadow.jsonl \
  --summary logs/quest_jaka_p2_summary.json \
  --metrics logs/quest_jaka_p2_worker.json \
  --capture logs/quest_jaka_p2_quest_capture.jsonl

.venv/bin/python tools/quest_jaka_model_parity.py \
  --worker-metrics logs/quest_jaka_p2_worker.json \
  --output logs/quest_jaka_p2_model_parity.json
```

The report records measured physical J1-J6, JAKA SDK TCP, MuJoCo FK at those
joints, shared-model TCP, millimetre position errors and degree orientation
errors, while keeping target and dynamic parity as separate fields.

## Offline verification

All commands ran from `/home/thor/projects/embodied_lab` without a JAKA
connection:

```bash
.venv/bin/python -m pytest tests/test_quest_jaka_shared_pipeline.py -q
# 25 passed in 5.30s

.venv/bin/python -m pytest \
  tests/test_quest_jaka_smooth.py \
  tests/test_quest_jaka_sim.py \
  tests/test_quest_jaka_se3.py \
  tests/test_quest_jaka_wrist_roll.py \
  tests/test_quest_live_controller_sim.py -q
# 48 passed in 4.80s

.venv/bin/python -m pytest tests/test_native_jaka_servo_worker.py -q
# 14 passed in 2.68s

.venv/bin/python -m pytest -q
# 571 passed, 1 skipped in 63.75s

cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
# configured and generated successfully

cmake --build build/jaka_servo_worker -j2
# [100%] Built target jaka_servo_worker

.venv/bin/python -m compileall -q src tools tests
# exit 0, no output

git diff --check
# exit 0, no output
```

The shared-pipeline suite covers exact discrete-state/metadata parity and a
`1e-12` tolerance for continuous pre-adapter pose/joint comparisons. It
includes independent XYZ, roll/pitch/yaw, combined 6D, downward wrist,
quaternion wraparound, repeated and bursty samples, release/recapture,
tracking loss/recovery, stale/invalid samples, IK/envelope rejection, shared
retreat continuation, and a blocked MuJoCo step while the plant-free target
generator continues. It also proves the JAKA representation adapter imports
without importing `mujoco` and that the physical entry contains no simulator
plant/viewer/step path.

P0 passed. No connected gate was entered, and no JAKA SDK connection, servo
enable, EDG entry, or physical command occurred.

## 2026-07-22 EDG command-time correction (E0 only)

This follow-up was performed from starting HEAD
`86e3c3927d554b4d3877dbf89e2c7eb4fae1c70c`. It did not connect to JAKA,
enter EDG, enable servo mode, clear a fault, or send a physical command. The
concurrent user change in `tools/teleop_mujoco_jaka_rh56.py` remained untouched
and is excluded from the checkpoint.

### Confirmed root cause and interface contract

The failed P4 transport was:

```text
60 Hz AcceptedArmTarget -> 125 Hz repeat latest -> edg_servo_j(ABS, step_num=1)
q0, q0, q1, q1, q2, ...
```

The shared target diagnostic divided `q[k]-q[k-1]` by the measured upstream
interval of about 16.67 ms. The controller instead received the changed point
as a new `step_num=1` point whose motion period is 8 ms. On the recorded stream,
this made the old controller-visible J4/J6 peaks 4.835/4.610 rad/s
(277.0/264.1 deg/s), above the configured and documented pi rad/s (180 deg/s)
boundary. This is the confirmed defect. It is not attributed to IK, a
singularity, or J5 being near zero. Payload/mounting uncertainty can amplify an
alarm but does not explain or replace the discontinuous command timing.

The official JAKA contracts used here are:

- [`edg_servo_j` running period is `step_num * 8 ms`](https://www.jaka.com/docs/en/guide/1.7.2/SDK/cpp.html);
- [ServoJ points must be continuously supplied every 8 ms and the user program
  plans the trajectory](https://www.jaka.com/docs/guide/V3/tcpip.html);
- [J4/J6 position-deviation and torque-feedforward errors explicitly call out
  command continuity, acceleration, payload and mounting checks](https://www.jaka.com/docs/en/guide/V3/errinfo.html).

The installed C++ SDK defines `JointValue::jVal[6]` in radians and the adapter
continues to pass J1 through J6 unchanged. The actual native call remains
`edg_servo_j(&value, ABS, 1)`.

### Selected design

The native transport now uses this bounded causal state:

```text
latest AcceptedArmTarget q/t (generated_monotonic_ns)
-> preserve last successfully emitted q and servo time
-> replace only the current destination (no FIFO/backlog)
-> PWL evaluation on each 8 ms native deadline
-> finite/manufacturer-limit/output-speed reject
-> edg_servo_j(sample, ABS, 1)
```

`generated_monotonic_ns` is selected because it is the local command-host
`CLOCK_MONOTONIC` timestamp at accepted-target generation. Source Quest time is
a different clock, input-receipt time describes transport rather than intended
target timing, and dispatch time includes adapter scheduling. The accepted
timestamp difference defines segment duration; the native deadline defines
execution time. Strictly non-monotonic timestamps abort before an SDK command.

An online interpolator cannot emit intermediate samples toward `q[k]` before
`q[k]` exists. It therefore incurs one bounded causal segment interval, without
buffering or replaying a target FIFO. A newly arrived destination preempts the
active destination from the last emitted point/time. The failed-stream offline
model measured a 19.997 ms maximum final-endpoint delay; the real-time fake
worker measured 20.693 ms including scheduling/receipt phase. The delay is
bounded and does not accumulate: source duration 900.003 ms became 920.000 ms,
and the final endpoint error was exactly zero.

The first target is still checked against measured startup joints. The first
emitted point is the measured state; a non-bit-exact but accepted aligned point
converges over one 8 ms startup segment. STOP, clutch release, tracking loss,
timeout, worker/SDK fault, or operator stop exits immediately and discards the
active segment. A future process starts from a new measured state and cannot
resume an old segment.

The worker computes per-joint `dq`, velocity and acceleration from actual SDK
call timestamps. Non-finite values and manufacturer position violations reject
the whole point. The shared `command_maximum_joint_velocity_rad_s` (pi rad/s)
is the single speed-boundary authority and aborts before the SDK call. The
shared 4*pi rad/s^2 acceleration value is reported diagnostically only; it is
not a hardware-only clamp or trajectory shaper. Jerk is diagnostic only.

### Candidate comparison: exact failed-run dispatched target stream

The source contains all 55 immutable targets successfully dispatched by the
Python adapter before the failed worker stopped (the failed worker had consumed
49 when its measured tracking-error abort fired). Controller-visible velocity
and acceleration below use the EDG command period, not 1/60 s.

| Candidate | points | duration | J4/J6 max delta | J4/J6 max velocity | J4/J6 max acceleration | >180 deg/s J4/J6 | repeats | discontinuous switches | endpoint |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| A: 125 Hz repeat-latest, step 1 | 114 | 904 ms | 2.216/2.113 deg | 277.0/264.1 deg/s | 34624.8/33017.1 deg/s^2 | 14/14 | 59 | 54 | exact |
| B: each target once, step 2 | 55 | 864 ms | 2.216/2.113 deg | 138.5/132.1 deg/s | 2330.8/2361.0 deg/s^2 | 0/0 | 0 | 54 | exact |
| C: 125 Hz PWL resampling, step 1 | 116 | 920 ms | 1.065/1.015 deg | 133.1/126.9 deg/s | 14064.7/13438.0 deg/s^2 | 0/0 | 2 | 0 | exact |

Candidate C is selected. Candidate B remains an offline comparison only:
`step_num=2` specifies a nominal 16 ms motion period but no stronger official
evidence establishes interpolation of arbitrary points. Its model is also
36.003 ms shorter than the recorded intended duration. The implementation does
not expose B as a live option.

The corrected native fake worker accepted all 55 targets, emitted 137 calls
(including the post-endpoint healthy repeat until the deliberate 200 ms stream
timeout), made zero IK calls, had zero speed rejects, reached the exact final
J1-J6 endpoint, and exited normally on `command_stream_timeout`. Its observed
J4/J6 peaks were 2.324/2.216 rad/s and 245.9/235.0 rad/s^2; acceleration is
reported rather than clamped. Evidence is in:

- `docs/measurements/jaka_edg_failed_run_resampling_20260722.json`
- `docs/measurements/jaka_edg_failed_run_native_fake_20260722/`
- `docs/measurements/jaka_edg_sim_initial_resampling_20260722.json`
- `docs/measurements/jaka_edg_sim_initial_native_fake_20260722/`

The same Quest capture was separately replayed from the successful simulation
initial joints. That run also ended exactly, with no output-speed crossing; it
is supporting coverage only and is not treated as the transport fix.

### Timing watchdog and isolation

- A single sub-period lateness such as 5 ms emits one point evaluated at current
  time, realigns the deadline, and never sends catch-up commands.
- A wake at least one full 8 ms period late, a start interval of at least 16 ms,
  or consecutive serious warnings hard-stops with a nonzero process exit.
- Viewer, MuJoCo physics and log writing do not share a lock, queue, event loop,
  callback or process with the native command deadline. Native joint mode still
  contains zero IK calls and no MuJoCo import/runtime dependency.
- The optional emitted-point recorder is rejected in connected modes and exists
  only to prove the fake SDK contract offline.

### E0 result and prepared gates

E0 passed. Focused resampler, fake SDK, native worker, adapter, parity, complete
repository, build, compile and whitespace checks passed. No physical gate ran.

E1 is a new, separately gated measured-position-only path through the same
resampler. It sends no motion target other than the initially measured J1-J6:

The first E1 evidence run exposed a measurement-handoff defect, not a timing or
controller fault: its fixed destination was read before EDG entry while the
resampler started from a fresh post-EDG measurement. The resulting maximum
startup offset was 1.6961e-5 rad. E1 now performs one atomic post-EDG handoff:
the fresh post-EDG J1-J6 value is simultaneously the hold destination,
resampler state, output-diagnostic state and tracking reference. The E1 native
mode does not consume the target socket, so neither a stale target nor the
normal live-target acceptance path can replace that hold value.

Metrics retain the prior global fields and add per-joint arrays (J1 through J6)
for target-to-measured tracking error, measured displacement from `q_hold`,
adjacent emitted-command delta, emitted velocity and emitted acceleration. The
pre-EDG measurement, post-EDG authoritative `q_hold`, their difference, fixed
destination, and first/last commands are recorded separately. Global maxima
must equal the maximum corresponding array element.

```bash
.venv/bin/python tools/jaka_edg_e1_zero_motion.py \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-sec 5 \
  --approval I_AUTHORIZE_E1_ZERO_MOTION_EDG_RESAMPLER \
  --estop-accessible --workspace-clear --rh56-command-path-absent \
  --metrics logs/quest_jaka_e1_resampler_zero_motion.json
```

Required E1 authorization phrase:

```text
I_AUTHORIZE_E1_ZERO_MOTION_EDG_RESAMPLER
```

Prepared E2 command (do not run until E1 passes and separate authorization is
given):

```bash
.venv/bin/python tools/quest_jaka_hardware.py e2-isolated \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-sec 20 \
  --approval I_AUTHORIZE_E2_ONE_SMALL_TCP_TRANSLATION \
  --estop-accessible --workspace-clear --rh56-command-path-absent \
  --log logs/quest_jaka_e2_isolated.jsonl \
  --summary logs/quest_jaka_e2_isolated_summary.json \
  --metrics logs/quest_jaka_e2_isolated_worker.json \
  --capture logs/quest_jaka_e2_isolated_capture.jsonl
```

E2 is operator-limited to one small TCP translation and return, then clutch
release; it adds no software scale or trajectory shaping. E3/P4 is not ready to
run before E1 and E2 pass.

The first E2 attempt stopped safely before any Quest-controlled command reached
the SDK. Its post-EDG hold was correct, but disengaged encoder polling repeatedly
reseeded the shared target generator; the resulting neutral target differed from
the immutable hold by 3.49066e-5 rad. The armed-session post-EDG J1-J6 value now
owns the resampler state, shared continuation seed and first-engagement baseline
until an explicit fresh startup handoff. Subsequent measured joints remain live
for monitoring but cannot rewrite those command states. E2 uses the same
0.001 rad startup-alignment contract already enforced by the native worker and
shared hardware configuration; it does not retain the conflicting 1e-7 rad
E2-only equality check. The guard still rejects larger mismatches and does not
modify an accepted target.

Before E2, read/report the active tool/user IDs and verify in the JAKA App the
RH56 payload mass and centre of mass plus robot mounting orientation. Tool 0,
user frame 0, order and radians were historically validated, but the repository
does not contain an authoritative RH56 payload/COM or mounting-orientation
record. Controller firmware/SDK interpolation details also remain unverified.
None of these uncertainties is automatically modified or used to excuse the
confirmed timing defect.
