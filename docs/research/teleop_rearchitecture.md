# Quest 3 → JAKA Mini2 teleoperation rearchitecture research

Status: **offline prototype only**.  No JAKA, EDG, ServoJ, Quest, or RH56
connection was made for this work.  This page is current for the isolated
`feature/quest-jaka-teleop-rearchitecture` worktree; it is not an operator
procedure and does not authorize physical execution.

## Scope, baseline, and evidence boundary

The research worktree was created with `git worktree add` from production
`7a02aa6fbdb01efddcc09605bc8e1e22eaf3bc8b` (2026-07-27).  The requested
`5401f3c` is an earlier pushed commit; production had two later PWL fixes
(`e619aed`, `7a02aa6`) before this worktree was made.  Production was clean;
the protected `tools/teleop_mujoco_jaka_rh56.py`, `learned_policy/`, and all
other worktrees were not changed.

The available tracked inputs are two 55-target AcceptedArmTarget reconstructions
(`jaka_edg_sim_initial` and historical `jaka_edg_failed_run`), matching native
fake-worker telemetry, the acceleration-failure fixture, and aggregated
singularity/output-feasibility reports.  Raw HTS captures, a matching physical
plant trace, and raw high-jerk/singularity command sequences are not tracked in
this worktree.  They were not reconstructed or added: the results below are
therefore command-model/FK replay, not measured Quest-to-robot latency or TCP
tracking.  The singularity report is used as policy evidence, not as a new
trajectory input.

## Current path and the coupling to remove

The authoritative production path is still:

```text
HTS + CTRL -> validation/bounded input -> release-before-press clutch
-> mapping/filter -> shared continuation IK/safety -> AcceptedArmTarget
-> MuJoCo adapter | JAKA adapter -> native PWL/SDK worker
```

It correctly has a single post-IK joint boundary, zero native IK, no target
backlog, and separates `HOLD_REJECTED` from liveness.  The remaining design
pressure is that PWL replacement, output feasibility, SDK lifecycle, status
polling, timing classification, and telemetry live too close together in the
native transport.  That makes a normal-rate policy, hard output boundary, and
transport defense difficult to reason about independently.  Historical PWL
data demonstrates the issue: before the current accepted-output acceleration
gate, the failed replay had J4/J6 peaks of 245.48/234.54 rad/s² and
51,171.73/49,257.05 rad/s³ while still having low 8 ms reconstructed tracking
lag.  Current production rejects the unsafe candidates; this research does not
claim the current branch retains those values.

## Source audit

Revisions, licenses, and non-vendoring decisions are in
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md).  These are primary
repository, official documentation, or paper sources only.

| Candidate | Maintenance / ROS | XR and command model | Safety/realtime observation | Reuse assessment |
| --- | --- | --- | --- | --- |
| Unitree `xr_teleoperate` | Active (v1.5 release in Dec 2025); Python/DDS, not ROS control | Native Quest 3, PICO, AVP; XR pose and dexterous-hand/data pipeline | Pinocchio retargeting and robot-specific Unitree DDS; no JAKA adapter | Borrow the process separation and recording ideas; no robot-control code. |
| Spes `teleop` | Active Python package; ROS 2 interface | WebXR phone/VR; hold Move button; publishes `PoseStamped`, subscribes current pose | Simple interface explicitly says it lacks filtering; optional Pinocchio velocity/acceleration-limited servo | Apache package is a possible dependency only after compatibility test; preserve current HTS timestamps/clutch semantics. |
| JAKA `jaka_ros2` | Active official ROS 2 repository; documented Ubuntu 22.04/Humble/x86_64, SDK 2.2.2 | No XR; services, ServoJ demo, FollowJointTrajectory/MoveIt | Service and trajectory-oriented; no documented EDG 8 ms real-time hardware interface; docs list MiniCobo, not Mini2 | Reference lifecycle/message mapping only. No license and wrong Thor platform/model prevent reuse. |
| MoveIt 2 / Servo | Active ROS 2, BSD-3-Clause | `TwistStamped`, `PoseStamped`, joint jog inputs | Differential IK, singularity scaling, collision checks, joint margins, smoothing plugin and composable/priority-aware Servo thread | Strong candidate dependency for an isolated ROS 2 deployment; not yet runnable on this Thor. |
| UM-ARM-Lab `vr_teleop` | Archived Feb 2024, ROS 1 | Vive / Unity grip “grab” interaction | MoveIt + robot-specific interface, no modern 8 ms adapter contract | Historical clutch/relative-pose reference only; do not reuse code. |
| `vr_ros2_bridge` | Small ROS 2 successor, last source update observed 2024 | ROS 2 bridge, not a complete servo stack | No JAKA-specific safety or output shaping | Interface reference only, license not confirmed. |
| OpenTeleVision / OpenTeach | Active research/data-collection ecosystem, Apache for OpenTeleVision | WebXR/Quest 3 visual feedback, hand data and dataset paths | Human interface/data capture, not safety servo or JAKA adapter | Optional future XR/recording study; keep arm and RH56 control isolated. |
| robosuite | Active MIT simulation/data package | No Quest transport by itself | Simulation/data collection, not real-time JAKA hardware | Useful only as a data/sim reference. |

For all sources, timestamp semantics, watchdog, Mini2 collision model, JAKA
alarm classification, and 8 ms output semantics are either absent or not
verified for this stack.  They must remain owned by this project.  No source
justifies copying code; unlicensed sources are explicitly non-reusable.

## Feature matrix

`✓` means implemented/observed; `~` means adaptable or partial; `—` means not
provided or not verified.  The current stack is the only column evaluated with
the Mini2 model and historical JAKA telemetry.

| Capability | Current | Unitree | Spes | JAKA ROS 2 | MoveIt Servo | UM VR | Proposed boundary |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Quest / WebXR input | ✓ HTS/CTRL | ✓ Quest | ✓ WebXR | — | ~ topic | Vive | preserve HTS adapter |
| timestamp/freshness | ✓ monotonic | ~ | ~ | ROS stamp | ROS stamp | ROS 1 | preserve local monotonic receipt |
| clutch/reference relative pose | ✓ release-before-press | ~ align/start | ✓ hold move | — | ~ joystick | grip | preserve unchanged |
| filtering/mapping | ✓ | ✓ | — by default | — | input-scale/smoothing | ~ | XR layer only |
| full IK / differential IK | ✓ / — | Pinocchio | optional Pinocchio | SDK/MoveIt | ✓ differential | MoveIt | selectable safety-servo |
| singularity / collision / limits | ✓ / ✓ / ✓ | robot dependent | ~ limits | MoveIt planning | ✓ / ✓ / ✓ | own full-IK checks or Servo |
| speed / accel / jerk | ✓ / ✓ / PWL transition | robot dependent | vel/accel only | trajectory limits | velocity + smoothing | — | explicit independent shaper |
| latest target / no backlog | ✓ | not verified | callback | action/service | topic latest | not verified | ✓ mailbox depth 1 |
| watchdog/liveness | ✓ separate feasibility | robot dependent | — | service/SDK state | stale cmd timeout | — | explicit health boundary |
| controller alarm / SDK lifecycle | ✓ single SDK | Unitree only | — | ✓ JAKA service lifecycle | hardware dependent | robot dependent | JAKA adapter only |
| 8 ms output | ✓ EDG step 1 | — | — | ServoJ demo, not EDG proof | controller dependent | — | retain adapter contract |
| simulation/replay/log/data | ✓ / ✓ / ✓ / ~ | ✓ / ✓ / ✓ | ~ / — / — / — | Gazebo/RViz | ✓/tests | demos | shared logger, no RT I/O |
| RH56 isolation | ✓ | supports other Inspire variants | — | — | — | grippers | retain hard prohibition |

Keep: Quest HTS receiver, release-before-press clutch and relative reference,
frame mapping, Mini2 model/collision checks, `AcceptedArmTarget` evidence (or
its exactly equivalent immutable `JointCommand`), controller-state record,
physical authorization gate, RH56 prohibition, and replay/history tests.
Rewrite or extract: post-IK output-shaping policy, hardware-health interface,
and telemetry ownership.  Delete nothing in this phase.

## Architecture choices

| Choice | Composition | Expected behavior | Main limitation / decision |
| --- | --- | --- | --- |
| A | Current XR/clutch → Pose/Twist → MoveIt Servo → official-style JAKA ROS 2/ServoJ adapter | Best mature collision/singularity facility; ROS can add 10–100 ms+ depending QoS, executor and controller | Existing reconstructed evaluation had 232–400 ms latency and 10–11 mm TCP RMS; official JAKA ROS 2 is Humble/x86_64/non-Mini2. Keep as a future ROS 2 evaluation, not current recommendation. |
| B | Current XR/clutch → independent Cartesian/differential safety servo → jerk-bounded joint velocity streamer → JAKA adapter | Lowest dependency/latency path; explicit dynamic bound and latest-wins | Must own/test all differential IK, collision, and singularity behavior. Implemented offline output prototype. |
| C | Current XR/clutch → existing shared full IK → jerk-bounded joint-position shaping → thin JAKA adapter | Reuses trusted Mini2 continuation/collision/singularity evidence and removes shaping from transport | Needs a mature OTG choice for production; Ruckig smooths well but its current candidate is 144–200 ms delayed. Implemented offline contract prototype. |

Both B and C retain: `q_hold` continuous engagement, no backlog, fresh
`HOLD_REJECTED` heartbeat, clutch-release controlled stop, and hard stops for
stale input, timing, controller alarm, and SDK errors.  A hardware adapter
must never rerun IK.  It accepts only pre-shaped immutable joints and reports
health; it performs no JSON serialization or file I/O on its timing thread.

Recommended end state is **C with an independently testable, mature
jerk-bounded OTG behind a narrow output interface**, while retaining B as the
lower-latency benchmark.  Do not select A until a supported ARM64/Jazzy-or-
containerized MoveIt/JAKA adapter exists and directly beats this result on the
same replay.  Do not select the current Ruckig branch as-is: its dynamic
quality is good but it misses the natural-following latency priority.

```text
XR input adapter -> teleop state machine -> RobotCommand (immutable)
 -> Mini2 safety/kinematics servo -> output shaper (latest-wins)
 -> JAKA hardware adapter -> sole SDK session
                                  |                 |
                         health/watchdog      RT ring telemetry
                                  \                 /
                                  non-RT logger/replay
```

Simulation and hardware share everything through the output shaper.  The
future hardware process remains responsible for SDK error/alarm hard-stop and
cleanup only; feasible-target rejection is not a timeout.

## Offline prototypes and results

Implementation is under `src/teleop_rearchitecture/`; it has no JAKA SDK,
socket, ROS, or hardware import.  `ResolvedRateVelocityServo` is B's
post-differential-IK output model.  `JerkBoundedPositionServo` is C's
post-full-IK stream with one-target feed-forward.  They use a one-entry mailbox,
125 Hz period, π rad/s velocity, 4π rad/s² acceleration, and a **project
prototype** 50 rad/s³ jerk bound (not a vendor Mini2 limit).  Both have
`native_ik_calls=0`, `rh56_commands=0`, mailbox max depth 1, and controlled
release stop in 8 ms on these replays.

| Same 55-target replay | PWL historical baseline | Ruckig selected baseline | MoveIt candidate baseline | B velocity prototype | C position prototype |
| --- | ---: | ---: | ---: | ---: | ---: |
| sim-initial TCP position RMS | 0.31 mm | 6.02 mm | 10.91 mm | 5.72 mm | 5.23 mm |
| sim-initial orientation RMS | 0.89 mrad | 13.47 mrad | 15.89 mrad | 14.11 mrad | 14.17 mrad |
| failed-run TCP position RMS | 0.23 mm | available in prior report | 10.78 mm | 5.68 mm | 5.19 mm |
| output jerk peak | 12,054 / 51,172 rad/s³ (normal / failed J4) | <=45 rad/s³ (selected sweep) | <=62.83 rad/s³ | <=50 rad/s³ | <=50 rad/s³ |
| reconstructed translation latency | 8 ms | 144–200 ms | 296–400 ms | not identifiable* | not identifiable* |
| mean shaper CPU | native not comparable | 6.08 µs OTG (prior) | 381–428 µs candidate | 25–38 µs | 32–36 µs |

\*The prototype stores the command age (mean about 95 ms including the explicit
post-input settling window), but no tracked physical/network timestamp can
identify translation/rotation latency.  It intentionally does not turn a
kinematic replay into a latency claim.  Results are checked in under
`docs/research/teleop_rearchitecture/results/`.

The prototype output peak acceleration never exceeds 4π rad/s² and peak jerk
never exceeds 50 rad/s³.  It computes FK TCP command-model RMS/peak,
orientation RMS/peak, per-joint velocity/acceleration/jerk, reversals, command
age, stop time, settling error, CPU cost, 125 Hz feasibility, and zero-backlog
metrics.  It does **not** fabricate physical tracking, joint-limit margin,
singularity, collision, or controller fault values.  Those remain generated by
the upstream shared safety pipeline and are tested as explicit policy events.

## JAKA and Thor assessment

The local JAKA C SDK 2.2.7 documents `edg_init`, servo enable, status/estop/
collision reads, and `edg_servo_j` with `step_num * 8 ms`; its `step_num=1`
contract supports the existing 125 Hz adapter.  It does not make the SDK a
trajectory generator.  Retain the sole-session pattern: a healthy lightweight
`get_robot_status_simple` read is separated from the event-triggered estop and
collision classification, and any SDK error/alarm/power-enable loss is a hard
stop.  No verified controller alarm-history/per-joint alarm API exists in the
current evidence.

This Thor is Jetson AGX Thor, Ubuntu 24.04.4, ARM64/aarch64, Python 3.12.3,
CMake 3.28/GCC 13.3.  ROS 2 is not installed.  The research venv successfully
runs MuJoCo 3.10 headlessly.  Official JAKA ROS 2 documents Ubuntu 22.04,
Humble and x86_64 and packages MiniCobo rather than Mini2; installation would
be a future manual/container feasibility gate, not a `sudo` action here.

## Migration and physical gates

1. Freeze production behavior and keep the present safety/replay suite as an
   oracle.  Add raw, privacy-reviewed HTS and native telemetry fixtures only
   if their provenance and retention permit it.
2. Lift the post-IK output contract and test it against B/C and current PWL;
   preserve exactly the acceptance/liveness split and physical gate.
3. Choose a production OTG only after CPU, jitter, target replacement, stop,
   output-limit, collision/singularity, and replay benchmarks on Thor.  Do not
   merge prototype code into production yet.
4. Build a thin fake-first JAKA adapter with a ring telemetry sink; prove no
   SDK calls in XR/safety layers and zero native IK.  Keep ROS/MoveIt in a
   separate evaluation worktree/container.
5. Before any separately authorized physical gate: verify model/TCP/payload
   state without writes, controller alarms/workspace/stop access, exact
   `q_hold` continuity, 8 ms deadline under load, stale input, clutch release,
   `HOLD_REJECTED` recovery, controller alarm/SDK/timing fault cleanup, no
   backlog, RH56=0, and evidence logging.  The unresolved J4 collision and
   unvalidated post-fix acceleration gate remain blockers to expansion.

## Validation performed

Offline only:

- `tests/test_teleop_rearchitecture.py`: mailbox no-backlog, non-active target
  exclusion, two shapers' v/a/j bounds, and clutch controlled-stop continuity.
- Both prototype replays on the two tracked AcceptedArmTarget streams, with
  MuJoCo FK and checked-in JSON results.
- Existing PWL acceleration fixture/replay and the critical Quest/JAKA set ran
  successfully: **130 passed** (including the nine new prototype tests); no
  test invokes a physical backend.
- `compileall` and the Release native build passed.  Full collection initially
  found 565 tests but five optional digital-twin/camera imports were absent.
  Adding the isolated optional packages exposed the host SciPy 1.11 / NumPy
  2.5 ABI mismatch, leaving two unrelated digital-twin collection errors.
  This branch does not change global packages to mask that environment issue;
  the targeted safety suite remains the validation result for this work.
- MuJoCo headless replay passed for both prototypes.  `git diff --check` and
  a final clean-worktree audit are run before commit.

No real hardware was used, no actuator setting was read or written, and this
document does not change the existing physical authorization boundary.
