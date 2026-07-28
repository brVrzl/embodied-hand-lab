# Quest 3 → JAKA Mini2 teleoperation rearchitecture research

Status: **offline prototype only**.  No JAKA, EDG, ServoJ, Quest, or RH56
connection was made for this work.  This page is current for the isolated
`feature/quest-jaka-teleop-rearchitecture` worktree; it is not an operator
procedure and does not authorize physical execution.

The research branch now has four local-only audit checkpoints above the
fetched remote `3e911f80ba8b02260fd68c1e7c8a9641521b3622`:

- `6f08b09b3a3d7584c517f8aec1ee9306cbb5003b` — corrected replay metric semantics;
- `d7661cc4d8d6e5c835d801a62114d8220bee5364` — unified evaluator and controlled-stop analysis;
- `afb54b3326cca482b508607f78dd3ab0bf5bd786` — ABI v1 and C++ shaping core;
- `a53ece339b945a79accaa70687f22c8f853c9344` — recoverable clutch lifecycle
  and SDK-free fake adapter.

They have not been pushed. The JAKA transport-contract audit and fake SDK seam
described later remain an auditable diff above the fourth checkpoint.

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

## Repository isolation audit (2026-07-27)

After `git fetch --prune origin`, `origin/HEAD` pointed to `origin/main` at
`891ae6f8c36c3c40a22a025cde288be3093fc5dc`.  Production local and remote
`feature/quest-jaka-pwl-acceleration-recovery` both remained at
`7a02aa6fbdb01efddcc09605bc8e1e22eaf3bc8b`.  Before this correction, the
research local branch, its upstream, and the fetched remote branch all matched
at `3e911f80ba8b02260fd68c1e7c8a9641521b3622`. The local checkpoints above were
then created without changing the upstream or fetched remote; nothing was
pushed.

All linked worktrees were inspected.  Production, ACT/Thor, MoveIt, Quest
input, Ruckig, TeleDex, and repository-cleanup were clean.  Four unrelated
worktrees already contained user changes and were left untouched: Quest
controller transport (13 status entries), Quest/JAKA arm audit (1), dual
clutch (7), and Quest/JAKA simulation (23).  Only this research worktree is
changed by the work reported below.  Both production and research copies of
`tools/teleop_mujoco_jaka_rh56.py` and `learned_policy/` remain unchanged.

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
release-stop semantics that are now measured separately during motion.

The subsequent isolated ABI phase adds an independent C++17 project under
`native/teleop_shaping/` and the versioned robot-independent contract described
in [`teleop_command_abi.md`](teleop_command_abi.md).  It preserves the Python C
active law only as a reference/conformance backend, while moving clutch release
into a distinct explicit-braking mode.  The C++ project and its Python test
bridge contain no JAKA SDK, ROS, Quest, MuJoCo, network, or RH56 dependency.

### Intended-target and timestamp correction

The first checked evaluator selected `samples[next_input]` after consuming all
targets with timestamps at or before the current servo tick.  During active
motion this is the *next future target*.  An independent active-window replay
with the pre-correction shapers reproduced the effect:

| Replay / prototype | future target RMS | causal latest-target RMS | artificial reduction |
| --- | ---: | ---: | ---: |
| sim initial / B | 9.064 mm | 8.454 mm | 0.610 mm |
| failed run / B | 8.835 mm | 8.214 mm | 0.621 mm |
| sim initial / C | 8.260 mm | 7.664 mm | 0.596 mm |
| failed run / C | 8.059 mm | 7.455 mm | 0.604 mm |

The bug is real, but it does **not** explain the old 5.19–5.72 mm values.  Those
values mixed roughly 0.9 s of active tracking with 2 s of final-target
settling; the long, low-error tail diluted active RMS.  The evaluator now
reports three disjoint meanings:

- `active_tracking`: causal latest target whose source timestamp is not later
  than the servo tick;
- `timestamp_interpolated_tracking`: explicitly non-causal joint-linear
  interpolation, retained only for comparison with older evaluators;
- `settling`: final-target error after the final source timestamp, including a
  declared threshold and first sustained settling time.

Candidate C feed-forward now divides target displacement by the two real
`control_monotonic_ns` target timestamps (mean source period 16.667 ms), not by
the 8 ms output period.  Non-increasing timestamps fail closed.  A clutch
release also clears any target feed-forward not yet consumed by a servo tick.

### Corrected B/C replay before the unified evaluator

This table records the previous correction round.  Its active window stopped
at the last grid tick *before* the final target timestamp, so only 54 targets
became active.  It remains here so the later unified-window change is not
silently overwritten.  Position is mm, orientation is
mrad, dynamics are the maximum over J1–J6 during the active window, and CPU is
the mean isolated Python shaper-tick time.  None is a physical latency or plant
tracking measurement.

| Replay / prototype | causal TCP RMS pos / rot | interpolated TCP RMS pos / rot | peak v / a / jerk | sustained settle | CPU mean |
| --- | ---: | ---: | ---: | ---: | ---: |
| sim initial / B | 8.454 / 14.273 | 8.720 / 15.278 | 0.347 / 1.652 / 38.834 | 1012 ms | 27.0 µs |
| sim initial / C | 7.742 / 14.121 | 8.003 / 15.117 | 0.376 / 1.773 / 50.000 | 772 ms | 26.5 µs |
| failed run / B | 8.214 / 13.048 | 8.508 / 14.092 | 1.048 / 4.473 / 50.000 | 1180 ms | 33.0 µs |
| failed run / C | 7.567 / 14.183 | 7.854 / 15.242 | 1.023 / 4.299 / 50.000 | 1196 ms | 24.4 µs |

All four runs stayed below π rad/s, 4π rad/s², and 50 rad/s³; their maximum
single measured tick CPU time was 1.41 ms, below the 8 ms offline budget.  Mean
causal command age was 7.54–8.29 ms, with a 16.00–16.74 ms peak.  Final
settling position error after the separate 2 s window was 0.00030–0.00047 mm.
Mailbox depth was one with zero queued/replaced backlog, native IK calls were
zero, and RH56 commands were zero.

### Moving clutch-release correction before the unified evaluator

The previous 8 ms result released only after the 2 s settling tail, when joint
velocity and acceleration were already effectively zero.  It was not a moving
stop test and is withdrawn.  The corrected replay triggers release at the
middle source target while moving and requires five consecutive ticks below
1e-3 rad/s velocity and 1e-2 rad/s² acceleration:

| Replay / prototype | pre-release peak | stop time | TCP stop displacement |
| --- | ---: | ---: | ---: |
| sim initial / B | 0.0227 rad/s | 808 ms | 0.267 mm |
| sim initial / C | 0.0318 rad/s | 816 ms | 0.355 mm |
| failed run / B | 0.0280 rad/s | 832 ms | 0.257 mm |
| failed run / C | 0.0278 rad/s | 792 ms | 0.376 mm |

These small-velocity command-model results preserve the acceleration/jerk
bounds, but 792–832 ms is too slow to claim an acceptable clutch stop.  The
result does not describe controller, SDK, or physical stop performance.

### Historical-baseline comparability audit

The source SHA-256 values match the Ruckig audit exactly (`04309b…663` and
`5aaefb…882`), all use a nominal 8 ms grid, and a direct MuJoCo check found
zero palm-FK difference between this prototype's `jaka_rh56.xml` and the
Ruckig evaluator's `jaka_rh56_visual_coacd.xml` on the 55 sim targets.  That is
enough for a qualified comparison to the *timestamp-interpolated* column, not
to the causal column.

| Historical result | sim / failed TCP RMS | sim / failed orientation RMS | status of comparison |
| --- | ---: | ---: | --- |
| old PWL | 0.311 / 0.251 mm | 0.886 / 0.727 mrad | Same inputs/palm FK and interpolated target, but the Ruckig audit extends active time 24 ms and historical PWL predates the current acceleration gate. Its 12,054 / 51,172 rad/s³ J4 peaks are incident evidence, not current production performance. |
| selected Ruckig | 6.017 / 7.924 mm | 13.470 / 12.691 mrad | Same inputs/palm FK and interpolated target; active-window +24 ms, end-triggered stop, and dynamics including stop differ. Its 144/200 ms lag is a trajectory-shift estimator, not measured latency. |
| MoveIt reference | 10.908 / 10.780 mm | 15.891 / 16.270 mrad | Same accepted streams and an interpolation window close to the corrected one, but it evaluates the zero-offset flange rather than the RH56 palm and compounds a custom differential-IK reference with Ruckig. It is explicitly not a MoveIt Servo runtime. |

The old PWL, Ruckig, and MoveIt numbers therefore remain context, not a single
strict A/B leaderboard.  The PWL 8 ms and MoveIt/Ruckig 144–304 ms figures are
best-lag reconstruction estimates.  This prototype deliberately reports no
translation or rotation latency because the tracked files contain no matching
network/physical observation timestamp.  Ruckig's 6.08–7.17 µs is OTG-only;
MoveIt's 381–428 µs differential-IK plus 14.6–16.8 µs Ruckig and this
prototype's Python tick times are likewise not whole-stack CPU comparisons.

Exact corrected artifacts, including active/settling dynamics and release
traces, are checked in under `docs/research/teleop_rearchitecture/results/`.
The prototype does **not** fabricate joint-limit margin, singularity,
collision, or controller fault values.  Those remain generated by the upstream
shared safety pipeline and are tested as explicit policy events.

## Unified evaluator (current result)

The checked evaluator in `src/teleop_rearchitecture/unified_evaluator.py`
replaces the earlier per-candidate metric code.  Every executable backend now
consumes the same versioned `AcceptedJointTarget` record: sequence,
`control_monotonic_ns`, six accepted joints, accepted/applied flags, and target
state.  The two input hashes remain `04309b…663` and `5aaefb…882`; each has 55
strictly increasing targets, explicit first/final joints, and a complete
target-interval distribution in the result JSON.

The common grid starts at the first target and runs every 8 ms.  Active ends at
the first grid tick at or after the final target, so all 55 targets are causal
and active: 114 samples over a 904 ms timestamp span for both fixtures.  A tick
may activate only the latest target with `control_monotonic_ns <= tick`.
Settling begins on the next tick for 250 samples, and the common moving release
event is the source-duration midpoint rounded down to the grid.  Active,
settling, and stop dynamics are never combined as the sole result.

All command separation is now named for the evaluated frame:
`rh56_R_hand_base_link` **palm-model command separation**, not physical TCP
tracking.  Both checked XML files have J1–J6 at qpos addresses 0–5, identical
joint axes/order, and zero position/quaternion difference at all 55 reference
targets with 1e-12 tolerances.

Each `teleop_unified_benchmark.record.v1` record contains the required
repository/dirty state, backend, fixture/hash, model/frame, period, reference
semantics, exact windows, causal and non-causal tracking, settling, active and
stop dynamics, accepted-target age, shift estimator, CPU distribution,
mailbox, stop result, and limitations.  All percentiles include mean, p50,
p95, p99, p99.9, and maximum.  JSON serialization rejects NaN/Infinity.  The
four older `teleop_rearchitecture_replay.v2` artifacts are retained as audit
history but are superseded for cross-backend comparison.

The generated current table is
[`results/unified_benchmark_table.md`](teleop_rearchitecture/results/unified_benchmark_table.md),
and its source is
[`results/unified_benchmark.json`](teleop_rearchitecture/results/unified_benchmark.json).
The generated table is produced from JSON by the replay tool and covered by a
test; it is not hand-maintained.

### Backend integration and evidence limits

- `historical_pre_gate_pwl_emitted_resampled` uses the recorded native output
  and linearly resamples its relative servo timestamps onto the common active
  grid.  The sim trace has one 9.617 ms realignment; the failed trace is an
  exact 8 ms recording.  Neither covers the common 2 s settling or a matching
  moving release, so those fields are unavailable rather than reconstructed.
- `production_style_pwl_reconstruction` executes an independent causal copy of
  the latest-segment PWL semantics for all windows.  It is not current gated
  production behavior.  Accepted-only files omit rejected candidates and the
  resulting holds, so current acceleration-gated PWL cannot be recovered.
- `selected_ruckig_position_otg` directly depends on MIT-licensed Ruckig 0.19.4
  with the old selected per-joint limits, time synchronization, discrete
  duration, zero target velocity/acceleration, and generated-state-preserving
  replacement.  It has no arbitrary target-horizon setting.
- B and C remain architecture-reference shapers.  C retains the real source
  timestamp feed-forward; neither is a production candidate.
- `candidate_c_cpp_reference` consumes the same causal ABI target stream.  Its
  active output conforms to Python C within floating-point rounding, while its
  controlled stop is a separate analytic jerk-limited mode.  Its measured CPU
  includes the Python/`ctypes` boundary and remains reference evidence only.
- MoveIt is excluded from machine comparison: the surviving evidence is a
  historical differential-IK-plus-Ruckig evaluation associated with that
  workstream, not a MoveIt Servo runtime, and its flange metric is not a palm
  metric.

### Unified comparable result

The final-target activation tick changes B/C active RMS slightly from the prior
table; the old → unified causal position values are B sim 8.454 → 8.527 mm,
C sim 7.742 → 7.808 mm, B failed 8.214 → 8.301 mm, and C failed 7.567 →
7.644 mm.  The settling times remain exactly 1012/772/1180/1196 ms when
measured from the final source timestamp.  Common midpoint release changes the
prior stop cases to B 800/832 ms and C 792/768 ms (sim/failed); this is a
release-state change, not a shaper improvement.

The old Ruckig evaluator's interpolation RMS 6.017/7.924 mm becomes
5.984/7.744 mm on the 114-sample common window.  Its old 144/200 ms trajectory
shift becomes 144/192 ms under the single shared estimator.  These are still
non-causal palm-trajectory shift estimates, never latency.  Historical PWL
interpolation is 0.312/0.254 mm after common-grid resampling; the independently
reconstructed PWL is 0.621/0.644 mm because its target activation phase is not
the recorded native worker phase.  Neither is current acceleration-gated PWL.

Finite-difference dynamics use backward differences on the common 8 ms grid;
the first sample uses fixture initial q and zero v/a.  Backend v/a/j are also
reported where available.  Candidate C reaches its 50 rad/s³ jerk bound on up
to 35.1% of sim ticks and 46.5% of failed-run ticks for at least one joint,
with 192/248 per-joint saturation transitions summed across the traces.  Its
tracking advantage over B therefore depends materially on jerk saturation.
Ruckig reaches its lower per-joint jerk limits even more often; the limits are
explicitly recorded and are not treated as equivalent tuning.

Algorithm-only mean CPU is about 2.4–2.5 µs for reconstructed PWL/Ruckig,
18.6–22.0 µs for B, and 15.5–16.3 µs for C in this run.  All observed maxima
are below 8 ms.  These numbers exclude scheduler wake-up jitter, IPC,
serialization, SDK send, controller, and network.

The new C++ active reference reproduces Python C's tracking, settling, and
dynamics values (rounding-only derivative differences).  Its checked
Python-to-C++ call mean is 4.67/4.72 µs, p99 8.44/7.02 µs, and maximum
29.32/46.12 µs for sim/failed.  That lower executable/reference cost is not a
scheduler, process-IPC, or realtime claim.  At the common moving release it
uses explicit braking and completes in 80 ms on both fixtures, versus the
Python active-law stop's 792/768 ms; palm-model stop displacement is
0.110/0.114 mm.  These are command-model values, not physical stop results.

## Controlled-stop policy sweep

The checked sweep has 60 deterministic release states: six peak-velocity bands
(0.02, 0.05, 0.10, 0.25, 0.50, and 1.00 rad/s) crossed with zero/positive/
negative acceleration, jerk ramp up/down, immediate replacement, direction
reversal, mixed six-axis, dominant wrist, and dominant shoulder conditions.
They are synthetic command states, not reconstructed Quest or physical data.
All policies use π rad/s, 4π rad/s², 50 rad/s³, and 8 ms.

| Policy | strict completed | strict mean / p95 / max | practical mean / max | direction-inconsistent cases | result |
| --- | ---: | ---: | ---: | ---: | --- |
| current stopping-point tracking | 60/60 | 1144.5 / 1648.4 / 1680 ms | 827.7 / 1360 ms | 60/60 | bounded but slow and reverses |
| explicit jerk-limited v=0/a=0 braking | 60/60 | 139.5 / 288.0 / 312 ms | 139.5 / 312 ms | 0/60 | selected semantic |
| C++ analytic explicit braking | 60/60 | 139.5 / 288.0 / 312 ms | 139.5 / 312 ms | 0/60 | selected C++ reference implementation |
| adaptive critically damped tracking | 40/60 | 524.2 / 608.0 / 640 ms among completions | 328.4 / 464 ms | 60/60 | rejected; all 0.5/1.0 rad/s cases fail |

The current stop target is built from velocity alone, ignoring incoming
acceleration, then handed back to the normal 36/10 position-tracking law.  It
crosses the fixed stop target and hunts in every matrix case.  The strict
threshold adds roughly 200–320 ms over the practical threshold, but it is not
the primary cause: at zero acceleration and 0.02 rad/s, the theoretical
jerk-limited bound is 40 ms, explicit braking finishes in 40 ms, while current
tracking needs 728 ms.  Gain adaptation improves low-speed time but becomes
unstable/saturated at 0.5–1.0 rad/s, reaching π rad/s, 4π rad/s², over 1.2 rad
joint travel and 0.648 m palm-model displacement.  It is rejected rather than
tuned around those failures.

Explicit velocity-interface braking is continuous at release, carries the
actual q/dq/ddq state, sets target velocity and acceleration to zero, and lets
the bounded trajectory generate the stop position.  It has zero limit
violations, zero rebound cases, and zero post-completion drift in this sweep.
Its strict envelope by speed is 40–48, 64–72, 88–96, 136–160, 184–224, and
272–312 ms.  Maximum palm-model stop displacement by band is 0.243, 0.895,
2.548, 10.360, 30.009, and 80.836 mm.  These are offline command-model bounds,
not physical stopping guarantees.

The independent C++ analytic profile is time-synchronized on the 8 ms grid.
Against Python/Ruckig explicit braking it has 0 completion or direction
mismatches and 0 limit violations.  Strict stop-time distributions are
identical.  Maximum per-case differences are 0.930 mrad joint displacement,
0.268 mm palm-model displacement, 0.000658 rad/s velocity, 1.242 rad/s²
acceleration, and 14.229 rad/s³ jerk.  The result JSON records explicit
tolerances and labels this as envelope conformance, not tick-identical Ruckig
equivalence.

Candidate C therefore remains the preferred *architecture reference* for
active full-IK shaping, but its normal tracking law does not define clutch
stop.  ABI v1 and the C++ reference now encode a separate explicit jerk-limited
zero-velocity/zero-acceleration mode, continuous state handoff, strict and
practical completion telemetry, stop distance, direction consistency, and
hard-stop preemption.  This still is not a production shaper.  The complete artifact is
[`results/controlled_stop_policy_sweep.json`](teleop_rearchitecture/results/controlled_stop_policy_sweep.json).

## Recoverable pause, residual acceleration, and fake lifecycle

Normal clutch release is now explicitly different from a fault. The
robot-independent engagement coordinator moves `ActiveTracking` or
`HoldRejected` into `ControlledBraking`, freezes the last accepted source
sequence, and emits no new active target until the shaper reports stopped.
Input remains a depth-one latest observation, so controller motion while
released is neither queued nor replayed. At `StoppedReady`, re-engagement
captures current measured q/dq/ddq and current controller pose, increments the
safety epoch, clears old target/feed-forward/filter/relative/rejection/brake
history, then emits an identity relative pose. Re-engagement while still
braking returns `WAIT_FOR_STOPPED`. A hard stop requires an explicit valid
measured-state reset; a clutch press alone cannot recover it.

Tests cover stationary, 0.75 m translated, and 90° rotated input movement
while paused; release from `HoldRejected`; repeated release and engagement;
long stopped dwell; residual measured velocity; invalid measured state;
delayed old-epoch targets; and 100 latest-slot replacements. With zero
measured dq/ddq, the first re-engaged C++ tick has exactly zero joint delta,
zero `rh56_R_hand_base_link` palm-model displacement, velocity, and
acceleration. This is model/command continuity evidence, not physical
continuity. The checked recovery artifact also records capture-to-first-output
as tick 0, zero first-tick jerk, one old-epoch rejection, mailbox depth one,
and zero queued release motion:
[`results/reengagement_continuity.json`](teleop_rearchitecture/results/reengagement_continuity.json).

The C++ stop planner also covers the previously unplannable
`|dq| approximately 0, |ddq| > 0` boundary. If the legacy common-duration
equation has no solution, it explicitly neutralizes acceleration at bounded
jerk, validates velocity and position excursion, then brakes residual velocity
independently per axis. The 115-case sweep includes 99 cross-products of
`dq={0, ±1e-6, ±1e-4, ±1e-3, ±1e-2}` rad/s and
`ddq={0, ±0.1, ±0.5, ±1, ±4, ±12}` rad/s², plus mixed axes, shoulder/wrist,
three jerk limits, 8 ms phase boundaries, and joint-limit cases.

| residual-acceleration result | value |
| --- | ---: |
| completed legal cases | 113 / 113 |
| expected outward position-limit failures | 2 / 2 |
| unexpected planning failures | 0 |
| direction-consistent completed cases | 113 / 113 |
| maximum stop time | 584 ms |
| maximum velocity excursion | 1.4513 rad/s |
| maximum joint displacement | 0.4854 rad |
| maximum palm-model displacement | 283.0 mm |
| observed peak v / a / jerk | 1.4584 / 12.0 / 100.0 SI |

The large worst-case displacement is the policy result: acceleration
neutralization prevents a spurious hard fault, but high residual acceleration
requires a wide stopping envelope. Upstream admission must avoid creating such
release states; the planner cannot promise a small stop merely because
instantaneous velocity is near zero. The complete artifact is
[`results/residual_acceleration_stop_sweep.json`](teleop_rearchitecture/results/residual_acceleration_stop_sweep.json).

`FakeJakaLifecycleAdapter` is a fully SDK-free lifecycle-shaped test double,
not a JAKA adapter. It owns one abstract session, validates only already-shaped
ABI commands and normalized health, enforces epoch/sequence/freshness/deadline,
classifies abstract send results, latches hard faults, and checks cleanup
ordering. It does no IK, collision/singularity, mapping, filtering,
interpolation, shaping, braking, Quest logic, JSON, or file I/O. Its states are
`Disconnected`, `Connecting`, `Connected`, `ServoReady`, `Streaming`,
`ControlledStopping`, `Stopped`, `Faulted`, and `CleaningUp`. A stopped session
may re-arm only with a newer epoch and valid measured state.

The follow-on read-only transport audit found that the current real native
worker cannot do this recovery: every stop exits its loop and the common path
disables servo, disables EDG, logs out, and terminates. SDK 2.2.7 exposes EDG
q+dq but no ddq; the worker currently discards dq. The header does not prove
whether a stopped session can remain command-free or must repeat q, nor whether
EDG must be reinitialized. The resulting default-disabled recovery contract,
measured-state hierarchy, fake SDK seam, and pre-physical gates are documented
in [`jaka_clutch_recovery_transport_contract.md`](jaka_clutch_recovery_transport_contract.md).

An SDK-free `ThinJakaTransportAdapter` now makes that proposed boundary
executable through a fixed fake function table. It implements the retained-
session stopped/measurement-refresh/new-epoch path, both explicit pause
policies, optional explicit EDG/servo restart, q/dq preservation, latest-only
consumption, and terminal cleanup/reset. Its 1,000-cycle fake run covers 7,000
exact 8 ms ticks with zero observed allocations or deadline misses. This is
still a skeleton: there is no vendor translation unit and no controller call.

| injected condition | fake classification | recovery |
| --- | --- | --- |
| clutch release / stopped command | controlled lifecycle | newer epoch + measured state |
| rejected feasible target | upstream `HoldRejected` | next accepted target or release |
| duplicate/old output or health sequence | invalid-command hard fault | cleanup + reconnect/re-arm |
| stale command or producer disappearance | stale-input hard fault | cleanup + reconnect/re-arm |
| missed deadline | timing-fault hard stop | cleanup + reconnect/re-arm |
| epoch mismatch / old epoch | epoch-mismatch hard stop | cleanup + newer epoch |
| non-finite/invalid ABI or measurement | invalid-command hard stop | cleanup + valid reinitialize |
| normalized controller alarm / estop / collision | controller hard stop | cleanup; external cause unresolved |
| abstract transport/send failure | SDK-failure category hard stop | cleanup + reconnect |

The C++ matrix injects 21 terminal cases. A skipped output sequence is accepted
as latest-wins and counted rather than queued. The fixed 256-record telemetry
ring wraps without allocation or blocking; the 300-command test observed 52
overwrites, and a separate terminal record survives wrap and cleanup. Records
include output/source sequence, epoch, mode, age, deadline slack, validation
result, lifecycle result, and cleanup event.

The executable no-SDK manifest is `tests/no_sdk_test_manifest.json`. It names
36 allowed Python files and explicitly forbids
`tests/test_native_jaka_servo_worker.py`, whose historical ELF dependency
would load `libjakaAPI.so`. The runner checks every listed research ELF file
with `readelf` and `nm`, runs CTest and pytest in one process, then audits
`/proc/self/maps`.
The current 340-test run loaded no JAKA SDK image and found no JAKA/ServoJ/EDG
dependency or symbol.

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
runs MuJoCo 3.10 headlessly.  Ruckig 0.19.4 was installed only in this
worktree's `.venv` from the existing ARM64 wheel cache and is declared under
the `teleop-research` optional dependency; no global or system package changed.
Official JAKA ROS 2 documents Ubuntu 22.04,
Humble and x86_64 and packages MiniCobo rather than Mini2; installation would
be a future manual/container feasibility gate, not a `sudo` action here.

## Migration and physical gates

1. Freeze production behavior and keep the present safety/replay suite as an
   oracle.  Add raw, privacy-reviewed HTS and native telemetry fixtures only
   if their provenance and retention permit it.
2. Use ABI v1 as the research post-IK contract and retain its measured-state
   restart, epoch, target-validity/liveness split, freshness, and fail-closed
   tests.  It remains isolated from production.
3. Choose a production OTG only after CPU, jitter, target replacement, stop,
   output-limit, collision/singularity, and replay benchmarks on Thor.  Do not
   merge prototype code into production yet.
4. Retain the completed **fake-only lifecycle adapter** and executable no-SDK
   manifest as boundary tests. Next define a process/IPC design and a separate
   thin fake transport only after scheduler/deadline requirements are agreed;
   do not add a real SDK transport yet. Keep ROS/MoveIt separate.
5. Before any separately authorized physical gate: verify model/TCP/payload
   state without writes, controller alarms/workspace/stop access, exact
   `q_hold` continuity, 8 ms deadline under load, stale input, clutch release,
   `HOLD_REJECTED` recovery, controller alarm/SDK/timing fault cleanup, no
   backlog, RH56=0, and evidence logging.  The unresolved J4 collision and
   unvalidated post-fix acceleration gate remain blockers to expansion.

The versioned ABI, independent in-process C++ shaping core, conformance bridge,
recoverable engagement coordinator, robot-independent in-memory consumer, and
SDK-free lifecycle-shaped fake are complete offline. No independent shaping
process, network IPC, real transport, or real JAKA adapter is implemented.
Those remain separately scoped work.

## Validation performed

Offline only:

- `tests/test_teleop_rearchitecture.py`: mailbox no-backlog, non-active target
  exclusion, two shapers' v/a/j bounds, causal intended-target selection,
  timestamp-based feed-forward, pending-feed-forward cancellation, metric
  separation, and moving clutch controlled-stop continuity.
- Both prototype replays on the two tracked AcceptedArmTarget streams, with
  MuJoCo FK and checked-in JSON results.
- The prior 15 focused tests remain, and nine unified-evaluator/stop-sweep
  tests cover causal final-target activation, schema finiteness, generated
  documentation rows, frame/input contracts, the release-state matrix, all
  stop bounds, and explicit-braking improvement.
- The expanded applicable Quest/JAKA, PWL/output-feasibility,
  singularity/liveness, native-worker, replay, unified-evaluator, and stop set
  passed: **175 passed**.  No test invokes a physical backend.
- `compileall`, all four legacy MuJoCo headless replays, both unified fixtures,
  the 180 policy/state stop cases, JSON validation, and `git diff --check`
  passed in the preceding evaluator round.
- ABI/C++ validation adds compile-time and Python runtime layout checks, pure
  validator cases, deterministic randomized enum/version/DOF/nonfinite tests,
  active Python/C++ tick conformance, explicit-stop envelope conformance,
  fake-consumer fault injection, hard-stop preemption, and a 100,000-tick
  stability run with zero observed dynamic allocations.
- Final ABI-round results are: Release CTest 1/1; focused Python 25 passed;
  ABI/conformance Python 12 passed; and the expanded no-SDK applicable subset
  163 passed.  GCC 13.3 built Release and ASan+UBSan variants; the sanitizer
  binary completed the same 100,000-tick test without a diagnostic.  The
  checked stop JSON rebuilt byte-for-byte, while unified JSON rebuilt exactly
  after excluding measured CPU and its generated Markdown rebuilt byte-for-byte.
- The earlier 175-test baseline included the pre-existing native fake-worker
  test executable, whose ELF has `libjakaAPI.so` as a required shared object.
  It made no hardware connection or SDK command, but the dynamic loader loads
  that library when the executable starts.  It was therefore not rerun in the final no-SDK subset;
  the new ABI/C++ library itself has no JAKA symbol or dependency.
- The recoverable-lifecycle round adds the 115-case residual sweep, engagement
  continuity tests, 21-case fake lifecycle fault matrix, latest-wins/epoch
  recovery, ring wrap/terminal retention, and the explicit no-SDK manifest.
  The current manifest run is CTest 1/1 plus 340 Python tests. Both native ELF
  files list only standard C/C++ runtime dependencies, expose no forbidden
  symbols, and `/proc/self/maps` contains no JAKA library after the suite.

Rebuild commands:

```bash
cmake -S native/teleop_shaping -B build/teleop_shaping \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/teleop_shaping -j2
ctest --test-dir build/teleop_shaping --output-on-failure
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/run_teleop_unified_benchmark.py \
  --output docs/research/teleop_rearchitecture/results/unified_benchmark.json \
  --markdown-output docs/research/teleop_rearchitecture/results/unified_benchmark_table.md \
  --cpp-library build/teleop_shaping/libteleop_shaping_c_api.so
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/run_teleop_stop_sweep.py \
  --output docs/research/teleop_rearchitecture/results/controlled_stop_policy_sweep.json \
  --cpp-library build/teleop_shaping/libteleop_shaping_c_api.so
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/run_residual_acceleration_sweep.py
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/run_reengagement_evidence.py
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/run_no_sdk_test_manifest.py
```

No real hardware was used, no actuator setting was read or written, and this
document does not change the existing physical authorization boundary.
