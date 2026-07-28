# Quest 3 to JAKA Mini2 physical parity audit

> **Status: historical snapshot, 2026-07-22.** This audit records the
> repository and gate state at that time. It is not the current operating
> guide. See
> [`docs/architecture/simulation_hardware_parity.md`](../../../architecture/simulation_hardware_parity.md).

Date: 2026-07-22
Scope: right-wrist JAKA arm control only; no Inspire RH56 command path
Physical gate reached: P0 only

## Repository and entry points

- Worktree: `/home/thor/projects/embodied_lab`
- Branch at audit start: `feature/jaka-teledex-control-foundation`
- HEAD at audit start: `ac7399be8951560bf154273974fe85c8927aabc9`
- Successful Quest/MuJoCo reference found in sibling branch
  `feature/quest-controller-transport-host` at `530d3a0`; its five commits were
  merged into this branch before the parity refactor.
- Successful simulation entry point:
  `tools/quest_jaka_mujoco_sim.py live-6dof` (or deterministic
  `replay-6dof`).
- Pre-change user-facing physical entry point:
  `scripts/run_real_jaka_teledex_arm_teleop.sh` ->
  `tools/run_real_jaka_teledex_arm_teleop.py` (legacy ROS Cartesian jog).
  The newer bounded foundation entry point was
  `tools/teleoperation/run_teledex_jaka_session.py` and native
  `jaka_servo_worker --mode bounded-teleop`. It consumed TeleDex Cartesian
  targets, not Quest right-wrist data, and was therefore not a physical adapter
  for the successful Quest simulation.
- Post-change physical entry point: `tools/quest_jaka_hardware.py`, with
  `p2-shadow` and `p4-live` stages. Both consume the same configuration and
  instantiate the same `SmoothQuestJakaSession`, mapping, filters, MuJoCo model,
  continuation IK, and candidate acceptance as the simulation entry point.

Concurrent user-owned edits to `README.md`, `tools/quest_jaka_mujoco_sim.py`,
`tools/teleop_mujoco_jaka_rh56.py`, `scripts/run_quest_jaka_sim_demo.sh`, and
the then-local simulation guide were present during the audit and are
intentionally excluded from the checkpoint. The current guide is
[`docs/operation/simulation_demo.md`](../../../operation/simulation_demo.md).

## Pre-change architecture and one-sample trace

The successful simulation path was:

```text
HTS/CTRL UDP
-> hts_protocol validation
-> hts_transport receipt timestamp
-> HtsCanonicalAssembler (Unity left-handed -> canonical right-handed)
-> 20 ms bounded pose interpolation
-> left-index clutch state machine
-> reference hand + simulated mounted-palm TCP + head-yaw capture
-> quaternion-safe One Euro wrist filter
-> inv(T_hand_ref) @ T_hand_current
-> latched head-horizontal change of basis
-> committed translation/orientation bases at 1:1 gain
-> mounted-palm TCP target
-> continuation MuJoCo IK seeded by the previous accepted solution
-> feasibility/branch/joint/collision checks
-> accepted J1..J6 target
-> MuJoCo position-actuator plant
```

The available physical bounded TeleDex path was:

```text
TeleDex websocket
-> independent TeleDex validator/filter/clutch/mapping
-> 0.05 Cartesian scale and bounded Cartesian shaper
-> relative Cartesian packet
-> native startup-TCP composition
-> JAKA SDK kine_inverse (second independent IK)
-> native soft envelope and branch checks
-> native 0.03 rad/s jerk-bounded tracker
-> JAKA edg_servo_j
```

Thus no valid Quest sample could traverse the old physical path, and the old
physical command differed after receipt, mapping, scaling, filtering, IK, and
joint command generation.

The older user-facing ROS path was independently divergent as well: its
`configs/teleop/teledex_jaka_arm.yaml` used 30 Hz input, 0.30 translation
scale, disabled orientation (with a dormant 0.50 rotation scale), a fixed
workspace box, a 45 ms target low-pass, 0.060 m/s and 0.40 m/s2 translation
limits, per-update position/rotation steps, target lead/jump clamps, tracking
error time scaling/pause logic, and a separate ROS bridge command path.

## Post-change authoritative boundary

`SmoothQuestJakaSession.control_tick()` now returns an immutable
`AcceptedArmTarget`. `JakaMujocoSimulation.evaluate()` remains the single IK and
acceptance implementation. Output begins only after acceptance:

```text
validated Quest wrist
-> shared reference/relative transform
-> shared mapped and filtered TCP target
-> shared continuation IK and acceptance
-> immutable AcceptedArmTarget (TCP + J1..J6 radians)
   |-> MujocoArmTargetAdapter
   `-> JakaAcceptedJointTargetAdapter
```

The JAKA adapter only packs the identical J1..J6 radian tuple. The native Quest
worker accepts only `JOINT_POSITION`, frame `NONE`, J1 through J6 in radians,
and sends it through `edg_servo_j(..., ABS, 1)`. It performs no Cartesian
conversion, scaling, filtering, IK, interpolation, trajectory shaping, or
unit conversion.

## Module responsibility audit

| Responsibility | Authoritative module |
|---|---|
| Quest hand/head packet receipt | `motion_input.hts_transport.HtsUdpReceiver` |
| Quest controller receipt | `motion_input.controller_provider.ControllerProvider` via `LiveQuestControllerRouter` |
| Packet syntax/finiteness | `motion_input.hts_protocol`, `controller_protocol` |
| Validity and age | `HtsCanonicalAssembler`, `ControllerProvider`, `ArmClutchMachine` |
| Canonical coordinate conversion | `motion_input.hts_canonical` |
| Input interpolation | `quest_jaka_sim.se3.PoseSampleBuffer` |
| Reference and clutch | `motion_input.clutch.ArmClutchMachine`, `SmoothQuestJakaSession._arm_target` |
| Head-yaw compensation | `precision_mapping.gravity_aligned_head_yaw` and `LatchedHeadYawArmMapper` |
| Relative translation/orientation | `quest_jaka_sim.se3.relative_pose` |
| Quaternion sign/shortest arc | `se3.align_quaternion_sign`, quaternion One Euro filter, SLERP |
| Position/orientation filter | `LatchedHeadYawArmMapper` One Euro filters |
| Target transform | `LatchedHeadYawArmMapper.target` |
| IK and accepted-solution continuation | `JakaMujocoSimulation.evaluate`, `PalmTargetIkState` |
| Rejected hold-last | `JakaMujocoSimulation.last_safe_*`, session rejection counter |
| MuJoCo output | `MujocoArmTargetAdapter`, then the MuJoCo actuator plant |
| JAKA output | `JakaAcceptedJointTargetAdapter`, native joint mode, `edg_servo_j` |
| Runtime safety state | shared arm clutch plus native lifecycle/timeout/fault cleanup |

## Parameter and processing-stage comparison

The shared authority is
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`; the physical launcher has no
second mapping file.

| Item | Successful Quest simulation / new shared value | Old physical TeleDex value or behavior | New physical behavior |
|---|---:|---:|---|
| Translation gain | `[1, 1, 1]` | ROS `0.30`; bounded foundation `0.05` | shared `[1, 1, 1]` |
| Rotation gain | scalar/per-axis `1.0` | ROS disabled with dormant `0.50`; bounded foundation `0.05` | shared `1.0` |
| Translation basis | rows `[-X, +Z, +Y]` | separate TeleDex calibration | shared exact basis |
| Orientation basis | `diag(-1,-1,+1)` conjugation | separate TeleDex mapping | shared exact conjugation |
| Transform order | local `inv(T_ref) @ T_current`; spatial translation; body-relative rotation; compose on captured TCP | startup-relative native composition | shared exact order |
| Head compensation | gravity-aligned yaw latched at capture; later head motion ignored | none in bounded TeleDex | shared exact behavior |
| Translation deadband | 1 mm | ROS 3 mm input plus 0.4/1.0 mm output hysteresis; bounded independent filter | shared 1 mm |
| Orientation deadband | 2 deg | ROS 1 deg input; bounded shaper | shared 2 deg |
| Operator displacement envelope | 0.30 m | old first-test workspace | shared 0.30 m rejection only |
| Target displacement envelope | 0.20 m | 0.015 m half extent | shared 0.20 m rejection only |
| Relative orientation envelope | 75 deg | 4 deg | shared 75 deg |
| Input freshness | 250 ms wrist/head | ROS phone 200 ms and feedback 300 ms; bounded 40/100/500/2000 ms age ladder | shared 250 ms source validity; separate 100 ms command-stream stop |
| Controller freshness | 150 ms | TeleDex Button A packet validity | shared 150 ms |
| Input interpolation | 20 ms, buffer 16 | none in native worker | shared 20 ms; none added in adapter |
| Position filter | One Euro `min=1.2`, `beta=18`, derivative cutoff 1 Hz, max dt 50 ms | ROS 45 ms target low-pass; bounded One Euro `min=2`, `beta=30`, derivative 1 Hz, max dt 100 ms | shared simulation profile |
| Orientation filter | One Euro `min=1.5`, `beta=4`, derivative cutoff 1 Hz, max dt 50 ms | ROS orientation disabled; bounded One Euro `min=2`, `beta=3`, derivative 1 Hz | shared simulation profile |
| Target/IK generation | 60 Hz | ROS 30 Hz plus bridge; bounded source-driven Python plus 125 Hz native IK | shared 60 Hz |
| JAKA transport | n/a | 125 Hz | 125 Hz repeat-latest, no interpolation |
| MuJoCo plant | 500 Hz | n/a | unchanged simulation-only plant |
| IK seed | previous accepted J1..J6 | native previous SDK IK solution/observation | shared previous accepted J1..J6 |
| IK iterations/gain/damping/max step | 24 / 0.70 / 0.05 / 0.04 rad | JAKA SDK inverse IK | shared exact values |
| IK tolerances | 2.5 mm / 3 deg | native 1 mm / 2 deg | shared 2.5 mm / 3 deg |
| Jacobian thresholds | condition 60, minimum singular 0.0125, 0.25 m rotational row scale | native condition up to 200 | shared exact values |
| Wrist singularity guard | J5 bend at least 15 deg | absent/SDK dependent | shared exact guard |
| Accepted target jump | TCP 0.04 m, orientation 8 deg, joint 0.22 rad | native IK step 0.10 rad | shared exact checks |
| IK candidate continuity thresholds | 14 rad/s, 1000 rad/s2 | native 0.03/0.15/1.5 command tracker | shared checks; old tracker bypassed |
| Shared command-reference limits | pi rad/s, 4pi rad/s2, 20pi rad/s3, 10 rad/s tracking frequency in MuJoCo plant | independent 0.03/0.15/1.5 tracker | simulation plant only; no physical duplicate |
| Rejection policy | hold last for 30 isolated rejections, then clutch fault | bounded worker independently rejected | shared exact policy; adapter receives nothing on reject |
| Joint order/unit | `jaka_joint_1..6`, radians | implicit native array/radians | explicit and contract-tested |
| Command mode | MuJoCo position target | `edg_servo_j ABS` after native shaping | `edg_servo_j ABS`, identical accepted radians |
| Tool/TCP | mounted `rh56_R_hand_base_link` FK in committed MJCF | controller TCP converted to startup-relative pose | shared mounted-palm FK; no vendor Cartesian conversion |
| Base | model JAKA base / committed operator basis | controller user frame 0 | joint output needs no Cartesian base conversion; P1 verifies user 0 |

### Complete limiter/shaper inventory

| Stage | Shared | Simulation-only | Hardware-only after change |
|---|:---:|:---:|:---:|
| 20 ms pose interpolation (no prediction) | yes |  |  |
| One Euro position/orientation filters | yes |  |  |
| Translation/orientation deadbands | yes |  |  |
| Operator/target pose envelopes | yes, rejection only |  |  |
| IK residual, branch, singularity, jump, candidate rate, joint-margin, collision checks | yes, rejection only |  |  |
| 500 Hz jerk-limited MuJoCo arm actuator reference |  | yes, part of simulated plant and after the accepted-target boundary |  |
| MuJoCo position servo gains/dynamics |  | yes |  |
| RH56 simulated hand slew |  | yes, out of arm scope |  |
| Old bounded Cartesian shaper (8 mm/s, 4 deg/s, acceleration/jerk limits) |  |  | bypassed by Quest entry point |
| Old native 0.03 rad/s, 0.15 rad/s2, 1.5 rad/s3 joint tracker |  |  | bypassed by native Quest joint mode |
| Native Cartesian envelope and second IK |  |  | bypassed by native Quest joint mode |
| Old ROS workspace box `[-.14,-.46,.16]` to `[.06,-.24,.46]` m |  |  | bypassed by Quest entry point |
| Old ROS 8 mm/1.5 deg per-update limits and 60 mm lead clamp |  |  | bypassed by Quest entry point |
| Old ROS 60 mm/s, 0.40 m/s2, 35 mm jump, 45 ms low-pass |  |  | bypassed by Quest entry point |
| Old ROS tracking-error time scaling (`min=0.25`) and pause thresholds |  |  | bypassed by Quest entry point |
| Repeat-latest JAKA transport at 8 ms |  |  | representation/transport only; repeats the exact tuple |
| Manufacturer joint-position check |  |  | retained hard rejection |
| Fixed excessive measured tracking-error abort |  |  | retained fault only; no scaling |

The old bounded TeleDex implementation remains available for its historical
workflow, but it is not called by the Quest physical entry point.
The YAML still contains an unselected `hardware_conservative` One Euro profile
for historical comparison; neither physical stage selects it. Both outputs use
the selected `simulation_exploration` profile.

## Retained hard protections

1. JAKA joint-position limits: every native Quest target is finite and checked
   against the six configured JAKA hard ranges. This only rejects an invalid
   target; it never clips or rescales one.
2. No startup motion: connection and state verification do not enter EDG. The
   first motion packet must equal measured startup J1..J6 within 0.001 rad.
3. E-stop/servo disable: P4 requires an E-stop confirmation; all exits call the
   worker cleanup that disables servo mode and exits EDG.
4. Command-stream timeout: no new accepted packet for 100 ms stops the joint
   stream and cleans up. Healthy 60 Hz publication remains far inside it.
5. Invalid command rejection: packet size, CRC, monotonic timestamps, sequence,
   finite payload, kind, frame, flags, and hard joint limits are checked.
6. Quest/controller loss: the shared clutch faults on stale/invalid wrist,
   shared hand tracking loss, stale controller, malformed input, or release.
   No new target reaches the adapter.
7. IK failure: the shared feasibility result is rejected and the last accepted
   solution remains the continuation seed; the adapter is not called.
8. Tracking-error abort: two consecutive cycles above 0.35 rad abort. It does
   not change commands below the threshold.
9. Communication/timing failures: SDK read/write errors, transport errors, or
   persistent hard 8 ms loop misses fault and clean up.
10. Operator stop: arm-clutch release and Ctrl+C send STOP and cease updates.
    P4 has no countdown or automatic transition from shadow.

## Offline verification and P0

Focused tests cover exact pre-adapter parity, independent translation axes,
all three rotation axes and combined rotation (including downward wrist pitch),
reference/clutch and head-yaw behavior, quaternion continuity, stale/tracking/
NaN/IK/communication rejection, wire ordering/units/mode, no pre-engagement
command, no post-stop update, and the native no-IK/no-shaping joint contract.

P0 PASS:

```text
.venv/bin/python -m pytest tests/test_quest_jaka_shared_pipeline.py tests/test_native_jaka_servo_worker.py -q
28 passed in 5.17s

.venv/bin/python -m compileall -q src tools tests
git diff --check
.venv/bin/python -m pytest -q
547 passed, 1 skipped in 59.86s
```

The skipped test is an existing environment-conditional test; it is not a
parity or JAKA adapter test. No JAKA network connection, EDG entry, or physical
command occurred during P0.

## Prepared physical gates (do not run without operator authorization)

All commands assume `cd /home/thor/projects/embodied_lab` and a freshly built
`build/jaka_servo_worker/jaka_servo_worker`.

P1, connected read-only:

```bash
.venv/bin/python tools/teleoperation/run_jaka_hardware_probe.py state-read \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-s 30 --expected-tool-id 0 --expected-user-frame-id 0 \
  --acknowledgement I_ACKNOWLEDGE_JAKA_HARDWARE_RISK \
  --metrics-file logs/quest_jaka_p1_state_read.json
```

P2, connected command shadow (no EDG and no command API):

```bash
.venv/bin/python tools/quest_jaka_hardware.py p2-shadow \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-sec 60 --approval I_AUTHORIZE_P2_QUEST_JAKA_COMMAND_SHADOW \
  --log logs/quest_jaka_p2_shadow.jsonl \
  --summary logs/quest_jaka_p2_summary.json \
  --metrics logs/quest_jaka_p2_worker.json \
  --capture logs/quest_jaka_p2_quest_capture.jsonl
```

P3, zero-motion servo validation:

```bash
.venv/bin/python tools/teleoperation/run_jaka_hardware_probe.py zero-motion \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-s 10 --expected-tool-id 0 --expected-user-frame-id 0 \
  --acknowledgement I_ACKNOWLEDGE_JAKA_HARDWARE_RISK \
  --metrics-file logs/quest_jaka_p3_zero_motion.json
```

P4 is prepared and remains operator-gated. The 2026-07-22 P2 direction shadow
and physical-seed twin review confirmed the relative translation and rotation
directions, so `hardware_adapter.physical_mapping_confirmed` is true. Run only
with both exact operator authorization lines and the现场 safety confirmations:

```bash
.venv/bin/python tools/quest_jaka_hardware.py p4-live \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --duration-sec 60 --approval I_AUTHORIZE_P4_LIVE_QUEST_JAKA_TELEOPERATION \
  --estop-accessible --workspace-clear --rh56-command-path-absent \
  --log logs/quest_jaka_p4_live.jsonl \
  --summary logs/quest_jaka_p4_summary.json \
  --metrics logs/quest_jaka_p4_worker.json \
  --capture logs/quest_jaka_p4_quest_capture.jsonl
```

## Physical-only uncertainties

- The shared mapping directions are confirmed, but the fixed model/installation
  frame discrepancy (about 7.63 mm position and 180 degrees orientation when
  comparing the SDK TCP with shared-model FK at the same measured joints) still
  requires later physical kinematic validation; no arbitrary offset is applied.
- The controller exposes tool/user IDs but did not expose a model identifier in
  prior read-only work. P1 must reconfirm IDs, firmware/SDK observation, joint
  order, units, and current pose.
- Actual live tracking lag and the suitability of the clearly-excessive 0.35
  rad abort can only be observed in P3/P4. The threshold is fault containment,
  not a normal motion-profile parameter.
- JAKA EDG requires its validated 8 ms transport period; P3 reconfirms timing
  and zero-motion tracking before any Quest-following command is authorized.
