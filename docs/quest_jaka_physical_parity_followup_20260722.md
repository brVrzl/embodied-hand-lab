# Quest-to-JAKA parity follow-up after `14b2909`

Date: 2026-07-22

Scope: offline audit and synchronization only. No JAKA connection, EDG entry,
servo enable, or physical command was performed.

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
| repeat latest accepted tuple at 8 ms | no | no | yes | transport requirement only; values are unchanged |
| finite/CRC/sequence/manufacturer-limit rejection, stale stop, tracking-error abort | no | no | yes | retained pass-through fault containment; no scaling or smoothing |
| old TeleDex workspace/slew/low-pass/tracking-error shaping | no | no | no | not imported or called by either Quest entry |

The 75 degree relative-rotation and 0.30 m operator values in the precision
mapper are warning thresholds, not clippers. The 0.20 m target envelope and
the feasibility jump/velocity limits are part of the current successful shared
simulation acceptance policy and therefore remain shared; no additional
physical workspace box, speed limiter, acceleration limiter, jerk limiter,
interpolator, or low-pass stage exists before the JAKA adapter.

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
| JAKA transport | separate native process, 8 ms / 125 Hz repeat-latest | Unix datagram, finite kernel buffer, never waits for Python/MuJoCo |
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
