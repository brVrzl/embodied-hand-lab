# JAKA–TeleDex bounded live-arm teleoperation handoff

Date: 2026-07-16  
Repository: `git@github.com:brVrzl/embodied-hand-lab.git`  
Worktree: `/home/thor/projects/embodied_lab`  
Branch: `feature/jaka-teledex-control-foundation`

## Checkpoint commits and repository state

The clean-slate foundation is checkpointed in three commits based on
`8ecd70b` (`feature/quest-motion-input-platform` at the point the dedicated
branch was created):

1. `bc15d55` — `feat: establish arm-only JAKA control foundation`
2. `8b02a44` — `docs: record JAKA Gates 1 through 3C evidence`
3. `dcadc17` — `test: tolerate additional host timing warnings`

This handoff is committed separately after those content checkpoints; its
commit hash is reported by the closing session response because a commit cannot
embed its own hash.

Foundation-owned changes are fully committed. The shared worktree is
intentionally **not clean** because other concurrent sessions own the following
unstaged tracked modifications:

- `Agents.md`, `README.md`, `docs/README.md`,
  `docs/project_rebuild_status.md`, and `pyproject.toml`;
- `configs/camera/realsense_thor.yaml`;
- `src/data_recorder/cli.py`, `src/embodiment_core/types.py`,
  `src/jaka_driver_adapter/servo_jog.py`;
- `src/teleop_tools/README.md` and
  `src/teleop_tools/relative_pose_lag_follow.py`;
- `src/vision_interface/{README.md,__init__.py,interfaces.py,mock_camera.py,realsense_adapter.py}`;
- `tests/{test_episode_recorder.py,test_jaka_servo_jog.py,test_realsense_adapter.py,test_relative_pose_lag_follow.py}`;
- `tools/{check_realsense_stream.py,iphone_mediapipe_hand_teleop.py,serve_realsense_viewer.py}`.

Untracked, excluded work includes the three root MOV files, `artifacts/`,
`digital_twin/`, `models/`, `tmp/`, camera/perception configuration and docs,
digital-twin docs/tools/tests, Quest/UMIP and prototype TeleDex scripts/configs,
legacy TeleDex calibration/shadow files, and camera/depth files. In particular,
the following are not part of this checkpoint:

- `configs/teleop/teledex_jaka_arm*.{yaml,json}`;
- `docs/teledex_jaka_arm_teleop.md`;
- `scripts/{calibrate_teledex_jaka_frame.sh,check_teledex_phone.sh,check_xbox_deadman.sh,run_real_jaka_teledex_arm_teleop.sh,run_teledex_rviz_shadow.sh}`;
- `src/teleop_tools/{pose_teleop_config.py,teledex_calibration.py,teledex_phone.py,teledex_rviz_shadow.py}`;
- `tools/{calibrate_teledex_jaka_frame.py,check_teledex_phone.py,check_xbox_deadman.py,run_real_jaka_teledex_arm_teleop.py,run_teledex_rviz_shadow.py}`.

Do not clean, reset, stage, or overwrite those paths. Do not use broad staging.

## What the completed gates proved

### Gate 1 — audit and legacy isolation

- The repository, JAKA/RH56 wrappers, legacy HEBI path, timing assumptions,
  hardware interfaces, and real-time risks were audited.
- The old HEBI phone/follower pipeline is explicitly classified as prototype
  legacy. It was neither deleted nor adopted by the clean design.
- Automated isolation tests prove that the arm-only runtime imports without
  HEBI and contains no RH56 dependency.
- Device-neutral, typed, serializable 6-DoF arm contracts define sequence,
  frames, health, safety, lifecycle, acknowledgement, robot state, timing, and
  source/local pipeline timestamps.

### Gate 2 — ownership and deterministic foundation

- A dedicated native process is the single owner of one JAKA SDK client and
  the EDG lifecycle. Python is supervisory and never executes in the 8 ms loop.
- The Python/native wire protocol uses fixed-size, versioned, CRC-protected
  Unix datagrams. Sockets and kernel buffers are bounded and non-blocking;
  overflow drops instead of growing a FIFO. The worker drains available
  datagrams and retains only the newest valid target, preventing stale backlog
  replay.
- Monotonic local receive, processing, dispatch, command, and observation
  timestamps are distinct from optional source-clock capture time.
- Startup is non-moving. Missing targets hold; invalid ages abort; configured
  thresholds are warning 40 ms, hold 100 ms, controlled stop 500 ms, and fatal
  communication timeout 2 s.
- Lifecycle, cleanup, signals, sequence rejection, stale targets, transport
  bounds, fake failures, repeated initialization, legacy isolation, RH56
  exclusion, and timing statistics have automated tests.
- Filtering, target shaping, safety limiting, trajectory generation, and servo
  execution remain separate interfaces. No full One Euro/Ruckig pipeline or
  live Cartesian control was enabled.
- IK placement was reviewed, not frozen: Cartesian SDK control, Python IK, and
  native/SDK IK each remain candidates. Branch continuity, singularities,
  joint-limit awareness, state freshness, and deterministic timing must be
  proven before selection.

### Gate 3A — physical read-only SDK validation

- JAKA mini2 login, state reads, frame IDs, cleanup, repeated sessions,
  unreachable-controller handling, and Ctrl-C cleanup were validated without
  command APIs.
- The physical SDK reported
  `libadd jakaAPI_version: V2.2.7stable_linux`; controller firmware and an
  SDK-reported model identifier were unavailable.
- At 10 Hz for 30 s, 300 cycles completed with zero failed reads/timeouts.
  Joint-position calls averaged 1.899 ms (p99 3.234 ms, max 6.032 ms), and TCP
  calls averaged 1.197 ms (p99 1.523 ms, max 3.878 ms).
- Slow combined-status calls averaged 30.823 ms and reached 58.066 ms in the
  archived summary; an earlier observation was approximately 66 ms. These
  calls must never enter the 8 ms command loop.
- The JAKA heartbeat consumed substantial CPU. Repeated logout left SDK-owned
  threads alive: baseline thread count was 1 and three sequential sessions
  left 4 threads. Logout is not process teardown.

### Gate 3B — physical EDG lifecycle and zero motion

- EDG entry/read/exit was validated first with no joint or Cartesian command.
- One-second and five-second invariant-current-joint loops then completed with
  paired servo-mode enable/disable and deterministic EDG cleanup.
- Each disposable run captured a fresh post-countdown joint vector and copied
  it exactly as that run's invariant target. Intentional command delta was
  exactly zero; cross-API encoder observations were recorded separately.
- The operator observed no visible motion, abnormal sound, vibration,
  collision indication, or alarm during the accepted five-second run.

### Gate 3C — predefined joint motion

- A +0.25-degree joint-6 outward/return probe validated the initial motion
  path, cleanup, trajectory generation, and recording.
- A separate +5-degree joint-6 outward/return probe then completed all 1,439
  commands using a seventh-order endpoint-stationary trajectory: 5 s outward,
  1 s hold, 5 s return, and 0.5 s settling.
- The operator observed the expected positive direction and approximately
  +5-degree amplitude. The RH56 assembly, wrist camera, adapter, connectors,
  and cables remained clear and untensioned. There was no abnormal sound,
  vibration, oscillation, collision indication, controller alarm, unexpected
  motion, or contact. The robot visibly returned to its starting pose.
- Gate 3C is accepted as successfully completed. It proves predefined
  joint-space control, not live input or general Cartesian safety.

## Physical hardware and validated lifecycle

- Robot: JAKA mini2
- Controller: `192.168.71.50`
- Local EDG-state address: `192.168.71.19`
- SDK: `V2.2.7stable_linux`
- Required tool/user IDs: `0/0`
- Units: joint positions in radians; device-neutral pose contracts use metres
  and quaternion `xyzw`; the vendor Cartesian API uses millimetres and RPY
  radians and therefore requires an explicit conversion boundary.

Validated command lifecycle:

```text
login
→ preflight
→ EDG initialization
→ servo-mode enable when required by the selected command API
→ 125 Hz bounded commands
→ cease commands
→ servo-mode disable
→ EDG exit
→ logout
→ disposable native process exit
```

The worker must continue to own both sides of every entered state. Never rely
on logout to disable servo mode or tear down heartbeat threads. One SDK client
and one disposable native process per physical run remain mandatory.

## Timing evidence

### Accepted five-second zero-motion run

- Requested period: 8 ms
- Commands: 625
- Period mean/median/p95/p99/max:
  8.000077/8.000003/8.006401/8.018465/8.473845 ms
- Command mean/p99/max: 0.035445/0.084362/0.550490 ms
- Wake mean/p99/max: 0.059122/0.074167/0.535689 ms
- Timing warnings, completion misses, hard misses: 0/0/0
- CPU migrations: 0; process CPU: 51.43%
- Intentional command delta: 0 rad
- SDK failures and cleanup errors: 0
- Post-cleanup thread count remained 2 versus baseline 1, reinforcing the
  disposable-process rule.

### Accepted +5-degree joint-6 run

- Commands: 1,439/1,439
- Observed positive displacement: 4.9972 degrees
- Peak raw command/observation difference: 0.1118 degrees
- Dynamic hard-threshold crossings: 0
- Maximum non-target observation: 0.00711 degrees
- Final return error: 0.00821 degrees
- Period mean/p99/max: 8.0000/8.0342/10.6821 ms
- Three isolated period warnings; zero completion or hard misses
- EDG state-read mean/p99/max: 0.023882/0.073402/0.360463 ms
- Joint-command mean/p99/max: 0.048746/0.134291/0.551333 ms
- Process CPU: 51.60%; CPU migrations: 2
- All SDK lifecycle calls returned zero; cleanup completed; no probe process
  remained.

An average near 8 ms is not, by itself, proof of hard real-time 125 Hz. Tail
latency, warnings, completion deadlines, and scheduling debt remain first-class
metrics.

## Current safety and timing semantics

- Tracking quality warning for the +5-degree validation: 0.2 degrees. It is
  recorded but is not a hard accuracy target.
- Expected lag allowance per cycle:
  `abs(commanded_joint_velocity) * 0.150 s`.
- Dynamic tracking hard boundary:
  `max(0.75 degrees, 2.5 * expected_lag)`, which ranged from 0.75 to
  0.8203125 degrees. One crossing is rechecked; two consecutive crossings
  abort. Rapidly increasing divergence and wrong-direction observation abort
  immediately.
- Joint-6 observation envelope for that test: fresh start minus 1 degree to
  fresh start plus 6 degrees. Non-target joints: fresh start ±0.1 degrees.
- Timing warning thresholds: wake lateness above 2 ms or start period above
  8.8 ms. Hard start/completion boundary: 12 ms. Two consecutive period or
  completion misses abort; accumulated debt of 8 ms aborts.
- Schedules use monotonic time, bounded/preallocated samples, and explicit
  re-alignment after an isolated warning so a delay does not create a command
  backlog.
- Same-cycle command minus observation is not called timestamp-aligned servo
  error unless timestamp evidence supports that interpretation.

These Gate 3C thresholds were probe-specific. The next implementation must
derive appropriate Cartesian, IK, and joint-space limits rather than copying a
single-joint envelope indiscriminately.

## Important paths

### Foundation source and configuration

- `configs/teleoperation/jaka_foundation.yaml`
- `src/teleoperation/contracts.py`
- `src/teleoperation/{sequence.py,state_machine.py,safety.py,timing.py,wire.py}`
- `src/teleoperation/motion_boundaries.py`
- `src/teleoperation/runtime/{arm_only.py,synthetic.py}`
- `src/teleoperation/jaka/{backend.py,fake_backend.py}`
- `native/jaka_servo_worker/`
- `native/jaka_readonly_diagnostic/`
- `native/jaka_zero_motion_probe/`
- `native/jaka_minimal_joint_probe/`
- `tools/teleoperation/`
- `src/teleop_tools/LEGACY.md`

### Focused tests

- `tests/test_teleoperation_{contracts,fake_jaka,isolation,state_and_safety,synthetic,timing,wire}.py`
- `tests/test_native_jaka_servo_worker.py`
- `tests/test_jaka_readonly_diagnostic.py`
- `tests/test_jaka_zero_motion_probe.py`
- `tests/test_jaka_minimal_joint_probe.py`

Final checkpoint validation: 99 focused tests passed in 41.62 s. The suite uses
fake backends for lifecycle/failure behavior and must not be described as
physical validation. The physical evidence is in the archived Gate reports.

### Reports and measurements

- `docs/jaka_teledex_teleoperation_foundation_audit_20260716.md`
- `docs/jaka_control_foundation_gates_1_2.md`
- `docs/jaka_control_foundation_gates_1_2_implementation_report_20260716.md`
- `docs/jaka_gate3a_readonly_validation_20260716.md`
- `docs/jaka_gate3a_physical_results_20260716.json`
- `docs/jaka_gate3b_stage3_state_preparation_review_20260716.md`
- `docs/jaka_gate3b_zero_motion_validation_20260716.md`
- `docs/gate3b_measurements/jaka_gate3b_stage5_timing_policy_retry_20260716.{json,csv}`
- `docs/jaka_gate3c_minimal_joint_validation_20260716.md`
- `docs/jaka_gate3c_5degree_joint6_plan_20260716.md`
- `docs/gate3c_measurements/jaka_gate3c_5deg_motion_20260716.json`
- `docs/gate3c_measurements/jaka_gate3c_5deg_trajectory_20260716.csv`

## Architectural boundaries

### Legacy HEBI

The old HEBI snapshot/follower, state machine, timing, mapping, filtering, and
trajectory logic are prototype history only. They must not be imported, copied,
incrementally refactored into, or used as the production TeleDex foundation.

### RH56

RH56 is absent from the active arm-only runtime. Do not instantiate its SDK,
mock it into arm composition, add 21-DoF active contracts, synchronize hand and
arm, or send hand commands. Hand control requires a later independent gate.

### Quest/UMIP

The Quest/Motion Input Platform exists on the separate
`feature/quest-motion-input-platform` history and has unrelated dirty worktree
changes. It is not an input, dependency, or fallback for the first TeleDex arm
test. Do not stage or alter its files from this branch.

## Known risks for the next session

1. TeleDex's source clock, capture timestamp, delivery behavior, packet loss,
   ordering, burst behavior, and reconnect semantics are not yet integrated.
2. TeleDex → world → robot base → tool → end-effector transforms require one
   centralized, calibrated transform chain with explicit handedness, axes, SI
   units, and quaternion convention.
3. Startup alignment must anchor the device pose to the freshly observed robot
   pose. An absolute first target can jump even when its data is valid.
4. Clutch and recenter must have explicit edge semantics. Motion must remain
   disabled until synchronization and an intentional clutch action complete.
5. Dropout, stale data, reconnect, sequence reset, and burst recovery must hold
   or controlled-stop without replaying old targets. Recovery must require
   re-clutch/recenter.
6. Cartesian IK placement is unresolved. State freshness, branch continuity,
   branch switching, singularities, unreachable targets, and joint-limit
   margins need measured safeguards.
7. Cartesian workspace, joint soft limits, predicted TCP displacement, and
   collision boundaries are not proven by a single-joint Gate 3C test.
8. Measurement filtering can add phase lag. It cannot repair missing
   timestamps or unstable transport.
9. Trajectory shaping must be state-to-state and interruptible by the newest
   target; it cannot hide stale targets or a backlog.
10. The non-real-time host exhibited isolated scheduling warnings. Continue to
    report tails, misses, CPU migration, and SDK heartbeat CPU—not only mean
    frequency.
11. A predefined physical Cartesian Gate 3D was not performed. The next
    session must explicitly justify its chosen joint/Cartesian command path and
    decide whether fixed Cartesian validation is required before live input.

## Exact next objective

Implement and validate the **first bounded TeleDex 6-DoF arm-only
teleoperation path** on top of this foundation. Implementation must proceed
through receive-only, timestamp/sequence validation, offline mapping, shadow,
and fake/native transport tests before a separately approved live session.

The current `ArmOnlyRuntime.dispatch()` deliberately sends targets with motion
permission disabled. Do not bypass that gate. Add a reviewed, explicit arm
enable/clutch authorization path rather than changing the default to moving.

## Prohibited shortcuts

- Do not reuse the legacy HEBI follower, filters, mapping, state machine,
  threading, or timing.
- Do not begin unrestricted or full-scale motion.
- Do not add RH56 control or Quest integration.
- Do not use a growing FIFO or execute delayed targets after a pause.
- Do not move before fresh robot-state startup synchronization and explicit
  clutch activation.
- Do not stream raw TeleDex poses directly to JAKA.
- Do not scatter coordinate transforms across adapters/controllers.
- Do not use filtering or Ruckig to conceal stale data, transport instability,
  or accumulated command debt.
- Do not put Python callbacks, filesystem writes, console output, dynamic
  configuration, or growing allocations in the 8 ms native loop.

## Recommended first bounded TeleDex constraints

The next session should implement these as configuration plus tested safety
invariants, then obtain new physical authorization before launching:

1. Arm 6-DoF only; TeleDex hand joints ignored; RH56 absent.
2. Receive-only and shadow validation first. Confirm all six signed axes and
   rotations against the centralized transform before any motion permission.
3. Relative pose mapping from a fresh robot/device anchor. Hold-to-run clutch,
   explicit recenter, no motion on startup, and mandatory re-clutch after stale
   data, reconnect, tracking loss, or sequence reset.
4. Translation and rotation scale no greater than 0.1; recommended first value
   0.05 for both.
5. Anchor-relative workspace no larger than ±20 mm translation per axis and
   ±5 degrees rotation per axis for the first session, further constrained by
   verified robot workspace and joint margins.
6. Initial TCP limits no greater than 10 mm/s linear speed and 5 deg/s angular
   speed, with conservative acceleration and jerk shaping. Exact values must
   be verified against the chosen IK/Cartesian path before hardware use.
7. Keep current sample-age thresholds as upper bounds: warning 40 ms, hold
   100 ms, controlled stop 500 ms, fatal 2 s. Never extrapolate blindly.
8. Fixed-size latest-sample transport only. Reject duplicate/reordered
   sequences and invalid/future/non-monotonic local timestamps; expose drops
   and target age.
9. Validate every generated joint target for finiteness, branch continuity,
   soft-limit margin, singularity risk, velocity, acceleration, jerk, and
   predicted Cartesian envelope before dispatch.
10. Require fresh tool/user IDs 0/0, fault code 0, E-stop/collision false,
    powered/enabled state, no competing JAKA command process, accessible
    E-stop, and a clear workspace.
11. Use one disposable native worker and one SDK client. No automatic EDG
    retry or post-fault resume.
12. First live exposure should be operator-approved, at most 10 seconds, with
    a visible countdown, immediate stop, full timing/target/observation logs,
    and no automatic repetition. Stop on any wrong direction, first-target
    discontinuity, unexpected motion/sound/vibration, persistent divergence,
    SDK error, stale input, hard/repeated timing miss, or cleanup uncertainty.

This handoff does not authorize that physical session; it defines the evidence
and constraints the new Codex session must use to request authorization.
