# Bounded TeleDex → JAKA arm architecture and operating boundary

Date: 2026-07-16  
Status: Stage T1 implemented and hardware-free validated; T2 live-phone evidence
and T3 connected command shadow still required; T4 motion is blocked

## Scope

The active runtime is arm-only:

```text
TeleDex WebSocket JSON
  -> TeleDex adapter boundary
  -> ArmPoseSample + RunGateSample + OperatorActionSample
  -> timestamp/sequence/tracking validation
  -> centralized source/normalized/operator/base/TCP frame mapping
  -> relative clutch anchor
  -> geodesic One Euro measurement filter
  -> scale, workspace clip, Cartesian jerk-bounded target shaper
  -> Cartesian safety supervisor
  -> fixed-size latest-only Unix datagram
  -> native JAKA worker
       -> startup-TCP-relative target composition
       -> feedback/prior-branch-seeded SDK IK
       -> FK residual, numerical Jacobian condition, joint envelope
       -> native joint velocity/acceleration/jerk tracker
       -> dynamic tracking/timing supervision
       -> existing single-owner EDG lifecycle
```

TeleDex is an input provider. No downstream module imports TeleDex classes.
HEBI, the legacy relative follower, Quest, RH56, ROS2, camera, digital-twin, and
hand-retargeting modules are absent from this composition. Automated AST and
blocked-import tests enforce the HEBI/RH56 boundary.

## TeleDex 0.0.7 audit

| Item | Repository/package evidence | Production treatment |
|---|---|---|
| Transport | Host-side `websockets.serve` TCP WebSocket server, default port 8888 | Dedicated audited server, one active client, `max_queue=1`, bounded latest sample |
| Packet | JSON object containing `position`, 3×3 `rotation`, `button`, `toggle`; README also names `button_secondary` | Strict parser; malformed packets invalidate run permission |
| Position units | Upstream API and `hand_offset` document metres | Contract stores metres; six-direction live diagnostic still required |
| Orientation | Incoming 3×3 matrix; upstream 0.0.7 transposes it before exposure | Adapter reproduces this transpose and outputs normalized quaternion `xyzw` |
| Axes/handedness | Not documented in package, GUIDE, or paper | Explicitly unknown; centralized basis remains an unconfirmed diagnostic placeholder |
| Pose direction | Paper calls it phone pose in AR reference frame, but package does not state matrix direction precisely | Named uncalibrated frame; no motion gate until live direction confirmation |
| Timestamp | No source/device timestamp in 0.0.7 packet or normalized API | `source_capture_ns=None`; monotonic receive and processing times remain separate |
| Sequence | No source sequence | Adapter assigns monotonic local sequence; `source_sequence=None` |
| Tracking quality | No ARKit tracking state/quality field | `TrackingState.UNKNOWN`, never silently promoted to `VALID` |
| Observed rate | Protected prototype report observed approximately 60 Hz live App updates | T2 must measure current App/network rate; not hard-coded |
| Packet loss | WebSocket/TCP is ordered, but loss/burst/source-frame omission cannot be measured without a source sequence | Interarrival gaps and stale age are recorded; no loss percentage is invented |
| Reconnect | Upstream connection/disconnection callbacks; no resume policy | Connection epoch increments; first reconnect sample requires release/re-clutch |
| Frame type | `position`/`rotation` is phone device pose; optional hand pose is separately computed by upstream | Production arm path consumes device pose only; landmarks/hand pose are ignored |
| Port behavior | Upstream repeatedly kills an existing port owner | Production preflights and refuses a busy port; it never kills another process |

Protected prototype imports that are forbidden in production include
`src/teleop_tools/teledex_phone.py`,
`src/teleop_tools/teledex_rviz_shadow.py`, and
`tools/run_real_jaka_teledex_arm_teleop.py`. They import
`HebiMobileIOSnapshot`, `RelativePoseLagFollower`, HEBI shadow helpers, and the
legacy combined bridge flow.

## Contracts and latest-sample behavior

`ArmPoseSample` records source ID, local sequence, optional source sequence,
optional source timestamp, monotonic receive/processing times, metres,
normalized quaternion `xyzw`, frame ID, validity, tracking state/quality,
current age, connection epoch, and discontinuity kind.

Hold-to-run state is a separate `RunGateSample`. Button A maps to hold-to-run.
Button B's rising edge maps to a generic `OperatorActionSample` recenter
request. For the first bounded session, recenter performs a controlled session
stop; relaunching the disposable worker captures a fresh robot TCP and requires
a new Button-A edge. No automatic resume is possible.

The WebSocket server has depth-one receive buffering. The adapter stores one
latest immutable snapshot. Replay skips all overdue records and returns only
the newest due sample. The native datagram worker drains all available packets
and retains only the highest sequence. No layer executes a pose FIFO.

## Frame convention

`T_A_B` maps coordinates in B into A. All source-axis and handedness conversion
is owned by `CentralFrameMapping`:

```text
TeleDex source frame
  -- source_to_normalized_basis --> normalized input frame
  -- device_to_operator_control --> operator control frame
  -- clutch-relative world delta --> robot base frame
  -- startup robot TCP anchor --> configured JAKA TCP target
```

An orthogonal basis with determinant -1 is allowed for an explicit handedness
conversion. Position maps as `C p`; orientation maps as `C R C^-1`, preserving
a proper rotation. Axis swaps, quaternion reordering, and handedness conversion
are prohibited outside this module.

The current configuration basis is identity only to make diagnostic output
auditable. `source_semantics_confirmed`, `confirmed_for_shadow`, and
`confirmed_for_motion` are all false. It cannot authorize T4.

Relative mapping uses world-frame changes:

```text
delta_p = p_device_now - p_device_anchor
delta_R = R_device_now * inverse(R_device_anchor)
p_tcp_target = p_tcp_anchor + translation_scale * delta_p_mapped
R_tcp_target = scale_SO3(delta_R, rotation_scale) * R_tcp_anchor
```

Clutch activation therefore produces exactly zero relative displacement.

## Filtering and shaping

Measurement filtering and command shaping are separate.

- Translation One Euro filter: 2 Hz minimum cutoff, beta 30 s/m, 1 Hz
  derivative cutoff.
- Orientation One Euro filter: geodesic SO(3) derivative/log map and SLERP,
  2 Hz minimum cutoff, beta 3 s/rad, 1 Hz derivative cutoff. Quaternion
  components are never averaged.
- Filters reset on clutch edge, recenter, reconnect, tracking recovery,
  discontinuity, and invalid/stale input.
- The Cartesian shaper is timestamp-aware and refuses gaps above 50 ms instead
  of smoothing over transport loss.
- Cartesian and native joint target tracking independently bound velocity,
  acceleration, and jerk. The native implementation is a local interruptible
  state-to-state tracker. Ruckig was not added because the repository contains
  no pinned/local Ruckig dependency; this implementation is tested against the
  configured derivative bounds and is not represented as Ruckig.

## Input and recovery policy

| Event | Action | Resume rule |
|---|---|---|
| No sample yet | No JAKA worker target; no EDG | Wait for valid pose, then explicit release/press |
| Invalid packet/pose | Run gate false, hold/stop | Valid tracking plus release/re-clutch |
| Age ≥40 ms | Timing/health warning | Continue only inside all other limits |
| Age ≥100 ms | Hold; native target velocity decays | Re-clutch after recovery |
| Age ≥500 ms | Controlled session stop and cleanup | New disposable session |
| Age ≥2 s | Fatal communication classification | Explicit safe reset/new session |
| Duplicate/reordered local sequence | Reject, retain newer target | No backlog replay |
| Future/nonmonotonic local timestamp | Abort/reject | New session after review |
| Source sequence reset, if future source supplies one | Recenter required | Release/re-clutch |
| Transport disconnect | Immediate run-gate invalidation | Reconnect epoch requires release/re-clutch |
| Reconnect/tracking recovery/relocalization | Never auto-resume | Release and fresh clutch anchor |
| Button A release | Controlled session stop in first T4 | New session captures fresh robot anchor |
| Button B rising edge | Recenter request -> controlled session stop | Relaunch, release, fresh Button-A edge |
| JAKA SDK/read/IK/FK error | Fatal, no retry | Deterministic cleanup; explicit investigation/reset |
| Two dynamic tracking crossings | Fatal | Cleanup and investigation |
| Hard/repeated timing miss | Fatal | Cleanup; no automatic restart |

## Native command generation and lifecycle

The existing worker's original four modes are retained. New modes are:

- `command-shadow-dry-run`: fake JAKA, IK/limit generation, no EDG/write;
- `command-shadow`: physical JAKA login/state/FK/IK only, no EDG/write;
- `bounded-teleop-dry-run`: fake full command path;
- `bounded-teleop`: explicitly gated physical path, never launched by default.

The physical bounded mode connects, verifies state/tool/user IDs, captures the
startup joint/TCP state, and waits without EDG. Only the first fresh target with
the explicit motion flag may enter the already validated EDG/servo lifecycle.
It then uses `kine_inverse` seeded from the prior accepted branch, checks the FK
residual (1 mm/2 degrees), numerical Jacobian condition, maximum IK update,
joint soft limits, native dynamics, dynamic observation error, target age, and
loop timing before issuing `edg_servo_j`.

Cleanup remains process-owned:

```text
cease target writes -> servo_move_enable(false) -> edg_init(false)
-> login_out -> disposable process exit
```

There is no reconnect/retry or automatic post-fault resume.

## Exact first-test configuration

From `configs/teleoperation/teledex_jaka_arm_bounded.yaml`:

| Limit | Value |
|---|---:|
| Translation/rotation scale | 0.05 / 0.05 |
| Absolute startup-TCP translation envelope | ±15 mm per robot-base axis |
| Shaped target guard inside translation envelope | 2.667 mm (effective target extent ±12.333 mm) |
| Absolute orientation envelope | 4 degrees |
| Shaped target guard inside orientation envelope | 1.533 degrees (effective target angle 2.467 degrees) |
| TCP linear speed/acceleration/jerk | 8 mm/s, 30 mm/s², 150 mm/s³ |
| TCP angular speed/acceleration/jerk | 4 deg/s, 15 deg/s², 60 deg/s³ |
| Joint speed/acceleration/jerk | 0.03 rad/s, 0.15 rad/s², 1.5 rad/s³ |
| Joint soft margin | 5 degrees from repository model limits |
| Maximum IK branch step | 0.10 rad per accepted source update |
| Maximum numerical Jacobian condition | 200 (infinity-norm estimate) |
| Session duration | at most 10 seconds; no automatic restart |
| Tracking warning | 0.2 degrees, diagnostic only |
| Dynamic hard tracking | `max(0.75 deg, 2.5 * abs(command_velocity) * 0.150 s)`, two consecutive crossings abort |
| Timing warning/hard | 8.8/12 ms start period; >2 ms wake warning; 8 ms debt fatal; two consecutive warning/completion misses abort |

The effective value is always the strictest active input, Cartesian, IK, joint,
tracking, timing, SDK, lifecycle, and operator limit.

## Exact staged commands

Build and T1:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_servo_worker -j2

PYTHONPATH=src .venv/bin/python tools/teleoperation/diagnose_frame_mapping.py \
  --config configs/teleoperation/teledex_jaka_arm_bounded.yaml
```

T2, no JAKA:

```bash
PYTHONPATH=src .venv/bin/python tools/teleoperation/diagnose_teledex_input.py \
  --config configs/teleoperation/teledex_jaka_arm_bounded.yaml \
  --port 8888 --duration-s 30 \
  --record logs/teleop/teledex_t2_$(date +%Y%m%d_%H%M%S).jsonl \
  --summary logs/teleop/teledex_t2_summary_$(date +%Y%m%d_%H%M%S).json
```

T3, physical login/read/FK/IK shadow but no EDG or command:

```bash
PYTHONPATH=src .venv/bin/python tools/teleoperation/run_teledex_jaka_session.py command-shadow \
  --config configs/teleoperation/teledex_jaka_arm_bounded.yaml \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --expected-tool-id 0 --expected-user-frame-id 0 --duration-s 10 \
  --t2-receipt <T2_SUMMARY_JSON> \
  --metrics-file <T3_NATIVE_METRICS_JSON> \
  --summary-file <T3_SUMMARY_JSON> \
  --target-log <T3_TARGET_JSONL> \
  --record-input <T3_INPUT_JSONL> \
  --allow-unconfirmed-source-semantics-for-no-motion-shadow \
  --acknowledgement I_ACKNOWLEDGE_JAKA_COMMAND_SHADOW_NO_EDG
```

The unconfirmed-source override exists only because T3 has no EDG/write path.
It is rejected by T4.

T4 exact command is documented for review but is currently blocked by the
false calibration gates and must not be run without a new explicit approval:

```bash
PYTHONPATH=src .venv/bin/python tools/teleoperation/run_teledex_jaka_session.py bounded-t4 \
  --config configs/teleoperation/teledex_jaka_arm_bounded.yaml \
  --robot-ip 192.168.71.50 --edg-state-ip 192.168.71.19 \
  --expected-tool-id 0 --expected-user-frame-id 0 --duration-s 10 \
  --t2-receipt <ACCEPTED_T2_SUMMARY_JSON> \
  --metrics-file <T4_NATIVE_METRICS_JSON> \
  --summary-file <T4_SUMMARY_JSON> \
  --target-log <T4_TARGET_JSONL> \
  --record-input <T4_INPUT_JSONL> \
  --execute-t4 \
  --operator-approval I_APPROVE_T4_BOUNDED_TELEDEX_JAKA_MOTION \
  --estop-accessible --workspace-clear --rh56-command-path-absent \
  --acknowledgement I_ACKNOWLEDGE_BOUNDED_TELEDEX_JAKA_MOTION
```

## T4 operator procedure, stop, and recovery

1. Review accepted T2/T3 receipts, current config hash, frame-direction sheet,
   current source/App version, native binary hash, and all unresolved risks.
2. Confirm source axes, handedness, pose direction, phone/control extrinsic,
   tool/user IDs 0/0, startup TCP, payload, joint margins, and clear ±15 mm /
   4-degree swept volume.
3. Prove no competing JAKA command process; prove no RH56 command process or
   serial/TIO connection exists.
4. Put the E-stop in the operator's hand, keep every person outside the swept
   volume, power/enable normally, and verify fault/E-stop/collision clear.
5. Start TeleDex first. Keep Button A released. Connect the phone and hold the
   neutral pose. Do not use Freeze Pose or Reset Pose.
6. Launch the one-shot T4 command after explicit approval. Observe the five
   second countdown. The worker captures current joints/TCP and remains
   non-commanding until a fresh Button-A rising edge.
7. Press and hold Button A. Initial target displacement is exactly zero. Move
   only one predeclared direction at a time and stop immediately on any wrong
   direction, jump, sound, vibration, oscillation, cable tension, unexpected
   motion, contact, warning escalation, or logging uncertainty.
8. Release Button A to stop. Button B requests recenter by ending the disposable
   session; relaunch captures a fresh robot/device origin. Do not expect resume.
9. Emergency: press E-stop for unexpected/untrusted motion. The software path
   sends stop/ceases targets, disables process-owned servo mode, exits EDG,
   logs out, and exits. Never wait for software if physical safety is uncertain.
10. Recovery: keep Button A released; do not auto-restart; inspect native and
    target logs plus controller state; verify process exit and EDG/servo state;
    clear controller faults only through the approved operator interface; use
    explicit safe fault reset/new process only after the cause is understood.

## Remaining approval blockers

- No TeleDex phone stream was connected, so T2 was **not executed** and live
  source behavior is not yet measured in this session. The zero-packet
  receive-only readiness probe is not a pass, failure, or acceptance result.
- All source/calibration confirmation gates remain false.
- T3 connected SDK IK/Jacobian timing and behavior have not yet been measured.
- The SDK source clock and source sequence remain unavailable; true capture-to-
  robot latency and packet-loss percentage cannot be computed.
- Numerical Jacobian conditioning uses SDK FK and must meet T3 timing margins;
  its cost cannot be assumed from fake tests.
- No predefined Cartesian Gate 3D has physically validated the selected SDK IK
  branch and multi-joint path. A fixed translation/rotation probe may still be
  required before live T4 approval.
- Environment/self-collision certification is outside this scope; the first
  envelope relies on a directly cleared local swept volume and joint margins.
- The custom native state-to-state tracker is bounded and tested but is not
  Ruckig. Promotion should review whether a pinned local Ruckig integration
  materially improves synchronized stopping without harming the 8 ms budget.
