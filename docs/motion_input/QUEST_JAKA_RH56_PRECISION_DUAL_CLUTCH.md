# Quest 3 precision teleoperation: dual-clutch simulation design and audit

Initial implementation audit date: 2026-07-20. Repository isolation was
re-audited on the same date before the provider-independent checkpoint. This
document describes simulation/offline behavior only. No physical JAKA or
Inspire connection or command path is used.

## Repository checkpoint and worktree audit

- Primary repository root: `/home/thor/projects/embodied_lab`
- Primary branch and HEAD at checkpoint isolation:
  `feature/jaka-teledex-control-foundation` at
  `ac7399be8951560bf154273974fe85c8927aabc9`; the worktree was clean.
- Original implementation worktree:
  `/home/thor/projects/embodied_lab_quest_jaka_sim`,
  branch `feature/quest-jaka-offline-simulation`, HEAD
  `8f72e7b40c6ea31674c81aa9c82eabfe134d1095`; it also contained relevant
  uncommitted SE(3), MuJoCo, and RH56 work before this audit.
- Isolated checkpoint worktree:
  `/home/thor/projects/embodied_lab_quest_jaka_dual_clutch`, branch
  `feature/quest-jaka-dual-clutch-checkpoint`, created from `8f72e7b` with only
  the validated runtime/test dependency closure transferred. The original
  dirty implementation worktree was left unchanged.
- Other worktrees: clean `feature/quest-hand-tracking-streamer-integration` at
  `7f4036eaffffde74c5ccb2698734e7c68094673d`, clean
  `feature/teledex-bounded-live-teleoperation` at
  `52b67fab057afd0a63c67fa6b6f9333dc807113a`, and clean
  `chore/repository-cleanup` at `d797dc5ce27a0d7cb1ca34df7a3ef23093d75e79`.

The implementation path is HTS UDP CSV -> `hts_protocol.py` validation ->
`hts_transport.py` receipt metadata -> `hts_canonical.py` Unity-to-canonical
conversion/freshness -> timestamped interpolation and quaternion-safe filtering
-> the independent clutch session -> project JAKA continuation IK/constraints ->
MuJoCo position actuators. The hand path is the same validated 21-joint
observation -> project-native `ProjectRh56Retargeter` -> canonical RH56 order ->
explicit MuJoCo actuator order -> existing hand slew limiter.

The root hardware lifecycle state machine remains
`DISCONNECTED -> CONNECTING -> CONNECTED -> ARMED -> EDG_READY -> HOLDING/RUNNING
-> CONTROLLED_STOP/FAULT -> SHUTDOWN`. The Quest simulation branch does not
import or bypass that hardware machine and has no hardware driver output.

Recorded data audited under both `logs/quest_input` and
`logs/quest_jaka_sim` includes live gate/retry/occlusion recordings and eleven
HTS JSONL captures with hand and optional head packets. No capture contains
controller trigger or grip state.

## Verified HTS capability

The inspected application is
[wengmister/hand-tracking-streamer at 5ff7c1c](https://github.com/wengmister/hand-tracking-streamer/tree/5ff7c1cfea0ead1bb8a0e233bc7770d94d31feb5).
`HandLandmarkStreamer.cs`, `HeadPoseStreamer.cs`, `AppManager.cs`, the Python
parser, local recordings, and configuration were inspected.

| Stream or property | Present in host packet contract | Validity/timing detail |
|---|---:|---|
| Right wrist 6-DoF | Yes | Paired with the right landmarks packet; freshness inferred by host |
| Right 21-joint skeleton | Yes | Positions only; paired with wrist validity |
| Head 6-DoF | Optional separate datagram | Separate freshness; debug source `f`/`t` only when enabled |
| Left controller pose | No | Not parsed or recorded |
| Left index-trigger analog | No | Not sent by the app |
| Left grip-trigger analog | No | Not sent by the app |
| Per-stream source timestamp | Conditional | Debug header `t`; otherwise host receipt time |
| Per-stream source sequence | Conditional | Debug header `f`; otherwise host-generated hand sequence |
| Explicit tracking confidence/status | No | Host infers valid/stale; wrist and skeleton are inseparable |

The Android project enables both hand-tracking and Touch-controller OpenXR
profiles, but application code uses the left controller only for a local menu
button and sends no controller data. Therefore simultaneous right bare-hand
wrist/skeleton plus host-visible left index/grip input is **not supported
end-to-end**. The right hand is never required to hold a controller. Live HTS
now leaves both clutches unavailable and outputs frozen. It does not claim live
support and does not silently substitute keyboard control. A later live
provider must implement the timestamped analog contract in `clutch.py`.

## Focused external reference review

| Repository and pinned commit | Trigger representation and behavior | Decision |
|---|---|---|
| [PickNik meta_quest_teleoperation `4bd4b5f`](https://github.com/PickNikRobotics/meta_quest_teleoperation/tree/4bd4b5fb50cb2438ed656b0fe2d9f34a508670db) | Unity Button actions for trigger/grip; publishes `WasPressedThisFrame` events and continuous booleans at 60 Hz. No exported analog values, explicit hysteresis, or per-button stale policy. | REFERENCE_ONLY: continuous state plus edge concepts. Controller-only ROS runtime is incompatible. |
| [Unitree xr_teleoperate `7dc9aa1`](https://github.com/unitreerobotics/xr_teleoperate/tree/7dc9aa1a6edbf4a9f4f887d8ab6fc449ea5135f6) | Analog trigger/squeeze values exist, but hand and controller are mutually selected input modes; no independent clutches or per-stream fault recovery in the inspected loop. | REFERENCE_ONLY: named analog channels and head-yaw-relative intent. NOT_COMPATIBLE: exclusive input modes, shared readiness, sample-count filters. |
| [Open-Teach `32a7d44`](https://github.com/aadhithya14/Open-Teach/tree/32a7d44b33953066ff27312a7b2b4c294f4f52c5) | Textual low/high pause and resolution signals. STOP-to-CONTINUE recaptures current robot and hand references; relative homogeneous transforms avoid a jump. Dynamic resolution scaling is present. | ADAPT_CONCEPT: authoritative robot/hand recapture. NOT_COMPATIBLE: dynamic gain/resolution and weak stale-input handling. |
| [XRoboToolkit Teleop Sample `79e5cb8`](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python/tree/79e5cb8a56e3455515ce1b476e993c764ec58739) | SDK exposes analog trigger/grip floats and controller/head poses plus hand-active state. Arm grip uses one `>0.9` threshold; falling release clears references and next press captures actual EE/controller references. No hysteresis or per-stream staleness. | ADAPT_CONCEPT: analog contract and recapture. REFERENCE_ONLY: controller/hand SDK surface. NOT_COMPATIBLE: one threshold, controller-only arm pose, no fault arming. |
| [AnyDexRetarget `77c0a10`](https://github.com/qqsq12321/AnyDexRetarget/tree/77c0a1074ba6eb003159da37b2bd3cec41792523) | No controller/clutch layer. Adaptive/key-vector objectives use joint limits, prior-solution regularization/warm start, and a fixed-alpha low-pass filter. Its Quest example uses the same hand-only HTS protocol. | ADAPT_CONCEPT: offline feature/error comparisons. REFERENCE_ONLY: optimization approach. NOT_COMPATIBLE: replacing project model/backend or receiver. |
| [dex-retargeting `3f56141`](https://github.com/dexsuite/dex-retargeting/tree/3f56141bc8bd2760d5e452e382937269554ebb21) | No controller/clutch layer. Sequential optimization warm-starts from last qpos, enforces model joint limits, returns the last solution on optimizer failure, and optionally low-pass filters. | ADAPT_CONCEPT: hold-last and offline timing/error metrics. REFERENCE_ONLY optional isolated backend; no new Pinocchio/NLopt dependency here. |

No external runtime, receiver, robot model, IK, or output code was copied.

## Coordinate frames and mapping

HTS reports Unity left-handed world coordinates (+X right, +Y up, +Z forward).
The existing canonicalizer reflects Z to form the project Quest world and
converts active `xyzw` quaternions. Poses represent `T_parent_child`, mapping
child coordinates into the parent. Quaternions are normalized, sign-aligned,
and used as active rotations.

At a valid arm index-trigger press edge:

1. The current filtered right wrist, authoritative simulated TCP, and head pose
   are sampled in the same control tick.
2. Head local forward (-Z) is projected onto the gravity-normal XZ plane. A
   projected local-right fallback handles near-vertical forward. This latches a
   Y-up horizontal yaw frame; head translation is never used.
3. The filter, derivatives, simulated command trajectory, and IK continuation
   reference are synchronized. The first target is exactly the authoritative
   current TCP.
4. Wrist motion is computed as
   `T_hand_delta = inverse(T_quest_hand_ref) * T_quest_hand_current`.
5. A fixed conjugation expresses that local delta in the latched horizontal
   frame, followed by the explicit Quest-to-JAKA signed-axis basis.
6. The target is right-composed as
   `T_robot_target = T_robot_tcp_ref * T_robot_delta`.

Translation and rotation gains are fixed at 1.0 in the default configuration.
No Euler subtraction, velocity control, hybrid control, runtime gain switching,
or target integration while released is used. Head pose is not accepted by the
engaged-update API, so later head movement or staleness cannot change or stop an
engaged arm.

## Dual-clutch and fault behavior

Both analog channels use press >= 0.75, release <= 0.55, and state retention in
between. Each requires its own observed valid release before its first press and
again after a fault. Repeated high samples create no repeated edge.

Arm states are `ARMED_WAITING_FOR_RELEASE`, `DISENGAGED`, `REFERENCE_CAPTURE`,
`ENGAGED`, and `TRACKING_FAULT`. Hand states are
`ARMED_WAITING_FOR_RELEASE`, `DISENGAGED`, `REACQUIRE`, `ENGAGED`, and
`TRACKING_FAULT`. They are coordinated but never combined into a mode enum.

An arm release freezes the last safe Cartesian/IK target and clears the mapping
reference so released wrist motion cannot accumulate. A new press captures the
current simulated TCP. A hand release does not run retargeting and holds the
last safe RH56 target. A new grip press blends from that held command over 200
ms before steady tracking; the existing model slew limiter remains downstream.
Arm recenter cycles do not touch the hand target.

Index/wrist faults stop only the arm; grip/skeleton faults stop only the hand
when validity can be independent. Entire left-controller loss faults both. HTS
right-hand validity is coupled, so its loss conservatively faults both active
channels. Recovery never automatically resumes either output.

Workspace/IK rejection retains the last safe target and does not accumulate the
candidate. Human-reference envelope telemetry warns at 80% without automatic
clutching or gain changes. Consecutive robot feasibility rejection faults only
the arm after the configured count.

## Validation and remaining gap

Automated tests cover analog hysteresis, startup arming, held-edge behavior,
reference capture, release freeze, no released accumulation, twenty varied
recenter cycles, stale/fault recovery, head requirements/latching, all rotation
axes including downward rotation, quaternion sign equivalence, hand freeze and
200 ms reacquisition, all four independent combinations, shared-controller
loss, deterministic synthetic pose/skeleton/trigger providers, and MuJoCo-only
session integration.

The default loop records source/control/IK/hand rates, sample ages,
trigger-to-engagement latency, target latency, reference capture duration,
reacquisition duration, IK time, retarget time, faults, and cycle counts. The
viewer overlay presents both clutch states/values/ages, wrist/skeleton status,
latched yaw, references/targets/deltas, IK/retarget/reacquisition status, faults,
and cycle counts.

### Bounded MuJoCo replay results

The 2026-07-20 recorded right-hand/head stream
`quest_jaka_live_6dof_20260720T150225+0800.hts.jsonl` was replayed with an
explicit `deterministic_replay_cli` clutch provider. A three-second ten-cycle
arm-only run produced 10 successful reference captures, 60 accepted targets,
zero feasibility rejections, zero capture-frame TCP translation jump, zero
change in the frozen commanded RH56 vector, and 1.327 mm maximum simulated-TCP
tracking error. Input/control rates were 71.34/59.99 Hz; mean/p95/max reference
capture was 0.114/0.171/0.220 ms and mean/p95/max IK was
2.52/2.81/2.98 ms.

A four-second overlapping dual-cycle run exercised all four natural
combinations with six arm cycles and four hand cycles. It produced 72 accepted
arm targets, no rejection/fault, zero capture-frame TCP translation jump, and
zero frozen-hand command change. Input/control rates were 71.55/59.99 Hz; active
hand-retarget and IK rates were 34.10/32.54 Hz because their clutches were
released for part of the run. Mean/p95/max capture was
0.123/0.197/0.226 ms, IK was 2.50/2.80/2.91 ms, retargeting was
0.177/0.197/0.326 ms, hand engagement completed at exactly 200 ms, and maximum
TCP tracking error was 1.550 mm.

The same three-second recording was evaluated at 150, 200, 250, and 300 ms hand
reacquisition. Measured completion was exactly the configured duration; the
maximum observed simulated hand-joint step was respectively 0.0169, 0.0177,
0.0156, and 0.0140 rad. The 200 ms middle value remains the default: it stays in
the requested 150--300 ms range without imposing the longest delay, while the
existing downstream slew limiter bounds the actual joint motion.

Focused precision, SE(3), hand, session, HTS, provider, and MuJoCo tests pass
92 cases. A repository-wide run
passed 228 tests and failed nine unrelated existing Correll-asset tests because
this worktree lacks `data/sim_assets/correll_rh56dfx` meshes/XML. No external
assets were restored or altered as part of this task.

The unresolved limitation is live left-controller transport. Interactive live
dual-clutch validation cannot honestly be completed until a reviewed provider
supplies controller-valid, index, grip, timestamp, and sequence fields while
right bare-hand tracking remains active. The external HTS app is outside this
repository and was not modified. Deterministic offline replay is explicitly
labelled `deterministic_replay_cli` and is not a live substitute.

An optional future hand-backend comparison should replay neutral, open, spread,
fist, pinch, and transitional recordings through (1) the project baseline, (2)
an AnyDex-informed project-native adaptive variant, and (3) isolated
dex-retargeting. Record output jitter, limit-hit rate, pinch/key-vector error,
clutch-cycle continuity, reacquisition transient, solve time, and failed frames.
The project-native backend remains the default.
