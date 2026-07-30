# RH56 H0 simulation self-test

This page records the 2026-07-27 repository review and the current H0-only
operation. H0 is offline MuJoCo validation. It is not Quest hand mapping and is
not physical RH56 or JAKA validation.

## Repository and historical call-chain review

The reviewed baseline is commit `0c30c2d2e550f7da7f3648e4d1849a78adaf38e1`.
The original dual-clutch hand path entered the history at commit `4a0b5e4` on
`feature/quest-jaka-dual-clutch-checkpoint`; commit `530d3a0` on
`feature/quest-controller-transport-host` added the live CTRL v1 sidecar and
thumb refinements. Both are ancestors of the reviewed baseline.

The committed simulation call chain was:

```text
scripts/run_quest_jaka_sim_demo.sh
  -> tools/quest_jaka_mujoco_sim.py main / _live_6dof
  -> QuestDatagramReceiverWorker
  -> LiveQuestControllerRouter.ingest
       CTRL -> parse_controller_datagram -> ControllerProvider
            -> ControllerClutchAdapter -> set_clutch_samples
       HTS  -> SmoothQuestJakaSession.ingest -> parse_hts_datagram
            -> HtsCanonicalAssembler
  -> SmoothQuestJakaSession.control_tick
  -> HandClutchMachine.step
  -> SmoothQuestJakaSession._update_hand
  -> QuestHandSkeleton.from_observation
  -> ProjectRh56Retargeter.retarget
  -> JakaMujocoSimulation.set_hand_actuator_target
  -> JakaMujocoSimulation.step_to
  -> six RH56 MuJoCo position actuators
```

HTS provides a right wrist pose and 21 wrist-local landmark positions. It does
not provide controller grip/index, curl scalars, Euler angles, or an RH56
command sidecar. CTRL v1 on the same UDP socket provides left index and left
grip analog values with source/host timestamps, session and sequence. The
index trigger is the arm clutch; grip is only the hand hold-to-run clutch.
Press/release thresholds are 0.75/0.55, stale timeout is 150 ms, repeated
sequence numbers create no edge, and release must be observed before a rising
edge. The right hand/landmark freshness timeout is 250 ms.

The four non-thumb commands are computed from joint-angle curl over HTS indices
`(5..8)`, `(9..12)`, `(13..16)`, and `(17..20)`, with a small MCP contribution
in the refined implementation. Thumb close blends thumb angle curl over
indices `(1..4)` with a palm-normalized thumb-tip-to-nearest-fingertip pinch
feature. Thumb lateral uses the cosine between thumb metacarpal-to-tip and
index-base-to-pinky-base vectors. These are computed features, not transported
curl values.

The explicit arm-only simulation mode deliberately calls
`build_viewer_mjcf(..., arm_only=True)`. Commit `0c30c2d` added that flag and
removes all `rh56_*` actuators from the generated viewer model. The hand bodies,
visual/collision geoms, 12 joints, and equality constraints remain in the XML,
but `JakaMujocoSimulation.hand_available` becomes false and
`SmoothQuestJakaSession.hand_enabled` disables `_update_hand`. Thus the current
runtime removed both actuator availability and command-path activation, while
retaining the dormant mapping code.

No committed Quest simulation path imports `rh56_driver`,
`RH56SerialBackend`, `RH56JakaToolBackend`, or a JAKA SDK adapter. Separate
hardware backends do exist in `src/rh56_driver/serial_backend.py` and
`src/rh56_driver/jaka_tool_backend.py`; they are not reusable in H0 and are not
reachable from this entry.

## Thumb defect evidence

The initial `4a0b5e4` implementation in
`src/quest_jaka_sim/hand_retarget.py` used:

```text
palm = |middle_proximal - wrist|
pinch = clip(1 - |thumb_tip - index_tip| / palm / 0.70, 0, 1)
thumb_bend = ordinary four-point finger-angle curl(1, 2, 3, 4)
thumb_close = (0.35 * thumb_bend + 0.65 * pinch) * 1.0
thumb_lateral = clip((cos(thumb_tip-thumb_metacarpal,
                              pinky_base-index_base) + 1) / 2, 0, 1)
```

The output was clipped and name-mapped correctly; no evidence of an Euler
angle, world-coordinate distance, thumb-close/lateral swap, output sign error,
or missing right-hand/landmark validation was found. The positions are
wrist-local, and dot products/distances are invariant to a common wrist-frame
rotation.

The concrete defects were instead:

1. The calibration was explicitly `quest_rh56_sim_uncalibrated_v1`; the old
   pinch range was a single hard-coded 0--0.70 palm ratio and lateral used an
   uncalibrated full cosine range. A 14,422-frame recording used only
   0.356/0.500 rad thumb close and 0.797/1.100 rad lateral.
2. Pinch considered only the index tip. It missed middle/ring/pinky contact and
   represented thumb close as one scalar.
3. Thumb bend reused the same generic four-point curl primitive as a finger.
   Thumb lateral was only one cosine against a palm-side vector; no complete
   palm-local orthonormal frame or full thumb key-vector/opposition objective
   existed.
4. The six-actuator model at that time coupled thumb-close MCP2 to PIP/DIP at
   unsupported fixed 0.6/0.8 ratios, so its distal motion did not match the
   later local vendor-angle table.
5. Grip reacquisition blended for 200 ms from the held actuator target to the
   current absolute gesture. It did not save a hand-skeleton reference, so
   clutching did not make finger posture relative.

Commit `530d3a0` addressed only part of this evidence: nearest-of-four pinch,
0.20/0.55 palm-normalized thresholds, thumb scale 1.2, and lateral calibration
0.08--0.74. It did not remove the single-close-DOF/model coupling or introduce a
relative hand reference. The repository cannot establish a physical RH56
thumb calibration or prove that the remaining semantic direction matches every
physical installation.

H2.1 retains the relative grip reference and replaces the later fixed
`0.45*bend + 0.55*pinch` blend with a bend-primary feature:

```text
base = bend_gain * normalized_thumb_bend
assist = pinch_assist_gain * max(0, normalized_pinch - normalized_thumb_bend)
feature = clip(base + assist, 0, 1)
```

The simulation defaults are `bend_gain=1.0` and
`pinch_assist_gain=0.4`, with no bend gate. The normalized feature is converted
to the audited 0--0.698131700798 rad actuator feature before the relative
hand-reference delta, so the unchanged relative gain of 1.0 can cover the
model's full direct thumb-close travel from an open reference. The retarget
warm-start limit and session slew limit bound speed, not the eventual target.

The local `关节角与0-1000 对应关系  .xls` workbook (SHA-256
`881d9693a01ee51086c928435df5ab66e12c8e886d100ebaf3dba8f950b305cc`)
contains one worksheet and 1001 contiguous command rows. Its absolute thumb
angles are not MuJoCo zero positions. The model instead uses endpoint-relative
travel: 40 degrees for MCP2 close, 44.99504 degrees for PIP, 35.614928 degrees
for DIP, and 80 degrees for lateral/opposition. Close and lateral columns are
linear. PIP and DIP are strictly monotonic but not linear; endpoint straight
lines miss them by 1.893 and 4.555 degrees respectively. Cubic MuJoCo equality
polynomials reproduce all 1001 PIP/DIP rows to below `1e-14` rad. The complete
audit is recorded in `data/sim_assets/rh56_thumb_table_calibration.json`.

The complete simulation hand mapping keeps the same grip reference lifecycle
and adds thumb lateral/opposition as a separate relative feature. It builds an
orthonormal wrist-local right-hand frame from index MCP to pinky MCP (positive
across-palm), wrist to middle MCP (forward after Gram--Schmidt projection), and
their cross product (normal). Thumb base-to-tip displacement along the positive
across-palm axis, divided by palm width, is calibrated from configurable
open/opposed values to `[0, 1]`. Grip press captures that feature and the
current simulated lateral target together with four-finger and thumb-close
references. Invalid or degenerate palm frames hold the complete last accepted
hand target.

## Four explicit RH56 orders

Upper layers use semantic names and never depend on MJCF order.

| Canonical channel | MuJoCo actuator | Direct MuJoCo joint | Protocol index | Legacy raw index | MuJoCo positive motion | Joint/ctrl range rad |
|---|---|---|---:|---:|---|---:|
| index | `rh56_R_index_MCP_joint_act` | `rh56_R_index_MCP_joint` | 3 | 2 | close | 0--1.70 |
| middle | `rh56_R_middle_MCP_joint_act` | `rh56_R_middle_MCP_joint` | 2 | 3 | close | 0--1.68 |
| ring | `rh56_R_ring_MCP_joint_act` | `rh56_R_ring_MCP_joint` | 1 | 4 | close | 0--1.70 |
| pinky | `rh56_R_pinky_MCP_joint_act` | `rh56_R_pinky_MCP_joint` | 0 | 5 | close | 0--1.70 |
| thumb_close | `rh56_R_thumb_MCP_joint2_act` | `rh56_R_thumb_MCP_joint2` | 4 | 0 | bend/close | 0--0.698132 |
| thumb_lateral | `rh56_R_thumb_MCP_joint1_act` | `rh56_R_thumb_MCP_joint1` | 5 | 1 | lateral/opposition | 0--1.396263 |

Canonical order is `[index, middle, ring, pinky, thumb_close, thumb_lateral]`.
Hardware protocol/feedback register order is
`[pinky, ring, middle, index, thumb_close, thumb_lateral]`. The retained legacy
raw/debug order is
`[thumb_close, thumb_lateral, index, middle, ring, pinky]`. Hardware raw counts
use 1000 open and 0 closed (`direction_sign=-1`), unlike positive MuJoCo joint
motion. MuJoCo actuator order is
`[thumb_lateral, thumb_close, index, middle, ring, pinky]`.

## Mounted model and H0 behavior

H0 loads `data/sim_assets/jaka_rh56_visual_coacd.xml`, the committed model
derived from `data/sim_assets/jaka_rh56.xml`. It retains the six JAKA actuators,
12 RH56 joints, six RH56 position actuators, 148 reviewed active convex hand
collision hulls, vendor visual geometry, seven adjacent-body exclusions, and
six equality couplings. The thumb PIP/DIP equalities use the audited cubic
table fits instead of fixed ratios. Each joint has damping 1.0 and armature
0.01; each hand actuator has `kp=8`, default gear 1, and an explicit control
range equal to its direct joint range.

The fixed hand body is a child of `jaka_Link_6` at position `[0, 0, 0.009]` m
and quaternion `[4.32978028e-17, 0.707106781, 0.707106781,
-4.32978028e-17]` in MuJoCo `wxyz` order. This transform already appears in the
initial repository commit `87684b2` and the mounted integration source; H0 did
not visually tune it. No independent calibration YAML/TF proving the physical
mount was found, and `data/sim_assets/README.md` still marks the mount transform
for future audit. It must therefore not be described as physically validated.

The model has no free joint under the hand, so it cannot float. At the configured
open hand pose the headless model reports no RH56 penetrating contact. MuJoCo
does report four duplicate -3 mm adjacent mesh contacts between existing
`jaka_Link_0_geom_0` and `jaka_Link_1_geom_0`; H0 does not alter the arm
collision model to hide them.

A forced-qpos 21-by-21 FK sweep over complete close/lateral travel remains
finite but exposes a maximum 6.62 mm intersection between the current thumb
and index convex hulls at an extreme combined command. This diagnostic bypasses
contact resolution. A dynamic full-target run keeps the commanded finite
limits and resolves the same contact below 1 mm penetration, consequently
holding actual close short of the requested endpoint. The collision geoms and
limits are retained; this is a model observation, not a collision-free claim.

`tools/rh56_h0_self_test.py` runs without Quest or network input. It prints the
three hardware-disable banners, keeps the six arm actuator targets constant,
and exercises only one canonical hand channel at a time in the order shown in
the table. Each channel uses continuous smoothstep motion from neutral to 15%
of its effective range and back. A negative excursion is attempted only when
the initial point leaves legal negative room; the current open pose is at the
lower limit, so H0 logs `negative_skipped_illegal` and holds neutral instead of
violating a range. After the last channel it restores the initial hand target.

The ignored default log directory is `logs/rh56_h0/`. Each JSONL row includes
simulation timestamp, host monotonic timestamp, phase, canonical channel,
actuator/joint names, requested and clipped control, actual qpos, joint/control
ranges, saturation/NaN flags, and phase progress.

Run on a graphical desktop from the repository root:

```bash
PYTHONPATH=src .venv/bin/python tools/rh56_h0_self_test.py --viewer
```

Close the viewer or press Ctrl-C to restore neutral and exit cleanly.

---

# 中文摘要

H0 只加载 MuJoCo 组合模型，不读取 Quest、不打开 UDP、不导入或初始化 JAKA/RH56
硬件后端。依次小幅测试 index、middle、ring、pinky、thumb_close 和
thumb_lateral，并将日志写到被忽略的 `logs/rh56_h0/`。当前 open 姿态位于手关节下限，
因此不合法的负向段会明确记录为 `negative_skipped_illegal`，不会越界。此结果仅是离线
仿真证据，不是物理验证。
