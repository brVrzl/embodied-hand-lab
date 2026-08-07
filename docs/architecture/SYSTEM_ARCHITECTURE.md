# System architecture

## Scope and authority

This page describes the current Quest-to-JAKA/RH56 implementation. It is based
on the active source and configuration, not on superseded designs under
`docs/history/`. The physical path is disabled unless a separately authorized
hardware gate satisfies its exact acknowledgements and preconditions.

The primary configuration is
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`. The current simulation and
physical entry implementations are `tools/quest_jaka_mujoco_sim.py` and
`tools/quest_jaka_hardware.py`, respectively.

## Runtime topology

```text
Quest HTS hand/head packets + CTRL v1 packets
        |
        v
QuestDatagramReceiverWorker
  receive timestamp, packet validation, bounded FIFO, drop oldest
        |
        v
LiveQuestControllerRouter
  canonical HTS state, controller validity/freshness
        |
        +---------------- left grip ----------------------+
        |                                                 |
        v                                                 v
release-before-press arm clutch                    RH56 relative retargeting
and fresh wrist/head/TCP capture                  and measured-first clutch
        |                                                 |
        v                                                 v
LatchedHeadYawArmMapper                            MuJoCo hand adapter OR
frame mapping, deadbands, filters                  PC-direct RH56 worker
        |
        v
SharedJakaTargetGenerator
bounded continuation IK and all shared feasibility checks
        |
        v
immutable AcceptedArmTarget
        |
        +-------------------------------+
        |                               |
        v                               v
MuJoCo arm adapter              JAKA accepted-joint adapter
                                        |
                                        v
                               bounded AF_UNIX datagram
                                        |
                                        v
                               sole native JAKA worker
                               8 ms PWL/latest destination
                                        |
                                        v
                               absolute J1--J6 ServoJ/EDG
```

The Quest receiver runs independently from target generation so packet I/O
does not perform IK. The Python producer owns the 60 Hz shared target pipeline.
Physical JAKA output runs in a separate native process. PC-direct RH56 serial
I/O uses its own worker so serial latency does not block the arm producer.

## Episode capture topology

```text
Quest/control producer -> JAKA/RH56 command and lightweight state publication
          |                     (never waits for camera or recorder)
          +-> canonical sampler: latest causal state/reference, no source wait

workspace D435 worker -> preallocated versioned RGB-D ring --+
wrist D435 worker     -> preallocated versioned RGB-D ring --+-> async writer
                                                        \-----> latest-only preview
```

Each camera ring has fixed slots. Queue and timeline entries contain frame
metadata plus a ring sequence, not ndarrays. The writer and preview copy a slot
only in their own threads and accept it only when version and sequence match
before and after the copy. The canonical sampler records an invalid quality row
when a causal frame is missing/stale; it never waits for the next frame.

## Hot-path ownership and lightweight telemetry

The Python producer runs the Quest/CTRL validation and clutch boundary, mapping,
one shared continuation IK attempt plus a small bounded number of retries, and
candidate legality checks at the target-generation rate. Its output check is a
coarse velocity/acceleration prefilter; it validates six finite joint values at
that boundary but does not copy the native active segment or predict jerk.
`AcceptedArmTarget` remains the immutable shared boundary.

The native worker owns the actual 8 ms velocity/acceleration/jerk transition
shaping, final hard output checks, watchdog, command publication, and cleanup.
The JAKA controller remains an additional hardware protection layer. These
layers are complementary: Python screens obviously impossible candidates, and
native/JAKA checks remain authoritative for actual output and liveness.

Normal Python ticks record fixed-width integer timing samples and a small
current event record. Full pose/metrics/continuation-attempt dictionaries are
sampled or emitted on reject/fault/reference/reacquisition events. Camera,
recorder, preview, and event-log work is asynchronous and cannot block the
native heartbeat. Episode recording keeps metadata-only invalid slots for
isolated data loss; it does not turn data quality into a robot stop.

The writer queue is bounded and non-blocking for producers. Queue overflow,
ring overwrite, preview lag, or an isolated stale frame is data loss with an
explicit counter, not a robot fault. Persistent acquisition failure stops
recording after configured evidence, while robot alarms, command-safety
violations, control liveness loss, and hard timing faults keep their existing
safety-stop authority. Final drain, validation, metadata fsync, and atomic
rename occur during bounded outer cleanup.

The combined teleoperation event log uses a separate bounded work queue;
JSON encoding and file flush run on its worker. An event-log drop or error is
reported as logging quality data and never enters the JAKA/RH56 stop path.

## Input, clutch, and frame contracts

`src/motion_input` and `src/quest_jaka_sim/live_input.py` own the untrusted
packet boundary. Validity, ordering, finite values, source identity where
configured, receive time, and freshness are established before the control
pipeline consumes a sample.

The arm uses the left index trigger. Engagement requires a release followed by
a fresh press; the press captures:

- the current Quest wrist pose;
- gravity-aligned head yaw;
- the current authoritative robot TCP/joint reference.

Subsequent head movement does not move the reference. In the confirmed operator
basis, Quest right/up/forward map to robot-base `-Y/+Z/+X`. Position is metres,
joint angles are radians, quaternions are explicitly labelled XYZW or WXYZ at
their boundary, and runtime liveness uses host monotonic timestamps.

The left grip controls only the RH56 path. Arm and hand clutch state are
independent.

## The acceptance boundary

`SmoothQuestJakaSession` advances a bounded SE(3) continuation and asks
`SharedJakaTargetGenerator` for a candidate. The generator starts each IK solve
from the last accepted branch and applies the current shared checks:

- pose residual and continuation progress;
- conservative joint bounds and software margin;
- branch/joint-step continuity;
- Jacobian condition, minimum singular value, and directional recovery;
- self-collision and the environment represented in the generator's base MJCF;
- accepted-output velocity and acceleration feasibility;
- the physical producer compute deadline when that budget is enabled.

Only a passing candidate becomes
`teleoperation.accepted_target.AcceptedArmTarget`. The immutable target carries
the J1--J6 position in radians, desired and filtered TCP poses, source/receive/
generation timestamps, clutch/reference generations, and acceptance
diagnostics.

This object is the last shared authority. Output adapters consume it without
changing its representation or recomputing feasibility.

## Recoverable rejection and hard faults

Candidate infeasibility and producer liveness are different states:

- A recoverable infeasible candidate creates no `AcceptedArmTarget`.
- The session sends a fresh `ArmControlHeartbeat` in `HOLD_REJECTED`.
- The output retains the last safe destination; the rejected candidate is not
  queued for later replay.
- A later feasible sample may recover without restarting the process.

Actual input/producer/IPC liveness loss, a controller or SDK fault, a hard
timing violation, or another terminal safety condition stops the physical
path. See [real hardware safety](../safety/REAL_HARDWARE_SAFETY.md) for the
complete distinction.

## Simulation and physical output

| Boundary | Simulation | Physical JAKA |
|---|---|---|
| Pre-adapter pipeline | Shared | Shared |
| Accepted representation | J1--J6 radians | Identical J1--J6 radians |
| Plant/output | MuJoCo accepted-joint adapter | Native ServoJ/EDG worker |
| Output timing | shaped 500 Hz default, or production-equivalent 125 Hz PWL | 8 ms native PWL/latest destination |
| IK after acceptance | None in adapter | None in adapter or joint-mode native worker |
| MuJoCo `qpos` authority | Simulation plant state | Never read or followed |

In current native `joint-teleop` mode, the JAKA `kine_inverse` call count must
remain zero. The native worker validates and emits the already accepted
absolute joint target. Its final position, velocity, acceleration, jerk,
tracking, timing, controller-state, liveness, and cleanup checks are defensive
physical boundaries; they are not a second planner.

The worker owns the sole JAKA SDK session. Lightweight controller status
polling occurs in that command worker. The rejected two-session monitor is
historical and must not be restored.

### Provisional table limitation

The live MuJoCo viewer/plant injects a provisional tabletop and mounting
members. `SharedJakaTargetGenerator` still loads the base MJCF named by the
configuration. Consequently, the injected table is not part of the shared
pre-acceptance collision authority. Plant contact with that table may be
visible in simulation, but it must not be described as a validated physical
workspace or collision guard.

## RH56 physical and simulation semantics

The maintained physical hand path is:

```text
RH56DFX -> PC-direct USB/RS485 -> RH56PcDirectWorker
```

It does not use JAKA tool-RS485. Canonical control order is
`[index, middle, ring, pinky, thumb_close, thumb_lateral]`; protocol order is
`[pinky, ring, middle, index, thumb_close, thumb_lateral]`.

The physical feedback fields have deliberately narrow meanings:

- `ANGLE_ACT`: measured feedback for the six commanded actuator axes;
- `CURRENT`: raw per-axis current register values;
- `FORCE_ACT`: raw per-axis load/force register values;
- `ERROR`: raw error values; nonzero is a hand fault;
- `STATUS`: raw status values whose nonzero code meanings are not validated.

These fields are not a full passive-joint state, tactile array, direct slip
sensor, or calibrated contact-force estimate. `CURRENT` and `FORCE_ACT` are
telemetry in the maintained PC-direct path, not invented force-safety limits.

The MuJoCo hand has six position actuators and twelve hand joints. Six equality
constraints couple thumb PIP/DIP motion to thumb close and each finger DIP to
its MCP. This is a kinematic approximation of six command axes. It does not
model tendon routing, compliance, backlash, motor/current control, calibrated
force limits, or the complete physical underactuation of RH56DFX.

## Logging and downstream integration

The current entries can preserve raw Quest packets, decoded control events,
accepted arm targets, 125 Hz emitted arm records, native metrics/cycle
telemetry, and RH56 feedback. These records share host monotonic time where the
runtime can establish it; device/source timestamps remain separately labelled.

`src/episode_dataset` provides an offline-tested episode layer shared by the
simulation path and the separately gated physical combined path. The physical
v2 wiring is not yet physically validated. Complete clock/geometry calibration,
a validated real multimodal dataset, and end-to-end ACT, Diffusion Policy, or
OpenPI training remain unavailable.

## Maintained entry points

The following help commands are non-motion inspection only:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

Running `--help`, tests, or plant-free checks does not authorize a hardware
connection. Current validation claims and unresolved work are summarized in
[current status](../status/current_status.md).

---

# 中文版：系统架构

## 范围和运行拓扑

本页描述当前 Quest-to-JAKA/RH56 实现，依据当前 source/config，不依据 `docs/history/` 中的旧设计。
真机路径只有在操作者单独授权并满足完整 gate 后才启用。

```text
Quest HTS/CTRL
  -> input validation / bounded queue
  -> release-before-press clutch/reference
  -> frame mapping / filter
  -> shared continuation IK / feasibility
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter 或 JAKA joint adapter
```

物理 adapter 与 MuJoCo adapter 在 `AcceptedArmTarget` 前共享同一目标生成路径。物理 adapter 不读
MuJoCo `qpos`，不重新映射、滤波、求 IK 或选择 branch；native joint mode 不调用 JAKA inverse kinematics。

相机 producer 只负责从 RealSense frameset 写入 bounded shared-memory ring；recorder/canonical sampler
负责因果选帧、JSONL 和 RGB MP4。camera、preview、writer 和 event log 不得阻塞 native control heartbeat。
RH56 通过独立 PC-direct serial worker 运行，arm/RH56 combined wrapper 只负责 session lifecycle 和
fault aggregation。

## 接受、拒绝和故障

candidate 的 finite、branch、joint range、collision、singularity、velocity/acceleration 和 continuation
检查属于共享 Python boundary；不通过的 candidate 不 commit，进入 `HOLD_REJECTED` 并保持最后 safe target
和 fresh heartbeat。transient Quest loss、clutch invalid、ordinary RH56 rate limit/contact clamp 和
录制质量事件不得无条件结束整个 process。

JAKA controller alarm、collision、E-stop、power/enable loss、SDK fault、native hard timing、watchdog/
liveness、tracking divergence、actual joint/output violation、branch/winding、RH56 nonzero ERROR 或
fatal serial/protocol，以及 cleanup 无法确定，保留 hard-stop path。

native worker 是实际 8 ms velocity/acceleration/jerk shaper 和 final hard-check authority；Python
predictor 不复制 native active segment，也不预测 jerk。JAKA controller 是最终硬件保护，但不是正常
轨迹筛选的唯一软件层。

## episode 和状态

有效 press 创建独立 episode，release 五秒 finalize；下一次 press 在同一 recorder process 中创建新
编号并重新初始化 clock、timeline、writer、trigger 和 quality state。Ctrl+C 或最后一次 fault 只影响
最后 partial episode，之前 finalized 的 episode 不再 outer cleanup 中重复 finalize。

当前 physical calibration、J4 collision 根因、latest acceleration fix、Quest recovery、dual-D435
同步和完整 Quest-driven RH56 仍按当前 status 标记为未完成或未物理验证。
