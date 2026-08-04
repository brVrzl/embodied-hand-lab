# JAKA arm teleoperation

The current physical entry point is `tools/quest_jaka_hardware.py`. It is not a
normal quick-start command. Inspecting help is safe:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

Its stages are deliberately separated (`p2-shadow`, `e2-isolated`, `p4-live`,
`post-payload-diagnostic`, `bounded-normal-teleop`, and
`combined-normal-teleop`) because they represent different operating modes.
Diagnostic, configuration-write, and normal teleoperation continue to enforce
their own runtime safety prerequisites. Never copy an old historical invocation;
always reconcile it with the current `--help`, current configuration, and the
maintained real-device entry before any physical operation.

## Current runtime contract

- Target generation is 60 Hz; native output is approximately 125 Hz (8 ms).
- Physical output consumes the shared immutable accepted six-joint target.
- Joint-teleop mode performs zero native JAKA IK calls.
- Output is absolute `edg_servo_j(..., ABS, 1)`.
- Resampling is piecewise-linear toward the latest destination with no stale
  queue replay.
- Production PWL destination changes use a short jerk-limited velocity
  transition (`command_maximum_joint_jerk_rad_s3`); this is a project-selected
  smoothing value, not a claimed Mini2 hardware limit.
- Post-EDG `q_hold` is authoritative and first engagement must be continuous.
- `HOLD_REJECTED` holds the last safe target with a fresh heartbeat.
- A transient Quest CTRL/wrist fault immediately holds output and permits at
  most 10 seconds for data recovery plus release-before-press re-reference.
  Recovery sends no motion target. A longer loss, actual producer/IPC liveness
  loss, controller alarm, collision, SDK error, or hard timing failure stops
  and cleans up.
- The sole SDK session performs lightweight health polling every two command
  cycles; extended collision/estop queries occur only after unhealthy status.
- During joint-teleop startup, the worker sends only the captured `q_hold`.
  A bounded grace window (25 cycles by default) records isolated sub-period
  wake/completion lateness and re-aligns the absolute schedule without catch-up
  or backlog. Lateness beyond the 12 ms completion hard boundary, persistent
  misses after grace and controller/SDK faults remain hard stops. Quest input
  loss becomes terminal only after the bounded recovery window; producer/IPC
  loss remains governed by the unchanged 100 ms native watchdog.

See [current status](../status/current_status.md) before proposing a physical
stage. The next recommended physical operation has not yet been executed and should be
performed only in a new operator session after the documented runtime safety
prerequisites have been satisfied.

## Current normal combined command entry

The current operator-facing physical wrapper is the normal JAKA + RH56 entry.
Inspecting help does not connect:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

The exact command and required physical prerequisites are documented in
[JAKA + RH56 combined teleoperation](jaka_rh56_combined_teleop.md). It reuses
the production 8 ms PWL arm path, the 20 ms producer compute budget, and the
PC-direct RH56 controller. Left index pauses/resumes only the arm; grip
holds/resumes only the hand. It does not use the post-payload 1 rad/s or hand
single-channel diagnostic restrictions. Production velocity, acceleration,
jerk, workspace, tracking, stale, collision, protocol, feedback, and cleanup
boundaries remain active.

`run_quest_jaka_bounded_teleop.sh` remains the arm-only isolation entry. It
sends zero RH56 commands and is no longer the normal combined operator entry.

---

# 中文版：JAKA 机械臂遥操作

当前真机入口是 `tools/quest_jaka_hardware.py`，它不是普通 quick start。安全的帮助检查：

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

stage 被明确分成 `p2-shadow`、`e2-isolated`、`p4-live`、
`post-payload-diagnostic`、`bounded-normal-teleop` 和
`combined-normal-teleop`，分别对应不同运行模式。诊断、配置写入和正常遥操作继续保留各自的运行时安全前置条件。不得直接复制历史命令，必须与当前 `--help`、当前配置以及维护中的真机入口保持一致。

## 当前运行契约

- 共享目标生成 60 Hz，native 输出约 125 Hz（8 ms）。
- 真机消费共享不可变六关节目标。
- joint-teleop 模式 native JAKA IK 调用数为零。
- 输出为 absolute `edg_servo_j(..., ABS, 1)`。
- 分段线性重采样靠近 latest destination，不重放旧队列。
- 进入 EDG 后的 `q_hold` 是启动权威，首次 engage 必须连续。
- `HOLD_REJECTED` 使用新鲜 heartbeat 保持最后安全目标。
- Quest CTRL/wrist 短时失效会立即保持输出，最多等待 10 秒；恢复后必须 release-before-
  press 重采参考，期间不发送运动 target。超过窗口、producer/IPC 真正失活、控制器报警、
  碰撞、SDK 或硬时序错误会停止并清理。native 100 ms producer watchdog 保持不变。
- 唯一 SDK 会话每两个命令周期执行一次轻量健康轮询。

在提出真机 gate 前先阅读[当前状态](../status/current_status.md)。当前推荐 gate 仍需操作者发起的独立运行，并满足全部运行时安全前置条件。

## 当前正常联合命令入口

以下 `--help` 不会连接真机：

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

精确命令与双授权见[JAKA + RH56 联合遥操作](jaka_rh56_combined_teleop.md)。该入口复用
production 8 ms PWL arm、20 ms producer compute budget 和 PC-direct RH56 controller。
left-index 只暂停/恢复 arm，grip 只保持/恢复 hand。正常入口不使用 post-payload 1 rad/s
或 hand 单通道诊断限制，但 production 的速度、加速度、jerk、workspace、tracking、stale、
collision、协议、feedback 和 cleanup 边界全部保留。

`run_quest_jaka_bounded_teleop.sh` 保留为 arm-only 隔离入口，发送零 RH56 命令，不再是
正常联合操作者入口。
