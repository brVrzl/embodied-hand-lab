# JAKA arm teleoperation

The maintained physical collection entry point is
`scripts/run_quest_jaka_rh56_teleop.sh`. It combines the live JAKA/RH56
teleoperation loop with the configured episode recorder; there is no separate
normal physical-teleoperation entry. Inspecting help is safe:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

The underlying tool still has explicitly separated diagnostic and combined
stages (`p2-shadow`, `e2-isolated`, `p4-live`, `post-payload-diagnostic`,
`bounded-normal-teleop`, and `combined-normal-teleop`). The arm-only bounded
stage is retained for isolation diagnostics, not as a second normal operator
entry. Never copy an old historical invocation; always reconcile it with the
current `--help`, current configuration, and the maintained collection entry
before any physical operation.

## Current runtime contract

- Target generation is 60 Hz. The default native output is 125 Hz (8 ms),
  `servo_step_num: 1`.
- Physical output consumes the shared immutable accepted six-joint target.
- Joint-teleop mode performs zero native JAKA IK calls.
- Output is absolute `edg_servo_j`; the default uses `step_num=1`.
- The explicitly selected experimental
  `hardware_adapter.transport_mode: jaka_62_5hz_step2` mode uses 62.5 Hz,
  `servo_step_num=2`, and a 16 ms ServoJ period. It is not the default and
  requires its own bounded physical validation.
- Resampling is piecewise-linear toward the latest destination with no stale
  queue replay.
- Production PWL destination changes use a short jerk-limited velocity
  transition (`command_maximum_joint_jerk_rad_s3`); this is a project-selected
  smoothing value, not a claimed Mini2 hardware limit.
- Post-EDG `q_hold` is authoritative and first engagement must be continuous.
- `HOLD_REJECTED` holds the last safe target with a fresh heartbeat.
- A transient Quest CTRL/wrist fault immediately holds output and permits at
  most 10 seconds for data recovery plus release-before-press re-reference.
  Recovery sends no motion target. A longer loss enters persistent disengaged
  hold; actual producer/process/IPC liveness loss, controller alarm, collision,
  SDK error, or hard timing failure stops and cleans up.
- The sole SDK session performs lightweight health polling every two command
  cycles; extended collision/estop queries occur only after unhealthy status.
- During joint-teleop startup, the worker sends only the captured `q_hold`.
  A bounded grace window (25 cycles by default) records isolated sub-period
  wake/completion lateness and re-aligns the absolute schedule without catch-up
  or backlog. Lateness beyond the 12 ms completion hard boundary, persistent
  misses after grace and controller/SDK faults remain hard stops. Quest input
  loss becomes a persistent disengaged hold after the bounded recovery window;
  producer/IPC loss remains governed by the unchanged 100 ms native watchdog.

See [current status](../status/current_status.md) before proposing a physical
stage. The next recommended physical operation has not yet been executed and should be
performed only in a new operator session after the documented runtime safety
prerequisites have been satisfied.

## Current physical collection command

The current operator-facing wrapper is the JAKA + RH56 collection entry.
Inspecting help does not connect:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

The exact command and required physical prerequisites are documented in
[JAKA + RH56 combined teleoperation](jaka_rh56_combined_teleop.md). It reuses
the configured PWL arm path, the 20 ms producer compute budget, and the
PC-direct RH56 controller. Left index pauses/resumes only the arm; grip
holds/resumes only the hand. It does not use the post-payload 1 rad/s or hand
single-channel diagnostic restrictions. Production velocity, acceleration,
jerk, workspace, tracking, stale, collision, protocol, feedback, and cleanup
boundaries remain active.

`run_quest_jaka_bounded_teleop.sh` remains only the arm-only isolation entry. It
sends zero RH56 commands and is not a normal collection entry.

---

# 中文版：JAKA 机械臂遥操作

当前维护的真机采集入口是
`scripts/run_quest_jaka_rh56_teleop.sh`，它把 JAKA/RH56 联合遥操作和
episode recorder 组合在一起；没有另一个普通真机遥操作入口。安全的帮助检查：

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

底层工具仍明确分成 `p2-shadow`、`e2-isolated`、`p4-live`、
`post-payload-diagnostic`、`bounded-normal-teleop` 和
`combined-normal-teleop`。其中 arm-only bounded stage 只保留用于隔离诊断，
不是第二个普通操作者入口。诊断、配置写入和采集入口继续保留各自的运行时安全前置条件。不得直接复制历史命令，必须与当前 `--help`、当前配置以及维护中的采集入口保持一致。

## 当前运行契约

- 共享目标生成 60 Hz。默认 native 输出为 125 Hz（8 ms），`servo_step_num: 1`。
- 真机消费共享不可变六关节目标。
- joint-teleop 模式 native JAKA IK 调用数为零。
- 输出为 absolute `edg_servo_j`，默认使用 `step_num=1`。
- 显式选择 `hardware_adapter.transport_mode: jaka_62_5hz_step2` 时，使用
  62.5 Hz、`servo_step_num=2` 和 16 ms ServoJ 周期。该模式不是默认值，
  必须单独进行有界真机验证。
- 分段线性重采样靠近 latest destination，不重放旧队列。
- 进入 EDG 后的 `q_hold` 是启动权威，首次 engage 必须连续。
- `HOLD_REJECTED` 使用新鲜 heartbeat 保持最后安全目标。
- Quest CTRL/wrist 短时失效会立即保持输出，最多等待 10 秒；恢复后必须 release-before-
  press 重采参考，期间不发送运动 target。超过窗口、producer/IPC 真正失活、控制器报警、
  碰撞、SDK 或硬时序错误会停止并清理。native 100 ms producer watchdog 保持不变。
- 唯一 SDK 会话每两个命令周期执行一次轻量健康轮询。

在提出真机 gate 前先阅读[当前状态](../status/current_status.md)。当前推荐 gate 仍需操作者发起的独立运行，并满足全部运行时安全前置条件。

## 当前真机采集命令入口

以下 `--help` 不会连接真机：

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

精确命令与双授权见[JAKA + RH56 联合遥操作](jaka_rh56_combined_teleop.md)。该采集入口复用
configured PWL arm、20 ms producer compute budget 和 PC-direct RH56 controller。
left-index 只暂停/恢复 arm，grip 只保持/恢复 hand。正常入口不使用 post-payload 1 rad/s
或 hand 单通道诊断限制，但 production 的速度、加速度、jerk、workspace、tracking、stale、
collision、协议、feedback 和 cleanup 边界全部保留。

`run_quest_jaka_bounded_teleop.sh` 仅保留为 arm-only 隔离入口，发送零 RH56 命令，不是
普通采集入口。
