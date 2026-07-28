# Current status

## What works

The Quest HTS/CTRL input boundary, release-before-press controller clutch,
fresh wrist/head/TCP reference capture, coordinate mapping, filters, bounded
continuation IK, Jacobian-based singularity handling, collision/limit/branch
checks, output velocity/acceleration feasibility, `HOLD_REJECTED`, immutable
accepted target, MuJoCo adapter, JAKA joint adapter, and native
latest-destination resampler are implemented and covered offline.

The live Quest/MuJoCo arm path and relative six-channel RH56 grip retargeting,
including the calibrated thumb-close and thumb-lateral model, are validated in
simulation. The integrated live configuration builds six JAKA and six RH56
actuators. The explicit JAKA-only model remains covered separately with exactly
six arm actuators and no hand command path. The default test suite and fake
native worker require no hardware.

## Latest physical evidence

A historical larger Quest/JAKA run produced a J4 servo collision alarm after
approximately 128 mm Quest/TCP displacement with substantial wrist motion. The
cause is unresolved. The operator subsequently reported applying payload
0.8 kg and COM `[9.289, 12.427, 36.961]` mm.

A synchronous/second-session health instrumentation attempt produced no motion:
the second SDK login prevented the primary worker from reaching `CONNECTED`.
The design was replaced with lightweight health polling on the sole SDK session.
A later bounded physical run completed 27.09 s and 3377 command ticks with no
timing warnings, hard misses, or controller alarm, validating that polling
timing path in that envelope. It stopped before a J4 point because the replayed
accepted targets contained controller-visible acceleration of
14.199679 rad/s².

The production `root_cause_fix` baseline adds a shared 4π rad/s²
output-acceleration gate before
`AcceptedArmTarget` construction. Offline replay now produces a safe
`HOLD_REJECTED` and recovers on the next feasible tick. That correction has not
yet been physically validated. TCP remains recorded as zero.

## Repository and research state

PWL/root-cause-fix and the RH56 simulation hand implementation are merged into
`main`. Four superseded Quest worktrees and their local branches were removed
on 2026-07-28 after the user explicitly abandoned their working-tree-only
content. MoveIt, Ruckig, ACT/Thor, TeleDex, and repository cleanup remain remote
archives. Offline teleoperation-rearchitecture contracts are present for
research review but are not the production baseline. OpenPI remains a pinned
sibling checkout used only by the inference-only π0.5-DROID shadow path.

## Exact next bounded physical gate

Open a new Codex session and obtain explicit authorization for a bounded repeat
of the post-payload diagnostic after the acceleration fix:

- maximum about 30 seconds in a known healthy posture;
- verify controller payload/COM, installation, zero TCP record, unchanged
  safety limits, alarms, workspace, and stop access without writing settings;
- release before press and confirm a still, jump-free first engagement;
- one gentle forward-and-return translation followed, separately, by one
  modest single-axis wrist motion;
- confirm any acceleration-infeasible candidate stays `HOLD_REJECTED`, recovery
  is immediate on a feasible retreat, native defensive acceleration rejection
  remains zero, tracking/timing/health stay bounded, and release stops/cleans
  up;
- preserve accepted/emitted targets, metrics, controller state, and stop reason.

Do not repeat the approximately 128 mm multi-axis/large-wrist run, combine axes,
approach a hard singularity, change payload/TCP/controller settings, or expand
the envelope in the same gate. This maintenance session does not authorize the
test.

---

# 中文版：当前状态

## 已实现并可用

Quest HTS/CTRL 输入边界、release-before-press clutch、新鲜 wrist/head/TCP 参考捕获、
坐标映射、滤波、有界 continuation IK、基于 Jacobian 的奇异性处理、碰撞/限位/分支检查、
输出速度/加速度可行性、`HOLD_REJECTED`、不可变已接受目标、MuJoCo/JAKA adapter 和
native latest-destination resampler 均已实现并有离线测试。

实时 Quest/MuJoCo 机械臂和相对式六通道 RH56 grip retarget（包括已标定的 thumb-close
与 thumb-lateral 模型）已通过仿真验证。集成实时配置构建 6 个 JAKA 与 6 个 RH56
actuator；显式 JAKA-only 模型另有测试，严格只有 6 个机械臂 actuator 且无手部 command
path。默认测试和 fake native worker 不需要硬件。

## 最新真机证据

一次较大的 Quest/JAKA 历史运行在约 128 mm Quest/TCP 位移和较大 wrist 运动后触发了
J4 servo collision alarm，原因未完全确定。之后操作者报告已应用 payload 0.8 kg 和
COM `[9.289, 12.427, 36.961]` mm。

双 SDK 会话健康监控尝试没有产生运动：第二次登录阻止主 worker 进入 `CONNECTED`。
设计已改为在唯一 SDK 会话中做轻量轮询。后续受限真机运行持续 27.09 秒、3377 个命令
tick，无时序 warning/hard miss/控制器报警，验证了该范围内的轮询时序。但运行在 J4
目标前停止，因为已接受目标包含 14.199679 rad/s² 的控制器可见加速度。

production `root_cause_fix` 基线在构造 `AcceptedArmTarget` 前增加共享 4π rad/s² 输出
加速度 gate。离线回放
会进入安全 `HOLD_REJECTED`，并在下一可行 tick 恢复。此修复尚未完成真机验证；TCP 仍
记录为零。

## 仓库与研究状态

PWL/root-cause-fix 与 RH56 仿真手实现均已进入 `main`。四个已取代 Quest worktree
及其本地分支在用户明确放弃仅存在于 working tree 的内容后，于 2026-07-28 删除。
MoveIt、Ruckig、ACT/Thor、TeleDex 和 repository cleanup 仅保留远程归档。
离线 teleoperation rearchitecture 契约保留用于研究审阅，不是 production baseline。
OpenPI 仍是固定版本的 sibling checkout，仅用于 inference-only π0.5-DROID shadow。

## 下一受限真机 Gate

需要新的显式授权，最多约 30 秒，在已知健康姿态执行 post-payload diagnostic：

- 只读取并确认 payload/COM、安装、零 TCP 记录、安全限制、报警、工作区和 stop access；
- release-before-press，首次 engage 静止且无跳变；
- 一次轻柔 forward-and-return 平移，然后单独一次小幅单轴 wrist；
- 加速度不可行候选必须保持 `HOLD_REJECTED`，可行撤退后立即恢复；
- native defensive acceleration reject 应保持为零，tracking/timing/health 有界；
- release 必须停止并清理；
- 保存 accepted/emitted target、metrics、控制器状态和停止原因。

当前手动入口见[真机遥操作文档](../operation/jaka_arm_teleoperation.md)。

不要重复约 128 mm 多轴/大 wrist 运行，不要组合轴、接近硬奇异点、修改 payload/TCP/
控制器设置或在同一 gate 扩大范围。阅读文档不构成授权。
