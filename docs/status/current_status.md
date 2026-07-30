# Current status

## What works

The Quest HTS/CTRL input boundary, release-before-press controller clutch,
fresh wrist/head/TCP reference capture, coordinate mapping, filters, bounded
continuation IK, Jacobian-based singularity handling, collision/limit/branch
checks, output velocity/acceleration feasibility, `HOLD_REJECTED`, immutable
accepted target, MuJoCo adapter, JAKA joint adapter, and native
latest-destination resampler are implemented and covered offline.

The physical producer now applies a 20 ms wall-time compute budget from the
start of each shared 60 Hz tick. The deadline is checked between IK iterations
and again before a candidate can become authoritative. An over-budget trial is
discarded, the last safe target remains authoritative, and the existing
`HOLD_REJECTED` heartbeat keeps the native command stream fresh. Simulation
sessions do not enable this physical-producer budget, so shaped 500 Hz and
JAKA-equivalent 125 Hz simulation target generation retain their existing
semantics. The `20260729_110336` event-level offline replay completed without a
producer-liveness stop; this correction has not yet been physically validated.

An independent RH56DFX PC-direct USB/RS485 gate is implemented with preferred
`/dev/serial/by-id/...` binding, an identity-checked custom-CH341 fallback,
read-only/hand-only/configuration approvals,
zero register writes on serial open, bounded command rate and delta, measured
`ANGLE_ACT` feedback, raw current/load/status/error reporting, and a deterministic
fake backend. Stage 1 now produces per-frame and summary read-only evidence;
Stage 2 is measured-relative and single-channel; Stage 3 reuses the shared Quest
router for hand-only operation. A formal combined entry reuses one Quest
receiver, one shared session, one JAKA SDK/native session, and the same
PC-direct hand controller, with arm/hand terminal-fault linkage. These additions
are offline tested. On 2026-07-29, the operator completed a 60 second Quest
hand-only run through this PC-direct path: 903 feedback records, 192 bounded
target writes, zero timeout/checksum/protocol errors, no hand fault, and zero
JAKA sessions. The combined JAKA + RH56 physical path remains unverified.

The first combined physical attempt on 2026-07-29 stopped after 27.34 seconds,
immediately after the first index release, with `joint target motion flag does
not match native execution mode`. `HOLD_CURRENT` existed in the wire enum but
had no native dispatch branch and was incorrectly validated as a normal joint
target. Native now performs bounded braking for `HOLD_CURRENT`, reports
`STOPPED_READY`, and the fresh disengaged shared session sends non-motion
producer heartbeats. The fix is offline-tested only.

The simulation entry now records joint arm/RH56 raw input and 60 Hz control
events, replays recorded CTRL clutches through the live router, and offers an
explicit simulation-only JAKA-equivalent 125 Hz arm output. That mode reuses
the production native PWL/transition implementation, bypasses the 500 Hz arm
command shaper, and retains 500 Hz MuJoCo physics plus the independent RH56
path. The existing shaped 500 Hz mode remains the default. Both modes are
offline tested; no new physical validation is claimed.

The live Quest/MuJoCo arm path and relative six-channel RH56 grip retargeting,
including the calibrated thumb-close and thumb-lateral model, are validated in
simulation. The integrated live configuration builds six JAKA and six RH56
actuators. The explicit JAKA-only model remains covered separately with exactly
six arm actuators and no hand command path. The default test suite and fake
native worker require no hardware.

The operator-aligned translation basis was corrected on 2026-07-29: operator
right/up/forward now map to robot-base `-Y/+Z/+X`. MuJoCo three-direction testing
and a subsequent bounded physical session were both observed by the operator to
move in the intended directions. Wrist orientation keeps its independent
anatomical basis, and RH56 retargeting is unchanged. The temporary
`physical_mapping_confirmed` configuration gate was removed after confirmation.

The live MuJoCo viewer now adds the existing provisional physical tabletop and
mounting members in robot-base world coordinates. The base stays upright at
identity, the table lies on base `+Y`, and offline FK confirms that J1=+90
degrees points the RH56 palm/TCP into the table workspace. This scene-only
installation does not change shared mapping, IK, accepted joints, or either arm
output adapter.

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

On 2026-07-29 a 60 s bounded post-payload command was stopped after 7.72 s by
the native sustained-output-hold policy. The controller reported no alarm,
collision, E-stop, tracking hard crossing, SDK failure, or hard timing miss;
selected output remained bounded and moving. The cause was a worker state
classification defect: every transition-limited point advanced the hold timer.
The worker now records progressing limited points separately and applies the
unchanged 2 s / 250-cycle escalation only to a real destination gap with no
material selected-command progress. The captured accepted targets pass the
corrected worker offline; subsequent physical runs exercised the corrected
classification without reproducing the false hold.

Later 2026-07-29 physical sessions exercised the corrected translation mapping.
The operator confirmed forward, right, and up moved the physical TCP in the
intended workspace directions, and progressing transition-limited output no
longer accumulated a false hold. The latest complete log
`quest_jaka_post_payload_manual_20260729_110336` is not an overall run PASS: it
ended after 38.224 s in the retained `command_stream_timeout` /
`producer_liveness_loss` hard stop. It recorded no RH56 commands and no true
output hold. Its tail contains a 138.211 ms continuation-IK computation and a
142.104 ms producer control gap while Quest wrist and controller input remained
fresh, immediately preceding the native liveness timeout. Direction mapping is
physically confirmed; full-envelope motion and producer-liveness robustness are
not.

Offline event-level replay of that input now bounds the shared control work to
20.405 ms maximum (20.365 ms p99.9), keeps the velocity, acceleration,
workspace, collision, singularity, and joint-limit gates enabled, and ends the
fake native worker by explicit operator stop rather than command-stream timeout.
This is regression evidence for the software correction, not a new physical
run or a claim that all future host scheduling stalls are impossible.

Four later combined JAKA/RH56 sessions on 2026-07-29 exercised the normal
combined entry. Two stopped at the retained native jerk hard boundary and are
not treated as software regressions here. One stopped after a Quest/CTRL input
gap triggered the retained stale-input/liveness policy. The latest session,
`quest_jaka_rh56_combined_20260729_170029_3454185`, stopped after 22.026 s at
`command_stream_timeout`: one control tick took 107.069 ms. Its three candidate
attempts recorded only about 4.177 ms, 4.736 ms, and 6 ms of actual IK/check
work; the third attempt's 96.301 ms wall time therefore contains an unaccounted
Python runtime suspension. The streaming physical path had also retained all
1208 already-persisted, deeply nested event records in memory. It now releases
each in-memory event after the complete JSONL line is written, removing this
episode-length-dependent GC workload without changing target generation,
safety checks, or either watchdog. This correction is offline-tested and has
not yet received a physical validation run.

## Repository and research state

PWL/root-cause-fix and the RH56 simulation hand implementation are merged into
`main`. Four superseded Quest worktrees and their local branches were removed
on 2026-07-28 after the user explicitly abandoned their working-tree-only
content. MoveIt, Ruckig, ACT/Thor, TeleDex, and repository cleanup remain remote
archives. Offline teleoperation-rearchitecture contracts are present for
research review but are not the production baseline. OpenPI remains a pinned
sibling checkout used only by the inference-only π0.5-DROID shadow path.

## Current normal physical entry

The current operator-facing entry is
`scripts/run_quest_jaka_rh56_teleop.sh`, the production JAKA + PC-direct RH56
combined gate. It uses normal arm limits (1.5 rad/s for J1-J3 and 1.2 rad/s for
J4-J6) and normal hand range/rate/delta rather than diagnostic restrictions.
All production safety boundaries remain active. Each run still requires both
exact approvals, E-stop access, a clear workspace, completed RH56 prerequisites,
a duration no greater than 60 seconds, and no automatic retry. Releasing left
index pauses only the arm; releasing grip holds only the hand; either may resume
without ending the process. `run_quest_jaka_bounded_teleop.sh` remains the
arm-only isolation gate, while the post-payload wrapper remains diagnostic.

---

# 中文版：当前状态

## 已实现并可用

Quest HTS/CTRL 输入边界、release-before-press clutch、新鲜 wrist/head/TCP 参考捕获、
坐标映射、滤波、有界 continuation IK、基于 Jacobian 的奇异性处理、碰撞/限位/分支检查、
输出速度/加速度可行性、`HOLD_REJECTED`、不可变已接受目标、MuJoCo/JAKA adapter 和
native latest-destination resampler 均已实现并有离线测试。

真机 producer 现在从每个共享 60 Hz tick 开始执行 20 ms wall-time 计算
budget。deadline 会在 IK 迭代之间以及 candidate 成为权威目标前再次检查。
超预算 trial 会被丢弃，上一安全目标仍为权威状态，并通过现有
`HOLD_REJECTED` heartbeat 保持 native command stream 新鲜。仿真 session 不启用
该真机 producer budget，因此 shaped 500 Hz 和 JAKA-equivalent 125 Hz 的仿真
target generation 语义不变。`20260729_110336` event-level 离线回放已不再
触发 producer-liveness 停止；该修复尚未进行新的真机验证。

已实现独立 RH56DFX PC-direct USB/RS485 gate：优先显式
`/dev/serial/by-id/...` 绑定，并支持身份核验的自定义 CH341 fallback；区分
read-only/hand-only/configuration 授权，
打开串口时零寄存器写入，并有 command rate/delta 限制、实测 `ANGLE_ACT`
feedback、raw current/load/status/error 和确定性 fake backend。Stage 1 会输出逐帧与
summary 只读证据；Stage 2 是 measured-relative 单通道测试；Stage 3 复用 shared Quest
router 做 hand-only。正式联合入口复用一个 Quest receiver、一个 shared session、一个
JAKA SDK/native session 和同一个 PC-direct hand controller，并实现 arm/hand terminal
fault 联动。实现已有离线测试；联合真机行为仍未验证。
2026-07-29 操作者已通过该 PC-direct 路径完成一次 60 秒 Quest hand-only：903 条 feedback、
192 次有界 target 写入、timeout/checksum/protocol error 均为 0、无 hand fault，JAKA
session 为 0。JAKA + RH56 联合真机路径仍未验证。

2026-07-29 首次联合真机运行在 27.34 秒、第一次释放 index 后立即以
`joint target motion flag does not match native execution mode` 停止。wire enum 已有
`HOLD_CURRENT`，但 native 缺少处理分支，误把它按普通 joint target 校验。当前 native 会对
`HOLD_CURRENT` 有界减速并发布 `STOPPED_READY`，shared session 在输入新鲜且 disengaged
时发送非运动 producer heartbeat。该修复仅完成离线测试。

仿真入口现已联合录制 arm/RH56 raw 输入和 60 Hz control event；回放会把录下的 CTRL
clutch 送入 live router，并提供显式选择的 simulation-only JAKA-equivalent 125 Hz arm
输出。该模式复用 production native PWL/transition 实现，旁路 500 Hz arm command
shaper，同时保留 500 Hz MuJoCo physics 与独立 RH56 路径。现有 shaped 500 Hz 仍为默认。
两种模式均只完成离线测试，不构成新的真机验证。

实时 Quest/MuJoCo 机械臂和相对式六通道 RH56 grip retarget（包括已标定的 thumb-close
与 thumb-lateral 模型）已通过仿真验证。集成实时配置构建 6 个 JAKA 与 6 个 RH56
actuator；显式 JAKA-only 模型另有测试，严格只有 6 个机械臂 actuator 且无手部 command
path。默认测试和 fake native worker 不需要硬件。

2026-07-29 已修正操作者同向时的平移基：操作者右/上/前现在分别映射到 robot-base
`-Y/+Z/+X`。MuJoCo 三方向测试及随后一次受限真机运行中，操作者均确认三个方向符合
预期。wrist orientation 继续使用独立的解剖基，RH56 retarget 不变；确认完成后已删除
临时 `physical_mapping_confirmed` 配置 gate。

实时 MuJoCo viewer 现会在 robot-base world 中加入已有 provisional 现场桌面和安装梁。
base 保持竖直 identity，桌面位于 base `+Y`；离线 FK 已确认 J1=+90 度时 RH56
palm/TCP 指向桌面工作区。该安装仅属于 scene，不改变 shared mapping、IK、accepted
joint 或任一 arm output adapter。

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

2026-07-29，一次配置为 60 秒的 post-payload 受限运行在 7.72 秒时被 native sustained
output hold 策略停止。控制器没有报告 alarm、collision、E-stop、tracking hard crossing、
SDK failure 或 hard timing miss；selected output 始终有界且持续运动。根因是 worker
状态分类错误：所有 transition-limited point 都推进了 hold timer。当前 worker 已将有进展
的 limited point 单独记录，原 2 秒/250 周期升级策略只作用于 destination 明显有差距且
selected command 没有实质进展的真正 hold。本次捕获的 accepted targets 已通过修复后
worker 离线回放；随后的真机运行也未复现该 false hold。

2026-07-29 随后的真机运行验证了修正后的平移方向；操作者确认前、右、上均使真机 TCP
沿预期工作区方向运动，有进展的 transition-limited 输出也不再误累计为 hold。最新完整
日志 `quest_jaka_post_payload_manual_20260729_110336` 不能作为整次运行 PASS：它在 38.224
秒时以保留的 `command_stream_timeout` / `producer_liveness_loss` 硬停止结束；该次没有 RH56
命令，也没有真正 output hold。尾部在 Quest wrist/controller 输入仍新鲜时出现一次
138.211 ms continuation IK 计算和 142.104 ms producer control gap，随后 native liveness
timeout。因此平移方向已获真机确认，但完整运动 envelope 和 producer-liveness 鲁棒性仍未
验证完成。

该输入的 event-level 离线回放现在将共享控制计算限制在最大
20.405 ms（p99.9 20.365 ms），速度、加速度、workspace、碰撞、奇异和
关节限位 gate 全部保持启用，fake native 最终由显式 operator stop 结束，而非
command-stream timeout。这是软件修复的回归证据，不是新的真机运行，也不声称
主机调度以后绝不会停顿。

随后 2026-07-29 的四次 JAKA/RH56 联合运行使用了正式联合入口。其中两次由保留的
native jerk hard boundary 停止，本轮不视为软件回归；一次因 Quest/CTRL 输入间断触发
保留的 stale-input/liveness 策略。最新
`quest_jaka_rh56_combined_20260729_170029_3454185` 在 22.026 秒因
`command_stream_timeout` 停止：单个 control tick 用时 107.069 ms。该 tick 三次
candidate 尝试记录的实际 IK/检查工作只有约 4.177 ms、4.736 ms 和 6 ms，第三次尝试
96.301 ms 的 wall time 因此包含一次未计入各阶段的 Python runtime 停顿。同时，真机
streaming 路径当时仍在内存中保留已写入 JSONL 的全部 1208 条大型嵌套 event。现在每条
完整 JSONL 写入后即释放对应内存 event，消除随 episode 长度增长的 GC 工作量；target
generation、全部安全检查与两个 watchdog 均未改变。该修复仅完成离线测试，尚未进行
新的真机验证。

## 仓库与研究状态

PWL/root-cause-fix 与 RH56 仿真手实现均已进入 `main`。四个已取代 Quest worktree
及其本地分支在用户明确放弃仅存在于 working tree 的内容后，于 2026-07-28 删除。
MoveIt、Ruckig、ACT/Thor、TeleDex 和 repository cleanup 仅保留远程归档。
离线 teleoperation rearchitecture 契约保留用于研究审阅，不是 production baseline。
OpenPI 仍是固定版本的 sibling checkout，仅用于 inference-only π0.5-DROID shadow。

## 当前正常真机入口

当前操作者入口是 production JAKA + PC-direct RH56 联合 gate
`scripts/run_quest_jaka_rh56_teleop.sh`。arm 使用 J1-J3 1.5 rad/s、J4-J6 1.2 rad/s，hand
使用正常 range/rate/delta，不继承诊断限制；所有 production 安全边界继续有效。每次仍需
arm/hand 两个精确授权、急停可触及、工作区清空、RH56 前置条件完成、最长 60 秒且禁止
自动重试。松开 left-index 只暂停 arm，松开 grip 只保持 hand，二者均可重新按下继续而不
结束程序。`run_quest_jaka_bounded_teleop.sh` 保留为 arm-only 隔离 gate，post-payload
wrapper 仅保留为诊断入口。
