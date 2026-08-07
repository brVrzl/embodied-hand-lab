# Shared target pipeline

## Acceptance is the adapter boundary

`SmoothQuestJakaSession` consumes validated input, owns clutch/reference state,
advances a bounded continuation, and asks `SharedJakaTargetGenerator` for a
candidate. A candidate becomes an immutable
`teleoperation.accepted_target.AcceptedArmTarget` only after all shared checks
pass.

Checks cover:

- finite and fresh input;
- pose residual and continuation progress;
- joint limits, self/environment collision, and branch continuity;
- Jacobian singularity quality and directional recovery;
- accepted-output joint velocity;
- accepted-output joint acceleration, including the first 8 ms emitted step
  and replacement of an active native interpolation segment.

Joint velocity and acceleration have one shared policy source:
`shared_target_generation.maximum_output_joint_*`. The Python output
prefilter screens accepted-target replacement against that contract, and the
native worker checks the final emitted 8 ms segment at the hardware boundary.
There is no separate IK velocity/acceleration gate and no second MuJoCo
command velocity/acceleration configuration.

The native production adapter additionally applies the project-selected
`command_maximum_joint_jerk_rad_s3` as a low-latency transition shaper. It is
not claimed as a Mini2 hardware maximum and is not a replacement for the
final velocity, acceleration, timing, tracking, controller, or SDK hard-stop
checks.

The configured continuation preserves the remote maximum of five backtracks and a minimum
fraction of 1/32. Each retry starts from the last authoritative branch and
reuses the same safety gates; reaching the compute budget immediately returns
`CONTROL_COMPUTE_BUDGET_EXHAUSTED` and holds the last target. The output
boundaries are currently π rad/s velocity and 4π rad/s² acceleration. These
are shared policy values, not native post-processing conveniences.

## Rejection and liveness

Candidate feasibility and liveness are deliberately separate:

- A recoverable candidate rejection produces no `AcceptedArmTarget`.
- The session emits a fresh `ArmControlHeartbeat` in `HOLD_REJECTED`.
- The native worker holds the last safe emitted destination; a rejected target
  is not queued for later replay.
- Actual producer/process/IPC liveness loss or another hard fault safely stops.

The next feasible input can recover without restarting the control session.
This behavior is protected by shared-pipeline, singularity, output-feasibility,
and native-worker tests.

The live profile distinguishes a bounded Quest transport interruption from a
dead command producer. Invalid/stale CTRL or wrist data immediately disengages
the clutches and requests `HOLD_CURRENT`; for at most 10 seconds the still-live
Python producer may send only no-motion heartbeats. Returning input must observe
both controls released and then capture a fresh reference before motion can
resume. Exceeding the window is `QUEST_INPUT_RECOVERY_TIMEOUT` and enters a
persistent disengaged hold; it does not end the process or reuse the old reference.
If the Python producer or IPC path itself dies, no heartbeat exists and the
unchanged native 100 ms watchdog still stops.

## Singularity policy

Actual full Jacobian quality is authoritative. Slowdown and hard-rejection use
condition number and minimum singular value, with hysteresis and directional
classification (toward, tangent, or away). J5 proximity to 15° remains warning
metadata; it is not a fixed hard gate. A safe retreat can therefore remain
possible without weakening the hard Jacobian boundary.

## Adapters

`MujocoArmTargetAdapter` writes the accepted six joints into the simulation
plant. `JakaAcceptedJointTargetAdapter` serializes the same J1–J6 radians in
absolute mode. The physical adapter contains no coordinate mapping, filter, IK,
branch selector, or feasibility policy. Native checks remain defensive
assertions against transport or implementation defects.

---

# 中文版：共享目标管线

## `AcceptedArmTarget` 是适配器边界

`SmoothQuestJakaSession` 消费通过验证的输入，管理 clutch/参考状态，推进有界
continuation，并让 `SharedJakaTargetGenerator` 计算候选。候选只有通过全部共享检查后，
才会成为不可变的 `teleoperation.accepted_target.AcceptedArmTarget`。

共享检查包括：

- 输入有限且新鲜；
- 位姿残差和 continuation 进度；
- 关节限位、自碰撞/环境碰撞和分支连续性；
- Jacobian 奇异质量与方向恢复；
- 已接受输出的关节速度；
- 已接受输出的关节加速度，包括第一个 8 ms 输出点以及替换活动插值段的情况。

关节速度和加速度只有一个共享策略来源：
`shared_target_generation.maximum_output_joint_*`。Python output prefilter 按该契约检查
accepted target 的替换，native worker 在硬件边界检查最终 8 ms 输出段；不再单独配置
IK 速度/加速度门限，也不再配置第二套 MuJoCo command 速度/加速度门限。

当前 continuation 保持远程的最多五次回退，最小比例为 1/32。达到 compute budget 后立即进入
`HOLD_REJECTED`，不再为了更小步长耗尽当前周期。共享输出边界为速度 π rad/s、加速度
4π rad/s²；它们属于共享策略，不是 native 后处理参数。

## 拒绝与活性分离

- 可恢复候选被拒绝时不产生新的 `AcceptedArmTarget`。
- session 在 `HOLD_REJECTED` 中继续发布新鲜 `ArmControlHeartbeat`。
- native worker 保持最后一个安全输出目标，不会排队以后重放被拒绝的目标。
- 只有真正的 producer/process/IPC 活性丢失或硬故障才停止。

新的可行输入可以在不重启控制进程的情况下恢复。共享管线、奇异性、输出可行性和 native
worker 测试覆盖了这一行为。

live profile 会区分“Quest 数据短时失效”和“命令 producer 真正死亡”。CTRL 或 wrist
失效时立即 disengage 并请求 `HOLD_CURRENT`；仍存活的 Python producer 最多 10 秒只发送
无运动 heartbeat。数据恢复后必须先观察到两个 trigger 均释放，再重新按下采集新参考，
不会沿用旧参考跳回。超过窗口以 `QUEST_INPUT_RECOVERY_TIMEOUT` 进入 persistent
disengaged hold；进程不退出，也不会沿用旧参考。Python/IPC 真正死亡时无法发送
heartbeat，native 原有 100 ms watchdog 保持不变。

## 奇异性策略

完整 Jacobian 质量是权威依据。slowdown 和 hard reject 使用 condition number、
最小奇异值、滞回以及 toward/tangent/away 方向分类。J5 接近 15° 只作为 warning
元数据，不是固定硬门，因此在不放松 Jacobian 硬边界的前提下仍允许安全撤离。

## 输出适配器

`MujocoArmTargetAdapter` 把共享六关节目标写入仿真 plant；
`JakaAcceptedJointTargetAdapter` 只以 absolute mode 序列化相同的 J1–J6 弧度值。
真机适配器不包含坐标映射、滤波、IK、分支选择或可行性策略。native 检查只作为传输和
实现错误的最终防御断言。
