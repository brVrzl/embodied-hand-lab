# JAKA Mini2 + RH56DFX combined teleoperation

This is the current normal physical teleoperation entry. The arm and hand use
their production operating envelopes rather than the temporary post-payload or
single-channel diagnostic restrictions. All production safety limits remain
active. The maintained command below is the same command used for the latest
combined run on 2026-08-03: it loaded the real RH56 calibration, accepted
10,035 arm targets, and stopped fail-closed after 234.32 seconds when Quest
input did not recover within the 10-second window. No arm/controller/RH56
transport fault occurred. This is operational evidence from one bounded physical run. 
It does not replace longer-duration validation or task-specific evidence.
parcel-box and tissue extraction outcomes were not completed in that run.

The first combined attempt on 2026-07-29 ran for 27.34 seconds and then exposed
a recoverable-clutch protocol defect: `HOLD_CURRENT` was misclassified as a
motion target without `ALLOW_MOTION`. The worker now handles it as bounded
braking, publishes `STOPPED_READY`, and keeps fresh non-motion producer
heartbeats while index is released. A later 2026-07-30 combined run exercised
this path but failed after 21.02 seconds at a separate retained arm hard-timing
gate; The recoverable-clutch path has since been corrected, but combined operation
still requires additional bounded physical validation.

```text
one Quest UDP receiver/router
  -> one SmoothQuestJakaSession
       left index -> shared 20 ms arm target generator -> AcceptedArmTarget
                  -> one native JAKA SDK/125 Hz PWL session
       left grip  -> shared relative RH56 retargeting
                  -> one PC-direct RH56 I/O worker/controller
```

The RH56 I/O worker prevents USB/RS485 latency from blocking the 60 Hz arm
producer. It does not duplicate the hand state machine. The next recommended physical operation has not yet been executed and should be
performed only in a new operator session after the documented runtime safety
prerequisites have been satisfied.
A stable `/dev/serial/by-id/...` path is preferred. On hosts where the custom
`usb_ch341` driver creates no by-id link, `/dev/ttyCH341USB<N>` is accepted only
with `--allow-direct-ch341-device` and exact VID:PID/driver identity checks.
If any required runtime safety prerequisite is not satisfied, neither hardware
path is opened and no physical command is issued.

Control semantics:

- Index released: no new arm `AcceptedArmTarget`; native holds current measured
  J1--J6. Wrist motion alone cannot move the arm.
- Index press: recapture wrist/head from current measured arm joints and resume
  continuously. Index release performs the existing bounded pause and
  `STOPPED_READY` synchronization; the process remains alive.
- Grip released: hand normally holds the last target, sends no new target, and
  does not open. If feedback-qualified contact detection observes a loaded
  channel while that held target remains active, the safety path may issue one
  bounded opening relief target; it is recorded as `CONTACT_RELIEF`. Grip press
  captures fresh measured `ANGLE_ACT` and resumes continuously.
- Index and grip are independent and may be active simultaneously. Arm pause
  does not reset or open the hand.
- Invalid/stale Quest CTRL or wrist input immediately pauses the arm and holds
  the hand. The live producer sends no-motion heartbeat for at most 10 seconds.
  If data returns, release both triggers and press again to capture fresh arm
  and hand references; if it does not, the session terminates. The native
  producer-death watchdog remains 100 ms.
- Hand transport/feedback/device fault makes the combined episode invalid and
  stops/holds the arm. An arm terminal hard fault stops new hand commands.

The entry uses the same normal production arm limits as
`run_quest_jaka_bounded_teleop.sh`: J1--J6 1.5 rad/s,
plus all shared/native position, workspace, velocity, acceleration, jerk,
tracking, controller, collision, stale, timing, and cleanup boundaries. It
does not use the post-payload diagnostic 1 rad/s limit. The hand retains its
0--1000 position range, physically selected `fast40` command profile, 0.05
delta limit, full normalized command domain, contact hold,
feedback/protocol/fault gates, and measured-first startup.
Both hand-only and combined physical entries load
`configs/hand/quest_rh56_real_retarget.yaml`, align the target to the current
Quest pose on grip press from measured activation, and enable the physically
validated index-pinch three-channel relationship. The simulation-only
uncalibrated mapping is rejected by physical entry assembly.
There is no unlimited or safety-disable option.

After completing Steps 1--8 in [RH56 operation](rh56_operation.md), inspect the
formal entry without connecting:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

Store the operator-confirmed robot, RH56, native-CPU, and Quest transport values
in the ignored `data/local/physical_collection.yaml` runtime config. The
explicit CH341 setting is kept in that config when the host exposes only the
identity-checked `/dev/ttyCH341USB<N>` fallback:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --runtime-config data/local/physical_collection.yaml \
  --hand-prerequisites-complete \
  --no-auto-retry --estop-accessible --workspace-clear \
  --log-dir logs
```

Executing this complete real-device command starts an operator-initiated,
bounded run for the current process. The operator must still verify current IP addresses,
payload/TCP/installation status, controller safety state, E-stop, workspace,
and the completed RH56 hand evidence before executing it. The runtime config
must contain an explicitly verified native control CPU. CPU 6 is the measured
low-load choice for the recorded 14-CPU host; it is not a portable robot
default and must be re-verified if the host or affinity set changes. The native
worker pins only its control thread there and moves Python, Quest, RH56, and
observer work to the remaining allowed CPUs. An unisolated combined invocation
is rejected before either hardware path opens. The wrapper creates HTS capture,
shared events, native metrics/cycle telemetry, event
extract, RH56 telemetry, and combined summary files under `logs/`.
When physical episode capture is enabled, the canonical dataset remains at
30 Hz and the diagnostic combined-event log is sampled at that same rate;
faults and clutch edges are still written immediately. The raw Quest, JAKA,
native, RH56, and dual-camera streams retain their source rates for offline
alignment.
The combined wrapper default and explicit upper bound are both 300 seconds;
arm-only and post-payload entries retain their existing shorter bounds. Every
run remains single-shot and requires `--no-auto-retry`.

When dual-D435 episode capture is enabled, the dataset runtime is process
isolated. Quest/mapping/IK/safety and the native target path remain in the
control process; each D435 has a producer process that writes only frames and
metadata into a preallocated versioned shared-memory ring. Canonical sampling,
frame materialization, and asynchronous writing run in the recorder process.
Preview, when enabled, is a separate latest-only consumer. Camera, recorder,
and preview children use ordinary scheduling and the verified non-native CPU
set; none may include `--native-control-cpu`.

This boundary is offline-tested only. Run the short software comparison with:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/validation/benchmark_combined_process_isolation.py --duration-sec 3
```

Do not use `--paced-seconds` from the older benchmark tool for this validation
and do not treat the software result as physical D435 evidence.

---

# 中文版：JAKA Mini2 + RH56DFX 联合遥操作

这是当前正常真机遥操作入口。arm 与 hand 使用 production 正常工作范围，不继承
post-payload 或单通道诊断的临时限制，但所有 production 安全边界仍保持启用。下方命令就是
2026-08-03 最近一次联合运行使用的最新入口：加载了真实 RH56 标定，接受 10,035 个 arm
目标，运行 234.32 秒后因 Quest 输入超过 10 秒未恢复而 fail-closed 停止；没有 arm、controller
或 RH56 transport fault。这只是部分证据，不是 300 秒 PASS；该次没有完成快递盒或抽纸抽出结果。

2026-07-29 首次联合运行持续 27.34 秒后暴露 recoverable clutch 协议缺陷：
`HOLD_CURRENT` 被误当成不带 `ALLOW_MOTION` 的运动 target。worker 现已将其处理为有界减速，
发布 `STOPPED_READY`，并在 index 释放且输入新鲜时维持非运动 producer heartbeat。
2026-07-30 后续 combined 运行覆盖了该路径，但在 21.02 秒因另一处保留的 arm hard-timing
gate 失败，因此 combined 仍未验证通过。

联合入口只有一个 Quest UDP receiver/router、一个 `SmoothQuestJakaSession`、一个 JAKA
SDK/native 125 Hz PWL session 和一个 PC-direct RH56 controller。RH56 I/O worker 只负责把
串口阻塞隔离出 60 Hz arm producer，不复制 hand clutch/state machine。两个精确授权和
RH56 设备身份会在任何硬件启动前校验；优先使用稳定 by-id。本机自定义 `usb_ch341` 没有
生成 by-id 时，只有显式提供 `--allow-direct-ch341-device` 且 VID:PID/driver 完全匹配，
才接受 `/dev/ttyCH341USB<N>`。缺任一授权时两边都不连接。

- index 未按：不生成新的 arm `AcceptedArmTarget`，native 保持当前实测 J1--J6；手腕移动
  不能让机械臂运动。
- index 按下：从当前实测关节连续重捕获；释放时按现有 bounded pause/`STOPPED_READY`
  同步，联合程序不退出。
- grip 未按/释放：hand 通常保持最后 target、不发新 target、不自动张开；如果反馈确认仍在
  持物且保持 target 导致接触负载上升，安全路径最多发送一次有界开向 `CONTACT_RELIEF`，
  并单独记录；新 grip press 从 fresh measured `ANGLE_ACT` 连续恢复。
- index 与 grip 完全独立，可以同时 ACTIVE；arm pause 不重置或张开 hand。
- Quest CTRL/wrist 失效时立即 pause arm 并 hold hand；仍存活的 producer 最多 10 秒只发
  无运动 heartbeat。数据恢复后先释放两个 trigger，再按下所需 clutch 重采 arm/hand
  参考；超过窗口则终止。native producer-death watchdog 仍为 100 ms。
- hand transport/feedback/device fault 会使 episode invalid 并让 arm 安全 hold/stop；arm
  terminal hard fault 会停止 hand 新命令。

入口复用 arm normal production 的 J1--J6 1.5 rad/s，以及所有关节/
workspace/速度/加速度/jerk/tracking/controller/collision/stale/timing/cleanup 安全边界；不使用
post-payload 的临时 1 rad/s。hand 保留 0--1000、已完成真机选择的 `fast40` profile、
0.05 delta、完整归一化 command domain、接触保持和全部
feedback/protocol/fault gate。没有 unlimited 或 disable-safety 参数。
hand-only 与 combined 真机入口统一加载
`configs/hand/quest_rh56_real_retarget.yaml`，每次 grip press 从实测激活值向当前 Quest
姿态对齐，并启用已完成真机验证的 index-pinch 三通道关系；真机配置装配会拒绝
simulation uncalibrated calibration。

正常联合真机运行使用上文命令模板。CPU6 是当前 14 核主机实测的低负载选择，并非可移植的
机器人默认值；主机或 affinity 变化后必须重新核实。入口会把 native control thread 固定到
该 CPU，并把 Python、Quest、RH56 和 observer 任务移到其余允许 CPU；未显式指定
`--native-control-cpu` 的 combined 调用会在打开任一硬件前拒绝。
仅进程级绑核不能排除普通调度任务造成的 4 ms tick 延迟。正式入口还会为 native 8 ms
control thread 使用固定 `SCHED_FIFO` priority 10；SDK helper、Python、Quest、RH56 和
observer 仍为普通调度，control thread 在 cleanup/日志序列化前恢复为 `SCHED_OTHER`。
运行前必须在同一会话确认：

```bash
ulimit -r
```

结果至少为 `10`；否则入口会在任何 JAKA/RH56 I/O 前拒绝。RT limit 必须由管理员单独授权，
并在新登录会话中继承；不要用 root 身份运行整套 combined 程序，也不要通过提高 priority
或修改 timing threshold 来绕过预检。
该模板不是某次运行的授权；操作者仍需核实 IP、
payload/TCP/安装、控制器安全状态、急停、工作区和已完成的 RH56 hand 证据。日志会写入
`logs/` 下的 HTS、shared events、native metrics/cycles、
event extract、RH56 telemetry 和 combined summary。
启用真机 episode 采集时，canonical dataset 仍为 30 Hz，诊断用 combined-event
日志也按 30 Hz 记录；fault 和 clutch 边沿仍立即写入。Quest、JAKA、native、RH56
以及双相机原始流保留各自采样率，供离线对齐。
combined wrapper 默认时长与显式上限均为 300 秒；arm-only 与 post-payload 入口保持原有
较短上限。每段仍为 single-shot，且必须使用 `--no-auto-retry`。

启用双 D435 episode 采集后，数据路径改为进程隔离：Quest/mapping/IK/safety 和
native target 路径仍在 control process；每台 D435 各自运行 producer process，只把帧和
metadata 写入预分配的 versioned shared-memory ring。canonical sampler、frame
materialization 和异步写盘都在 recorder process；preview（若启用）是独立的
latest-only consumer。camera、recorder、preview 只使用不包含
`--native-control-cpu` 的普通调度 CPU 集合，不使用 `SCHED_FIFO`。

该边界目前只完成离线验证。短时软件对照命令为：

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/validation/benchmark_combined_process_isolation.py --duration-sec 3
```

不要在本轮使用旧 benchmark 工具的 `--paced-seconds`，软件结果也不能当作双 D435
真机证据。
