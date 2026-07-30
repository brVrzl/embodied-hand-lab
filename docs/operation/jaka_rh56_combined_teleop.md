# JAKA Mini2 + RH56DFX combined teleoperation

This is the current normal physical teleoperation entry. The arm and hand use
their production operating envelopes rather than the temporary post-payload or
single-channel diagnostic restrictions. All production safety limits remain
active. RH56 Quest hand-only completed a 60 second physical run on 2026-07-29.
After the timing fixes described below, a 60 second `fast40` combined run
completed on 2026-07-30 with no arm, hand, heartbeat, controller, or transport
fault. Five-minute combined operation remains physically unvalidated.

The first combined attempt on 2026-07-29 ran for 27.34 seconds and then exposed
a recoverable-clutch protocol defect: `HOLD_CURRENT` was misclassified as a
motion target without `ALLOW_MOTION`. The worker now handles it as bounded
braking, publishes `STOPPED_READY`, and keeps fresh non-motion producer
heartbeats while index is released. A later 2026-07-30 combined run exercised
this path but failed after 21.02 seconds at a separate retained arm hard-timing
gate; combined operation therefore remains unvalidated.

```text
one Quest UDP receiver/router
  -> one SmoothQuestJakaSession
       left index -> shared 20 ms arm target generator -> AcceptedArmTarget
                  -> one native JAKA SDK/125 Hz PWL session
       left grip  -> shared relative RH56 retargeting
                  -> one PC-direct RH56 I/O worker/controller
```

The RH56 I/O worker prevents USB/RS485 latency from blocking the 60 Hz arm
producer. It does not duplicate the hand state machine. Both exact approvals
and the RH56 device identity are validated before either hardware path starts.
A stable `/dev/serial/by-id/...` path is preferred. On hosts where the custom
`usb_ch341` driver creates no by-id link, `/dev/ttyCH341USB<N>` is accepted only
with `--allow-direct-ch341-device` and exact VID:PID/driver identity checks.
Missing either approval connects neither device.

Control semantics:

- Index released: no new arm `AcceptedArmTarget`; native holds current measured
  J1--J6. Wrist motion alone cannot move the arm.
- Index press: recapture wrist/head from current measured arm joints and resume
  continuously. Index release performs the existing bounded pause and
  `STOPPED_READY` synchronization; the process remains alive.
- Grip released: hand holds the last target, sends no new target, and does not
  open. Grip press captures fresh measured `ANGLE_ACT` and resumes continuously.
- Index and grip are independent and may be active simultaneously. Arm pause
  does not reset or open the hand.
- Hand transport/feedback/device fault makes the combined episode invalid and
  stops/holds the arm. An arm terminal hard fault stops new hand commands.

The entry uses the same normal production arm limits as
`run_quest_jaka_bounded_teleop.sh`: J1--J3 1.5 rad/s and J4--J6 1.2 rad/s,
plus all shared/native position, workspace, velocity, acceleration, jerk,
tracking, controller, collision, stale, timing, and cleanup boundaries. It
does not use the post-payload diagnostic 1 rad/s limit. The hand retains its
0--1000 position range, physically selected `fast40` command profile, 0.05 delta limit, configured 0.8
closure boundary, feedback/protocol/fault gates, and measured-first startup.
There is no unlimited or safety-disable option.

After completing Steps 1--8 in [RH56 operation](rh56_operation.md), inspect the
formal entry without connecting:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

Set `RH56_DEVICE` to the operator-confirmed path before a normal combined run.
The explicit CH341 flag is harmless for a by-id path and required for the
identity-checked `/dev/ttyCH341USB<N>` fallback:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --rh56-device "$RH56_DEVICE" \
  --allow-direct-ch341-device \
  --duration-sec 300 \
  --arm-approval I_AUTHORIZE_BOUNDED_NORMAL_QUEST_JAKA_TELEOPERATION \
  --hand-approval I_AUTHORIZE_ONE_JAKA_RH56_PC_DIRECT_COMBINED_RUN \
  --hand-prerequisites-complete \
  --no-auto-retry --estop-accessible --workspace-clear \
  --rh56-scheduler-profile fast40 \
  --log-dir logs
```

This command is a template, not authorization. The operator must verify current
IP addresses, payload/TCP/installation status, controller safety state, E-stop,
workspace, and the completed RH56 hand evidence before executing it. The wrapper
creates HTS capture, shared events, native metrics/cycle telemetry, event
extract, RH56 telemetry, and combined summary files under `logs/`.
The combined wrapper default and explicit upper bound are both 300 seconds;
arm-only and post-payload entries retain their existing shorter bounds. Every
run remains single-shot and requires `--no-auto-retry`.

---

# 中文版：JAKA Mini2 + RH56DFX 联合遥操作

这是当前正常真机遥操作入口。arm 与 hand 使用 production 正常工作范围，不继承
post-payload 或单通道诊断的临时限制，但所有 production 安全边界仍保持启用。2026-07-29
RH56 Quest hand-only 已完成一次 60 秒真机运行。完成下述 timing 修复后，2026-07-30
一次 60 秒 `fast40` combined 运行无 arm、hand、heartbeat、controller 或 transport fault
并正常结束；5 分钟 combined 仍未完成真机验证。

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
- grip 未按/释放：hand 保持最后 target、不发新 target、不自动张开；新 grip press 从 fresh
  measured `ANGLE_ACT` 连续恢复。
- index 与 grip 完全独立，可以同时 ACTIVE；arm pause 不重置或张开 hand。
- hand transport/feedback/device fault 会使 episode invalid 并让 arm 安全 hold/stop；arm
  terminal hard fault 会停止 hand 新命令。

入口复用 arm normal production 的 J1--J3 1.5 rad/s、J4--J6 1.2 rad/s，以及所有关节/
workspace/速度/加速度/jerk/tracking/controller/collision/stale/timing/cleanup 安全边界；不使用
post-payload 的临时 1 rad/s。hand 保留 0--1000、已完成真机选择的 `fast40` profile、
0.05 delta、0.8 closure 和全部
feedback/protocol/fault gate。没有 unlimited 或 disable-safety 参数。

正常联合真机运行使用上文命令模板。该模板不是某次运行的授权；操作者仍需核实 IP、
payload/TCP/安装、控制器安全状态、急停、工作区和已完成的 RH56 hand 证据。日志会写入
`logs/` 下的 HTS、shared events、native metrics/cycles、
event extract、RH56 telemetry 和 combined summary。
combined wrapper 默认时长与显式上限均为 300 秒；arm-only 与 post-payload 入口保持原有
较短上限。每段仍为 single-shot，且必须使用 `--no-auto-retry`。
