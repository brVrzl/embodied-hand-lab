# Physical hardware prerequisites

This page is a prerequisite checklist, not authorization to run hardware.
Every physical gate needs explicit user authorization in a new/current session
for its exact stage, duration, motion bound, and acknowledgement flags.

Before any connection or command:

1. Confirm the intended worktree, branch, commit, clean task-owned diff, config,
   and executable hashes.
2. Confirm no other control client is logged in and no stale process remains.
3. Inspect the robot, hand, cabling, workspace, fixtures, and stop access.
4. Confirm controller state, power/enable, emergency stop, alarms, collision
   state, payload, installation, TCP, and safety limits at the controller.
5. Confirm the operator understands the displacement/orientation envelope,
   clutch, stop conditions, and abort procedure.
6. Perform only the separately authorized read-only/no-motion stages before
   considering motion.

Recorded operator-supplied state from the latest evidence:

| Item | Recorded value | Ownership |
|---|---|---|
| Payload mass | 0.8 kg | controller/operator state |
| Center of mass | `[9.289, 12.427, 36.961]` mm | controller/operator state |
| Installation | upright/floor; X=0°, Z=0° | controller/operator state |
| TCP1–TCP10 | zero | controller/operator state |
| Safety limits | unchanged | controller/operator state |

These values are not hard-coded runtime truth. The software must not silently
write them, and a future operator must verify them before motion.

Do not repeat the earlier approximately 128 mm multi-axis translation with
large wrist motion. Do not use a maintenance session to enter servo/EDG,
identify payload, calibrate TCP, or change controller settings.

---

# 中文版：真机硬件前置条件

本页只是前置检查清单，不构成真机授权。每个真机 gate 都必须在当前/新会话中针对精确的
stage、时长、运动边界和 acknowledgement flags 获得显式授权。

任何连接或命令之前：

1. 确认 worktree、branch、commit、任务 diff、配置和可执行文件身份。
2. 确认没有其他控制客户端登录，也没有残留进程。
3. 检查机器人、灵巧手、线缆、工作区、夹具和停止按钮。
4. 在控制器端确认状态、power/enable、急停、报警、碰撞状态、payload、安装方向、TCP
   和安全限制。
5. 确认操作者理解位移/旋转范围、clutch、停止条件和 abort 流程。
6. 只执行另行授权的 read-only/no-motion gate，然后才考虑运动。

最近证据中的操作者记录值：

| 项目 | 记录值 | 所有权 |
|---|---|---|
| Payload | 0.8 kg | 控制器/操作者状态 |
| 质心 | `[9.289, 12.427, 36.961]` mm | 控制器/操作者状态 |
| 安装 | upright/floor，X=0°，Z=0° | 控制器/操作者状态 |
| TCP1–TCP10 | 全零 | 控制器/操作者状态 |
| 安全限制 | 未更改 | 控制器/操作者状态 |

这些值不是软件硬编码真值。未来运动前必须由操作者在控制器上重新确认；软件不能静默写入。

不要重复先前约 128 mm、多轴并带大幅 wrist 的运行。不要利用维护会话进入 servo/EDG、
自动识别 payload、标定 TCP 或修改控制器设置。
