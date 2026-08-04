# Physical hardware prerequisites

This page defines the runtime safety prerequisites for physical operation.

Reading this page, running --help, or performing repository maintenance does not open or command hardware.

Each maintained hardware entry is responsible for selecting the intended operation mode, validating the target device, enforcing bounded execution duration, verifying controller state, checking workspace conditions, and performing deterministic cleanup before any physical connection is established.

Configuration writes, fault reset, and force-sensor calibration remain separate operation modes with their own safety prerequisites.

Before any connection or command:

1. Confirm the intended worktree, branch, commit, clean task-owned diff, config,
   and executable hashes.
2. Confirm no other control client is logged in and no stale process remains.
3. Inspect the robot, hand, cabling, workspace, fixtures, and stop access.
4. Confirm controller state, power/enable, emergency stop, alarms, collision
   state, payload, installation, TCP, and safety limits at the controller.
5. Confirm the operator understands the displacement/orientation envelope,
   clutch, stop conditions, and abort procedure.
6. Perform only the read-only and no-motion stages before considering physical motion.

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

本页定义真机运行前必须满足的安全前置条件。

阅读本文档、运行 --help 或执行仓库维护都不会打开、连接或控制真实设备。

所有维护中的真机入口都负责选择对应运行模式、确认目标设备、检查控制器状态、验证工作区、限制运行时长、施加命令边界并完成确定性的退出和清理。

配置写入、故障复位和力传感器校准仍属于独立运行模式，并保留各自的安全前置条件。

任何连接或命令之前：

1. 确认 worktree、branch、commit、任务 diff、配置和可执行文件身份。
2. 确认没有其他控制客户端登录，也没有残留进程。
3. 检查机器人、灵巧手、线缆、工作区、夹具和停止按钮。
4. 在控制器端确认状态、power/enable、急停、报警、碰撞状态、payload、安装方向、TCP
   和安全限制。
5. 确认操作者理解位移/旋转范围、clutch、停止条件和 abort 流程。
6. 只执行 read-only 和 no-motion 阶段，然后才考虑实际运动。

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
