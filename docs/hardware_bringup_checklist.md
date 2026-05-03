# Hardware Bring-Up Checklist

Date: 2026-05-04

This checklist supports the active plan in `docs/active_research_and_control_plan.md`.

It replaces the old failure-debug checklist that focused on a low-frequency envelope-grasp pipeline.

## 1. Lab Safety

- [ ] Physical estop or power cutoff is reachable by the operator.
- [ ] JAKA workspace is clear.
- [ ] Table, object fixtures, camera mounts, and cables are secured.
- [ ] JAKA speed/acceleration limits are set conservatively before any real motion.
- [ ] RH56 force and speed limits are loaded from config before contact tests.
- [ ] Every real-motion script has a dry-run mode and requires explicit `--execute`.

## 2. JAKA Trajectory Mode

- [ ] Run connection check.
- [ ] Read joint state and robot status.
- [ ] Confirm no estop, protective stop, soft-limit, program-running, or servo-mode blocker.
- [ ] Move to home with low speed.
- [ ] Move to pregrasp with low speed.
- [ ] Repeat home -> pregrasp 10 times.
- [ ] Record visible repeatability issues or endpoint error if measured.

Pass gate:

- [ ] 10/10 trajectory moves complete without protective stop.
- [ ] No visible cable snag or hand/table collision.
- [ ] Pregrasp pose is inside a safe reachable workspace.

## 3. JAKA EDG Servo Mode

- [ ] Confirm SDK exposes `edg_init`, `servo_move_enable`, `edg_servo_j` or `edg_servo_p`.
- [ ] Run read-only SDK capability probe:

```bash
./scripts/check_jaka_edg_servo_capability.sh \
  --config configs/robot/jaka_mini2_real.yaml
```

- [ ] Run connected SDK capability probe:

```bash
./scripts/check_jaka_edg_servo_capability.sh \
  --config configs/robot/jaka_mini2_real.yaml \
  --connect
```

- [ ] Confirm trajectory mode is disabled before entering servo mode.
- [ ] Enter EDG servo with conservative filter and bounded motion.
- [ ] Test `step_num=4` small joint or Cartesian increments.
- [ ] Test `step_num=2` small joint or Cartesian increments.
- [ ] Run 60 seconds at `step_num=2`.
- [ ] Log command interval, missed calls, SDK errors, protective stops, and oscillation.
- [ ] Exit servo mode cleanly.
- [ ] Confirm trajectory mode can be re-entered after servo mode exits.

Pass gate:

- [ ] `step_num=2` runs for 60 seconds without protective stop.
- [ ] No visible oscillation or uncontrolled drift.
- [ ] Mode manager prevents simultaneous trajectory and servo commands.

Stretch gate:

- [ ] Test `step_num=1` only after `step_num=2` is stable.
- [ ] Do not use `step_num=1` for contact experiments until it has passed no-contact validation.

## 4. RH56 PC Direct USB-RS485

- [ ] Connect RH56 directly to PC via USB-RS485.
- [ ] Confirm power supply and common ground/wiring are safe.
- [ ] Confirm the configured protocol order is `[pinky, ring, middle, index, thumb_close, thumb_lateral]`.
- [ ] Confirm published/canonical order is `[index, middle, ring, pinky, thumb_close, thumb_lateral]`.
- [ ] Read `ANGLE_ACT`.
- [ ] Read `FORCE_ACT`.
- [ ] Read `CURRENT`.
- [ ] Read `ERROR`.
- [ ] Read `STATUS`.
- [ ] Read `TEMP`.
- [ ] Run read-only feedback frequency probe:

```bash
./scripts/rh56_pc_direct_bringup.sh \
  --config configs/hand/rh56_real.yaml \
  --port /dev/ttyUSB0 \
  --polls 20
```

- [ ] Set default speed.
- [ ] Set default force.
- [ ] Execute the four low-amplitude asymmetric channel-identification commands from `docs/rh56dfq_official_materials_notes.md`.
- [ ] Execute open.
- [ ] Execute close or a safe hand-code.
- [ ] Run 100 safe commands while logging feedback.

Pass gate:

- [ ] State feedback reaches at least 20 Hz.
- [ ] Command timeout rate is below 2%.
- [ ] No finger direction, range, or order mismatch is observed.
- [ ] Error/status/temp remain safe.

## 5. RH56 JAKA Tool RS485 Fallback

- [ ] Confirm JAKA tool channel mode setup.
- [ ] Execute safe open.
- [ ] Execute safe preset grasp.
- [ ] Read available angle feedback.
- [ ] Compare command latency against PC direct.
- [ ] Mark the backend as fallback unless it matches the feedback needs of the active experiment.

Pass gate:

- [ ] Preset commands execute reliably.
- [ ] Backend mode switching does not break arm control.
- [ ] Limitations are recorded in experiment metadata when this backend is used.

## 6. Camera and TF

- [ ] Mount Orbbec Gemini 2 or selected RGB-D camera rigidly.
- [ ] Confirm RGB stream.
- [ ] Confirm depth stream.
- [ ] Confirm camera info.
- [ ] Calibrate `jaka_base -> camera`.
- [ ] Calibrate or measure `jaka_tool0 -> rh56_palm`.
- [ ] Record calibration date, method, board, residual, and file paths.

Pass gate:

- [ ] RGB-D stream is stable at 15 Hz or higher.
- [ ] TF tree is complete.
- [ ] Object-relative palm pose can be visualized before robot execution.

## 7. Integrated Motion Gate

- [ ] MoveIt or trajectory mode moves to pregrasp.
- [ ] Mode manager switches to EDG servo.
- [ ] EDG servo refines palm pose with no object contact.
- [ ] RH56 executes hand-code through PC direct backend.
- [ ] RH56 feedback is recorded during close.
- [ ] Pseudo-tactile rule classifies the contact state.
- [ ] Arm lifts and holds.
- [ ] Recorder writes arm, hand, camera, TF, action, and review metadata.

Pass gate:

- [ ] One complete no-learning episode can be recorded and reviewed.
- [ ] `manual_review.yaml` and synchronized replay are present.
- [ ] Any failure is labeled with a concrete failure mode and backend/mode context.

## 8. Experiment Readiness

- [ ] Public-data hand-code artifact is selected.
- [ ] Object list is fixed.
- [ ] Baselines are configured:
  - fixed palm.
  - continuous 6D hand command.
  - no public data.
  - no pseudo-tactile correction.
- [ ] Real trial count is defined.
- [ ] Safety limits are written in config.
- [ ] Recorder schema version is set to `jaka_rh56_palm_handcode_v0.1` or a documented successor.

# 中文版本

本文档服务当前主计划 `docs/active_research_and_control_plan.md`，用于替代早期只围绕低频 envelope-grasp pipeline 的 failure debug checklist。

## 1. 实验室安全

- [ ] 操作者能立即触碰物理急停或断电开关。
- [ ] JAKA 工作空间内没有无关物体。
- [ ] 桌面、物体治具、相机支架和线缆都已固定。
- [ ] 真实运动前，JAKA 速度和加速度限制已设置为保守值。
- [ ] 接触测试前，RH56 force 和 speed limit 已从配置加载。
- [ ] 所有真实运动脚本默认 dry-run，只有显式 `--execute` 才允许下发运动。

## 2. JAKA trajectory mode

- [ ] 运行连接检查。
- [ ] 读取 joint state 和 robot status。
- [ ] 确认没有 estop、protective stop、soft-limit、program-running 或 servo-mode blocker。
- [ ] 低速移动到 home。
- [ ] 低速移动到 pregrasp。
- [ ] 重复 home -> pregrasp 10 次。
- [ ] 记录可见重复定位问题或测量末端误差。

通过门槛：

- [ ] 10/10 次 trajectory move 无 protective stop。
- [ ] 没有线缆拉扯、手/桌碰撞。
- [ ] pregrasp 位姿处于安全可达工作空间。

## 3. JAKA EDG servo mode

- [ ] 确认 SDK 暴露 `edg_init`、`servo_move_enable`、`edg_servo_j` 或 `edg_servo_p`。
- [ ] 运行只导入 SDK 的能力探测：

```bash
./scripts/check_jaka_edg_servo_capability.sh \
  --config configs/robot/jaka_mini2_real.yaml
```

- [ ] 运行连接控制器后的能力探测：

```bash
./scripts/check_jaka_edg_servo_capability.sh \
  --config configs/robot/jaka_mini2_real.yaml \
  --connect
```

- [ ] 进入 servo mode 前确认 trajectory mode 已停止。
- [ ] 使用保守滤波和小幅有界运动进入 EDG servo。
- [ ] 使用 `step_num=4` 测试小幅关节或笛卡尔增量。
- [ ] 使用 `step_num=2` 测试小幅关节或笛卡尔增量。
- [ ] 在 `step_num=2` 下连续运行 60 秒。
- [ ] 记录命令间隔、丢失调用、SDK 错误、protective stop 和可见抖动。
- [ ] 干净退出 servo mode。
- [ ] 确认退出 servo 后可以重新进入 trajectory mode。

通过门槛：

- [ ] `step_num=2` 连续运行 60 秒无 protective stop。
- [ ] 无明显抖动或失控漂移。
- [ ] mode manager 能阻止 trajectory 与 servo 同时发命令。

扩展门槛：

- [ ] 只有在 `step_num=2` 稳定后才测试 `step_num=1`。
- [ ] `step_num=1` 通过无接触验证前，不用于接触实验。

可选的无运动 servo enable/disable 检查：

```bash
./scripts/check_jaka_edg_servo_capability.sh \
  --config configs/robot/jaka_mini2_real.yaml \
  --connect \
  --execute-enable-cycle
```

## 4. RH56 PC direct USB-RS485

- [ ] 通过 USB-RS485 将 RH56 直接接到 PC。
- [ ] 确认供电、接线和共地安全。
- [ ] 确认配置中的协议顺序是 `[pinky, ring, middle, index, thumb_close, thumb_lateral]`。
- [ ] 确认 ROS2/数据集使用的 canonical 顺序是 `[index, middle, ring, pinky, thumb_close, thumb_lateral]`。
- [ ] 读取 `ANGLE_ACT`。
- [ ] 读取 `FORCE_ACT`。
- [ ] 读取 `CURRENT`。
- [ ] 读取 `ERROR`。
- [ ] 读取 `STATUS`。
- [ ] 读取 `TEMP`。
- [ ] 运行只读反馈频率探测：

```bash
./scripts/rh56_pc_direct_bringup.sh \
  --config configs/hand/rh56_real.yaml \
  --port /dev/ttyUSB0 \
  --polls 20
```

- [ ] 设置默认 speed。
- [ ] 设置默认 force。
- [ ] 执行 `docs/rh56dfq_official_materials_notes.md` 中的四组低幅度非对称通道识别命令。
- [ ] 执行 open。
- [ ] 执行 close 或安全 hand-code。
- [ ] 连续运行 100 次安全命令并记录反馈。

通过门槛：

- [ ] state feedback 至少达到 20 Hz。
- [ ] command timeout rate 低于 2%。
- [ ] 没有手指方向、幅度或顺序错误。
- [ ] error/status/temp 保持安全。

可选写命令测试，必须确认手部周围安全后执行：

```bash
./scripts/rh56_pc_direct_bringup.sh \
  --config configs/hand/rh56_real.yaml \
  --port /dev/ttyUSB0 \
  --polls 20 \
  --execute \
  --command-cycles 3
```

## 5. RH56 JAKA tool RS485 fallback

- [ ] 确认 JAKA tool channel mode 设置。
- [ ] 执行安全 open。
- [ ] 执行安全 preset grasp。
- [ ] 读取可用 angle feedback。
- [ ] 和 PC direct 比较命令延迟。
- [ ] 如果使用该 backend，必须在 metadata 记录反馈受限和低频限制。

通过门槛：

- [ ] preset 命令可靠执行。
- [ ] backend mode switching 不破坏 arm control。
- [ ] 使用该 backend 时，实验 metadata 明确记录限制。

## 6. 相机与 TF

- [ ] 刚性安装 Orbbec Gemini 2 或其他 RGB-D 相机。
- [ ] 确认 RGB stream。
- [ ] 确认 depth stream。
- [ ] 确认 camera info。
- [ ] 标定 `jaka_base -> camera`。
- [ ] 标定或测量 `jaka_tool0 -> rh56_palm`。
- [ ] 记录标定日期、方法、标定板、残差和文件路径。

通过门槛：

- [ ] RGB-D 稳定达到 15 Hz 或更高。
- [ ] TF tree 完整。
- [ ] 真实执行前可以可视化 object-relative palm pose。

## 7. 集成运动门槛

- [ ] MoveIt 或 trajectory mode 移动到 pregrasp。
- [ ] mode manager 切换到 EDG servo。
- [ ] EDG servo 在无接触情况下微调 palm pose。
- [ ] RH56 通过 PC direct backend 执行 hand-code。
- [ ] close 过程中记录 RH56 feedback。
- [ ] pseudo-tactile rule 分类接触状态。
- [ ] 机械臂 lift 并 hold。
- [ ] recorder 写入 arm、hand、camera、TF、action 和 review metadata。

通过门槛：

- [ ] 能记录并复核一条完整的非学习 episode。
- [ ] 存在 `manual_review.yaml` 和同步 replay。
- [ ] 所有失败都带有明确 failure mode 和 backend/mode 上下文。

## 8. 实验准备

- [ ] 已选择 public-data hand-code artifact。
- [ ] 已固定 object list。
- [ ] 已配置 baseline：
  - fixed palm。
  - continuous 6D hand command。
  - no public data。
  - no pseudo-tactile correction。
- [ ] 已定义真实 trial 数量。
- [ ] safety limits 已写入配置。
- [ ] recorder schema version 设置为 `jaka_rh56_palm_handcode_v0.1` 或有文档说明的后续版本。
