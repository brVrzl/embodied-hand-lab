# Active Research and Control Plan

Date: 2026-05-04

## Summary

The current project direction is:

**Palm-Frame Hand-Code Transfer for Data-Efficient Dexterous Grasping on JAKA mini2 + Inspire RH56**.

This replaces the earlier first-paper direction centered on failure-aware envelope-grasp data collection. Failure analysis and episode curation remain useful support infrastructure, but they are no longer the main research novelty.

The active research question is:

**Can public Inspire/RH56-like hand data be transferred to a single low-cost JAKA mini2 + RH56 system by learning a low-dimensional hand-code prior, grounding actions in an object-relative palm frame, and using RH56 low-level feedback as pseudo-tactile correction?**

The control stack should support that question directly:

- JAKA keeps both MoveIt/trajectory execution and high-frequency EDG servo control.
- RH56 uses PC direct USB-RS485 as the main experimental path and JAKA tool RS485 only as fallback.
- ROS2 Humble coordinates arm, hand, camera, policy, TF, and recording.
- Policies output low-dimensional `delta_palm_pose + hand_code + close_strength`, not raw high-dimensional whole-system commands.

## System Architecture

### JAKA Mini2 Arm Control

Use two mutually exclusive arm modes.

**Trajectory mode**

- Purpose: home, pregrasp, large safe motions, MoveIt planning, obstacle-aware approach.
- Backends: existing `joint_move`, `linear_move`, and future MoveIt trajectory execution.
- ROS2 interfaces:
  - `/arm/joint_states`
  - `/arm/ee_pose`
  - `/arm/command/joints`
  - `/arm/command/pose`
  - `/arm/follow_joint_trajectory`

**EDG servo mode**

- Purpose: visual servoing, palm-frame refinement, final 2-5 cm approach, contact-preparation motion.
- SDK path: `edg_init`, `servo_move_enable`, `edg_servo_j`, `edg_servo_p`.
- Initial control target: `step_num=2`, about 62.5 Hz.
- Stretch target after stability tests: `step_num=1`, about 125 Hz.
- ROS2 interfaces:
  - `/arm/servo/enable`
  - `/arm/servo/command_delta_pose`
  - `/arm/servo/command_twist`
  - `/arm/servo/state`

A mode manager must prevent trajectory commands and EDG servo commands from controlling the arm at the same time. Estop, protective stop, soft-limit, and servo abort must be recorded in every affected episode.

### Inspire RH56 Hand Control

Use two switchable hand backends.

**PC direct USB-RS485 backend**

- This is the main research and data-collection backend.
- It must control:
  - `ANGLE_SET`
  - `FORCE_SET`
  - `SPEED_SET`
- It must read:
  - `ANGLE_ACT`
  - `FORCE_ACT`
  - `CURRENT`
  - `ERROR`
  - `STATUS`
  - `TEMP`
- Initial targets:
  - hand state: at least 20 Hz, with 20-50 Hz as the working range.
  - hand command: 10-20 Hz.
  - command timeout rate: below 2% during a 100-command smoke test.

**JAKA tool RS485 backend**

- This remains a fallback and demo backend.
- It is not the primary backend for pseudo-tactile experiments because JAKA TIO exposes only limited feedback signals and the current command path has significant delay.
- It can still be used to execute safe preset grasps when PC direct wiring is unavailable.

Hand action representation:

```yaml
hand_action:
  code_id: int
  target_angles: [6]
  speed: [6]
  force_limit: [6]
  close_strength: float
```

Canonical RH56 order:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

### ROS2 Humble Nodes

Implement the stack as separate ROS2 nodes so control, perception, policy, and logging can be tested independently.

`jaka_driver_node`

- Publishes `/arm/joint_states`, `/arm/ee_pose`, `/arm/edg_state`.
- Accepts trajectory and EDG servo commands.
- Owns arm mode state and exposes mode transitions.

`rh56_driver_node`

- Supports `serial_protocol` and `jaka_tool_rs485` backends.
- Publishes `/hand/state`, `/hand/raw_feedback`, `/hand/backend_mode`.
- Accepts `/hand/command_code`, `/hand/command_angles`, `/hand/command_force`.
- Provides backend switching only when the hand is idle.

`camera_node`

- Uses Orbbec Gemini 2 as the primary fixed RGB-D camera.
- Optionally uses Intel RealSense D405 for wrist or close-range contact observation.
- Publishes RGB, depth, and camera info at 15-30 Hz.

`tf_calibration_node`

- Publishes `jaka_base -> camera`.
- Publishes `jaka_tool0 -> rh56_palm`.
- Stores calibration metadata with dataset exports.

`policy_node`

- Inputs RGB-D, arm state, hand state, and TF.
- Outputs `delta_palm_pose`, `hand_code`, and `close_strength`.
- Must not output raw JAKA joint actions and raw six-finger commands as the default policy interface.

`episode_recorder_node`

- Records synchronized arm, hand, camera, TF, policy action, operator notes, and success/failure metadata.
- Records both raw feedback and normalized/canonical fields used for learning.

Recommended frequencies:

- arm EDG servo internal loop: 62.5-125 Hz.
- arm ROS state: 50-100 Hz.
- hand PC direct state: 20-50 Hz.
- hand command: 10-20 Hz.
- RGB-D: 15-30 Hz.
- policy: 5-20 Hz.
- recorder aligned export: 10-20 Hz.

## Research Method

### Public Data Transfer

Use public Inspire-like datasets to learn a RH56 hand-code prior.

Primary roles:

- Unitree Inspire datasets: auxiliary hand prior with explicit normalized Inspire hand order.
- HRDexDB / Inspire F1 datasets: target-domain calibration source after axis-order and range verification.
- Project real data: evaluation and calibration data first; training data only after episodes pass manual review and schema validation.

Do not clone external arm joint values onto JAKA. Only transfer:

- hand state/action sequences.
- hand phase priors.
- object-relative or palm-relative action abstractions when available.
- low-dimensional hand-code priors.

### Palm-Frame Representation

The central action representation is object-relative palm pose:

```text
object observation -> T_object_to_palm + approach direction + hand_code sequence + close_strength
```

JAKA executes the reachable palm pose. RH56 executes the hand-code sequence. This makes wrist/palm orientation a first-class variable without making "palm orientation matters" the entire novelty.

The first implementation can use retrieval or lightweight classifiers before any large model:

- retrieve nearest object category / geometry / width.
- select candidate `T_object_to_palm`.
- select candidate hand-code and close strength.
- use EDG servo for final alignment.

### Pseudo-Tactile Correction

Use RH56 low-level feedback as a practical pseudo-tactile signal:

- angle residual: `target_angle - actual_angle`.
- force estimate.
- current.
- closure time and closure progress.

Detect:

- empty grasp.
- blocked closure.
- slip risk.
- over-closure / object ejection risk.
- hand response mismatch.

The first version should be rule-based:

- increase close strength within configured safety bounds.
- switch to a backup hand-code.
- run a small palm-pose correction.
- abort and label the failure mode.

Do not claim high-resolution tactile perception. The contribution is low-cost pseudo-tactile contact correction for RH56-style hardware.

## Experiments

### Hardware Bring-Up

JAKA:

- Verify trajectory mode for home and pregrasp.
- Verify EDG servo at `step_num=4`, then `2`, then optionally `1`.
- Run 60 seconds of small bounded servo motion at `step_num=2`.
- Record command interval, dropped calls, protective stops, and visible oscillation.

RH56:

- Verify PC direct USB-RS485 read/write.
- Run 100 open/close or hand-code commands.
- Log angle, force, current, status, error, and temp.
- Compare PC direct latency and feedback richness against JAKA tool RS485.

Integrated:

- MoveIt or trajectory mode moves to pregrasp.
- EDG servo refines palm pose.
- RH56 executes hand-code.
- pseudo-tactile correction runs.
- arm lifts and holds.
- episode recorder writes complete logs.

### Real Robot Tasks

Start with 4-8 lightweight tabletop objects:

- foam cube.
- foam cylinder.
- plastic cup.
- light bottle.
- optional YCB or household small objects.
- optional tool/handle object for functional grasp.

Core tasks:

- grasp-lift-hold.
- functional grasp with orientation constraints.
- optional place into fixed tray after grasp-lift becomes stable.

Each object/task setting should run at least 20 real trials for comparisons.

### Ablations

Palm representation:

- fixed top-down palm.
- random reachable palm.
- retrieved or learned object-relative palm.

Hand action representation:

- continuous 6D RH56 command.
- KMeans hand-code.
- RVQ/VQ hand-code with RH56 thumb anchors.

Public-data transfer:

- no public data.
- Unitree Inspire only.
- HRDexDB/Inspire F1 only.
- Unitree + HRDexDB after normalization.

Pseudo-tactile correction:

- no feedback.
- angle residual only.
- angle residual + force/current threshold.

Low-data learning:

- 5 demos.
- 10 demos.
- 20 demos.

### Metrics

- grasp success rate.
- first-attempt success rate.
- lift/hold success rate.
- slip count.
- empty grasp count.
- over-closure/ejection count.
- palm reachability failure rate.
- demos needed to reach 70% success.
- arm servo command interval and failure rate.
- hand feedback frequency and command timeout rate.
- policy inference latency.

## Documentation Policy

This document is the single source of truth for the active research and control direction.

Older failure-aware collection plans are historical references only. They should not be used as the active first-paper direction. Useful pieces from those documents remain valid as support infrastructure:

- failure labels.
- manual review.
- replay validation.
- episode schema.
- clean/weak/failure data curation.

Any future planning document should either update this file or explicitly state that it is a historical/alternative plan.

## Assumptions

- OS and ROS target: Ubuntu 22.04 + ROS2 Humble.
- JAKA high-frequency control uses EDG servo first; ordinary servo is fallback.
- MoveIt remains in the stack for planning and large motions.
- RH56 PC direct USB-RS485 is the main research backend.
- JAKA tool RS485 is retained as fallback.
- The first paper should emphasize palm-frame hand-code transfer and pseudo-tactile correction, not a generic data collection pipeline.

# 中文版本

## 总结

当前项目主线是：

**面向 JAKA mini2 + Inspire RH56 的 Palm-Frame Hand-Code Transfer 少样本灵巧抓取。**

这条主线替代了早期“failure-aware envelope-grasp 数据采集 pipeline”方向。失败分析、人工复核、episode schema 和数据筛选仍然保留，但它们只是支撑实验的基础设施，不再作为第一篇文章的主要创新点。

当前研究问题是：

**能否利用公开 Inspire/RH56 类手部数据，在只有一台 JAKA mini2 + RH56 的低成本系统上学习低维 hand-code 先验，并通过 object-relative palm frame 与 RH56 低层反馈实现数据高效的真实抓取？**

核心约束：

- 策略默认输出 `delta_palm_pose + hand_code + close_strength`。
- 不默认让策略直接输出 JAKA 6 关节和 RH56 6 指连续高维命令。
- JAKA 负责执行可达 palm pose。
- RH56 负责执行 hand-code，并通过 angle / force / current 等反馈做 pseudo-tactile correction。

## 系统架构

### JAKA mini2 控制

机械臂保留两个互斥模式。

**Trajectory mode**

- 用于 home、pregrasp、大范围安全移动、MoveIt 规划和避障。
- 后端保留现有 `joint_move`、`linear_move`，后续可接 MoveIt trajectory。
- 相关接口包括 `/arm/joint_states`、`/arm/ee_pose`、`/arm/command/joints`、`/arm/command/pose` 和 `/arm/follow_joint_trajectory`。

**EDG servo mode**

- 用于视觉 servo、palm-frame 微调、接触前最后 2-5 cm 的精细 approach。
- 使用 JAKA SDK 的 `edg_init`、`servo_move_enable`、`edg_servo_j`、`edg_servo_p`。
- 第一阶段目标为 `step_num=2`，约 62.5 Hz。
- 稳定后再测试 `step_num=1`，约 125 Hz。
- 必须由 mode manager 仲裁，禁止 trajectory 与 EDG servo 同时控制机械臂。

### RH56 控制

RH56 使用两个可切换 backend。

**PC direct USB-RS485**

- 这是论文实验与数据采集的主链路。
- 需要控制 `ANGLE_SET`、`FORCE_SET`、`SPEED_SET`。
- 需要读取 `ANGLE_ACT`、`FORCE_ACT`、`CURRENT`、`ERROR`、`STATUS`、`TEMP`。
- 初始目标为 hand state 至少 20 Hz，工作范围 20-50 Hz；hand command 10-20 Hz。

**JAKA tool RS485**

- 只作为备用和演示链路。
- 因为 JAKA TIO 暴露信号有限、当前命令路径延迟较大，所以不作为 pseudo-tactile 实验主数据源。

RH56 canonical order 固定为：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

## ROS2 节点

建议拆成独立 ROS2 Humble 节点：

- `jaka_driver_node`：发布 arm state，接收 trajectory 与 EDG servo 命令，并管理 arm mode。
- `rh56_driver_node`：支持 PC direct 与 JAKA tool backend，发布 hand state/raw feedback，接收 hand-code 和 raw angle 命令。
- `camera_node`：主相机推荐 Orbbec Gemini 2，可选 RealSense D405 作为腕部或近距接触观察。
- `tf_calibration_node`：发布 `jaka_base -> camera` 与 `jaka_tool0 -> rh56_palm`。
- `policy_node`：输入 RGB-D、arm state、hand state、TF，输出 `delta_palm_pose`、`hand_code`、`close_strength`。
- `episode_recorder_node`：同步记录 arm、hand、camera、TF、policy action、operator notes 和 success/failure metadata。

推荐频率：

- JAKA EDG servo 内部控制：62.5-125 Hz。
- arm ROS state：50-100 Hz。
- RH56 PC direct state：20-50 Hz。
- RH56 command：10-20 Hz。
- RGB-D：15-30 Hz。
- policy：5-20 Hz。
- recorder 对齐导出：10-20 Hz。

## 研究方法

### 公开数据迁移

使用公开 Inspire 类数据学习 RH56 hand-code 先验：

- Unitree Inspire 数据：作为 auxiliary hand prior。
- HRDexDB / Inspire F1：作为目标域校准数据，前提是完成轴顺序与数值范围验证。
- 本项目真实数据：优先作为评估和校准数据；只有通过人工复核和 schema 检查后才进入训练集。

不要把外部机器人手臂关节动作直接克隆到 JAKA。只迁移：

- 手部状态/动作序列。
- 手部阶段先验。
- object-relative 或 palm-relative action abstraction。
- 低维 hand-code 先验。

### Palm-frame 表示

核心动作表示为：

```text
object observation -> T_object_to_palm + approach direction + hand_code sequence + close_strength
```

也就是说，模型或检索模块先决定物体坐标系下的掌心姿态，再由 JAKA 执行可达 palm pose，由 RH56 执行 hand-code。这样可以把“掌心/腕部朝向”作为重要变量研究，但不会把“掌心朝向很重要”本身包装成全部创新。

第一版可以先使用 retrieval 或轻量 classifier，不需要一开始上大模型。

### Pseudo-tactile correction

利用 RH56 低层反馈构造低成本 pseudo-tactile 信号：

- `target_angle - actual_angle`。
- force estimate。
- current。
- 闭合时间和闭合进度。

用于判断：

- 夹空。
- 闭合受阻。
- 滑落风险。
- 过闭合或物体被挤出。
- 手部响应异常。

第一版 correction 使用规则即可，例如增大 close strength、切换备用 hand-code、小幅修正 palm pose，或中止并写入 failure mode。不要宣称这是高分辨率触觉感知。

## 实验

硬件 bring-up：

- JAKA trajectory mode 完成 home 和 pregrasp。
- JAKA EDG servo 按 `step_num=4 -> 2 -> 1` 逐步测试。
- RH56 PC direct 完成读写、100 次命令压力测试、完整反馈记录。
- 比较 PC direct 与 JAKA tool RS485 的延迟和反馈信息量。

真实任务：

- foam cube。
- foam cylinder。
- plastic cup。
- light bottle。
- 可选 YCB 或日用品。
- 可选工具/把手类 functional grasp。

核心任务包括 grasp-lift-hold、带朝向约束的 functional grasp，以及稳定后可加入固定托盘 place。

核心消融：

- fixed top-down palm vs random reachable palm vs object-relative palm。
- continuous 6D RH56 command vs KMeans hand-code vs RVQ/VQ hand-code。
- no public data vs Unitree only vs HRDexDB only vs mixed public data。
- no feedback vs angle residual only vs residual + force/current。
- 5 / 10 / 20 demos 少样本设置。

指标：

- 抓取成功率。
- 首次尝试成功率。
- lift/hold 成功率。
- 滑落次数。
- 夹空次数。
- 过闭合/挤出次数。
- palm reachability failure rate。
- 达到 70% 成功率所需 demo 数。
- arm servo 命令间隔与失败率。
- hand feedback 频率和 command timeout rate。
- policy 推理延迟。

## 文档策略

本文档是当前研究与控制方向的唯一主计划。旧 failure-aware collection 文档仅作为历史参考。未来任何新计划都应该更新本文档，或者明确说明自己只是历史/备选方案。

## 假设

- 系统目标为 Ubuntu 22.04 + ROS2 Humble。
- JAKA 高频控制优先使用 EDG servo，普通 servo 只是备选。
- MoveIt 保留用于规划和大范围运动。
- RH56 PC direct USB-RS485 是主要研究 backend。
- JAKA tool RS485 保留为 fallback。
- 第一篇文章应强调 palm-frame hand-code transfer 和 pseudo-tactile correction，而不是通用数据采集 pipeline。
