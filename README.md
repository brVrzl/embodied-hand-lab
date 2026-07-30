# Embodied Lab

Embodied Lab is a research and engineering repository for a JAKA Mini2 arm,
Inspire RH56DFX hand, Meta Quest 3 input, MuJoCo simulation, perception, and
robot-learning experiments. Its most developed current path is a shared
Quest-to-JAKA target pipeline whose accepted joint targets can drive either
MuJoCo or a separately authorized physical ServoJ/EDG adapter.

```text
Quest wrist/head + left Touch controller
  -> validated input and clutch/reference capture
  -> frame mapping and filtering
  -> continuation IK and safety feasibility
  -> immutable AcceptedArmTarget
  -> MuJoCo simulation | physical JAKA adapter
```

Simulation and hardware are identical up to the adapter boundary. The physical
path does not follow MuJoCo `qpos`, does not independently solve IK, and does
not write payload, TCP, installation, or controller safety settings.

## Start here

- [Documentation index](docs/README.md)
- [Current status and next safe step](docs/status/current_status.md)
- [Architecture overview](docs/architecture/overview.md)
- [Quest recording, replay, and 125 Hz simulation](docs/operation/simulation_demo.md)
- [RH56 staged PC-direct operation](docs/operation/rh56_operation.md)
- [Current normal JAKA + RH56 teleoperation](docs/operation/jaka_rh56_combined_teleop.md)
- [Development setup and testing](docs/development/setup.md)
- [Safety model](docs/safety/safety_model.md)
- [Validation matrix](docs/status/validation_matrix.md)

## Simulation-first quick start

Set up the supported Python environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
```

Run the offline suite:

```bash
.venv/bin/python -m pytest -q
```

Inspect the simulation entry point without connecting to devices:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
```

The live simulation demo receives Quest UDP packets but imports and initializes
no JAKA or RH56 hardware SDK. Its two simulation clutches are
release-before-press: left index captures and holds the arm reference; left
grip captures and controls the simulated RH56 hand. The integrated live model
has six JAKA and six RH56 actuators. The explicit arm-only model builder,
retained for JAKA-only regression and operation, removes the RH56 command path
and still exposes exactly six JAKA actuators. Recording, shaped/125 Hz live
control, two-mode replay, log outputs, and full setup are in the
[simulation guide](docs/operation/simulation_demo.md).

## Current validation boundary

The shared target generator, continuation IK, Jacobian-based singularity
policy, `HOLD_REJECTED`, output velocity and acceleration feasibility,
piecewise-linear native resampling, startup continuity, and zero-native-IK
joint mode are covered offline. The Quest/MuJoCo path is simulation validated.

Historical bounded physical gates established parts of the JAKA foundation and
later exercised Quest teleoperation. A larger run encountered a J4 collision
alarm. After the operator corrected payload data, the sole-session lightweight
health polling implementation completed a bounded physical timing run, but the
run then revealed an excessive accepted-output acceleration. The current
acceleration-feasibility fix is tested offline and has **not** yet been
physically validated. The J4 collision cause remains unresolved.

The PC-direct RH56 Quest hand-only path completed a 60 second physical run on
2026-07-29 without timeout, checksum, protocol, or hand fault. The current
normal operator entry is the combined JAKA + RH56 wrapper; combined physical
motion itself remains unvalidated and retains separate exact approvals.

Physical execution is deliberately not a quick-start workflow. It requires a
new, explicit authorization for the exact bounded gate and the prerequisites in
[hardware prerequisites](docs/operation/hardware_prerequisites.md). Repository
maintenance or running `--help` never authorizes robot login, enable, servo
mode, EDG, or motion.

## Project areas

- `src/quest_jaka_sim`, `src/teleoperation`, `src/motion_input`: current Quest
  input and shared arm-target pipeline.
- `native/jaka_servo_worker`: 125 Hz JAKA EDG transport and safety worker.
- `src/rh56_driver`, `src/jaka_driver_adapter`, `src/robot_bringup`: robot and
  hand adapters plus legacy/parallel bring-up tools.
- `data/sim_assets`, `models`: MuJoCo robot and integrated-workspace assets.
- `src/vision_interface`: perception and RealSense calibration workflows.
- `docs/digital_twin`: current integrated-workspace project, which remains
  below “Simulation Ready” pending its documented calibration and collision
  issues.
- `docs/history`: preserved gate, incident, audit, and design evidence; it is
  not the current command reference.
- `learned_policy/pi05_shadow`: inference-only OpenPI/DROID shadow work. It
  remains blocked from JAKA/RH56 execution by an intentional schema boundary
  and uses the separately pinned local `openpi` checkout.

PWL/root-cause-fix and the RH56 simulation hand work are in `main`. MoveIt,
Ruckig, ACT/Thor, TeleDex, and repository-cleanup results are retained as remote
archive branches rather than the production baseline. The offline
teleoperation-rearchitecture contracts are present for research review but do
not replace the production control path.

The worktree may contain untracked datasets, models, captures, calibration
assets, or concurrent experiments. They are not part of the repository merely
because they are present locally; preserve them and stage changes
intentionally.

---

# 中文版

Embodied Lab 是面向 JAKA Mini2 机械臂、Inspire RH56DFX 灵巧手、Meta Quest 3
输入、MuJoCo 仿真、感知和机器人学习实验的研发仓库。目前最成熟的链路是一条共享的
Quest 到 JAKA 目标生成管线；它产生的已接受关节目标既可以驱动 MuJoCo，也可以交给
需要单独授权的真机 ServoJ/EDG 适配器。

```text
Quest 手腕/头部 + 左 Touch 控制器
  -> 输入验证与 clutch/参考位姿捕获
  -> 坐标映射与滤波
  -> continuation IK 与安全可行性检查
  -> 不可变 AcceptedArmTarget
  -> MuJoCo 仿真 | JAKA 真机适配器
```

仿真和真机在输出适配器之前完全共用同一条逻辑。真机路径不跟随 MuJoCo `qpos`，
不独立求解 IK，也不写入 payload、TCP、安装方向或控制器安全配置。

## 从这里开始

- [文档索引](docs/README.md)
- [当前状态和下一安全步骤](docs/status/current_status.md)
- [架构概览](docs/architecture/overview.md)
- [Quest 录制、回放与 125 Hz 仿真](docs/operation/simulation_demo.md)
- [开发环境与测试](docs/development/setup.md)
- [安全模型](docs/safety/safety_model.md)
- [验证矩阵](docs/status/validation_matrix.md)
- [JAKA 真机遥操作](docs/operation/jaka_arm_teleoperation.md)
- [RH56 分阶段 PC-direct 操作](docs/operation/rh56_operation.md)
- [当前正常 JAKA + RH56 联合遥操作](docs/operation/jaka_rh56_combined_teleop.md)

## 仿真优先快速开始

创建支持的 Python 环境：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
```

运行离线测试：

```bash
.venv/bin/python -m pytest -q
```

只检查仿真入口，不连接设备：

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
```

实时仿真接收 Quest UDP 数据，但不会导入或初始化 JAKA/RH56 真机 SDK。两个仿真 clutch
都采用 release-before-press：左手食指捕获并保持机械臂参考，左手 grip 捕获并控制仿真
RH56。集成实时模型包含 6 个 JAKA 和 6 个 RH56 actuator；用于 JAKA-only 回归和运行的
显式 arm-only builder 会移除 RH56 command path，并仍然只暴露 6 个 JAKA actuator。
录制、shaped/125 Hz 实时控制、双模式回放、日志和完整设置见
[仿真指南](docs/operation/simulation_demo.md)。

## 当前验证边界

共享目标生成、continuation IK、基于 Jacobian 的奇异性策略、`HOLD_REJECTED`、输出速度/
加速度可行性、原生线性重采样、启动连续性和 native zero-IK joint mode 均有离线覆盖。
Quest/MuJoCo 路径已经过仿真验证。

历史上的受限真机 gate 验证了部分 JAKA 基础能力并执行过 Quest 遥操作。一次较大范围运行
触发了 J4 collision alarm。操作者修正 payload 后，单 SDK 会话轻量健康轮询在一个受限
运行中通过了时序验证，但该运行又暴露了已接受目标的输出加速度过大。当前加速度可行性及
true-hold 分类修复已有受限真机证据，修正后的 Quest 平移方向也由操作者确认；最新完整运行
仍以 producer-liveness timeout 停止，J4 碰撞原因也未完全确定。

PC-direct RH56 Quest hand-only 已于 2026-07-29 完成一次 60 秒真机运行，无 timeout、
checksum、protocol 或 hand fault。当前正常操作者入口已切换为 JAKA + RH56 联合 wrapper；
联合真机运动本身仍未验证，并继续要求两套精确授权。

真机运行不是普通 quick start。每次都必须针对精确 gate 获得新的显式授权，并满足
[硬件前置条件](docs/operation/hardware_prerequisites.md)。仓库维护或运行 `--help` 永远不
代表允许登录机器人、enable、进入 servo/EDG 或执行运动。

## 当前项目区域

- `src/quest_jaka_sim`、`src/teleoperation`、`src/motion_input`：当前 Quest 输入和共享
  机械臂目标管线。
- `native/jaka_servo_worker`：125 Hz JAKA EDG 传输和安全 worker。
- `src/rh56_driver`、`src/jaka_driver_adapter`、`src/robot_bringup`：机器人/灵巧手适配器
  以及并行 bring-up 工具。
- `data/sim_assets`、`models`：MuJoCo 机器人和集成工作区资产。
- `src/vision_interface`：感知和 RealSense 标定。
- `docs/digital_twin`：数字孪生；在完成所记录的标定和碰撞问题前仍未达到
  “Simulation Ready”。
- `docs/history`：保留的 gate、事故、审计和设计证据，不是当前命令入口。
- `learned_policy/pi05_shadow`：仅推理的 OpenPI/DROID shadow 工作；它通过明确的 schema
  边界与 JAKA/RH56 执行隔离，并使用单独固定版本的本地 `openpi` checkout。

PWL/root-cause-fix 和 RH56 仿真手成果均已进入 `main`。MoveIt、Ruckig、ACT/Thor、
TeleDex 和 repository-cleanup 只保留为远程归档分支，不是 production baseline。
离线 teleoperation rearchitecture 契约已进入当前仓库供研究审阅，但不替换 production
控制链路。

工作区中可能存在未跟踪的数据集、模型、采集、标定资产或并行实验。它们不会因为存在于本地
就自动成为仓库内容；修改和暂存时必须明确区分。
