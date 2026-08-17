# Embodied Lab

## English

Embodied Lab is an offline-first research stack for a JAKA Mini2 arm and
Inspire RH56DFX hand. Current maintained functions are:

- Quest 3 input parsing and safe target generation;
- MuJoCo simulation and replay;
- explicit, separately authorized JAKA/RH56 hardware adapters;
- review-first RGB-D/robot episode collection and dataset preparation;
- ACT/LeRobot and π0.5/OpenPI training infrastructure.

The current arm path is:

```text
Quest HTS + CTRL
  -> validate and order input
  -> release-before-press reference capture
  -> map, filter, continuation IK, and feasibility checks
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter or JAKA joint adapter
```

The physical adapter never follows MuJoCo `qpos`, remaps a target, or solves
IK. Native joint teleoperation makes zero JAKA `kine_inverse` calls.

### Safety boundary

Tests, replay, simulation, `doctor`, and `--help` are offline actions. They do
not connect to JAKA, RH56DFX, Quest, RealSense, or any actuator. Physical
operation requires an explicit operator procedure and the safety rules in
[`docs/safety/REAL_HARDWARE_SAFETY.md`](docs/safety/REAL_HARDWARE_SAFETY.md).

### Quick start

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim smoke
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider
```

Start with the [documentation index](docs/README.md). The maintained entry
points are grouped in [capabilities](docs/CAPABILITIES.md),
[dataset](docs/data/DATASET.md), [ACT training](docs/training/ACT.md), and
[π0.5 training](docs/training/PI05.md).

### Repository layout

| Path | Current role |
| --- | --- |
| `src/` | Reusable input, control, safety, RH56, camera, and dataset packages |
| `native/` | Offline-buildable JAKA and teleoperation workers |
| `tools/` | Current operator and developer entrypoints |
| `scripts/` | Thin wrappers for current simulation, collection, and training workflows |
| `configs/` | Current runtime and training configuration |
| `training/` | Project-owned ACT and π0.5 training boundaries |
| `assets/` | Current MuJoCo and robot assets |
| `docs/` | Current functional documentation |
| `third_party/` | Pinned submodules and vendor/reference assets |

## 中文

Embodied Lab 是面向 JAKA Mini2 机械臂和 Inspire RH56DFX 灵巧手的离线优先研究栈。当前维护的功能包括：

- Quest 3 输入解析和安全 target 生成；
- MuJoCo 仿真与 replay；
- 必须单独授权的 JAKA/RH56 真机适配器；
- 先人工审核的 RGB-D/机器人 episode 采集和数据准备；
- ACT/LeRobot 与 π0.5/OpenPI 训练基础设施。

当前机械臂管线为：

```text
Quest HTS + CTRL
  -> 输入校验和排序
  -> release-before-press reference capture
  -> 映射、滤波、continuation IK 和 feasibility 检查
  -> 不可变 AcceptedArmTarget
  -> MuJoCo adapter 或 JAKA joint adapter
```

物理适配器不会跟随 MuJoCo `qpos`、重新映射 target 或重新求 IK。原生 joint teleoperation
不会调用 JAKA `kine_inverse`。

### 安全边界

测试、replay、仿真、`doctor` 和 `--help` 都是离线操作，不会连接 JAKA、RH56DFX、Quest、RealSense
或任何 actuator。真机操作必须经过明确的 operator procedure，并遵守
[`docs/safety/REAL_HARDWARE_SAFETY.md`](docs/safety/REAL_HARDWARE_SAFETY.md)。

### 快速开始

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim smoke
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider
```

请从[文档索引](docs/README.md)开始。当前入口集中在[能力说明](docs/CAPABILITIES.md)、
[数据集](docs/data/DATASET.md)、[ACT 训练](docs/training/ACT.md)和
[π0.5 训练](docs/training/PI05.md)。

### 仓库布局

| 路径 | 当前职责 |
| --- | --- |
| `src/` | 可复用的输入、控制、安全、RH56、相机和数据集包 |
| `native/` | 可离线构建的 JAKA 和 teleoperation worker |
| `tools/` | 当前 operator/developer 入口 |
| `scripts/` | 当前仿真、采集和训练流程的薄 wrapper |
| `configs/` | 当前 runtime 和 training 配置 |
| `training/` | 项目维护的 ACT 和 π0.5 training boundary |
| `assets/` | 当前 MuJoCo 和机器人资产 |
| `docs/` | 当前功能文档 |
| `third_party/` | 固定版本的 submodule 和 vendor/reference 资产 |
