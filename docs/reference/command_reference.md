# Command reference

Run commands from the repository root unless a guide says otherwise. `--help`,
tests, replay, and simulation do not connect to or command hardware.

## Offline installation and checks

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q
.venv/bin/embodied-lab doctor
```

`doctor` is read-only. A skipped hardware-dependent test is not physical
validation.

## Main CLI

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor --help
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

## Simulation

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/rh56_h0_self_test.py --help
```

The simulation path has no physical hardware backend. Replay and simulation
results are not physical PASS evidence.

## Dataset inspection and conversion

```bash
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset inspect <episode-directory>
.venv/bin/embodied-lab dataset review-staging <root> <episode>
.venv/bin/embodied-lab dataset approve-staging <root> <episode> --status approved
.venv/bin/embodied-lab dataset convert-staging <root> <episode> <output>
.venv/bin/embodied-lab dataset manifest <root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
.venv/bin/embodied-lab dataset export <episode-directory> act-hdf5 <output.hdf5>
```

Live collection writes review-first staging data. Conversion and any Parquet
or framework export happen only after human review.

## Native worker build

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

Building or inspecting help does not authorize login, enable, servo mode, EDG,
or motion.

## Physical entry inspection

These commands only inspect parsers and help when invoked with `--help`:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
.venv/bin/python tools/quest_rh56_hand_test.py --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

The combined physical collection entry is documented in
[combined teleoperation](../operation/jaka_rh56_combined_teleop.md). A real
run requires its exact operator acknowledgements, explicit device identity,
bounded duration, E-stop access, workspace check, no-retry policy, and cleanup.
Do not add device arguments merely to validate documentation.

---

# 中文版：命令参考

除非具体指南另有说明，命令都从仓库根目录运行。`--help`、测试、回放和仿真不会连接或驱动
任何硬件。

## 离线安装和检查

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q
.venv/bin/embodied-lab doctor
```

`doctor` 是只读检查。跳过硬件依赖测试不等于真机验证。

## 主 CLI

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor --help
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

## 仿真

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/rh56_h0_self_test.py --help
```

仿真路径没有真机 hardware backend。回放和仿真结果都不能作为真机 PASS 证据。

## 数据检查和转换

```bash
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset inspect <episode-directory>
.venv/bin/embodied-lab dataset review-staging <root> <episode>
.venv/bin/embodied-lab dataset approve-staging <root> <episode> --status approved
.venv/bin/embodied-lab dataset convert-staging <root> <episode> <output>
.venv/bin/embodied-lab dataset manifest <root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
.venv/bin/embodied-lab dataset export <episode-directory> act-hdf5 <output.hdf5>
```

live 采集只写 review-first staging 数据。人工确认之前不能转换，也不会生成 Parquet 或其他
framework export。

## Native worker 构建

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

构建或查看 help 不授权 login、enable、servo mode、EDG 或运动。

## 真机入口检查

以下 `--help` 只检查 parser 和帮助文本：

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
.venv/bin/python tools/quest_rh56_hand_test.py --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
```

联合真机采集入口见[联合遥操作](../operation/jaka_rh56_combined_teleop.md)。真机运行必须
使用精确的操作者确认、显式 device identity、有界时长、E-stop 可达、workspace 检查、禁止
自动重试和确定性 cleanup。不要为了验证文档而添加 device 参数。
