# Command reference

All commands run from the repository root. `--help` commands below are safe and
do not authorize a live device.

## Environment and offline validation

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
.venv/bin/python -m pytest -q
```

## Simulation and plant-free replay

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/rh56_h0_self_test.py --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

## Native build

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

## Physical entry-point inspection

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/check_jaka_connection.sh --help
./scripts/check_rh56_connection.sh --help
```

Do not remove `--help` or add device arguments during documentation
verification. Actual JAKA/RH56 connection, enable, servo/EDG, or motion requires
a separately approved physical gate and exact acknowledgement flags.

## Current bounded physical manual entry

Help only:

```bash
./scripts/run_quest_jaka_post_payload_manual.sh --help
```

Exact operator command after separate authorization:

```bash
./scripts/run_quest_jaka_post_payload_manual.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --approval I_AUTHORIZE_ONE_POST_PAYLOAD_TELEOP_RERUN \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

Verify both addresses first. This wrapper is intentionally limited to
`post-payload-diagnostic`; it does not expose automatic P4 escalation.

## Bounded normal-speed physical teleoperation

This independent entry uses the production immutable `AcceptedArmTarget` and
native 8 ms piecewise-linear output path. It does not replace the post-payload
diagnostic:

```bash
./scripts/run_quest_jaka_bounded_teleop.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --approval I_AUTHORIZE_BOUNDED_NORMAL_QUEST_JAKA_TELEOPERATION \
  --output-generator pwl-8ms \
  --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2 \
  --log-dir logs \
  --no-auto-retry \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

The wrapper creates timestamped accepted-target, summary, worker, capture,
native-cycle, and event-extract logs under `logs/`. The J1–J3 1.5 rad/s and
J4–J6 1.2 rad/s limits are project-selected bounded normal-teleoperation
parameters, not official JAKA Mini2 maximum speeds. Shared and native hard
velocity/acceleration checks, controller/SDK/timing/tracking hard stops, and
clutch/keyboard stop remain active. This entry never commands RH56 and never
writes payload, COM, TCP, installation, or controller safety settings.

Validate a complete invocation without opening sockets or connecting hardware
by adding `--plant-free-no-network-check`.

---

# 中文版：命令参考

所有命令都从仓库根目录运行。下面的 `--help` 不构成真机授权。

## 环境与离线验证

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
.venv/bin/python -m pytest -q
```

## 仿真和 plant-free 回放

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/rh56_h0_self_test.py --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

## Native 构建

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

## 真机入口检查

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/run_quest_jaka_post_payload_manual.sh --help
./scripts/check_jaka_connection.sh --help
./scripts/check_rh56_connection.sh --help
```

文档验证时不要去掉 `--help` 或添加设备参数。任何 JAKA/RH56 连接、enable、servo/EDG 或
运动都需要单独批准的真机 gate 和精确 acknowledgement。

## 当前受限真机手动入口

在另行获得精确授权后：

```bash
./scripts/run_quest_jaka_post_payload_manual.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --approval I_AUTHORIZE_ONE_POST_PAYLOAD_TELEOP_RERUN \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

运行前核实两个 IP。该 wrapper 只支持 `post-payload-diagnostic`，不会自动升级到 P4。

## 有界正常速度真机遥操作

这是独立入口，使用 production 不可变 `AcceptedArmTarget` 和 native 8 ms 分段线性输出
路径，不替换 post-payload diagnostic：

```bash
./scripts/run_quest_jaka_bounded_teleop.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --approval I_AUTHORIZE_BOUNDED_NORMAL_QUEST_JAKA_TELEOPERATION \
  --output-generator pwl-8ms \
  --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.2 1.2 1.2 \
  --log-dir logs \
  --no-auto-retry \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

wrapper 会在 `logs/` 下生成带时间戳的 accepted-target、summary、worker、capture、native
cycle 和 event-extract 日志。J1–J3 的 1.5 rad/s、J4–J6 的 1.2 rad/s 是项目选择的有界
正常遥操作参数，不是 JAKA Mini2 官方最大速度。

共享和 native 的硬速度/加速度检查、控制器/SDK/时序/跟踪硬停止以及 clutch/键盘停止仍然
有效。该入口不会命令 RH56，也不会写入 payload、COM、TCP、安装方向或控制器安全参数。

在命令末尾增加 `--plant-free-no-network-check`，可在不创建 socket、不连接硬件的条件下
验证整条命令。
