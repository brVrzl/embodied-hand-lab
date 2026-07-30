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

The canonical shaped/125 Hz live recording and two-mode replay commands are in
the [Quest/JAKA MuJoCo simulation guide](../operation/simulation_demo.md). That
page is authoritative for output filenames, viewer environment, emitted logs,
and current validation boundaries.

## Native build

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

## Physical entry-point inspection

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/check_jaka_connection.sh --help
./scripts/check_rh56_connection.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

Do not remove `--help` or add device arguments during documentation
verification. Actual JAKA/RH56 connection, enable, servo/EDG, or motion requires
a separately approved physical gate and exact acknowledgement flags.

RH56 PC-direct preflight, read-only, bounded-command, Quest hand-only, and the
formal combined entry are documented in [RH56 operation](../operation/rh56_operation.md)
and [combined teleoperation](../operation/jaka_rh56_combined_teleop.md). They
prefer an explicit `/dev/serial/by-id/...` device; the identity-checked custom
CH341 fallback is separately acknowledged.

## Current normal combined physical entry

Help only:

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

The exact operator command, dual approvals, device selection, and logging are
maintained in [combined teleoperation](../operation/jaka_rh56_combined_teleop.md).
This production entry removes diagnostic-only restrictions, not production
safety boundaries.

## Arm-only isolation entry

Help only:

```bash
./scripts/run_quest_jaka_bounded_teleop.sh --help
```

Exact operator command after separate authorization:

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

Verify both addresses first. This wrapper selects `bounded-normal-teleop`, uses
the production 8 ms PWL path, and performs exactly one run with no automatic
retry. Its 60 second maximum and all safety hard stops remain active.
It writes timestamped accepted-target, summary, worker, capture, native-cycle,
and event-extract logs under `logs/`, never commands RH56, and never writes
controller configuration. The configured 1.5/1.2 rad/s values are
project-selected run limits, not vendor maximums. Add
`--plant-free-no-network-check` to validate a complete invocation without
opening sockets or connecting hardware.

For this arm-only entry, releasing left index requests a bounded native pause;
pressing it again captures a fresh reference from the stopped measured joints
and resumes. Ctrl+C, duration elapsed, stale input, and all controller/native
hard faults remain terminal. The post-payload diagnostic below retains
release-to-stop behavior.

## Post-payload diagnostic

The earlier lower-speed diagnostic remains available when specifically needed:

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

The diagnostic is not the current normal-operation entry.

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

规范的 shaped/125 Hz 实时录制与双模式回放命令见
[Quest/JAKA MuJoCo 仿真指南](../operation/simulation_demo.md)。输出文件名、viewer 环境、
emitted 日志与当前验证边界均以该操作页为准。

## Native 构建

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

## 真机入口检查

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_post_payload_manual.sh --help
./scripts/check_jaka_connection.sh --help
./scripts/check_rh56_connection.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

文档验证时不要去掉 `--help` 或添加设备参数。任何 JAKA/RH56 连接、enable、servo/EDG 或
运动都需要单独批准的真机 gate 和精确 acknowledgement。

RH56 PC-direct 的 preflight、read-only、bounded-command、Quest hand-only 和正式联合入口
见 [RH56 操作](../operation/rh56_operation.md)及
[联合遥操作](../operation/jaka_rh56_combined_teleop.md)。设备优先显式使用
`/dev/serial/by-id/...`；自定义 CH341 fallback 需要独立显式确认并核验身份。

## 当前正常联合真机入口

只查看帮助：

```bash
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

精确运行命令、双授权、设备选择和日志见
[联合遥操作](../operation/jaka_rh56_combined_teleop.md)。production 联合入口取消的是
诊断阶段临时限制，不是 production 安全边界。

## Arm-only 隔离入口

在另行获得精确授权后：

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

运行前核实两个 IP。该 wrapper 选择 `bounded-normal-teleop`，使用 production 8 ms PWL，
只运行一次且禁止自动重试；最长 60 秒和全部安全硬停止仍然有效。
它会在 `logs/` 下生成带时间戳的完整日志，不命令 RH56，也不写控制器配置。1.5/1.2 rad/s
是项目选择的运行边界，不是厂商最大速度。增加 `--plant-free-no-network-check` 可在不创建
socket、不连接硬件的条件下验证完整命令。

正常入口松开 left-index 会请求 native 有界暂停；再次按下会从停止后的实测关节状态重新
捕获参考并继续。Ctrl+C、时长到期、stale input 和所有控制器/native 硬故障仍会结束
运行。下面的 post-payload 诊断入口仍保持松开即停止。

## Post-payload 诊断入口

较低速度的历史诊断入口仍保留，在明确需要诊断时使用：

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

该诊断不再是当前正常操作入口。
