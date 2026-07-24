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
