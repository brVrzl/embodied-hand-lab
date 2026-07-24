# RH56 operation

The repository has three distinct RH56 roles:

1. Mounted MuJoCo RH56 in the Quest/JAKA simulation, with left grip retargeting.
2. A PC-direct USB-RS485 driver/ROS2 bridge in `src/rh56_driver`.
3. Experimental iPhone, HEBI, and JAKA-tool-RS485 workflows.

Canonical command order is:

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

Do not infer normalized values versus raw counts. Messages and logs should
state schema/version, source, timestamp, unit, and canonical order.

Safe local inspection:

```bash
./scripts/check_rh56_connection.sh --help
./scripts/rh56_pc_direct_bringup.sh --help
.venv/bin/python tools/iphone_mediapipe_hand_teleop.py --help
```

Those commands are help only. A serial port, physical hand, or JAKA tool bus
must not be opened without explicit authorization and a clear stop policy.
Quest grip control is simulation-validated; it is not a validated
Quest-to-physical-RH56 path.

Current model roles and limitations are documented in
`data/sim_assets/README.md`.

---

# 中文版：RH56 操作

仓库中的 RH56 有三种不同角色：

1. Quest/JAKA MuJoCo 仿真中的 mounted RH56，由左 grip retarget。
2. `src/rh56_driver` 中的 PC 直连 USB-RS485 driver/ROS2 bridge。
3. iPhone、HEBI 和 JAKA-tool-RS485 实验流程。

规范命令顺序：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

不能凭空判断数值是 normalized 还是 raw count。消息和日志必须包含 schema/version、source、
timestamp、unit 和 canonical order。

安全的本地帮助检查：

```bash
./scripts/check_rh56_connection.sh --help
./scripts/rh56_pc_direct_bringup.sh --help
.venv/bin/python tools/iphone_mediapipe_hand_teleop.py --help
```

这些命令只有在保留 `--help` 时才是帮助检查。打开串口、真机 RH56 或 JAKA tool bus 需要
明确授权和 stop policy。Quest grip 已通过仿真验证，但 Quest 到真机 RH56 尚未验证。

模型角色和限制见 `data/sim_assets/README.md`。
