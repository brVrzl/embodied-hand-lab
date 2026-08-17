# Troubleshooting

## English

Start with read-only checks:

```bash
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim smoke
.venv/bin/embodied-lab dataset --help
```

Common boundaries:

- MuJoCo import/display errors: install the `simulation` extra and use the
  repository root; headless hosts may need a working `MUJOCO_GL` backend.
- Missing Python command: run `.venv/bin/python -m pip install -e ".[dev]"`.
- Missing native worker: build `native/jaka_servo_worker` with CMake and check
  the Linux SDK architecture before any physical procedure.
- Missing RealSense package/device: install the `realsense` extra and use
  `tools/check_realsense_stream.py --list-devices`; do not auto-discover or
  connect to an unverified device.
- Quest input: use the transport gate and inspect sequence/staleness reports;
  stale or malformed input must remain disengaged.
- Training dependencies: run `scripts/check_training_dependencies.sh --help`
  or `training/pi05/scripts/check_openpi.sh --help`; container dependencies are
  separate from the repository venv.

Never fix a failed physical precondition by adding a retry, fallback device,
or controller-setting write. Follow [real-device operation](operation/REAL_ROBOT.md)
and [safety](safety/REAL_HARDWARE_SAFETY.md).

## 中文

先执行只读检查：

```bash
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim smoke
.venv/bin/embodied-lab dataset --help
```

常见边界：

- MuJoCo import/display error：安装 `simulation` extra，且从仓库根目录运行；headless 主机可能需要可用的
  `MUJOCO_GL` backend。
- Python command 缺失：执行 `.venv/bin/python -m pip install -e ".[dev]"`。
- native worker 缺失：使用 CMake 构建 `native/jaka_servo_worker`，在真机流程前确认 Linux SDK 架构。
- RealSense package/device 缺失：安装 `realsense` extra，执行 `tools/check_realsense_stream.py --list-devices`；
  不要自动发现或连接未经确认的设备。
- Quest 输入问题：使用 transport gate 检查 sequence/staleness report；过期或格式错误输入必须保持 disengaged。
- Training dependency：执行 `scripts/check_training_dependencies.sh --help` 或
  `training/pi05/scripts/check_openpi.sh --help`；container dependency 与仓库 venv 分开。

不要用 retry、fallback device 或 controller-setting write 来绕过真机前置条件失败。遵循
[真机操作](operation/REAL_ROBOT.md)和[安全](safety/REAL_HARDWARE_SAFETY.md)。
