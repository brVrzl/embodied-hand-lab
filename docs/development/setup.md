# Development setup

Python 3.10 or newer is required:

```bash
cd "$(git rev-parse --show-toplevel)"
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

Optional extras are declared in `pyproject.toml`: `vision-teleop`, `realsense`,
`phone-teleop`, `motion-input-viz`, and `asset-tools`. Install only the extras
needed for the current offline task.

ROS2 Humble and vendor SDK dependencies are system/environment concerns and are
not fully installed by pip. The default pytest suite uses fake/offline
backends; it must not need a robot, hand, headset, camera, or vendor login.

Do not commit `.venv`, local SDK builds, captures, calibration data, credentials,
or runtime logs.

---

# 中文版：开发环境

需要 Python 3.10 或更高版本：

```bash
cd "$(git rev-parse --show-toplevel)"
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

`pyproject.toml` 中的可选依赖包括 `vision-teleop`、`realsense`、
`phone-teleop`、`motion-input-viz` 和 `asset-tools`。只安装当前离线任务所需部分。

ROS2 Humble 和 vendor SDK 属于系统环境依赖，不能完全通过 pip 安装。默认 pytest 使用
fake/offline backend，不应要求机器人、灵巧手、头显、相机或 vendor 登录。

不要提交 `.venv`、本地 SDK build、采集、标定数据、凭据或运行日志。
