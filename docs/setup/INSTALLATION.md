# Installation

## English

Python 3.10 or newer is required. Install only the extras for the machine role:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/embodied-lab doctor
```

Useful extras are `simulation`, `hardware`, `realsense`, `dataset-collection`,
`dataset-export`, `asset-tools`, and `dev`. Extras install Python packages;
they do not install NVIDIA drivers, librealsense system rules, JAKA controller
software, or a Quest application.

Linux JAKA builds additionally require CMake and the locally supplied SDK:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

Initialize the pinned source submodules when training is needed:

```bash
git submodule update --init --recursive
scripts/check_training_dependencies.sh --help
```

Installation, `doctor`, and build commands are offline and do not open devices.

## 中文

需要 Python 3.10 或更新版本。根据机器职责安装对应 extras：

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e ".[dev]"
.venv/bin/embodied-lab doctor
```

常用 extras 包括 `simulation`、`hardware`、`realsense`、`dataset-collection`、`dataset-export`、
`asset-tools` 和 `dev`。extras 只安装 Python package，不安装 NVIDIA driver、librealsense 系统规则、
JAKA controller 软件或 Quest application。

Linux JAKA 构建还需要 CMake 和本地 SDK：

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

需要训练时初始化固定版本的 source submodule：

```bash
git submodule update --init --recursive
scripts/check_training_dependencies.sh --help
```

安装、`doctor` 和 build 命令都是离线操作，不会打开设备。
