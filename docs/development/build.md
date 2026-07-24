# Build

## Python

Editable installation is the normal Python build:

```bash
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
```

## Current native EDG worker

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

The build links the locally available JAKA SDK according to its CMake
configuration. Running `--help` is offline; invoking a real backend is a
separate physical action.

The other native directories are dated foundation diagnostics. Build them only
when a specifically authorized gate or regression investigation requires them;
their reports are in the history index.

---

# 中文版：构建

## Python

通常通过可编辑安装完成 Python 构建：

```bash
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
```

## 当前原生 EDG worker

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

构建过程按照 CMake 配置链接本机已有的 JAKA SDK。运行 `--help` 是离线操作；调用真实
后端属于另一个需要单独授权的真机动作。

其他原生目录属于带日期的基础 gate 诊断。只有在特定 gate 已获授权或回归调查确实需要
时才构建它们；对应报告见历史索引。
