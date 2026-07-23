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
