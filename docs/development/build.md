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
```

Every supported build host compiles the portable resampler library. Linux
x86_64/aarch64 additionally links `build/jaka_servo_worker/jaka_servo_worker`
against the vendored platform snapshot. macOS intentionally has no worker
executable. On Linux, inspecting
`build/jaka_servo_worker/jaka_servo_worker --help` is offline; selecting its
real backend is a separate physical action.

The offline shaping research contract has portable tests:

```bash
cmake -S native/teleop_shaping -B build/teleop_shaping
cmake --build build/teleop_shaping -j
ctest --test-dir build/teleop_shaping --output-on-failure
```

The dated foundation diagnostics are built only when a specifically authorized
gate or regression investigation requires them. The offline
`native/teleop_shaping` and `native/research_thin_jaka_worker` projects are
research contracts, not production or physical validation.

中文摘要：macOS 只能验证 portable resampler；Linux 才能链接当前 JAKA worker。
任何构建或 `--help` 都不构成真机授权，研究 shaper 也不代表真机验证。
