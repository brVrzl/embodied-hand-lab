# Testing

## 中文摘要

默认 pytest 套件只做离线验证，硬件访问被隔离在专门的 CLI 门和 fake/static 合约测试
之外。长期测试应保护稳定的功能、安全行为或真实回归；一次性探针、环境可选 smoke、
私有 helper 测试和对调度抖动敏感的单次实验不应长期保留。

## Suite policy

The default pytest suite is offline. Hardware access is kept outside collection
behind dedicated CLI acknowledgements and fake/static contract tests. Tests are
organized by subsystem rather than pytest markers; adding markers now would
not improve the single current CI-equivalent workflow.

Permanent coverage is deliberately selective. Keep tests for stable external
contracts, safety-critical control behavior, real regressions, and a few
representative workflows. Temporary tests and result generators are normal
during development, but they are removed before task completion unless they
meet that bar. Do not optimize for test count or coverage percentage, and do
not preserve generated benchmark artifacts merely so a test can compare them
byte-for-byte.

Full validation:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider
```

Critical Quest/JAKA safety regression:

```bash
.venv/bin/python -m pytest -q \
  tests/test_quest_jaka_shared_pipeline.py \
  tests/test_quest_jaka_output_feasibility.py \
  tests/test_quest_jaka_singularity_liveness.py \
  tests/test_jaka_edg_resampler.py \
  tests/test_native_jaka_servo_worker.py
```

The suite preserves coverage for startup continuity, liveness,
`HOLD_REJECTED`, Jacobian singularity policy, output velocity/acceleration,
resampling, tracking errors, controller-health polling, zero native IK,
simulation/hardware parity, physical authorization, no-motion gates, and
cleanup.

## Categories

- Unit: schemas, mappings, filters, config, geometry, small state machines.
- Integration/contract: shared target path, adapters, CLI authorization,
  dataset lifecycle, simulation asset compilation.
- Regression/replay: recorded Quest/JAKA sequences, output feasibility,
  singularity, EDG resampling.
- Native: fake-worker execution plus source/binary contract inspection.
- Simulation: MuJoCo model and controller behavior.
- Physical: dedicated executables/CLI stages only, never run by default pytest.

The integrated-workspace render test may skip without a usable headless MuJoCo
render backend. Linux-only JAKA SDK suites skip as complete modules on other
platforms, and the optional collective smoke skips when PyTorch is absent.
Every skip must be reported with its reason. Do not remove a failing or old
test merely to make totals smaller.

中文摘要：默认 pytest 完全离线；只保留稳定契约、安全行为、真实回归和代表性流程。
所有 skip 都必须说明平台或依赖原因，不能把离线通过描述成真机 PASS。
