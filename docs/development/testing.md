# Testing

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
.venv/bin/python -m pytest -q
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
- Integration/contract: shared target path, adapters, CLI authorization, ROS2
  helpers, simulation asset compilation.
- Regression/replay: recorded Quest/JAKA sequences, output feasibility,
  singularity, EDG resampling.
- Native: fake-worker execution plus source/binary contract inspection.
- Simulation: MuJoCo model and controller behavior.
- Physical: dedicated executables/CLI stages only, never run by default pytest.

The integrated-workspace render test may skip when a usable headless MuJoCo
render backend is unavailable. Any other skip must be investigated and reported.
Do not remove a failing or old test merely to make totals smaller.

---

# 中文版：测试

## 测试策略

默认 pytest 套件完全离线。硬件访问位于带专用 CLI acknowledgement 的独立入口之外，
pytest 只使用 fake/static contract。当前按子系统组织测试，不额外引入 marker。

永久覆盖应保持克制，只保护稳定外部契约、安全关键控制行为、真实回归和少量代表性流程。
开发中可以使用临时测试和结果生成器，但任务结束前默认删除；不要以测试数量或覆盖率为目标，
也不要仅为了逐字比较生成 benchmark 产物而永久保留它们。

完整验证：

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
.venv/bin/python -m pytest -q
```

Quest/JAKA 关键安全回归：

```bash
.venv/bin/python -m pytest -q \
  tests/test_quest_jaka_shared_pipeline.py \
  tests/test_quest_jaka_output_feasibility.py \
  tests/test_quest_jaka_singularity_liveness.py \
  tests/test_jaka_edg_resampler.py \
  tests/test_native_jaka_servo_worker.py
```

它们覆盖启动连续性、活性、`HOLD_REJECTED`、Jacobian 奇异策略、输出速度/加速度、
重采样、tracking error、控制器健康、native zero-IK、仿真/真机一致性、真机授权、
no-motion gate 和 cleanup。

## 分类

- Unit：schema、映射、滤波、配置、几何和小型状态机。
- Integration/contract：共享目标、adapter、CLI 授权、ROS2 helper、仿真资产编译。
- Regression/replay：Quest/JAKA 记录、输出可行性、奇异性、EDG 重采样。
- Native：fake worker 执行和源码/二进制契约检查。
- Simulation：MuJoCo 模型和控制行为。
- Physical：专用 CLI stage，默认 pytest 永不执行。

无可用 headless MuJoCo 渲染后端时，integrated-workspace render 测试可能 skip。其他 skip
必须调查并报告。不得仅为减少数量而删除失败或旧测试。
