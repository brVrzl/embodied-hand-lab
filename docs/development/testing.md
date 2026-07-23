# Testing

## Suite policy

The default pytest suite is offline. Hardware access is kept outside collection
behind dedicated CLI acknowledgements and fake/static contract tests. Tests are
organized by subsystem rather than pytest markers; adding markers now would
not improve the single current CI-equivalent workflow.

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
