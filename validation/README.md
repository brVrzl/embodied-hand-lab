# On-demand validation suites

These suites are development and asset-validation checks, not part of the
default pytest regression suite. Run them from the repository root when
changing the corresponding calibration, scene, asset, or perception code:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/digital_twin
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/perception
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/rh56
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/motion_input
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/training
```

The RH56 H0 MuJoCo self-test is included in `validation/rh56`; run it
explicitly when changing the H0 model or mapping:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q validation/rh56/test_h0_self_test.py
```

They are offline checks only. A passing suite reports software consistency or
asset/calibration behavior; it is not evidence of JAKA, RH56, Quest, camera,
or other physical-device validation.
