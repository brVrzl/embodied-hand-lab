# Command reference

All commands run from the repository root. `--help` commands below are safe and
do not authorize a live device.

## Environment and offline validation

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
.venv/bin/python -m pytest -q
```

## Simulation and plant-free replay

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

## Native build

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

## Physical entry-point inspection

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/check_jaka_connection.sh --help
./scripts/check_rh56_connection.sh --help
```

Do not remove `--help` or add device arguments during documentation
verification. Actual JAKA/RH56 connection, enable, servo/EDG, or motion requires
a separately approved physical gate and exact acknowledgement flags.
