# Embodied Lab

Embodied Lab is a research and engineering repository for a JAKA Mini2 arm,
Inspire RH56DFX hand, Meta Quest 3 input, MuJoCo simulation, perception, and
robot-learning experiments. Its most developed current path is a shared
Quest-to-JAKA target pipeline whose accepted joint targets can drive either
MuJoCo or a separately authorized physical ServoJ/EDG adapter.

```text
Quest wrist/head + left Touch controller
  -> validated input and clutch/reference capture
  -> frame mapping and filtering
  -> continuation IK and safety feasibility
  -> immutable AcceptedArmTarget
  -> MuJoCo simulation | physical JAKA adapter
```

Simulation and hardware are identical up to the adapter boundary. The physical
path does not follow MuJoCo `qpos`, does not independently solve IK, and does
not write payload, TCP, installation, or controller safety settings.

## Start here

- [Documentation index](docs/README.md)
- [Current status and next safe step](docs/status/current_status.md)
- [Architecture overview](docs/architecture/overview.md)
- [Simulation demo](docs/operation/simulation_demo.md)
- [Development setup and testing](docs/development/setup.md)
- [Safety model](docs/safety/safety_model.md)
- [Validation matrix](docs/status/validation_matrix.md)

## Simulation-first quick start

Set up the supported Python environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
```

Run the offline suite:

```bash
.venv/bin/python -m pytest -q
```

Inspect the simulation entry point without connecting to devices:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
```

The live simulation demo receives Quest UDP packets but imports and initializes
no JAKA or RH56 hardware SDK. Its controller clutch is release-before-press:
left index captures and holds the arm reference; left grip controls the
simulated RH56 hand. Full setup is in the
[simulation guide](docs/operation/simulation_demo.md).

## Current validation boundary

The shared target generator, continuation IK, Jacobian-based singularity
policy, `HOLD_REJECTED`, output velocity and acceleration feasibility,
piecewise-linear native resampling, startup continuity, and zero-native-IK
joint mode are covered offline. The Quest/MuJoCo path is simulation validated.

Historical bounded physical gates established parts of the JAKA foundation and
later exercised Quest teleoperation. A larger run encountered a J4 collision
alarm. After the operator corrected payload data, the sole-session lightweight
health polling implementation completed a bounded physical timing run, but the
run then revealed an excessive accepted-output acceleration. The current
acceleration-feasibility fix is tested offline and has **not** yet been
physically validated. The J4 collision cause remains unresolved.

Physical execution is deliberately not a quick-start workflow. It requires a
new, explicit authorization for the exact bounded gate and the prerequisites in
[hardware prerequisites](docs/operation/hardware_prerequisites.md). Repository
maintenance or running `--help` never authorizes robot login, enable, servo
mode, EDG, or motion.

## Project areas

- `src/quest_jaka_sim`, `src/teleoperation`, `src/motion_input`: current Quest
  input and shared arm-target pipeline.
- `native/jaka_servo_worker`: 125 Hz JAKA EDG transport and safety worker.
- `src/rh56_driver`, `src/jaka_driver_adapter`, `src/robot_bringup`: robot and
  hand adapters plus legacy/parallel bring-up tools.
- `data/sim_assets`, `models`: MuJoCo robot and integrated-workspace assets.
- `src/pregrasp`, `src/vision_interface`, `src/data_recorder`: pregrasp,
  perception, and data workflows.
- `docs/digital_twin`: current integrated-workspace project, which remains
  below “Simulation Ready” pending its documented calibration and collision
  issues.
- `docs/history`: preserved gate, incident, audit, and design evidence; it is
  not the current command reference.

The worktree may contain untracked datasets, models, captures, calibration
assets, or concurrent experiments. They are not part of the repository merely
because they are present locally; preserve them and stage changes
intentionally.
