# Command reference

Run commands from the repository root unless a linked guide says otherwise.
`--help`, tests, replay, and simulation do not authorize a live device.

## Verification labels

- **Help verified**: the exact parser/help command was executed on the
  2026-07-31 maintenance host without opening hardware.
- **Repository recipe**: current source/build command, but not re-executed as
  part of this documentation-only check.
- **Physical template — not executed**: requires an operator-initiated run,
  explicit device selection, and all listed runtime safety gates in the
  session where it is run.

## Install and validate

Repository recipe:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[dev]"
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
.venv/bin/python -m pytest -q
```

This transferred source bundle has no project `.git` directory. Do not let
Git walk upward into an unrelated parent repository. Run `git diff --check`
only after the project is placed in a real worktree whose repository root is
this directory (that is, `test -d .git` succeeds here).

After editable installation, the maintained offline CLI is
`.venv/bin/embodied-lab`. Both the installed entry point and the
source-equivalent module forms below were **Help verified**:

```bash
.venv/bin/python -m embodiment_core.cli --help
.venv/bin/python -m embodiment_core.cli doctor --help
.venv/bin/python -m embodiment_core.cli sim smoke --help
.venv/bin/python -m embodiment_core.cli dataset --help
.venv/bin/python -m embodiment_core.cli benchmark --help
.venv/bin/python -m embodiment_core.cli distributed-smoke --help
```

Canonical installed forms are:

```bash
.venv/bin/embodied-lab doctor --help
.venv/bin/embodied-lab sim smoke --help
.venv/bin/embodied-lab dataset --help
.venv/bin/embodied-lab benchmark --help
.venv/bin/embodied-lab distributed-smoke --help
```

These installed forms were **Help verified** after editable installation.
`doctor` is read-only. The distributed smoke command requires optional
PyTorch and is not a model trainer.

## Simulation, data, benchmark, and distributed inspection

The following were **Help verified**:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
.venv/bin/python tools/rh56_h0_self_test.py --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
.venv/bin/python tools/episode_dataset_cli.py --help
.venv/bin/python tools/export_episode_dataset.py --help
.venv/bin/python tools/run_benchmark.py --help
.venv/bin/python tools/distributed_smoke_test.py --help
```

Use the owning guides for executable non-help examples and their output
contracts:

- [MuJoCo simulation](../operation/simulation_demo.md)
- [Dataset schema and lifecycle](../data/DATASET_SCHEMA.md)
- [Offline benchmark](../benchmark/BENCHMARKS.md)
- [Distributed training readiness](../training/DISTRIBUTED_TRAINING.md)

## Native JAKA worker build

Repository recipe:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

The portable resampler builds on supported non-Linux hosts. The real worker
binary is Linux-only; after a successful Linux x86_64/aarch64 build it can be
inspected with:

```bash
build/jaka_servo_worker/jaka_servo_worker --help
```

Building or reading help does not authorize login, enable, servo mode, EDG, or
motion. A missing worker binary on macOS is expected, not a build failure.

## Physical entry-point inspection

These commands were **Help verified**:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_post_payload_manual.sh --help
./scripts/run_quest_rh56_hand_test.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
```

Do not remove `--help` or append device arguments merely to validate
documentation. Any JAKA connection, enable, servo/EDG, or motion requires its
separate physical gate. RH56 hand-only debugging is dry-run unless the
operator explicitly supplies `--real --device ...` and an operation mode; see the
[RH56 session-debug entry](../operation/rh56_session_debug.md). Configuration
writes, fault reset, and force calibration keep their separate operation modes
and configuration/physical confirmations;
combined JAKA/RH56 operation uses its complete explicit real-device command.
See [real-hardware safety](../safety/REAL_HARDWARE_SAFETY.md),
[RH56 operation](../operation/rh56_operation.md), and
[combined operation](../operation/jaka_rh56_combined_teleop.md).

The maintained combined entry is
`scripts/run_quest_jaka_rh56_teleop.sh`. Its complete real-device invocation,
stable serial-device selection, duration, no-retry rule, E-stop/workspace
checks, and logging command are kept in the combined-operation guide. The script permits
at most 300 seconds; no 300-second physical PASS exists.

## Arm-only isolation template

**Physical template — not executed:**

```bash
./scripts/run_quest_jaka_bounded_teleop.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --output-generator pwl-8ms \
  --joint-velocity-limits-rad-s 1.5 1.5 1.5 1.5 1.5 1.5 \
  --log-dir logs \
  --no-auto-retry \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

The addresses are examples and must be verified locally. This wrapper performs
one run, is bounded to at most 60 seconds, never commands RH56, and never
writes payload, TCP, installation, collision, or controller safety settings.
The 1.5 rad/s values are project-selected run limits, not vendor maximums.
Its documented `--plant-free-no-network-check` mode validates argument and
pipeline setup without opening sockets; it is still not physical evidence.

Releasing left index requests a bounded native pause. Pressing it again
captures a fresh reference from stopped measured joints. Ctrl+C, duration,
stale/invalid input, controller alarms, collision, E-stop, SDK errors, hard
timing faults, and actual liveness loss remain terminal.

## Post-payload diagnostic template

**Physical template — not executed:**

```bash
./scripts/run_quest_jaka_post_payload_manual.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 30 \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

This lower-speed, release-to-stop diagnostic is not the normal combined or
arm-only production entry. Recorded payload/COM values are operator state, not
permission for software to write controller configuration.
