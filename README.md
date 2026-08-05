# Embodied Lab

Embodied Lab is a simulation-first research stack for teleoperating and
studying a JAKA Mini2 arm with an Inspire RH56DFX hand. The maintained control
path accepts Meta Quest 3 hand/wrist tracking and a left Touch controller,
generates one safety-checked joint target, and sends that same target to either
MuJoCo or the explicitly selected physical JAKA adapter.

```text
Quest HTS + CTRL
  -> validate and queue
  -> release-before-press reference capture
  -> map and filter
  -> continuation IK and feasibility
  -> immutable AcceptedArmTarget
  -> MuJoCo adapter OR physical JAKA joint adapter
```

The physical adapter never follows MuJoCo `qpos`, remaps the target, or solves
IK. In native joint-teleop mode it makes zero JAKA `kine_inverse` calls.

## Safety boundary

Everything in the quick start below is offline. Tests, replay, simulation,
`doctor`, and `--help` do **not** open, connect to, or command a JAKA,
RH56DFX, Quest headset, RealSense camera, or any actuator.

Physical operation must always be started explicitly by the operator through a
maintained real-device entry. Before any hardware connection is established, the
selected entry is still required to enforce all runtime safety conditions,
including explicit device selection, bounded execution duration, verified
controller state, operator stop access, workspace clearance, command limits,
timing supervision, and deterministic shutdown.

Read [current status](docs/status/current_status.md) and
[real-hardware safety](docs/safety/REAL_HARDWARE_SAFETY.md) before interpreting
physical evidence or opening an operator guide.

## What is available now

| Area | Current capability |
| --- | --- |
| Quest/JAKA control | Shared input, clutch, mapping, continuation IK, collision/singularity/limit checks, output feasibility, and immutable accepted-target boundary |
| MuJoCo | Headless smoke, replay/live simulation, six arm plus six hand actuators, and a deterministic joint reach/pre-shape benchmark |
| Physical JAKA | Explicitly selected ServoJ/EDG joint adapter with sole-session status polling and final native safety checks; only partially physically validated |
| RH56DFX | PC-direct USB/RS485 scheduler, bounded six-actuator commands, and raw actuator feedback; independently operated and only partially physically validated |
| Dataset tools | Atomic canonical episodes, integrity validation, episode-level splits, train-only statistics, ACT-style HDF5 export, and optional LeRobot v3 export |
| Training infrastructure | Host inspection, global-batch validation, rank handling, and a PyTorch distributed communication smoke test |
| Policy training | Integration boundaries are documented; no maintained ACT, Diffusion Policy, or OpenPI trainer is implemented in this repository |
| Cameras | RealSense adapters, processing utilities, and example configuration exist; synchronized dual-D435 physical collection is not end-to-end validated |

The RH56 MuJoCo model is a six-command-axis kinematic approximation. Its
equality couplings do not reproduce tendon compliance, backlash, calibrated
force control, tactile sensing, or the complete physical underactuation.

## Offline quick start

Python 3.10 or newer is required. A development install includes MuJoCo and the
offline test dependencies:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -e ".[dev]"
```

Inspect the host and repository without opening devices, then run the default
headless model:

```bash
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim smoke
```

Run the offline suite:

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m pytest -q -p no:cacheprovider
```

The Linux-only JAKA SDK tests and optional PyTorch collective test are skipped
when their platform or dependency is unavailable. A skip is not physical
validation.

## Maintained workflows

### Simulation and replay

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py replay-6dof --help
.venv/bin/embodied-lab benchmark \
  configs/benchmark/smoke.yaml \
  --output /tmp/embodied-lab-benchmark.json
```

The live simulation receives Quest UDP packets but imports no JAKA or RH56
hardware SDK. See [simulation operation](docs/operation/simulation_demo.md) and
[benchmarking](docs/benchmark/BENCHMARKS.md).

### Dataset preparation

```bash
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset inspect <episode-directory>
.venv/bin/embodied-lab dataset manifest <dataset-root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
.venv/bin/embodied-lab dataset export <episode-directory> \
  act-hdf5 <episode.hdf5>
```

Canonical schema, atomic completion, missing-frame semantics, collection
limits, and framework adapters are documented in
[the dataset collection entry](docs/data/DATA_COLLECTION.md),
[dataset schema](docs/data/DATASET_SCHEMA.md),
[collection guide](docs/data/COLLECTION_GUIDE.md), and
[training integration](docs/training/TRAINING_INTEGRATION.md).

### Training-server preparation

Install the optional PyTorch dependency on a compatible training host:

```bash
.venv/bin/python -m pip install -e ".[training]"
.venv/bin/embodied-lab distributed-smoke --check
```

Single-process is the baseline and DDP is the intended scaling path when a
trainer is added. The repository does not claim validated GPU, multi-GPU,
multi-node, Slurm, FSDP, or DeepSpeed training. See
[distributed training](docs/training/DISTRIBUTED_TRAINING.md) for `torchrun`
and Slurm templates, global-batch semantics, checkpoint requirements, storage
guidance, and Jetson Thor deployment boundaries.

### Physical operation

Physical commands are intentionally absent from the quick start. The current
operator pages are:

- [Hardware prerequisites](docs/operation/hardware_prerequisites.md)
- [JAKA arm teleoperation](docs/operation/jaka_arm_teleoperation.md)
- [RH56 operation](docs/operation/rh56_operation.md)
- [Combined JAKA and RH56 teleoperation](docs/operation/jaka_rh56_combined_teleop.md)

Inspecting these pages or running a wrapper with `--help` grants no hardware
authority.

## Repository layout

| Path | Responsibility |
| --- | --- |
| `src/motion_input` | Quest packet transport, validation, canonical state, recording, and replay |
| `src/quest_jaka_sim` | Shared mapping, retargeting, simulation, accepted-target output, and resampling |
| `src/teleoperation` | Target, safety, sequencing, supervision, and wire contracts |
| `native/jaka_servo_worker` | Linux JAKA EDG transport and 8 ms final command boundary |
| `src/rh56_driver`, `src/rh56_sim` | PC-direct RH56 protocol path and simulation approximation |
| `src/episode_dataset` | Episode lifecycle, synchronization, validation, manifests, statistics, and export |
| `src/training_infra` | Optional distributed-runtime inspection and collective smoke test |
| `src/benchmarking` | Deterministic offline benchmark harness |
| `src/vision_interface`, `digital_twin` | Camera interfaces and workspace reconstruction/calibration research |
| `configs` | Versioned examples and runtime policies |
| `docs/history` | Dated evidence and superseded designs, never current operating authority |

`learned_policy/` is preserved inference research, not the maintained training
stack or a physical command path.

## Environment choices

Install only the extras needed by a host:

- `.[simulation]` for MuJoCo-only runtime;
- `.[hardware]` for serial support, still subject to physical authorization;
- `.[realsense]` or `.[vision-teleop]` for optional camera/input tooling;
- `.[dataset-export]` for ACT-style HDF5 and LeRobot export;
- `.[training]` for PyTorch training-server preparation;
- `.[asset-tools]` for reconstruction and collision-asset development;
- `.[dev]` for the complete offline development and test environment.

Linux JAKA SDK workers, x86_64 training servers, and ARM64 Jetson deployment
have different system dependencies. Do not reuse one environment definition
as proof that another platform is ready. See
[installation](docs/setup/INSTALLATION.md) and
[configuration](docs/configuration/CONFIGURATION.md).

## Common questions

- If MuJoCo cannot load, run `embodied-lab doctor`, verify the development or
  simulation extra, and use the repository root as the working directory.
- If a camera or robot is missing, do not add automatic discovery-and-connect
  fallbacks. Verify the explicit device identity and follow the relevant
  operator gate.
- `ANGLE_ACT` is six-axis actuator feedback. `CURRENT`, `FORCE_ACT`, `ERROR`,
  and `STATUS` are raw register fields; they are not passive-joint state,
  tactile arrays, direct slip sensing, or calibrated contact force.
- Simulation, replay, fake workers, and a successful benchmark do not imply a
  physical PASS or sim-to-real equivalence.

More diagnostics are in [troubleshooting](docs/TROUBLESHOOTING.md). The
[documentation index](docs/README.md) separates current authority from dated
evidence, and the [execution roadmap](docs/roadmap/NEXT_STEPS.md) records the
remaining data, training, benchmark, calibration, and deployment work.

## 中文说明

本仓库默认只进行离线和仿真工作。任何测试、回放、`doctor` 或 `--help` 都不构成真机
授权。当前最成熟的是 Quest 到 JAKA 的共享安全目标管线；数据集验证、最小 MuJoCo
benchmark 和分布式通信检查已经具备，但 ACT、Diffusion Policy、OpenPI/π0 的完整训练
闭环、双 D435 真机同步采集以及长期联合真机验证尚未完成。操作前请以
[当前状态](docs/status/current_status.md)和[真机安全边界](docs/safety/REAL_HARDWARE_SAFETY.md)
为准。
