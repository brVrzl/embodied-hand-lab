# Installation

This page is the authoritative installation guide for the current source
bundle. Installation and `--help` checks do not authorize a JAKA, RH56DFX,
RealSense, Quest, or other physical-device connection. Read
[hardware prerequisites](../operation/hardware_prerequisites.md) and the
[validation matrix](../status/validation_matrix.md) before any separately
authorized hardware work.

## Support status

| Component | Current repository support | Validation boundary |
| --- | --- | --- |
| Core Python and `embodied-lab doctor` | Implemented | Offline tested; doctor only inventories paths and software |
| MuJoCo JAKA/RH56 simulation | Implemented | Offline/simulation validated; not proof of physical behavior |
| JAKA Mini2 | Linux native C++ ServoJ/EDG worker for x86_64 and aarch64 | Partially physically validated within bounded gates; full envelope and latest fixes are not fully validated |
| RH56DFX | Linux PC-direct USB/RS485 driver | Hand-only read/command/Quest evidence exists; long combined operation remains incomplete |
| RealSense D435 | Python adapter, stream checker, RGB-D processing, and example configs | Offline tested; target dual-camera profiles and synchronization are not physically validated |
| Meta Quest 3 | Host-side HTS and CTRL UDP parsing, recording, replay, and simulation | Host path implemented; headset APK/runtime installation is external and must be verified separately |
| Dataset tools | Simulation-backed Quest + dual-D435 episode capture, validation, manifest, statistics, ACT-style HDF5 and LeRobot export boundaries | Offline-tested data contracts; physical JAKA/RH56 multimodal collection is not integrated or validated |
| Training | PyTorch/distributed smoke infrastructure only | No ACT, Diffusion Policy, OpenPI/pi0, or general trainer exists |
| Jetson Thor | aarch64 JAKA SDK snapshot and device-oriented configs exist | End-to-end Thor installation, camera stack, policy deployment, and latency are not validated |

The maintenance host used during the repository overhaul was macOS arm64.
It could validate Python and MuJoCo offline behavior but could not link or run
the Linux JAKA worker, use CUDA/NCCL, or validate target hardware.

## Python baseline

[`pyproject.toml`](../../pyproject.toml) requires Python 3.10 or newer.
Python 3.11 is the conservative starting point for a new robotics/ML
environment because optional binary wheels may lag the newest Python release.
Use one environment per machine role; do not reuse an x86_64 server
environment on ARM64 Jetson.

From the extracted repository root:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install -e .
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor
```

If an existing environment predates the current `pyproject.toml`, reinstall
editable mode so the `embodied-lab` entry point is regenerated:

```bash
.venv/bin/python -m pip install -e .
```

`doctor` is read-only. It parses repository YAML, checks required paths and
Python packages, inventories device filenames, and reports host, storage,
NVIDIA, PyTorch, container, Slurm, and network-interface facts. It never opens
a serial port, camera, headset socket, or robot session. A nonzero exit means
the reported offline prerequisites are incomplete; it does not mean a physical
device failed.

The equivalent development invocation before installing the entry point is:

```bash
PYTHONPATH=src .venv/bin/python -m embodiment_core.cli doctor
```

## Optional dependency groups

Install only the role-specific extras needed on a machine:

| Extra | Python dependencies and intended role |
| --- | --- |
| `simulation` | MuJoCo runtime |
| `hardware` | pyserial for RH56 PC-direct communication |
| `vision-teleop` | MediaPipe and OpenCV for the experimental camera hand path |
| `realsense` | `pyrealsense2`; still requires a compatible OS/device stack |
| `dataset-export` | HDF5, OpenCV, and LeRobot 0.6 export support |
| `motion-input-viz` | Matplotlib visualization |
| `asset-tools` | COACD, image/video, OpenCV, pycolmap, SciPy, and trimesh tools |
| `training` | HDF5 and PyTorch; distributed smoke only, not a model trainer |
| `dev` | pytest, native build helpers, MuJoCo, HDF5, image/geometry tools, pyserial |

Examples:

```bash
.venv/bin/python -m pip install -e ".[simulation]"
.venv/bin/python -m pip install -e ".[simulation,hardware,realsense]"
.venv/bin/python -m pip install -e ".[dataset-export]"
.venv/bin/python -m pip install -e ".[dev]"
```

`dev` is broad but does not include every other extra: notably it does not
install MediaPipe, `pyrealsense2`, LeRobot, PyTorch, COACD, or pycolmap.
Extras install Python packages only. They do not install GPU drivers,
librealsense udev rules, firewall policy, robot controller software, or a
Quest application.

## Linux workstation

Linux is required for the current physical JAKA worker. A Debian/Ubuntu-style
workstation commonly needs:

```bash
sudo apt-get update
sudo apt-get install \
  build-essential \
  cmake \
  ninja-build \
  pkg-config \
  python3 \
  python3-dev \
  python3-venv
```

Package names differ by distribution. MuJoCo viewer or headless rendering may
also need the distribution's OpenGL/EGL or OSMesa runtime. Install those only
for the selected rendering backend.

Recommended role installs are:

```bash
# Offline simulation/development workstation
.venv/bin/python -m pip install -e ".[simulation,dev]"

# Robot-side Python without camera packages
.venv/bin/python -m pip install -e ".[hardware]"

# Perception workstation, after installing a compatible librealsense stack
.venv/bin/python -m pip install -e ".[realsense]"
```

Run:

```bash
.venv/bin/embodied-lab doctor --json
.venv/bin/embodied-lab sim smoke
```

The simulation smoke loads, resets, and steps
`configs/sim/quest_hts_jaka_mini2_offline.yaml` headlessly. It imports no
physical JAKA or RH56 backend.

## MuJoCo

Install the simulation extra:

```bash
.venv/bin/python -m pip install -e ".[simulation]"
.venv/bin/embodied-lab sim smoke
```

The default model is
`data/sim_assets/jaka_rh56_visual_coacd.xml`. Keep the repository layout
intact because MJCF includes and mesh paths are repository-relative.

For interactive Quest simulation, inspect the current wrapper without opening
network sockets:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
```

On a desktop, the viewer requires a valid display. On a headless Linux GPU
host, `MUJOCO_GL=egl` is appropriate only when EGL is installed and usable.
`MUJOCO_GL=osmesa` requires an OSMesa runtime. Do not set either globally
without testing the selected host.

## GPU training server

The `training` extra declares `torch>=2.4` and HDF5, but the correct CUDA-enabled
PyTorch build depends on the server driver and package channel. First record:

```bash
nvidia-smi
nvidia-smi topo -m
nvcc --version
```

Then install the PyTorch build recommended for that server's supported driver
and CUDA runtime, followed by the repository:

```bash
.venv/bin/python -m pip install -e ".[training,dataset-export]"
.venv/bin/embodied-lab doctor --json
.venv/bin/embodied-lab distributed-smoke --check
```

Do not infer the host driver from `nvcc`, or the system toolkit from
`torch.version.cuda`. The project currently has no trainer; a successful
distributed smoke does not make ACT or Diffusion Policy available. See
[distributed training readiness](../training/DISTRIBUTED_TRAINING.md).

No Dockerfile, Conda lock, or Apptainer image is currently authoritative.
Create a role-specific environment only after selecting the trainer and target
server stack.

## JAKA Mini2 native worker

The maintained physical arm transport is the C++ worker in
[`native/jaka_servo_worker`](../../native/jaka_servo_worker). It links the
locally supplied JAKA SDK 2.2.7 on Linux:

- `x86_64-linux-gnu/libjakaAPI.so` on x86_64;
- `aarch64-linux-gnu/libjakaAPI.so` on ARM64.

The vendor files have no redistributable license text in this bundle; follow
[`THIRD_PARTY_NOTICES.md`](../../THIRD_PARTY_NOTICES.md) before transferring
them independently.

Build offline:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
build/jaka_servo_worker/jaka_servo_worker --help
```

On non-Linux hosts CMake builds only the portable resampler library and skips
the real worker. On Linux, inspect linkage before any authorized controller
session:

```bash
file build/jaka_servo_worker/jaka_servo_worker
ldd build/jaka_servo_worker/jaka_servo_worker
```

The operator-facing help commands are offline:

```bash
./scripts/run_quest_jaka_bounded_teleop.sh --help
./scripts/run_quest_jaka_rh56_teleop.sh --help
./scripts/setup_jaka_lan2_route.sh --help
```

`setup_jaka_lan2_route.sh` is dry-run by default and is Linux-only. Its
execution mode flushes every address from the selected interface, so it is
appropriate only for a verified, dedicated robot interface with another
management path available. Set `IFACE`, `PC_IP`, and `JAKA_IP` explicitly,
inspect the dry-run output, and use both `--execute` and
`--acknowledge-interface-address-replacement` only inside an independently
authorized physical-network procedure.

Do not convert these into a physical command from this installation page.
Every real invocation needs its current exact approval phrase, bounded
duration, workspace and stop acknowledgements, verified controller state, and
the procedure in [JAKA arm teleoperation](../operation/jaka_arm_teleoperation.md).
The software must not write payload, TCP, installation, collision, or
controller safety settings.

## RH56DFX PC-direct hand

The maintained hand path is:

```text
RH56DFX -> USB/RS485 adapter -> Linux/Jetson host
```

Install:

```bash
.venv/bin/python -m pip install -e ".[hardware]"
./scripts/run_quest_rh56_hand_test.sh --help
```

The physical tool requires an explicit stable `/dev/serial/by-id/...` path.
Configure udev permissions according to the host administrator's policy; do
not use a mutable `/dev/ttyUSB0` name. Some target systems expose a custom
`/dev/ttyCH341USB<N>` path, but it is an explicit, audited fallback rather
than a portable default.

A no-open preflight is available after selecting the intended device path:

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID \
  --preflight-only
```

Preflight does not authorize a serial-open read or a command. Read-only,
bounded command, runtime configuration write, and Quest hand modes each have
separate approval requirements. Follow
[RH56 PC-direct operation](../operation/rh56_operation.md).

## RealSense D435

Install Intel's compatible librealsense runtime and udev rules for the target
Linux distribution first, then:

```bash
.venv/bin/python -m pip install -e ".[realsense]"
.venv/bin/python tools/check_realsense_stream.py --help
```

Wheel and system-package availability varies across x86_64 Linux and Jetson.
Do not assume a desktop `pyrealsense2` wheel is compatible with JetPack.
Use `rs-enumerate-devices` from the installed librealsense tools to record
serial, firmware, USB mode, and supported stream profiles.

`tools/check_realsense_stream.py --list-devices` enumerates RealSense
identities without starting a stream. Without that flag, the tool opens the
selected camera and writes RGB, metric-depth, point-cloud, and metadata
snapshots; run the streaming form only when that device probe is intended.

`configs/camera/realsense_thor.yaml` is a site-specific two-camera snapshot,
not a portable default. For a new system start from
`configs/data_collection/dual_d435_episode.example.yaml`, replace both role
serials, and retain calibration identity. Workspace and wrist roles must never
be inferred from `/dev/video*` numbering. Target dual-D435 hardware/profile
validation remains incomplete.

## Meta Quest 3 host input

No Quest APK or Unity project is built by this repository. The deployed
Hand Tracking Streamer and left-controller sender are external installation
facts. The host-side UDP parser and recorder are available with the core
installation:

```bash
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
.venv/bin/python tools/quest_jaka_mujoco_sim.py live-6dof --help
```

Use a host address on the intended trusted interface, open only the selected
UDP port in the firewall, and optionally restrict the accepted sender IP.
Hand/head and CTRL packets share the current input port. Quest source time has
an independent monotonic epoch, so host receipt time is required for local
freshness and absolute one-way latency is not directly available.

See [Quest host setup](../operation/quest_setup.md) for packet and clutch
requirements. Source inspection does not prove which APK is installed on the
headset.

## Jetson Thor

Jetson Thor should be prepared as a separate ARM64 deployment/data-collection
target, not as the default large-scale training machine.

1. Install the vendor-supported JetPack, CUDA, TensorRT, camera, and PyTorch
   builds for the exact Thor image.
2. Create a fresh ARM64 virtual environment.
3. Install the base repository and only compatible extras, typically
   `hardware` first.
4. Build the aarch64 JAKA native worker and inspect its linkage.
5. Install/validate librealsense separately before adding `realsense`.
6. Run `embodied-lab doctor --json`.
7. Validate replay and inference consistency before any device connection.

Generic x86 CUDA wheels, desktop librealsense packages, and an x86 container
must not be copied to Thor. Large ACT, Diffusion Policy, or OpenPI/pi0 training
belongs on a GPU server or cluster. The current repository has no validated
Thor policy export, TensorRT conversion, camera timing, or end-to-end robot
deployment result.

## Post-install offline checks

Use only checks supported by the installed role:

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor --json
.venv/bin/embodied-lab sim smoke
.venv/bin/embodied-lab dataset --help
.venv/bin/embodied-lab distributed-smoke --check
.venv/bin/embodied-lab benchmark --help

PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools tests
.venv/bin/python -m pytest --collect-only -q -p no:cacheprovider
```

`distributed-smoke --check` returns nonzero when PyTorch or a usable backend is
absent. Skip the simulation command on a core-only installation. Neither
condition should be hidden by deleting tests.

For configuration rules continue with
[Configuration](../configuration/CONFIGURATION.md). For installation and
runtime failures see [Troubleshooting](../TROUBLESHOOTING.md).
