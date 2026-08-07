# Troubleshooting

Start with evidence, not a configuration change:

```bash
.venv/bin/embodied-lab doctor --json \
  --output artifacts/doctor/doctor.json
```

`doctor` is read-only: it checks software, repository paths, YAML, storage,
network-interface names, and device filenames without opening a robot, serial
port, camera, or Quest socket. Preserve the full error, command, configuration
path/hash, host identity, and timestamp.

Never bypass a controller alarm, collision, emergency stop, SDK error,
communication timeout, stale input, hard timing fault, output bound, or
cleanup contract to continue a physical run. See
[incident response](safety/incident_response.md).

## Unified CLI is missing

If `.venv/bin/embodied-lab` does not exist, the environment may predate the
current entry point:

```bash
.venv/bin/python -m pip install -e .
.venv/bin/embodied-lab --help
```

Until reinstall, the same offline CLI can be inspected with:

```bash
PYTHONPATH=src .venv/bin/python -m embodiment_core.cli --help
```

Run commands from the extracted repository root unless a maintained wrapper
explicitly resolves its own root. If the source bundle has no `.git`, do not
use a parent directory's `git rev-parse --show-toplevel` as the project path.
Set `EMBODIED_LAB_ROOT` explicitly only when the selected directory really
contains this project's `pyproject.toml` and assets.

## Python or dependency import failure

Record the interpreter and package state:

```bash
.venv/bin/python --version
.venv/bin/python -m pip --version
.venv/bin/python -m pip check
.venv/bin/python -m pip list
```

Confirm the failing feature's extra in
[Installation](setup/INSTALLATION.md). Common mappings are:

- `ModuleNotFoundError: mujoco`: install `.[simulation]`;
- `ModuleNotFoundError: serial`: install `.[hardware]`;
- `ModuleNotFoundError: pyrealsense2`: install a compatible librealsense stack
  and `.[realsense]`;
- `ModuleNotFoundError: h5py` or LeRobot export failure: install
  `.[dataset-export]`;
- `ModuleNotFoundError: torch`: install a platform-appropriate PyTorch build
  or `.[training]`;
- MediaPipe/OpenCV teleoperation import failure: install `.[vision-teleop]`.

Do not solve a binary-wheel failure by mixing unrelated system, Conda, and pip
libraries in the same environment. Python 3.11 is a conservative retry when an
optional robotics/ML wheel is unavailable for a newer interpreter. Jetson
requires JetPack-compatible ARM64 packages rather than desktop wheels.

## Doctor reports `not_ready`

Inspect `problems`, `repository.required_paths`, and
`repository.configurations.errors` in the JSON report.

- Missing NumPy or PyYAML is a base-install failure.
- A missing default simulation config or MJCF usually means the source bundle
  is incomplete or `EMBODIED_LAB_ROOT` points at the wrong directory.
- A YAML error identifies the exact file; fix it through the owning schema,
  not merely until `yaml.safe_load` succeeds.
- Missing MuJoCo, pyserial, RealSense, HDF5, or PyTorch is reported as an
  optional capability and should be installed only for the intended role.

Doctor does not prove hardware connectivity. Seeing `/dev/tty*` or
`/dev/video*` only proves that a device path exists at inventory time.

## CUDA or PyTorch mismatch

Collect distinct layers:

```bash
nvidia-smi
nvcc --version
.venv/bin/python - <<'PY'
import torch
print("torch", torch.__version__)
print("compiled CUDA", torch.version.cuda)
print("CUDA available", torch.cuda.is_available())
print("device count", torch.cuda.device_count())
print("cuDNN", torch.backends.cudnn.version())
PY
```

Interpret them correctly:

- `nvidia-smi` reports the host driver and visible GPUs;
- `nvcc` reports the optional system CUDA toolkit;
- `torch.version.cuda` reports the CUDA version used to build PyTorch;
- `torch.cuda.is_available()` tests whether that build can use the current
  driver/device environment.

A system toolkit is not required for every prebuilt PyTorch wheel, and its
version need not equal `torch.version.cuda`. If `nvidia-smi` fails, fix the
host driver/runtime before Python. If `nvidia-smi` works but PyTorch sees no
GPU, inspect the installed PyTorch build, `CUDA_VISIBLE_DEVICES`, container GPU
exposure, permissions, and driver compatibility.

This repository currently has no model trainer. Use:

```bash
.venv/bin/embodied-lab distributed-smoke --check
```

before attempting collectives. See
[Distributed training](training/DISTRIBUTED_TRAINING.md) for Gloo/NCCL,
multi-node, Slurm, shared-memory, and profiling diagnosis.

## MuJoCo import or model-load failure

First isolate the headless maintained path:

```bash
.venv/bin/python -c "import mujoco; print(mujoco.__version__)"
.venv/bin/embodied-lab sim smoke
```

If import fails, reinstall `.[simulation]` in the active interpreter. If model
loading fails:

1. confirm `assets/jaka_rh56_visual_coacd.xml` exists;
2. preserve the repository-relative MJCF/mesh layout;
3. check case-sensitive filenames on Linux;
4. inspect the full MuJoCo XML error, including referenced line/path;
5. verify `EMBODIED_LAB_ROOT` and current config selection.

Do not replace a missing mesh or equality constraint with an empty placeholder.
The RH56 underactuated/mimic approximation depends on the committed model
constraints.

For display failures, separate headless stepping from rendering:

```bash
./scripts/run_quest_jaka_sim_demo.sh --help
```

- `DISPLAY is not set` or X authorization failure: select a real desktop X11
  display and matching `XAUTHORITY`, or use headless smoke.
- EGL error on an NVIDIA server: verify EGL libraries, driver visibility, and
  container GPU access before setting `MUJOCO_GL=egl`.
- OSMesa error: install the host OSMesa runtime before selecting
  `MUJOCO_GL=osmesa`.
- Do not force a rendering backend globally; a setting valid on one host may
  break another.

## RealSense camera is not found

Do not guess the device from `/dev/video0`. Check:

```bash
lsusb
ls -l /dev/video*
rs-enumerate-devices
.venv/bin/python -c "import pyrealsense2 as rs; print(rs.__version__)"
.venv/bin/python tools/check_realsense_stream.py --help
```

If no device appears, inspect cable, power, USB port, hub, kernel log, and
librealsense udev rules. If the device appears only under `sudo`, fix udev/group
permissions; do not run the robot stack as root.

If a configured serial is wrong, update
`configs/data_collection/physical_collection.yaml` and preserve role
assignment. Verify the identity before a physical run; the YAML itself does
not authorize opening a camera or robot connection.

`tools/check_realsense_stream.py` opens a camera and writes snapshots. Use it
only when an actual camera probe is intended:

```bash
.venv/bin/python tools/check_realsense_stream.py \
  --serial REPLACE_WITH_SERIAL \
  --width 640 --height 480 --fps 30 \
  --duration-sec 3 \
  --snapshot-dir artifacts/realsense/check
```

Review observed FPS, RGB/depth shape, timestamp skew, depth-valid fraction,
intrinsics, and point count.

## RealSense USB bandwidth or profile errors

Two D435 streams can exceed a weak hub or shared USB controller. Symptoms
include profile-start errors, frame timeouts, low FPS, corrupted frames, or one
camera disappearing.

1. Confirm each camera negotiates the expected USB mode.
2. Test each camera alone with the intended profile.
3. Place cameras on separate host controllers where possible.
4. Avoid unpowered hubs and marginal cables.
5. Reduce resolution/FPS only as an explicit experiment/config change.
6. Record any profile fallback; do not silently train with a different image
   shape or frame rate.

On Linux, inspect topology and bandwidth with the host's USB tools such as
`lsusb -t`. The target dual-camera/profile combination is not currently
physically validated.

## Camera freezes, depth artifacts, or RGB/depth mismatch

Detect freezes using frame number, device timestamp, host receive timestamp,
and image-content change—not host FPS alone. A repeated frame must remain
marked as repeated rather than relabelled as fresh.

For depth:

- verify the stored array is meters where the schema says meters;
- verify alignment status before pairing RGB pixels with depth;
- inspect zeros/non-finite values and valid-depth fraction;
- do not enable temporal or hole-filling filters blindly around moving
  fingers/objects;
- keep camera intrinsics tied to the actual stream profile.

If RGB and depth timestamp domains differ or skew exceeds the configured
threshold, reject/mark the sample according to the collector contract. Do not
shift timestamps after recording to make a quality check pass.

## Quest packets do not arrive

Inspect the host-side tool first:

```bash
.venv/bin/python tools/quest_hand_tracking_streamer.py --help
```

Check:

- headset and host are on the intended network;
- the configured project IP is the host's reachable IPv4, not loopback,
  VPN, stale DHCP, or another interface;
- host firewall allows the selected UDP port only on the trusted interface;
- bind address/port match the headset senders;
- `--allowed-sender`, if set, matches the current headset address;
- HTS and CTRL packet senders use the current protocol/port;
- no second process owns the port.

Use `ss -lunp` on Linux to inspect UDP ownership. A packet capture can contain
personal motion data; handle it according to local retention policy.

The current arm clutch requires a fresh left-controller signal,
release-before-first-press, right wrist data, and head pose at the capture
edge. Hand Tracking Streamer data alone does not synthesize a Touch-controller
clutch. If Quest input becomes stale during a physical session, the retained
policy stops/holds according to the owning safety state; do not extend timeout
values to conceal packet loss.

If packets arrive but engagement does not occur, fully release the left index
before pressing it again and inspect the effective configuration's controller,
wrist, and head freshness limits. Do not substitute a keyboard/SPACE clutch
from an older prototype.

## Quest timestamp or latency confusion

Quest source monotonic time and host monotonic time have independent epochs.
Host receipt time supports freshness and ordering but cannot produce absolute
one-way network latency without a separately validated clock relation.

Record:

- source sequence/timestamp;
- host kernel/application receive time;
- parsed observation time;
- control target and emitted command time;
- packet loss, reordering, duplicates, and frozen duration.

Synchronize multiple Linux hosts with the site's NTP/PTP/chrony policy, but
retain device and host clocks separately. Wall-clock synchronization does not
make a camera hardware clock or Quest monotonic epoch equal to host monotonic
time.

## JAKA native worker does not build

The real worker is Linux-only. Configure and build:

```bash
cmake -S native/jaka_servo_worker -B build/jaka_servo_worker
cmake --build build/jaka_servo_worker -j
```

On macOS or another non-Linux host, only the portable resampler is expected;
the JAKA executable is intentionally skipped. On Linux:

```bash
file third_party/jaka_sdk/v2.2.7/linux/python3/*/libjakaAPI.so
file build/jaka_servo_worker/jaka_servo_worker
ldd build/jaka_servo_worker/jaka_servo_worker
```

The host architecture must match the selected x86_64 or aarch64 vendor
library. Preserve the build RPATH or otherwise provide the same reviewed SDK
library. Do not replace the vendor SDK snapshot casually; ABI and controller
compatibility must be revalidated.

## JAKA connection or command gate fails

Inspect only help until the operator has selected a specific stage and checked
its runtime safety prerequisites:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
./scripts/run_quest_jaka_bounded_teleop.sh --help
```

Before any connection, verify the controller-side payload/COM, installation,
TCP/user frame, alarms, collision status, safety limits, enable state,
network route, exclusive SDK ownership, emergency stop, workspace, and manual
stop access. The current command worker intentionally uses one SDK session;
do not add a second monitor session as a workaround.

Failure classes:

- native executable/library missing: build/linkage problem;
- robot IP unreachable: host route/interface/controller network problem;
- login/enable/servo/EDG failure: controller/SDK state; do not auto-clear or
  auto-retry;
- `command_stream_timeout` or producer liveness loss: real hard stop;
- `HOLD_REJECTED`: candidate infeasibility with a fresh heartbeat; do not
  mislabel it as transport liveness loss;
- tracking, output velocity/acceleration/jerk, singularity, joint limit, or
  collision rejection: preserve the gate and diagnose the input/trajectory;
- controller collision/alarm, SDK error, emergency stop, or timing hard fault:
  stop, clean up, and review evidence before another run.

J5 proximity is diagnostic metadata; the full Jacobian and directional
recovery checks remain the singularity authority. Do not raise a boundary
merely to remove a `HOLD_REJECTED` message.

Do not increase timeouts or safety limits merely to complete a trial. Current
physical status and unresolved risks are in
[Known limitations](status/known_limitations.md).

## RH56 serial device is not found

Inspect:

```bash
lsusb
ls -l /dev/serial/by-id/
fuser -v /dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID
./scripts/run_quest_rh56_hand_test.sh --help
```

Use a stable by-id path and correct group/udev permissions. Do not run the
stack as root and do not choose `/dev/ttyUSB0` by enumeration order.
Custom Thor CH341 drivers may expose `/dev/ttyCH341USB<N>` without a by-id
link; the explicit fallback requires verifying VID:PID, driver, permissions,
and zero occupants.

The preflight mode does not open serial:

```bash
./scripts/run_quest_rh56_hand_test.sh \
  --device /dev/serial/by-id/REPLACE_WITH_ACTUAL_ADAPTER_ID \
  --preflight-only
```

Serial-open read-only and command modes require their applicable explicit
operation mode and runtime safety prerequisites, plus the staged procedure in
[RH56 operation](operation/rh56_operation.md).

## RH56 timeout, checksum, protocol, or feedback fault

Stop the gate and preserve telemetry. Check adapter identity, baud/address,
wiring/termination, exclusive port ownership, cable/USB stability, and host
scheduling. Do not clear errors or write speed/force as an automatic recovery.

Review independently:

- `ANGLE_ACT`: measured six-channel actuator feedback;
- `CURRENT` and `FORCE_ACT`: raw device feedback/proxies, not calibrated joint
  torque or tactile force;
- `ERROR`: nonzero remains a fault;
- `STATUS`: raw response whose nonzero meanings are not validated;
- read latency, feedback age, duplicate ratio, checksum/protocol errors, and
  command attempts.

A stale or invalid response enters the hand fault path. Grip release holds the
last safe target; it does not automatically open the hand. Do not treat a
command attempt counter as confirmed physical motion.

## Control rate is too low

Measure before changing configuration:

- input arrival and accepted-target generation rate;
- IK/control compute time and queue age;
- native wake/completion lateness;
- RH56 command and each feedback-register rate;
- camera capture/decode/write time;
- disk flush and telemetry overhead.

Avoid high-frequency `print`, synchronous image compression, or checkpoint
writes in a control thread. A slower observed rate can be a device or I/O
limit, not just a scheduler setting. Preserve command freshness and hard timing
contracts; never relax watchdogs to raise an average-rate number.

## Device time synchronization

Store device time, host receive monotonic time, and control/emission time as
separate fields. For multiple cameras and hosts:

1. configure NTP/PTP/chrony according to the deployment;
2. record clock source and timestamp domain per stream;
3. measure skew and drift before collection;
4. preserve missing, duplicate, and out-of-order markers;
5. resample only in a versioned training adapter, never by rewriting raw
   timestamps.

Host wall-clock synchronization does not align RealSense device clocks, Quest
monotonic time, or robot controller time automatically.

## Data writing is slow or disk fills

Check capacity, inode count, filesystem type, and write latency:

```bash
df -h data artifacts
df -i data artifacts
du -sh data/episodes artifacts
```

Keep raw episodes and final checkpoints on durable storage. Use node-local
NVMe only for reproducible caches and temporary shards. Do not put the only
copy on `/tmp` or ephemeral cluster scratch.

Symptoms of blocking I/O include control-loop jitter, camera queue growth,
missing frames, and long finalize/checkpoint time. Separate capture from heavy
export/compression where the current architecture allows it, flush bounded
metadata, and validate the finalized episode:

```bash
.venv/bin/embodied-lab dataset validate \
  data/episodes/EPISODE_ID \
  --output artifacts/dataset/EPISODE_ID.validation.json
```

Do not rename an incomplete temporary episode as complete. Preserve the
manifest/status and repair the writer or storage problem before training.

## Dataset validation or export fails

Run deep validation first; use `--fast` only for a clearly labelled quick
inventory:

```bash
.venv/bin/embodied-lab dataset validate data/episodes/EPISODE_ID
.venv/bin/embodied-lab dataset manifest \
  data/episodes \
  artifacts/dataset/manifest.json
```

Typical failures are missing files, count/shape/dtype mismatch, timestamp
regression, non-finite values, missing camera frames, invalid schema, or an
episode not finalized as training-eligible. Do not delete the failing sample
to make export pass; retain it with a failure category unless it is proven
corrupt and recoverable from authoritative raw data.

ACT HDF5 and LeRobot exports require `.[dataset-export]`. They are derived
artifacts; regenerate them from the canonical episode rather than editing them
in place. Dataset splits must remain episode-level so adjacent frames from one
demonstration do not leak across train and validation.

## Checkpoint cannot load or resume

There is no repository model trainer or implemented model-checkpoint resume
path yet. A failure from an external ACT, Diffusion Policy, or OpenPI/pi0
integration must be diagnosed in that integration without claiming the core
repository produced the checkpoint.

For future integrations verify:

- checkpoint completed its temporary-write/atomic-rename sequence;
- file/directory hash and format are valid;
- model architecture, action dimension, camera order, image size, temporal
  horizon, and action chunk match;
- dataset/schema version and normalization statistics match;
- optimizer, scheduler, scaler, RNG, epoch, and global step exist for full
  resume;
- precision, world size, global batch, and learning rate changes are explicit;
- DDP saved unwrapped weights and only rank zero wrote the main checkpoint;
- framework-native sharded checkpoints were merged/exported by that framework.

Do not load a partially written `.tmp` file or silently fall back to weights
only. See the checkpoint contract in
[Distributed training](training/DISTRIBUTED_TRAINING.md).

## Jetson Thor memory pressure

First identify which memory is exhausted: system RAM, unified GPU memory,
TensorRT workspace, page cache, or disk. Record:

```bash
free -h
df -h
tegrastats
```

Then profile a bounded replay, not a live robot. Possible measured reductions
include lower camera resolution/FPS, fewer cameras, shorter observation/action
horizons, smaller inference batch, FP16/BF16 where supported, fewer DataLoader
workers, bounded queues, and delayed/offline visualization.

Do not enable swap as a substitute for meeting real-time latency, and do not
assume TensorRT conversion is lossless or supported. Compare fixed replay
outputs before/after export, measure peak memory and end-to-end latency, and
only then consider a separately scheduled and operator-initiated hardware gate. The current
repository has no validated Thor inference deployment.

## When to stop troubleshooting

Stop and request new authority rather than continuing when resolution would
require:

- connecting to or commanding hardware not explicitly selected and gated for this
  session;
- changing payload, TCP, installation, collision, or controller safety state;
- widening a motion envelope or weakening a watchdog/safety threshold;
- deleting raw data, calibration, model weights, or physical evidence;
- substituting an uncalibrated transform or guessed register meaning;
- adopting a different trainer/framework/checkpoint format.

Installation details are in [Installation](setup/INSTALLATION.md), and
configuration ownership is in
[Configuration](configuration/CONFIGURATION.md).
