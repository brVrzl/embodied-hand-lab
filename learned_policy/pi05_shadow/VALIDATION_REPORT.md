# OpenPI π0.5-DROID shadow validation on Jetson AGX Thor

> **Status: historical environment snapshot dated 2026-07-22.** Versions,
> device paths, resource measurements, and backend failures below apply to that
> audit. For the current inference-only safety boundary, see
> [`README.md`](README.md).

Date: 2026-07-22 (Asia/Shanghai)

## Result

The independent OpenPI checkout, official `uv` environment, dual-camera probe,
and inference-only websocket client are installed and validated. Full policy
inference is blocked before model load by the official pinned GPU runtimes:

- JAX sees NVIDIA Thor but JAX/XLA 0.5.3 does not support compute capability
  11.0. Its first trivial GPU operation fails with `XlaRuntimeError` after XLA
  reports `Unknown compute capability 11.0`, substitutes `sm_101`, and reports
  that the bundled `ptxas` is too old.
- The same official ARM64 lock resolves `torch==2.7.1+cpu`, so the documented
  OpenPI PyTorch path has no CUDA device in this environment.

No physical robot connection was initiated. No JAKA or RH56 process was found,
and none of the new files imports a robot, teleoperation, target, servo, or
command package.

## Host audit

- Architecture: `aarch64`.
- OS: Ubuntu 24.04.4 LTS, kernel `6.8.12-1021-tegra`.
- Jetson Linux: R39.2 (`nvidia-l4t-*` 39.2.0). This corresponds to JetPack 7.2;
  the `nvidia-jetpack` meta-package itself is not installed.
- NVIDIA driver: 595.78; `nvidia-smi` reports NVIDIA Thor and CUDA 13.2.
- CUDA runtime packages are installed, but `nvcc` is not on `PATH`.
- System Python: 3.12.3. OpenPI isolated Python: CPython 3.11.15 ARM64.
- `uv`: 0.11.31, installed user-locally at `/home/thor/.local/bin/uv`.
- Storage before installation: 860 GiB available of 936 GiB. OpenPI `.venv`
  uses 6.5 GiB; 855+ GiB remained after installation.
- Memory at final audit: 122 GiB total, 5.1 GiB non-cache used, 117 GiB
  available, no swap. `tegrastats` reported roughly 99.6/125.7 GB resident plus
  cache/allocation accounting. No model was loaded.
- Thor uses unified memory; `nvidia-smi` reports GPU memory accounting as “Not
  Supported.” It showed only Xorg (108 MiB) and GNOME Shell (49 MiB), and 0%
  GPU utilization at the final audit.
- Docker 29.1.3 is installed and the daemon is running, but the current user
  does not have permission to access `/var/run/docker.sock`.

NVIDIA identifies Jetson Linux 39.2 as JetPack 7.2 with CUDA 13.2.1 in its
[JetPack release page](https://developer.nvidia.com/embedded/jetpack/downloads).

## OpenPI checkout and install

- Repository: `https://github.com/Physical-Intelligence/openpi.git`
- Path: `/home/thor/projects/openpi` (a sibling of `embodied_lab`).
- Commit: `15a9616a00943ada6c20a0f158e3adb39df2ccac`.
- Submodules:
  - `third_party/aloha`: `d1dc83afd89ded4379851257fe5d85632d31d5ec`
  - `third_party/libero`: `f78abd68ee283de9f9be3c8f7e2a9ad60246e95c`
- Checkout status after install: clean `main`, tracking `origin/main`.

Both requested commands completed without altered pins:

```text
GIT_LFS_SKIP_SMUDGE=1 uv sync                 # exit 0, 233 packages
GIT_LFS_SKIP_SMUDGE=1 uv pip install -e .     # exit 0
```

`openpi` imports from the checkout, and JAX 0.5.3 / jaxlib 0.5.3 import. JAX
enumerates `[CudaDevice(id=0)]` and selects the GPU backend, but cannot execute
the first array creation. PyTorch imports as `2.7.1+cpu`; `torch.version.cuda`
is `None` and `torch.cuda.is_available()` is false.

`scripts/serve_policy.py --help` imports and exits successfully. A DROID policy
server cannot complete model initialization because of the JAX blocker above.

## Checkpoint

- Selected model/config: official `pi05_droid`.
- URI: `gs://openpi-assets/checkpoints/pi05_droid`.
- Remote resolution: successful, anonymous GCS listing of 20 files totaling
  12,429,488,598 bytes. It contains `assets/droid/norm_stats.json` and JAX
  Orbax parameters.
- Intended final cache:
  `/home/thor/.cache/openpi/openpi-assets/checkpoints/pi05_droid`.
- Current partial cache:
  `/home/thor/.cache/openpi/openpi-assets/checkpoints/pi05_droid.partial`
  (811 MiB at stop).

The full 11.6 GiB transfer was stopped after the backend blocker was proven;
at the observed 2.3 MiB/s it had about 85 minutes remaining and could not be
executed by either installed backend. Nothing was substituted for the official
checkpoint.

Model load time, cold inference latency, warm inference latency, and model GPU
memory are **unavailable**, not zero: execution is blocked before model load.

## Camera audit and mapping

Both devices are Intel RealSense D435 RGB-D cameras. Their RGB nodes use YUYV
and advertise 320×180, 320×240, 424×240, 640×360, 640×480, 848×480, and
960×540 up to 60 FPS, plus 1280×720 and 1920×1080 up to 30 FPS. The probe uses
848×480 at 30 FPS.

| Policy role | Serial | Stable RGB device | Orientation |
|---|---|---|---|
| Fixed scene / exterior | `315223123328` | `/dev/v4l/by-id/...315223123328-video-index0` (`/dev/video4`) | 0° |
| Wrist | `315223123181` | `/dev/v4l/by-id/...315223123181-video-index0` (`/dev/video10`) | rotate 180° |

The mapping and wrist correction were verified visually from live frames: the
scene camera has the fixed overhead workspace view, while the wrist image shows
the hand/fingers at its mounting edge and was physically upside-down before
rotation.

Five-second synchronized probe at 848×480/30 FPS:

| Metric | Scene | Wrist |
|---|---:|---:|
| Frames | 140 | 141 |
| Measured FPS | 29.163 | 29.360 |
| Read failures | 0 | 0 |
| Inferred dropped frames | 4 | 3 |
| Median inter-frame interval | 33.359 ms | 33.362 ms |
| Maximum interval | 64.933 ms | 65.877 ms |

Nearest-frame host-monotonic timestamp skew: median 1.433 ms, p95 1.592 ms,
maximum 31.301 ms. The timestamp is sampled immediately after each independent
V4L2 `read`; it is not a hardware-trigger timestamp.

Evidence:

- `artifacts/camera_probe_20260722_203539/report.json`
- `artifacts/camera_probe_20260722_203539/scene_wrist.jpg`

## Checked-out π0.5-DROID schemas

These fields come from `src/openpi/policies/droid_policy.py` and the
`pi05_droid` entry in `src/openpi/training/config.py` at the recorded commit.

External request:

- `observation/exterior_image_1_left`: scene HWC RGB `uint8`; shadow client
  sends `(224, 224, 3)`.
- `observation/wrist_image_left`: wrist HWC RGB `uint8`; shadow client sends
  `(224, 224, 3)` after the verified 180° mount correction.
- `observation/joint_position`: seven DROID/Franka joint positions, shape `(7,)`.
- `observation/gripper_position`: one gripper position, shape `(1,)`.
- `prompt`: string.

`DroidInputs(ModelType.PI05)` concatenates the state to shape `(8,)`, maps the
images to `base_0_rgb` and `left_wrist_0_rgb`, creates a zero-valued
`right_wrist_0_rgb`, and uses image masks `(true, true, false)`. Model transforms
resize to 224×224, tokenize the prompt, normalize using checkpoint DROID stats,
and pad state/actions to the model action dimension of 32.

The model produces an internal `(15, 32)` chunk because `pi05_droid` sets
`action_horizon=15` and inherits `action_dim=32`. `DroidOutputs` returns only
the first eight dimensions, so the external action chunk is `(15, 8)`:

- dimensions 0–6: DROID/Franka joint-velocity actions;
- dimension 7: gripper-position action.

The current `examples/droid/main.py` comment/assert still says `(10, 8)`, but
that is stale relative to the checked-out `pi05_droid` config. The shadow client
therefore validates `(15, 8)` and never copies the stale example assumption.

JAKA mini2 has six arm joints. The client rejects a six-element state and never
pads, drops, clips, binarizes, publishes, or executes a DROID action.

## Shadow pipeline validation

Files are isolated under this directory:

- `camera_probe.py`: V4L2-only synchronized capture and timing report.
- `shadow_client.py`: strict state-file reader, RGB preprocessing, websocket
  query, action validation, and JSONL prediction logger.
- `test_shadow_safety.py`: static import boundary and schema regression tests.

Validation completed:

- `pytest`: 3 passed.
- `ruff`: all checks passed.
- Live synchronized camera probe: passed.
- Observation-only smoke for all three requested prompts: passed using an
  explicitly tagged synthetic zero DROID state; no policy query was attempted.
- Artifact: `artifacts/shadow_20260722_203552/run_metadata.json`.

No state was read from the physical JAKA because the task explicitly prohibited
connecting to it and no already-running read-only state process/topic was found.
The shadow client accepts only an external `openpi.pi05_droid_state.v1` JSON
snapshot and rejects the JAKA six-joint shape.

## Smallest isolated PyTorch next step

Do not change the official OpenPI `.venv`. Use a separate NVIDIA container and
make the backend deviation explicit:

1. Grant the operator access to the existing Docker daemon (an administrative
   decision outside this task).
2. Validate an NVIDIA Framework PyTorch image on Thor. NVIDIA’s current
   [Thor Docker guide](https://docs.nvidia.com/jetson/agx-thor-devkit/user-guide/latest/setup_docker.html)
   demonstrates `nvcr.io/nvidia/pytorch:25.08-py3` with CUDA-visible NVIDIA
   Thor. Use that known-good baseline or a newer image only after a one-line
   CUDA tensor smoke test.
3. Mount `/home/thor/projects/openpi` and a separate checkpoint/cache directory
   read-only where possible. Preserve the container’s Thor-enabled PyTorch and
   install the remaining OpenPI dependencies in a container-local environment;
   do not let `uv sync` replace it with the lockfile’s CPU-only ARM64 torch.
4. Apply the OpenPI-documented Transformers 4.53.2 replacement inside only that
   container environment, using copied files rather than the shared `uv` cache.
5. Finish the official JAX checkpoint download and run the official
   `examples/convert_jax_model_to_pytorch.py` with `JAX_PLATFORMS=cpu`, writing
   `model.safetensors` to a separate directory. Then point the unmodified
   `serve_policy.py` at that converted directory.
6. Re-run model load, three cold/warm shadow queries, action validation, and
   unified-memory measurement before considering data-collection integration.

This is the smallest fallback because it changes only the backend runtime and
checkpoint representation in an isolated container. It does not alter OpenPI’s
lockfile, the validated `embodied_lab` environment, teleoperation, IK, EDG,
accepted-target logic, or physical safety behavior.

## Readiness

- Dual-camera RGB shadow acquisition: ready, with saved evidence and measured
  timing. The existing checked-in RealSense config still names older,
  disconnected serials and was intentionally not modified.
- π0.5-DROID GPU inference on this host: not ready due to the exact JAX and
  PyTorch ARM64/Thor blockers above.
- Physical execution: intentionally absent and not authorized.
- Data collection / later fine-tuning for JAKA+RH56: not yet ready as an
  end-to-end labeled pipeline. It still needs an approved existing read-only
  state feed, synchronized six-DOF arm/hand action labels, an explicit
  JAKA/RH56 observation/action schema, and a fine-tuned or validated embodiment
  adapter. The current DROID checkpoint’s 7+1 action semantics must not be used
  as JAKA/RH56 targets.
