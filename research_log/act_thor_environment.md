# ACT Thor environment

Validation date: 2026-08-11 (Asia/Shanghai)

## Scope and safety

This investigation was entirely offline. It did not access or command either physical device, start a collection/teleoperation process, or change a raw/materialized episode. The source of truth was the existing worktree and the already-materialized master dataset.

## Thor host

| Item | Observed value |
|---|---|
| Hostname | `thor` |
| Board | NVIDIA Jetson AGX Thor Developer Kit |
| OS | Ubuntu 24.04.4 LTS, aarch64 |
| Kernel | `6.8.12-1021-tegra` |
| Jetson/L4T packages | `39.2.0-20260601141651` |
| GPU | NVIDIA Thor |
| Driver | 595.78 |
| `nvidia-smi` CUDA API | 13.2 |
| PyTorch-visible GPU | Yes: `NVIDIA Thor`, capability `(11, 0)` |

## ACT installations inspected

The maintained checkout at `/home/thor/LeRobot/src` is the official Hugging Face LeRobot repository. It was at clean commit `f66e5128ecb2456e8c54a63d15404fa59c16aebc` and its remote points to `huggingface/lerobot`. The host-side `/home/thor/LeRobot/.venv` did not contain a usable PyTorch installation. The existing OpenPI environment contains an old LeRobot 0.1.0/CPU dependency and was not selected. This repository's `act-smoke` is intentionally only an adapter smoke path, not a maintained trainer.

The already-working LeRobot development image was therefore selected:

| Item | Selected value |
|---|---|
| Container image | `jaka-lerobot-dev:snapshot-before-raw-mount` |
| Image ID | `sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e` |
| Persistent user environment | `/home/thor/LeRobot/container-home:/home/lerobot` |
| Python | 3.12.3 |
| LeRobot | 0.6.2, `/usr/local/lib/python3.12/dist-packages/lerobot` |
| PyTorch | `2.13.0a0+8145d630e8.nv26.06` |
| torchvision | `0.27.0a0+499ca510.nv26.06` |
| PyTorch CUDA build/runtime | 13.3; NVIDIA compatibility layer over the host driver |
| Accelerate | 1.14.0 |

The difference between the CUDA 13.2 value reported by the host driver API and the CUDA 13.3 PyTorch build is expected for this existing NVIDIA container compatibility environment; a real CUDA tensor allocation and all training/backward operations succeeded.

## Exact implementation selected

The validation used the maintained LeRobot 0.6.2 ACT path, not a new or copied implementation:

- policy/config/processor: `/home/thor/LeRobot/src/src/lerobot/policies/act/`
- training entry point: `/home/thor/LeRobot/src/src/lerobot/scripts/lerobot_train.py`
- dataset loader: `/home/thor/LeRobot/src/src/lerobot/datasets/lerobot_dataset.py`
- installed trainer command: `python -m lerobot.scripts.lerobot_train`

This path was chosen because it is the current official local checkout and has a working train, checkpoint, processor serialization, reload, and inference path on the Thor GPU. No second LeRobot copy was vendored into `embodied_lab`.

## Re-entering the environment

The validation environment can be re-entered without exposing robot devices or networking:

```bash
docker run --rm --runtime=nvidia --ipc=host --network none \
  -v /home/thor/LeRobot/container-home:/home/lerobot:rw \
  -v /home/thor/projects/embodied_lab:/workspace/embodied_lab:rw \
  -w /workspace/embodied_lab \
  jaka-lerobot-dev:snapshot-before-raw-mount bash
```

The experiment output is `outputs/act/physical_bottle_5demo_validation_20260811_150941`. This timestamped directory was new and did not overwrite another experiment.
