# pi0.5 LoRA / JAKA Mini2 + RH56 baseline

This is a training-only experiment. It does not contain robot rollout or
actuator commands. The source ACT data is not modified.

## Fixed experiment definition

Task prompt: `Pick up the bottle and place it on the cardboard box.`

The latest exact-match `physical_bottle_nominal52` audit set contains 52
explicitly accepted logical demonstrations and 33,111 frames at 30 Hz. The
machine-readable run manifest is `outputs/training/pi05_rh56/audited_manifest.json`.
Its acceptance evidence is:

- `configs/training/shared/physical_bottle.yaml`
- `docs/data/PHYSICAL_TRAINING_DATASET.md` and its generated manifest
- `data/training/physical_bottle_v4_nominal52/manifests/logical_segments.json`

The locked ACT split is 37 train / 15 validation logical episodes, recorded in
`configs/training/shared/physical_bottle.yaml` and
`data/training/physical_bottle_v4_nominal52/act/manifests/splits.json`. The
OpenPI training view is
`outputs/training/pi05_rh56/lerobot_home_v2/local/pi05_rh56_train`; it contains
only the 37 ACT training episodes. The 52-episode audited manifest remains the
complete acceptance record, and the 15 validation episodes are explicitly
held out rather than silently mixed into training.

The derived view contains copied parquet files and source-linked videos; the
source master remains immutable. Four source relative timestamps contain one
60 ms interval despite contiguous frame indices and 30 Hz video. The derived
view regularizes only the LeRobot `timestamp` column to `frame_index / 30`; the
original `timestamp_ns`, state, action, force, and video data are preserved.
The four episode IDs are recorded in `meta/openpi_provenance.json`.

Each state and native command has 12 dimensions: six JAKA arm channels
followed by six active RH56 actuator channels in
`index, middle, ring, pinky, thumb_close, thumb_lateral` order. Actions are
absolute native targets; no delta conversion is applied. Raw force observations
remain in source parquet but are excluded from this baseline.

OpenPI's released `pi05_base` action projections are 32-wide. The adapter
keeps the real 12-dimensional command at the dataset/output boundary and uses
OpenPI's standard zero-padding to a 32-dimensional internal model interface,
so the official base checkpoint can load without inventing RH56 channels.

## Pinned software and configuration

- OpenPI checkout: `third_party/openpi` (override with `PI05_OPENPI_REPO` when using a separately audited checkout)
- OpenPI commit: `15a9616a00943ada6c20a0f158e3adb39df2ccac`
- Project pin: `configs/training/pi05/openpi.lock.json`
- Runtime check: `training/pi05/scripts/check_openpi.sh --require-image`
- Backend: official upstream JAX
- Model: `pi05_base` initialization, flow-matching pi0.5
- LoRA variants: `gemma_2b_lora` and `gemma_300m_lora`
- Actual command action dimension: 12
- Internal OpenPI model action dimension: 32
- Action horizon: 16 frames
- Images: workspace and wrist, source 640x480 RGB, mapped to `base_0_rgb`
  and `left_wrist_0_rgb`; the masked third slot is synthetic and is not a
  claimed camera
- Initial batch: 4, effective batch: 4, data workers: 0
- Learning rate: 100-step warmup, peak `1e-5`, cosine decay to `1e-6`
- AdamW, gradient clipping 1.0, EMA disabled
- Seed: 20260814
- Checkpoint interval: 100 steps; keep interval: 500 steps
- Default run length: 20,000 optimizer steps
- W&B: disabled; local logs are authoritative

The current upstream checkout contains two pre-existing Thor-specific local
patches in `src/openpi/models/model.py` and
`src/openpi/training/sharding.py`. The exact dirty state is recorded with the
run provenance, and the checkout is mounted read-only into training containers.
The project runner also applies two runtime-only compatibility shims for the
Thor image: Flax 0.12.2 NNX variable selection uses key identity, and Orbax
0.11.39 reads `StepMetadata.item_metadata.tree` while restoring the upstream
base checkpoint. Neither shim changes model weights, LoRA filters, or
checkpoint contents.

## Validation and operation

The output directory is under ignored `outputs/`; do not commit datasets,
caches, logs, or checkpoints. The staged commands are `manifest`,
`build-view`, `validate-view`, `norm-stats`, `data-smoke`, `model-smoke`, and
`checkpoint-smoke`, and `val-eval`. They validate the audited data, OpenPI q01/q99/mean/std,
decoded images, language, finite values, LoRA parameter freezing, one JAX
forward/backward/update, and actual checkpoint resume.

Start or resume the supervised run:

```bash
training/pi05/scripts/train_weekend.sh start
```

Inspect it later:

```bash
training/pi05/scripts/status.sh
cat outputs/training/pi05_rh56/status.json
```

Stop it without deleting checkpoints:

```bash
training/pi05/scripts/train_weekend.sh stop
```

After a reboot, the single recovery command is
`training/pi05/scripts/train_weekend.sh resume`.

The supervisor uses the newest checkpoint containing `train_state`, `params`,
and `assets`, passes `--resume`, refuses overwrite, serializes launches with a
lock, appends timestamped logs, and stops after three crashes in 15 minutes so
a broken run is not left retrying indefinitely. Dynamic provenance and status
files are written under `outputs/training/pi05_rh56/`.
