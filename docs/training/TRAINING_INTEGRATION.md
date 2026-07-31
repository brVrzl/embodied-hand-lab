# Training integration

## Current status

This repository prepares canonical episodes and derived training views; it
does not contain a model trainer.

| Capability | Repository status |
|---|---|
| Canonical single-episode state/action/RGB-D archive | Implemented; the wired producer uses simulated arm/hand state |
| Deep/fast archive validation | Implemented |
| Deterministic episode-level manifest and split | Implemented |
| Train-only low-dimensional normalization statistics | Implemented |
| ACT-layout per-episode HDF5 export | Implemented 12-D project view; not directly loadable by the unmodified upstream ACT reference trainer |
| LeRobot v3 per-episode export | Implemented exporter; no multi-episode training loader |
| ACT training/evaluation | Not implemented |
| Diffusion Policy adapter/training/evaluation | Not implemented |
| OpenPI/π0 fine-tuning | Not implemented |
| OpenPI π0.5-DROID shadow query | Inference-only, embodiment-incompatible, and unable to command a robot |
| Single-/multi-GPU model training | Not implemented; only a distributed communication smoke tool exists |
| Checkpoint/resume/export pipeline | Planned |
| Jetson Thor policy deployment | Planned |

The `training` optional dependency currently installs PyTorch and HDF5 support,
not ACT, Diffusion Policy, OpenPI, or a repository trainer. Do not invent a
`train` command or treat `tools/distributed_smoke_test.py` as training
validation.

The source dataset contract is
[Canonical episode dataset schema](../data/DATASET_SCHEMA.md). Distributed
launch, storage, precision, Slurm, and cluster requirements are documented in
[Distributed training readiness](DISTRIBUTED_TRAINING.md).

## Training contract supplied by the repository

### Observation

Each canonical frame provides:

- a 25-D low-dimensional state:

  ```text
  arm q measured (6)
  + arm dq measured/estimated (6)
  + TCP [x_m,y_m,z_m,qx,qy,qz,qw] (7)
  + hand actuator-space observation (6)
  ```

- `workspace` RGB, HWC `uint8`;
- `wrist` RGB, HWC `uint8`;
- lossless workspace/wrist raw depth, two-dimensional `uint16` device counts;
- optional aligned-to-RGB depth, also `uint16`;
- source provenance, trigger state, camera frame metadata, and source timing.

The camera order is always:

```text
[workspace, wrist]
```

Adapters must convert HWC RGB to the layout required by the selected framework
and must not accidentally preserve OpenCV BGR order. Raw depth becomes metric
only after applying the per-camera depth scale from episode metadata. The
smallest training baseline should omit depth rather than silently normalize
unscaled counts or mix raw and aligned depth.

### Action

Each canonical frame provides a 12-D actuator-space action:

```text
[J1,J2,J3,J4,J5,J6,H1,H2,H3,H4,H5,H6]
```

Canonical-v1 arm and hand values are absolute targets in radians. The arm
target is the immutable shared `AcceptedArmTarget`; it is not a Cartesian
delta, velocity action, MuJoCo state, EDG interpolation point, or rejected IK
candidate.

`action.arm_status` distinguishes a newly `accepted` target from
`held_rejected`, where the last accepted target was safely held after candidate
infeasibility. Current normalization includes both. A framework adapter must
choose and record one of these behaviors:

1. include `held_rejected` as the actually emitted safe hold action;
2. mask those steps from action loss while retaining their observations; or
3. exclude windows containing them.

The choice changes the learned behavior and cannot be implicit. The rejected
candidate is not stored as the action and must never be reconstructed as a
supervision target.

### Current physical-data limitation

The canonical-v1 hand fields require radians, but the physical RH56 driver
reports `ANGLE_ACT`, `CURRENT`, `FORCE_ACT`, `ERROR`, and `STATUS` as raw
controller fields plus a normalized closure view. A physical archive must use
a new versioned schema/training view with calibration identity and masks. Raw
counts cannot be inserted into the existing 12-D radian action or 25-D state.

The current end-to-end archive therefore supports simulation-backed data
infrastructure work, not physical JAKA/RH56 policy training.

## Reproducible preparation pipeline

### 1. Validate source episodes

```bash
.venv/bin/embodied-lab dataset validate \
  data/episodes/episode-<uuid> \
  --output data/reports/episode-<uuid>.validation.json
```

Use deep validation for release acceptance. `training_eligible=true` requires a
completed, non-empty, structurally valid episode with zero canonical missed
slots and an explicit `success_label` of `success` or `failure`. A freshly
finalized `unlabeled` episode can be structurally valid but is intentionally
ineligible.

### 2. Freeze episode-level splits

```bash
.venv/bin/embodied-lab dataset manifest \
  data/episodes \
  data/manifests/dataset-v1.json \
  --seed embodied-lab-v1 \
  --train-fraction 0.8 \
  --validation-fraction 0.1
```

The manifest hashes `<seed>:<episode_uuid>`, so every frame from an episode
stays in one split. This prevents adjacent-frame leakage but does not prevent
object, task, scene, operator, or session leakage. Before a benchmark, choose
the evaluation unit and create a group-aware policy when needed. Never choose
a new split after looking at test performance.

Current eligibility rejects `success_label=unlabeled`. The repository does not
yet provide an audited annotation command, so a controlled post-collection
review must establish `success` or `failure` before the manifest can assign a
split. Record reviewer/provenance and retained episode UUIDs outside the
current schema until that workflow is implemented. Never infer success from
`completion_status`.

Manifest creation deep-validates NPY payloads and raw JSONL by default. A
damaged finalized archive remains catalogued with validation errors and
`split=excluded`. Explicit `--fast` is an inventory-only mode and assigns no
training/validation/test splits.

### 3. Freeze train-only normalization

```bash
.venv/bin/embodied-lab dataset statistics \
  data/manifests/dataset-v1.json \
  data/manifests/dataset-v1.statistics.json
```

The statistics file contains population mean, population standard deviation,
minimum, and maximum for the 25-D state and 12-D action using only manifest
`train` episodes. It does not normalize RGB or depth.

Every adapter must:

- verify the statistics `manifest_sha256`;
- preserve the deep manifest's episode UUID, metadata hash, and canonical-index
  hash checks rather than silently rebasing it onto edited episodes;
- apply dimensions in the recorded state/action order;
- replace a zero standard deviation with 1 and record affected dimensions;
- never fit statistics on validation or test episodes;
- store the exact statistics file/hash with checkpoints and deployment
  bundles;
- use the same inverse action transform during inference;
- perform final finite-value and physical/action-bound checks after inverse
  normalization.

Framework-specific image normalization belongs in the model config and must be
identical at training, validation, replay, and deployment.

### 4. Select a derived view

ACT HDF5:

```bash
.venv/bin/embodied-lab dataset export \
  data/episodes/episode-<uuid> \
  act-hdf5 \
  data/exports/act/episode-<uuid>.hdf5
```

LeRobot v3:

```bash
.venv/bin/embodied-lab dataset export \
  data/episodes/episode-<uuid> \
  lerobot-v3 \
  data/exports/lerobot/episode-<uuid> \
  --repo-id <LOCAL_NAMESPACE>/<DATASET_NAME>
```

Both commands export one training-eligible episode. There is no current command
to merge a collection into one ACT index or one multi-episode LeRobot
repository. Until that is implemented and tested, preserve the canonical
manifest as the split authority and keep an explicit mapping from every
derived artifact to its episode UUID and canonical-index hash.

## Temporal windows, chunks, and masks

The canonical archive stores fixed-clock frames, not ready-made temporal
windows. Current training-eligible episodes have no canonical slot gaps, and
window/chunk padding is not implemented.

A framework adapter must define:

| Contract | Required decision |
|---|---|
| observation horizon | Number of past/current canonical frames supplied to the policy |
| prediction horizon | Number of future 12-D targets predicted |
| action/execution horizon | Subsequence consumed before replanning |
| stride | Frame spacing in canonical slots |
| episode boundary | Windows may not cross UUID boundaries |
| left padding | Repeat, zero, or mask before the first frame |
| right/action padding | Repeat final safe action, zero, or mask after episode end |
| padding mask | Boolean meaning and shape consumed by the model/loss |
| data-quality mask | Treatment of `held_rejected`, unavailable provenance, or future physical missing feedback |
| camera mask | Required if an embodiment/model has optional camera slots |
| control-rate conversion | Explicit relation between dataset FPS, model chunk rate, and accepted-target control rate |

Padding values must never contribute to loss where the framework expects a
padding mask. Do not turn a missing camera into an unmarked black image.
Training eligibility currently excludes any canonical timing gap rather than
interpolating it; future gap-tolerant adapters must retain an explicit temporal
mask and validate the resampling policy.

At inference, a predicted chunk is a proposal. It must pass schema,
normalization, freshness, finite-value, joint/actuator boundary, velocity and
acceleration, feasibility, liveness, and existing control-pipeline checks. A
model must never send a whole chunk directly to the physical JAKA or RH56
transport.

## ACT integration boundary

### What exists

The ACT HDF5 exporter writes:

| HDF5 path | Shape per episode | Meaning |
|---|---:|---|
| `/observations/qpos` | `[T, 12]` | arm q (6) + hand observation (6) |
| `/observations/qvel` | `[T, 6]` | arm dq only; no fabricated hand velocity |
| `/observations/images/workspace` | `[T,H,W,3]` | RGB `uint8` |
| `/observations/images/wrist` | `[T,H,W,3]` | RGB `uint8` |
| `/observations/depth/{workspace,wrist}` | `[T,H,W]` | project-specific lossless `uint16` extension |
| `/action` | `[T,12]` | absolute accepted arm + hand target |
| `/timestamps` | `[T]` | relative seconds |
| `/timing_valid` | `[T]` | canonical timing flag |
| `/arm_action_status` | `[T]` | `accepted=0`, `held_rejected=1` |

Root attributes preserve UUID, schema, order, units, completion/success
metadata, FPS, and simulation status. The exporter stages a temporary HDF5
file, flushes it, and atomically replaces the destination name. It is not a
loader, trainer, evaluator, or checkpoint format.

### Minimal integration path

1. Keep ACT in a separately pinned environment or optional integration; do not
   copy the upstream repository wholesale.
2. Build a dataset index from the canonical manifest and open each HDF5 inside
   its DataLoader worker.
3. Fix camera order to `[workspace, wrist]`; initially omit the project depth
   extension.
4. Adapt and test every upstream 14-D state/action assumption for this 12-D
   embodiment. The ACT reference implementation hard-codes 14 dimensions in
   multiple policy and data paths, so changing one configuration field is not
   sufficient.
5. Construct action chunks entirely inside an episode and emit the padding
   mask ACT's loss consumes.
6. Apply only train-split statistics and persist their hash.
7. Choose chunk size, visual encoder, hidden size, KL weight, and batch size
   from a recorded experiment config. The pinned reference loader supplies one
   current observation; a multi-frame observation context would be an
   additional project model/loader change. This repository supplies no
   validated values.
8. Run a tiny single-GPU overfit first, then held-out offline loss, replay, and
   MuJoCo rollout. Add DDP only after the single-process path is correct.

The official ACT repository exposes `imitate_episodes.py` for training and
evaluation and documents `chunk_size`, `kl_weight`, hidden size, and
camera/joint data. Its reference loader assumes `episode_<integer>.hdf5`,
expects the `sim` attribute, samples one current qpos/image observation, and
then reads the future action sequence. It also computes normalization
statistics over all episodes rather than only its later training split. Do not
reuse that loader unchanged: a project integration must preserve this
repository's UUID-to-derived-file index, frozen episode manifest, and
train-only statistics. These are upstream interfaces and assumptions, not
commands or compatibility guarantees implemented here.

### Still missing

- a multi-episode ACT dataset index/loader;
- episode padding/action-chunk masks;
- a repository ACT model/trainer/evaluator;
- DDP and AMP integration;
- atomic training checkpoint and resume;
- a policy-to-safe-accepted-target inference adapter;
- physical data and physical rollout validation.

## Diffusion Policy integration boundary

No Diffusion Policy code, dependency, converter, or trainer exists in this
repository.

The smallest maintainable adapter should read canonical episodes directly or
from one reviewed derived representation and return:

```text
low-dimensional observation: (B, To, 25)
workspace RGB:             (B, To, C, H, W)
wrist RGB:                 (B, To, C, H, W)
action target:             (B, Tp, 12)
project data-quality mask: custom adapter extension, if used
```

Here `To` is the observation horizon, `Tp` is the prediction horizon, and `Ta`
is the action subsequence executed before replanning. The upstream sequence
sampler repeats the first or last sample at episode boundaries; it does not
return a padding mask, and the upstream image-policy path rejects a
`valid_mask`. Therefore a mask for `held_rejected` or other project data
quality is a deliberate custom loss/adapter extension, not an existing
upstream-compatible field.

The integration must make these choices explicit:

- action representation remains absolute joint/actuator target or is converted
  to a reversible delta representation;
- observation/prediction/action horizons and execution replanning horizon;
- RGB crop/resize/augmentation and fixed camera order;
- whether depth is excluded or converted to metric data with its own
  preprocessing;
- diffusion step count, scheduler, noise seed policy, and validation sampling
  cost;
- state/action normalization and action inverse transform;
- window padding and `held_rejected` loss policy;
- EMA update rule, validation weights, and checkpoint state.

If DDP is added, every rank begins from synchronized model and EMA weights.
Either every rank performs the identical EMA update after synchronized model
steps, or a measured rank-zero design broadcasts the result; this choice must
be tested. Checkpoints must include EMA, optimizer, scheduler, precision/scaler,
RNG, and data identities. Validation sampling and noise seeds must not
silently duplicate every sample on every rank.

Start with state-only or two-RGB-camera single-GPU overfit on a few episodes.
Do not add FSDP or DeepSpeed for a model that fits on one GPU.

## OpenPI / π0 integration boundary

### Protected local shadow code

`learned_policy/pi05_shadow/` is inference-only evidence and remains
unmodified. It:

- accepts exactly `openpi.pi05_droid_state.v1`;
- requires seven DROID/Franka joint positions plus one gripper position;
- sends two HWC RGB images and a prompt to a websocket policy server;
- validates a `(15, 8)` action chunk at its recorded OpenPI commit:
  seven DROID/Franka joint-velocity actions plus one gripper-position action;
- rejects a six-joint JAKA state;
- never pads, drops, clips, maps, publishes, or executes the returned action;
- imports no JAKA/RH56 command path.

That DROID checkpoint and schema are incompatible with the canonical 25-D
JAKA/RH56 observation and 12-D absolute action. Do not crop seven joints to six,
pad six to seven, collapse six hand actuators to one gripper scalar, or execute
the DROID velocities as JAKA targets.

The dated Thor validation report describes one historical host and pinned
checkout. Its JAX/PTX and PyTorch/ARM findings are evidence for that run, not a
claim about every current Thor or current OpenPI release.

### Minimal future integration

Keep OpenPI as an external, pinned upstream environment. A JAKA/RH56
integration needs:

1. a multi-episode LeRobot dataset or another upstream-supported input rooted
   in the canonical manifest;
2. a repository-specific OpenPI data config with explicit `repo_id`, asset and
   normalization identity;
3. repack transforms from `workspace`/`wrist`, 25-D state, 12-D action, and
   task text into the selected model interface;
4. a deliberate action transform matching the trained checkpoint—absolute,
   delta, or velocity—not a convenience mapping;
5. a language/task policy with reviewed natural-language prompts rather than
   assuming an arbitrary task ID is suitable text;
6. quantile or standard normalization generated through the selected OpenPI
   release and cross-checked against the canonical train split;
7. camera masks for any model slots not populated by the two-camera
   embodiment;
8. a JAKA/RH56-specific fine-tuned checkpoint and offline inference
   equivalence tests;
9. a separate safety adapter that proposes canonical actions to the shared
   accepted-target pipeline and cannot command hardware directly.

The current LeRobot exporter produces one episode at a time. Its output and
lossless depth sidecar are not automatically an OpenPI training dataset. Depth
is not a model input until an explicit upstream-compatible transform is
implemented.

### Current upstream distribution facts

As checked against the official OpenPI `main` documentation on 2026-07-31,
upstream provides both JAX and PyTorch implementations. Its PyTorch trainer
documents single-GPU, single-node `torchrun` DDP, and multi-node `torchrun`
launches. That path documents full BF16 or FP32 training while listing mixed
precision, PyTorch FSDP, LoRA, EMA, and π0-FAST support as unavailable. The JAX
route has its own precision and `fsdp_devices` sharding control, but the
official trainer says multi-node JAX training is unsupported; `fsdp_devices`
is not evidence of multi-node readiness.

OpenPI's training `batch_size` is a global batch and its loader requires that
value to divide across `WORLD_SIZE`. The pinned upstream loader currently uses
integer division without rejecting a remainder, so the project integration
must enforce divisibility itself. This differs from this repository's proposed
`per_device_batch_size` contract. An adapter must translate and log the
effective global batch explicitly rather than pass the same integer through
both configuration systems. It must also test epoch-to-epoch shuffling for the
selected release rather than assuming the upstream loader exposes
`DistributedSampler.set_epoch`.

Follow the chosen upstream release's native JAX or PyTorch distribution and
checkpoint system. Do not wrap OpenPI in this repository's future ACT/DP DDP
trainer, and do not maintain parallel custom FSDP and DeepSpeed paths.

## Data loading and scaling

The canonical format favors auditability: one directory and many lossless NPY
payloads per episode. The ACT view is one HDF5 per episode; the LeRobot v3 view
uses the official SDK's tabular/video organization plus a project depth
sidecar. No distributed loader or measured shared-filesystem profile exists.

For the first training loop:

- load the manifest once and partition complete episodes before sampling;
- open HDF5 handles inside each worker, never in the parent before fork;
- cache parsed canonical indexes rather than rescanning the tree per sample;
- use node-local NVMe for reproducible disposable cache and shared durable
  storage for canonical data, manifests, statistics, selected checkpoints, and
  run records;
- do not let every rank independently copy the full dataset;
- measure image decode, NPY/HDF5 open, host-to-device transfer, and GPU idle
  time before changing formats.

If small-file pressure becomes measured at scale, add a deterministic sharded
cache with a source-manifest hash. Do not replace the canonical archive or
adopt WebDataset/Zarr/LMDB solely because a future cluster might need it.
Raw/aligned depth can dominate I/O; exclude it from a model only through an
explicit model config, while retaining it in the source archive.

DDP sampling must shard within a precomputed split. Changing world size must
not change train/validation/test membership. See
[Distributed training readiness](DISTRIBUTED_TRAINING.md) for sampler,
global-batch, storage, NCCL, and Slurm details.

## Planned common policy interface

No common policy class exists today. A future minimal inference boundary should
be framework-neutral:

```text
resolved observation window
    -> schema/order/unit validation
    -> frozen preprocessing and normalization
    -> policy inference
    -> action chunk in a declared representation
    -> inverse normalization
    -> finite/shape/latency checks
    -> one proposed 12-D canonical action at a time
    -> existing acceptance, feasibility, rate, liveness, and safety pipeline
```

The boundary must expose:

- policy/checkpoint identity;
- expected schema version and exact state/action orders;
- camera order, sizes, color layout, temporal horizon, and masks;
- action representation, bounds, chunk length, execution horizon, and expected
  control rate;
- normalization identity;
- device/dtype and measured inference latency;
- a fault result that causes safe hold/stop through the control owner.

A policy result must never bypass `AcceptedArmTarget`, write JAKA/RH56
transports directly, follow MuJoCo `qpos`, remap in the physical adapter, or
recompute hardware IK.

## Checkpoint and experiment contract

There is no model checkpoint implementation in the repository. A future common
training checkpoint must atomically contain:

- unwrapped model weights;
- optimizer and learning-rate scheduler state;
- FP16 scaler state when applicable;
- framework state such as Diffusion Policy EMA;
- epoch, global optimizer step, and gradient-accumulation position;
- Python, NumPy, framework CPU/GPU, sampler, and DataLoader seed/state where
  resumable;
- resolved model/training/data config;
- dataset schema, manifest hash, retained episode IDs, split policy, and
  normalization file/hash;
- state/action orders, units, action representation, camera order, image
  preprocessing, horizons, chunks, and masks;
- source revision/bundle identity;
- precision, world size, per-device/global batch, learning rate, and any
  scaling rule;
- best metric and comparison direction.

Write on rank zero to a temporary path on the destination filesystem, flush and
rename, then update `latest`. Preserve `best`, `latest`, and configured
milestones. Resume must reject incompatible model architecture, action
dimension/representation, camera order, dataset schema, or normalization.
World-size/global-batch/learning-rate changes require an explicit recorded
decision.

An inference bundle is separate from a resumable training checkpoint. It
should include only selected model/EMA weights and everything required to
reproduce preprocessing and action interpretation:

```text
inference_bundle/
  weights.<framework format>
  model_config.json
  observation_schema.json
  action_schema.json
  normalization.json
  preprocessing.json
  manifest.json
  checksums.json
```

The manifest must record checkpoint hash, source revision, framework/version,
camera order and resolution, temporal dimensions, action bounds, expected
control frequency, device/dtype validation, and replay/equivalence results.

## Evaluation sequence

Use the same ordering for ACT, Diffusion Policy, and OpenPI-derived policies:

1. schema/normalization unit tests with synthetic data;
2. one-batch forward/backward test;
3. tiny single-GPU overfit on a few train episodes;
4. deterministic loader/window/padding tests at episode boundaries;
5. checkpoint save, resume, and weights-only load;
6. held-out episode metrics with a frozen manifest;
7. fixed replay inference and action-bound/latency checks;
8. MuJoCo closed-loop rollout and benchmark;
9. exported-model equivalence on x86 GPU;
10. Jetson replay inference and end-to-end latency;
11. command-disabled shadow integration;
12. only then, a separately authorized bounded physical gate through the
    existing safety pipeline.

Training or replay success is not physical validation. Report simulation and
physical results separately and preserve failure categories.

## Training server to Jetson Thor

The training server or cluster should own preprocessing, training, validation,
checkpoint selection, model export, and profiling. Jetson Thor should own data
collection, online preprocessing, inference, latency measurement, and
eventually robot deployment. Large-scale training on Thor is not the default.

Handoff procedure:

1. select a checkpoint using frozen validation criteria;
2. create and checksum the inference bundle;
3. freeze normalization, schemas, camera order, image dimensions, horizons,
   action representation/bounds, and control rate;
4. test the bundle against fixed canonical replay inputs on x86;
5. install a separate ARM64/JetPack-compatible inference environment;
6. verify operator support and run native framework inference in FP32;
7. test supported FP16/BF16 only with output and stability comparisons;
8. try ONNX or TensorRT only after inspecting the actual graph, dynamic shapes,
   and unsupported/custom operators;
9. compare converted and reference action chunks numerically;
10. measure preprocessing, transfer, model, postprocessing, and complete
    observation-to-action latency;
11. run recorded-episode replay and MuJoCo before any robot connection.

TorchScript, `torch.compile`, ONNX, TensorRT, and TensorRT-LLM are candidates,
not guaranteed export formats. Choose the simplest one that the selected model
and Thor software stack demonstrably support.

## Framework-specific acceptance criteria

### ACT

- [ ] Multi-episode loader follows the canonical manifest.
- [ ] State/action dimension and camera order are explicit.
- [ ] Chunk padding and loss mask are tested at episode ends.
- [ ] KL/chunk/model parameters are captured in the run config.
- [ ] Single-GPU overfit, checkpoint resume, replay, and MuJoCo rollout pass.
- [ ] DDP/AMP claims are backed by separate validation.

### Diffusion Policy

- [ ] Observation/prediction/action/execution horizons are distinct and tested.
- [ ] Absolute/delta action transform is reversible and versioned.
- [ ] RGB/depth preprocessing and normalization are frozen.
- [ ] Noise seed, scheduler, diffusion steps, and validation sampling are
      recorded.
- [ ] EMA is synchronized, checkpointed, and used consistently for validation.
- [ ] Single-GPU overfit precedes any DDP work.

### OpenPI / π0

- [ ] A pinned upstream release and backend (JAX or PyTorch) are selected.
- [ ] Multi-episode data and prompt policy are upstream-loadable.
- [ ] Repack, normalization, camera masks, and action transforms are explicit.
- [ ] The checkpoint is fine-tuned/validated for 6-arm + 6-hand semantics.
- [ ] No DROID 7+1 action is mapped to JAKA/RH56.
- [ ] Upstream-native distributed/checkpoint behavior is tested unchanged.
- [ ] Inference is command-disabled until replay, simulation, Thor latency, and
      safety-adapter checks pass.

## Official upstream references

These links describe external frameworks and were checked on 2026-07-31. They
are not evidence that the corresponding framework is installed, integrated, or
validated here.

- [ACT pinned model dimensions](https://github.com/tonyzhaozh/act/blob/742c753c0d4a5d87076c8f69e5628c79a8cc5488/detr/models/detr_vae.py#L55-L70)
  and [dataset loader/statistics](https://github.com/tonyzhaozh/act/blob/742c753c0d4a5d87076c8f69e5628c79a8cc5488/utils.py#L23-L128)
  — 14-D reference-model assumptions, HDF5 naming/loading, action padding,
  split behavior, and normalization scope.
- [Diffusion Policy pinned configuration](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/image_pusht_diffusion_policy_cnn.yaml#L15-L43),
  [sequence sampler](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/diffusion_policy/common/sampler.py#L77-L153),
  and [image-policy loss](https://github.com/real-stanford/diffusion_policy/blob/5ba07ac6661db573af695b419a7947ecb704690f/diffusion_policy/policy/diffusion_unet_hybrid_image_policy.py#L261-L323)
  — horizon mapping, edge-value padding, and the absence of a standard
  padding/data-quality mask input.
- [OpenPI pinned JAX requirements](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/README.md#L24-L31),
  [PyTorch support and launch matrix](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/README.md#L190-L305),
  [training configuration](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/src/openpi/training/config.py#L503-L535),
  and [PyTorch loader](https://github.com/Physical-Intelligence/openpi/blob/15a9616a00943ada6c20a0f158e3adb39df2ccac/src/openpi/training/data_loader.py#L305-L322)
  — backend support, native launchers, global-batch semantics, and JAX
  sharding boundaries.
- [LeRobotDataset v3 official documentation](https://github.com/huggingface/lerobot/blob/main/docs/source/lerobot-dataset-v3.mdx)
  — multimodal time-series storage, episode metadata, Parquet/video layout,
  loading, and finalization.

Pin exact commits for an experiment. `main` branch behavior can change after
the date above.
