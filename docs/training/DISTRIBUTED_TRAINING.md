# Distributed training readiness

## Current support boundary

This repository does not yet contain an ACT, Diffusion Policy, OpenPI/pi0, or
other model trainer. It therefore cannot currently train on one GPU or many
GPUs. The distributed-training code that does exist is deliberately small:

- [`DistributedContext`](../../src/training_infra/distributed.py) parses the
  complete `LOCAL_RANK`, `RANK`, and `WORLD_SIZE` triplet and rejects partial
  launch environments.
- `GlobalBatchConfig` calculates global batch size without hiding any scaling
  factor.
- `write_rank_zero_json` performs same-filesystem, atomic, rank-zero-only JSON
  replacement.
- [`distributed_smoke_test.py`](../../tools/distributed_smoke_test.py) lazily
  imports PyTorch and, when PyTorch is available, exercises process-group
  initialization, rank/device mapping, an all-reduce, a
  `DistributedSampler` partition, barriers, and process-group destruction.

The smoke tool is not a trainer. It does not construct a model, wrap one in
`DistributedDataParallel` (DDP), run automatic mixed precision, aggregate
model metrics, or save and restore training checkpoints.

[`distributed.example.yaml`](../../configs/training/distributed.example.yaml)
is a proposed contract for a future trainer. No current code consumes it. The
Slurm files in [`scripts/slurm`](../../scripts/slurm) are parameterized launch
templates; they cannot become end-to-end training jobs until a real training
module implements this contract.

DDP is the intended default when a model fits on one GPU. FSDP and DeepSpeed
are not implemented. Do not add either until a measured model/optimizer memory
failure demonstrates that DDP cannot fit and the selected upstream framework
does not already own its distribution strategy.

| Scenario | Recommended strategy |
| --- | --- |
| Small correctness/debug run | One process, one GPU |
| Routine ACT training | One GPU or DDP |
| Routine Diffusion Policy training | One GPU or DDP |
| Larger visual encoder that still fits per GPU | DDP plus measured AMP |
| Model/optimizer cannot fit one GPU | Upstream-supported FSDP or existing DeepSpeed, selected once |
| Jetson Thor | Single-process inference, not distributed training |

## Validation status on 2026-07-31

The maintenance host was macOS 15.6.1 (Darwin 24.6.0), arm64 Apple M1, 8 CPU
cores, and 16 GiB unified memory. At baseline it had no PyTorch. The temporary
validation environment later installed PyTorch 2.13.0 for macOS arm64 and its
CPU/Gloo backend. The host still had no NVIDIA GPU or driver, CUDA toolkit or
runtime, cuDNN, NCCL, Docker, Apptainer/Singularity, or Slurm. macOS also has
no Linux `/dev/shm` to measure. No GPU topology, NVLink, InfiniBand, RoCE, or
storage-network bandwidth was inferred.

The repository filesystem had approximately 176 GiB free, and the active
network was Wi-Fi with no claimed bandwidth. Reported limits included
1,048,575 open files, 2,666 processes, an 8 MiB stack, and unlimited CPU time
and address space. These macOS facts are baseline evidence only, not sizing
recommendations for a Linux training server.

The following capability and local collective checks were run:

```bash
.venv/bin/python -m pip install -e ".[training]"
.venv/bin/python tools/distributed_smoke_test.py --check
.venv/bin/python tools/distributed_smoke_test.py \
  --device cpu \
  --backend gloo \
  --result-json build/distributed-smoke-single.json
```

The capability check reported `torch_importable: true`, PyTorch 2.13.0,
`gloo_available: true`, no CUDA devices, and no NCCL. The one-process
collective passed all-reduce, sampler, barrier, rank mapping, atomic result
write, and process-group cleanup. A two-process CPU/Gloo run with explicit
`MASTER_ADDR=127.0.0.1`, distinct ranks, and `GLOO_SOCKET_IFNAME=lo0` also
passed: all-reduce observed `3.0`, both eight-sample shards were disjoint, and
only rank zero wrote the result.

The macOS `torchrun` workers produced the same passing two-process result, but
the PyTorch 2.13.0 launcher parent did not reap its completed workers and was
manually terminated. Therefore the communication code is locally exercised,
but `torchrun` clean-launcher exit is **not** verified on this host. No
CUDA/NCCL, GPU, model training, multi-node, Slurm, or container training test
was run.

### Second-round Linux/aarch64 validation

Independent integration validation on 2026-07-31 used Linux
`6.8.12-1021-tegra`, aarch64, one NVIDIA Thor GPU, driver 595.78, and PyTorch
`2.11.0+cu130`. The following offline communication checks passed:

- one-process CPU/Gloo: all-reduce `1.0`, one 16-sample shard, clean exit;
- two-process CPU/Gloo under `torchrun`: all-reduce `3.0`, disjoint even/odd
  eight-sample shards, two rank mappings, rank-zero-only atomic output, and a
  clean launcher/worker exit with no residual process;
- one-process/one-GPU NCCL: all-reduce `1.0`, rank 0 mapped to CUDA device 0,
  clean process-group destruction.

This resolves only the earlier local `torchrun` parent-reaping uncertainty.
It does not validate multi-GPU NCCL, multi-node rendezvous, model DDP,
checkpointing, Slurm execution, or training throughput. The optional
LeRobot/training environment also exposes an NVIDIA packaging issue:
`nvidia-cusparselt-cu13` installs from an aarch64-named wheel whose internal
tag is `manylinux2014_sbsa`, so `pip check` reports it as unsupported on this
Jetson host. The independent base `.[dev]` environment passes `pip check`;
do not interpret the optional-wheel warning as a distributed-training PASS.

## Target-server inventory

Run the applicable read-only checks on every proposed training node and save
their output with the experiment configuration:

```bash
uname -a
cat /etc/os-release
lscpu
free -h
df -h
df -h /dev/shm
ulimit -a

nvidia-smi
nvidia-smi --query-gpu=index,name,uuid,memory.total,driver_version \
  --format=csv
nvidia-smi topo -m
nvcc --version

python - <<'PY'
import torch
print("torch", torch.__version__)
print("torch build CUDA", torch.version.cuda)
print("cuda available", torch.cuda.is_available())
print("device count", torch.cuda.device_count())
print("distributed", torch.distributed.is_available())
print("gloo", torch.distributed.is_gloo_available())
print("nccl", torch.distributed.is_nccl_available())
if torch.cuda.is_available():
    for index in range(torch.cuda.device_count()):
        properties = torch.cuda.get_device_properties(index)
        print(index, properties.name, properties.total_memory)
PY

ldconfig -p | grep -E 'libcudnn|libnccl' || true
ip -brief link
ibstat
ibv_devinfo
docker version
apptainer version
sinfo
scontrol show config
```

Missing commands must be recorded as unavailable. Keep these concepts
separate:

- The NVIDIA driver is a host kernel/userspace installation reported by
  `nvidia-smi`.
- The system CUDA toolkit supplies `nvcc`; it may be absent even when PyTorch
  CUDA wheels work.
- A pip or Conda environment may include CUDA runtime libraries without a
  system toolkit.
- `torch.version.cuda` identifies the CUDA version against which that PyTorch
  build was compiled; it is not the driver version.
- cuDNN and NCCL may be bundled with a PyTorch package and need not match
  unqualified system-library searches.

Also record local NVMe capacity, shared filesystem type and quota, measured
network bandwidth, firewall restrictions, GPU heterogeneity, container
permissions, Slurm account/partition/QoS rules, and whether all nodes expose
the same code, data, and checkpoint paths.

## Example configuration contract

The example YAML keeps training policy in one place while distinguishing
values that belong to the launcher from values that belong to a future
trainer. Its `status.consumed_by_current_trainer` field is intentionally
`false`.

Important meanings:

- `per_device_batch_size` is the number of samples processed by one process in
  one optimizer micro-step.
- `gpu_count_per_node` and `node_count` describe the requested data-parallel
  topology.
- `gradient_accumulation_steps` is the number of micro-steps per optimizer
  update.
- `precision.mode` is one of `fp32`, `fp16`, or `bf16`.
- `data.manifest` is the only source of episode split membership. It must be a
  default deep-validation manifest; an inventory-only `--fast` manifest
  assigns no training splits.
- `data.normalization_statistics` must identify statistics fitted only on that
  manifest's training split.
- `data.split_unit: episode` prevents adjacent samples from one demonstration
  leaking across train and validation sets.
- `checkpoint.resume` describes a future interface; checkpoint recovery is
  not implemented today.

The launcher and configuration must agree on world size. A future trainer
must compare the configured topology with `WORLD_SIZE` at startup and fail
with a useful error instead of silently changing global batch size.
It must also recompute and verify `expected_global_batch_size`; the example
value is review evidence, not a second source of truth.

## Global batch size and learning rate

Use the explicit relationship:

```text
world_size = gpu_count_per_node * node_count

global_batch_size =
    per_device_batch_size
    * gpu_count_per_node
    * node_count
    * gradient_accumulation_steps
```

For example, per-device batch 8, four GPUs per node, two nodes, and three
accumulation steps produce a global batch of 192.

Do not apply automatic linear learning-rate scaling. Whether it is appropriate
depends on the optimizer, model, loss, and reference recipe. If scaling is
chosen, record the reference batch size, reference learning rate, rule, and
final learning rate in both the configuration and run log. Resume must reject
an unexplained change in global batch or learning rate.

## Precision and numerical stability

Use FP32 for the first correctness/overfit run. BF16 is generally preferable
on GPUs that support it efficiently because it has a wider exponent range
than FP16. Older GPUs may require FP16. Detect capability; never infer BF16
support from the presence of CUDA alone.

A future trainer must:

- select the local CUDA device before creating model tensors;
- use autocast only around supported forward/loss work;
- use `GradScaler` for FP16 and save/restore its state;
- normally omit loss scaling for BF16;
- keep sensitive normalization, diffusion scheduling, or reduction operations
  in FP32 when measurements require it;
- check the scalar loss and gradient norm at a configurable interval;
- record the first non-finite step, scaler scale, overflow, and skipped
  optimizer step;
- include the precision mode in the checkpoint compatibility check.

Do not add finite-value checks to every tensor in the steady-state hot path.

## DDP trainer contract

When a trainer is added, the same entry point must support one process and
`torchrun`. It must:

1. Parse `LOCAL_RANK`, `RANK`, and `WORLD_SIZE` as one complete contract.
2. Set `cuda:LOCAL_RANK`; never hard-code `cuda:0`.
3. Initialize the process group once and destroy it in `finally`.
4. Wrap the model in DDP only when `WORLD_SIZE > 1`.
5. Use one process per GPU and a consistent per-device batch size.
6. Use `DistributedSampler` for training and call `sampler.set_epoch(epoch)`.
7. Partition validation or explicitly account for duplicated padded samples
   before reducing metrics.
8. Seed Python, NumPy, PyTorch CPU/CUDA, augmentations, and DataLoader workers
   from a recorded base seed plus a deliberate rank/worker derivation.
9. Reduce metric numerators and denominators, not pre-averaged rank means.
10. Restrict the main log, run manifest, and checkpoint writes to global rank
    zero.
11. Put rank and hostname in nonzero-rank diagnostic logs.
12. Treat a process failure as a job failure and always clean up the process
    group.

DDP does not make an episode split safe automatically. Generate stable
train/validation/test episode IDs before sampling, then shard only within each
split. Changing GPU count must not change which episodes belong to a split.
The current manifest command deep-validates payloads by default and assigns a
split only to completed, gap-free episodes carrying an explicit `success` or
`failure` label. `unlabeled` episodes remain visible but are excluded.
`dataset manifest --fast` is inventory-only and deliberately assigns no
training, validation, or test split.

## Smoke and future launch commands

Capability reporting does not initialize a process group:

```bash
python tools/distributed_smoke_test.py --check
```

After installing a compatible PyTorch build, run a single-process CPU/Gloo
collective:

```bash
python tools/distributed_smoke_test.py \
  --device cpu \
  --backend gloo \
  --result-json artifacts/training/distributed-smoke-cpu.json
```

Then test two local CPU processes:

```bash
torchrun \
  --standalone \
  --nnodes=1 \
  --nproc_per_node=2 \
  tools/distributed_smoke_test.py \
  --device cpu \
  --backend gloo \
  --sampler-size 16 \
  --result-json artifacts/training/distributed-smoke-gloo-2.json
```

On a CUDA server, use only visible homogeneous GPUs for the first test:

```bash
CUDA_VISIBLE_DEVICES=0,1 torchrun \
  --standalone \
  --nnodes=1 \
  --nproc_per_node=2 \
  tools/distributed_smoke_test.py \
  --device cuda \
  --backend nccl \
  --sampler-size 16 \
  --result-json artifacts/training/distributed-smoke-nccl-2.json
```

The result JSON is written atomically by rank zero. Verify the all-reduce,
rank-to-device mapping, disjoint sampler shards, and clean process exit. These
commands are not model-training validation.

Do not mix substantially different GPU models or memory capacities in one DDP
job unless a measured need outweighs the slowest-rank bottleneck. Record every
visible GPU's model, memory, compute capability, BF16 support, and
rank-to-device mapping. All ranks need a batch that fits the smallest GPU.

Once a real trainer module exists, the intended forms are:

```bash
# Placeholder: no such trainer module exists in this repository yet.
TRAINER_MODULE=your_integration.train
python -m "$TRAINER_MODULE" \
  --config configs/training/distributed.example.yaml

CUDA_VISIBLE_DEVICES=0,1,2,3 torchrun \
  --standalone \
  --nnodes=1 \
  --nproc_per_node=4 \
  -m "$TRAINER_MODULE" \
  --config configs/training/distributed.example.yaml
```

For a launcher outside Slurm, all nodes need the same rendezvous values:

```bash
torchrun \
  --nnodes="$NODE_COUNT" \
  --nproc_per_node="$GPUS_PER_NODE" \
  --node_rank="$NODE_RANK" \
  --master_addr="$MASTER_ADDR" \
  --master_port="$MASTER_PORT" \
  -m "$TRAINER_MODULE" \
  --config "$TRAINING_CONFIG"
```

`NODE_RANK` is zero-based, and world size is `NODE_COUNT * GPUS_PER_NODE`.
The master address must be reachable from every node and the selected port
must be allowed by host firewalls.

## Checkpoint contract

Only `write_rank_zero_json` exists today; it is not a model checkpoint system.
A future rank-zero checkpoint must atomically save:

- unwrapped model state;
- optimizer and scheduler state;
- FP16 scaler state, when used;
- epoch, global optimizer step, and micro-step/accumulation position;
- Python, NumPy, CPU torch, and CUDA random states;
- resolved training and model configuration;
- dataset/schema version, split manifest hash, and normalization statistics;
- source revision or source-bundle hash;
- world size, global batch size, learning rate, and precision;
- best metric and its comparison direction;
- framework-specific state such as Diffusion Policy EMA weights.

Write a temporary file in the destination filesystem, flush it, then rename it
atomically. Validate structure and hashes before updating a small `latest`
pointer. Keep `latest`, `best`, and a configured number of milestones. The
inference export should contain model/EMA weights plus schemas and
normalization, not optimizer state.

Resume must support explicit path and `auto` latest selection, and distinguish
full training resume from weights-only initialization. Reject incompatible
model architecture, action dimension, camera ordering, dataset/schema
version, normalization statistics, or precision. Warn and require explicit
approval for world-size/global-batch/learning-rate changes. A trainer should
handle the cluster's preemption signal early enough to request a final
checkpoint, but it must never report success if that save fails.

DDP checkpoints should be portable between single-process and DDP execution by
saving the unwrapped module state. FSDP/DeepSpeed sharded checkpoint merge is
not applicable until one of those strategies is actually implemented.

## Data loading and storage

The repository's current durable unit is an episode; see
[`DATASET_SCHEMA.md`](../data/DATASET_SCHEMA.md).
There is no distributed training dataset loader yet.

For initial ACT or Diffusion Policy scale, retain episode files plus a compact,
versioned manifest instead of prematurely converting the whole corpus. Build
train/validation/test splits at episode, task, object, or scene level. Cache
the parsed manifest once per worker/process rather than scanning the complete
directory for every sample.

Recommended responsibilities:

- Shared durable storage: raw episodes, split manifests, normalization
  statistics, selected checkpoints, and final run records.
- Node-local NVMe: disposable decoded-image/features cache, staged shards, and
  profiler traces.
- Never leave the only valid checkpoint on node-local scratch.
- Copy or stage data before launching workers; avoid every rank independently
  copying the same corpus.

Open HDF5 files inside each DataLoader worker rather than before process fork,
and do not share file handles. Measure before changing formats. If many small
episode files overload a shared filesystem, evaluate tar/WebDataset or another
sharded representation with an explicit conversion manifest; do not rewrite
the canonical data spec without evidence.

Tune `num_workers`, `persistent_workers`, `prefetch_factor`, pinned memory,
non-blocking transfers, image decoding, and local cache placement with a short
profile. More workers are not always faster, and each DDP process owns its own
workers.

## Logging, reproducibility, and profiling

Rank zero should create the primary offline JSONL/CSV and TensorBoard record.
Online services may be optional but must not be required. Record:

- run ID, UTC start time, hostname(s), rank/world-size mapping;
- GPU UUID/model/memory, driver, PyTorch/CUDA/NCCL versions;
- resolved configuration and command line;
- dataset/split/schema versions and normalization hash;
- parameter and trainable-parameter counts;
- per-device and global batch sizes, learning rate, precision, seed;
- image size, camera order, observation horizon, and action chunk;
- data/transfer/forward/backward/optimizer/communication/validation times;
- throughput, peak allocated/reserved GPU memory, losses, and metrics;
- checkpoint duration and path.

Record the base seed, rank/worker derivation, deterministic-algorithm setting,
cuDNN benchmark setting, and TF32 policy. Multi-GPU runs across different GPU,
CUDA, NCCL, or kernel versions should target experiment/statistical
reproducibility; do not promise bitwise identity.

Start performance work with tens of synthetic or tiny real batches. Use simple
step timers and `torch.cuda.max_memory_allocated()` first, then PyTorch
Profiler. Use `nvidia-smi dmon` or Nsight Systems only on a suitable NVIDIA
host. Look for data starvation, decode saturation, load imbalance, unnecessary
CPU/GPU synchronization, repeated validation, blocking checkpoint writes, and
communication dominating step time before adding optimization complexity.

Memory levers, in preferred order after measuring, include AMP, gradient
accumulation, smaller per-device batch/image/horizon/chunk/camera count,
freezing the visual encoder, activation checkpointing, and cached visual
features. CPU offload or parameter sharding belongs only to a demonstrated
large-model need.

## NCCL troubleshooting

Do not bake NCCL tuning into scripts. For a failing diagnostic run, selectively
set:

```bash
NCCL_DEBUG=INFO
NCCL_DEBUG_SUBSYS=INIT,NET
TORCH_DISTRIBUTED_DEBUG=DETAIL
NCCL_SOCKET_IFNAME=eno1
NCCL_IB_DISABLE=0
NCCL_P2P_DISABLE=0
```

Choose `NCCL_SOCKET_IFNAME` from measured inter-node interfaces, not from an
example name. Do not leave verbose debug enabled for routine jobs. Do not
disable IB or P2P without evidence; a fallback to sockets is slower but useful
when the fabric is unavailable. `CUDA_DEVICE_MAX_CONNECTIONS=1` can change
execution/communication behavior and is not a universal default.

Check that every node resolves and reaches `MASTER_ADDR:MASTER_PORT`, firewall
rules allow the rendezvous and NCCL connections, clocks are synchronized for
usable logs, container networking exposes the required interfaces, and
`/dev/shm` is large enough. One failed rank normally leaves peers waiting in a
collective until the process-group timeout; inspect every rank's first error.

## Slurm templates

The two `.example.sbatch` files intentionally contain literal angle-bracket
site placeholders in `#SBATCH` directives. Copy a template, replace or remove
every placeholder-bearing directive according to cluster documentation,
create the log/output directories, and then submit it. Some sites use
`--gpus-per-node` instead of `--gres`. Do not submit an unedited example.

For example, keep the site-specific copies under ignored local data rather
than editing the reviewed examples in place:

```bash
mkdir -p data/local/slurm
cp scripts/slurm/train_single_node.example.sbatch \
  data/local/slurm/train_single_node.sbatch
cp scripts/slurm/train_multi_node.example.sbatch \
  data/local/slurm/train_multi_node.sbatch
```

After replacing every placeholder, the copies accept a Python module, a
config path, and optional trainer arguments:

```bash
sbatch data/local/slurm/train_single_node.sbatch \
  your_integration.train \
  configs/training/act.yaml \
  --output artifacts/training/act-run

sbatch data/local/slurm/train_multi_node.sbatch \
  your_integration.train \
  configs/training/act.yaml \
  --output /shared/checkpoints/act-run
```

`your_integration.train`, `configs/training/act.yaml`, and
`configs/training/run.yaml` are future placeholders and do not exist in the
current repository. Set `PYTHON_BIN` to the environment's interpreter,
`GPUS_PER_NODE` to match the Slurm GPU request, and optionally
`TRAINING_ACTIVATE` to a sourceable environment activation file.

The multi-node template launches one `torchrun` parent per node with `srun`;
`SLURM_PROCID` becomes node rank, and the first allocated hostname becomes
master address. Some sites require a different `srun`/PMI pattern or
`--rdzv_backend=c10d`; follow administrator guidance.

Common operations are:

```bash
sbatch copied_job.sbatch your_integration.train configs/training/run.yaml
squeue -u "$USER"
scontrol show job "$SLURM_JOB_ID"
scancel "$JOB_ID"
srun --pty --gres=gpu:1 --time=01:00:00 bash
```

Account, partition, QoS, interactive syntax, modules, requeue permission, and
preemption signals are site-specific. Before enabling requeue, implement
atomic checkpoints and verify that an interrupted job resumes the correct
global step instead of overwriting a completed run.

## Containers and environment separation

Docker is appropriate on a self-managed NVIDIA server only when the NVIDIA
Container Toolkit is installed. Pin a compatible PyTorch/CUDA base, run with
`--gpus`, mount data and outputs, set a practical `--shm-size`, and map the
host UID/GID. Do not bake datasets or checkpoints into the image. Host
networking may simplify multi-node NCCL but changes isolation and must follow
site policy.

Apptainer/Singularity is usually preferable on daemonless HPC systems. Build
or pull an approved image outside the job when required, use `--nv`, bind
shared data/output and local scratch explicitly, and place image/cache/temp
paths according to quota rules. Slurm launch conventions remain site-specific.

No container definition is supplied because neither runtime is installed here
and the repository does not yet have a trainer dependency set. Hardware
drivers for JAKA, RH56DFX, RealSense, and Quest must be evaluated separately;
training-container readiness does not authorize device access.

Do not reuse an x86_64 training-server environment on Jetson Thor. Jetson's
ARM64 JetPack/CUDA/TensorRT stack requires its own compatible inference
environment.

Illustrative runtime forms, after an image is built and approved, are:

```bash
docker run --rm --gpus all --shm-size=16g \
  --user "$(id -u):$(id -g)" \
  -v /shared/datasets:/datasets:ro \
  -v /shared/checkpoints:/checkpoints \
  TRAINING_IMAGE COMMAND

apptainer exec --nv \
  --bind /shared/datasets:/datasets:ro \
  --bind /shared/checkpoints:/checkpoints \
  TRAINING_IMAGE.sif COMMAND
```

Replace every capitalized placeholder. Multi-node Docker networking and
Apptainer cache/temp directories must follow site policy.

For a venv or Conda environment, pin the critical Python, framework, CUDA
runtime, and model-integration versions without exporting platform-specific
build noise. Do not confuse a Conda CUDA runtime with the host driver or
toolkit. On module-based clusters, record the loaded GCC, CUDA, cuDNN, and any
required communication modules in the run manifest. MPI is not automatically
required by `torchrun`; load it only when the site launcher or selected
framework requires it. A sourceable `TRAINING_ACTIVATE` file used by the Slurm
templates can contain the reviewed module/venv/Conda activation.

## Framework integration boundaries

### ACT

ACT is not integrated. Its smallest path is an external or optional adapter
that reads the canonical episode schema, fixes camera order and action/state
normalization, constructs padded temporal/action chunks and masks, and uses
one common trainer entry point. The official reference code hard-codes the
ALOHA 14-D state/action size in multiple paths; the 12-D project HDF5 layout is
not directly compatible and needs a reviewed adaptation. Start with FP32
single-GPU overfit, then DDP and AMP. Record transformer size, image encoder,
camera count, temporal context, action chunk, KL weight, padding, and global
batch. Partition episodes before a `DistributedSampler`.

### Diffusion Policy

Diffusion Policy is not integrated. The adapter must define observation,
prediction, and action horizons; camera/depth preprocessing; action
normalization; diffusion steps/scheduler; and validation sampling cost. EMA
state must be in every checkpoint and updated equivalently on every rank (or
updated on rank zero from synchronized weights by a measured design).
Validation metric aggregation and noise seeding must be explicit. Use DDP
before considering model sharding. The official sequence sampler repeats
boundary samples and does not emit a padding mask; a project data-quality mask
therefore requires an explicit custom adapter/loss rather than an assumed
framework field.

### OpenPI/pi0

The protected OpenPI shadow code is inference-only and is not a trainer.
Follow the selected upstream OpenPI/pi0 release's native PyTorch or JAX
distribution and checkpoint system rather than wrapping it in this project's
DDP layer. Keep the project dataset schema separate from the adapter's image,
language/tokenizer, action, and normalization format. Do not copy the large
upstream framework into this repository and do not implement FSDP merely in
anticipation of a future model. The upstream OpenPI `batch_size` is global,
not this example config's per-device value. Its current JAX `fsdp_devices`
control is single-host sharding and the official JAX trainer does not support
multi-node training; use only the selected upstream backend's documented
launcher.

## Training server to Jetson Thor

Training servers or clusters own preprocessing, training, validation,
benchmark evaluation, checkpoint selection, export, and profiling. Jetson
Thor should primarily own data collection, online preprocessing, inference,
latency measurement, and robot deployment—not large-scale distributed
training.

The deployment handoff must:

1. select a validated checkpoint;
2. export inference weights separately from optimizer state;
3. freeze observation/action schemas, camera order, image sizes, temporal
   dimensions, normalization, action bounds, and control frequency;
4. record model configuration and a checkpoint/source hash;
5. compare exported and training-model outputs on fixed replay inputs on x86;
6. transfer to a Jetson-compatible environment;
7. test native FP32, then supported FP16/BF16;
8. attempt ONNX/TensorRT only after checking dynamic shapes and unsupported or
   custom operators;
9. quantify conversion output error and end-to-end latency;
10. validate on recorded episodes before any separately authorized robot
    connection.

TorchScript, `torch.compile`, ONNX, TensorRT, and TensorRT-LLM are options, not
promises. Select one only after the actual model graph is known.

## Official PyTorch references

The generic DDP contract in this page follows the current official
[DistributedDataParallel](https://docs.pytorch.org/docs/stable/generated/torch.nn.parallel.DistributedDataParallel.html),
[DistributedSampler](https://docs.pytorch.org/docs/stable/data.html#torch.utils.data.distributed.DistributedSampler),
[`torchrun`](https://docs.pytorch.org/docs/main/elastic/run.html), and
[AMP](https://docs.pytorch.org/docs/stable/amp.html) documentation. Pin the
actual PyTorch/CUDA build in an experiment record; these online references can
change independently of this source bundle.

## Readiness ratings

| Area | Rating | Evidence or blocker |
| --- | --- | --- |
| Single-GPU training | Planned | PyTorch CPU/Gloo was installed for infrastructure checks, but no trainer, model, optimizer loop, CUDA GPU, or one-GPU overfit run exists. |
| Single-node multi-GPU | Partially implemented | Rank/global-batch helpers and a two-process CPU/Gloo collective/sampler run passed; no DDP trainer or CUDA/NCCL test exists, and the macOS `torchrun` parent required manual termination after its workers completed. |
| Multi-node | Partially implemented | Environment-based smoke code and a parameterized Slurm template exist; no nodes, fabric, shared data, trainer, or collective validation. |
| Slurm | Implemented but not verified | Single- and multi-node example templates are syntax-checked; Slurm is absent and site placeholders require customization. |
| Data I/O | Partially implemented | Canonical episodes, deep validation, label-gated episode splits, train-only statistics, and per-episode exporters exist; no distributed loader, manifest cache, or measured multi-worker/shared-storage path exists. |
| Checkpoint recovery | Planned | Rank-zero atomic JSON helper exists, but model/optimizer/scaler/RNG checkpoint and resume do not. |
| ACT multi-GPU | Planned | No ACT dependency, adapter, or trainer. DDP is the recommended future strategy. |
| Diffusion Policy multi-GPU | Planned | No Diffusion Policy dependency, adapter, trainer, or EMA checkpoint implementation. |
| OpenPI/pi0 large-model training | Planned | Only protected inference shadow work exists; upstream-native distribution must be selected later. |
| Jetson Thor deployment | Planned | Handoff contract is documented, but no selected trained model, export, Jetson environment, or latency/equivalence result exists. |

The next meaningful gate is a Linux training environment with a compatible
CUDA PyTorch build, followed by a clean `torchrun` CPU/Gloo check, one-GPU
correctness/overfit, two-GPU NCCL smoke, and only then a real DDP trainer test
with equivalent global batch and portable checkpoint recovery.

For that trainer test, use a tiny deterministic model/dataset and retain the
following acceptance evidence:

1. A short one-GPU run and a two-GPU run with equivalent global batch have
   reasonably close loss trajectories; define tolerance before running.
2. Every rank receives a distinct sampler shard, and `set_epoch` changes the
   shared shuffle deterministically.
3. Validation numerator/denominator aggregation matches a one-process
   reference.
4. Only rank zero writes the main manifest and checkpoint.
5. A DDP checkpoint resumes optimizer, scheduler, scaler, RNG, epoch, and
   global step, and its inference weights load in one process.
6. A deliberately failed rank terminates the job without orphaned worker
   processes.
7. A short profile records data, transfer, forward, backward, optimizer,
   communication, checkpoint, and validation time plus peak memory.

None of these model-training acceptance checks has been executed in the
current repository.
