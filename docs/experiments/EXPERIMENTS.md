# Experiment and result discipline

## Reproducibility unit

Every result must be recoverable from a source identity, resolved
configuration, immutable data split, seed policy, environment report, and
output record. This repository copy has no local Git metadata, so the current
maintenance run uses a source-bundle inventory rather than inventing a commit
hash. On a normal Git checkout, record the commit and whether tracked files
were modified.

A run record should contain:

- run ID and UTC start/end time;
- operator or scheduler job identity;
- source revision or source-bundle hash;
- command line and resolved configuration snapshot;
- host, CPU/GPU, driver, Python, framework, CUDA, cuDNN, and NCCL versions
  where applicable;
- dataset schema/version, manifest hash, split IDs, and normalization hash;
- model architecture and parameter counts;
- base seed and rank/worker seed derivation;
- per-device and global batch size, precision, and learning rate;
- checkpoint input/output hashes;
- metrics, failure category, logs, and media paths;
- validation level: offline, simulation, replay, or physical.

## Directory contract

Use a unique run directory. A recommended layout is:

```text
artifacts/experiments/<run-id>/
  run.json
  config.resolved.yaml
  environment.json
  metrics.jsonl
  checkpoints/
  evaluation/
  media/
```

`artifacts/` is output, not source data. Raw canonical episodes and formal
checkpoints must also be stored on durable backed-up storage. Never leave the
only valid checkpoint on node-local scratch.

Only rank zero writes shared run metadata and checkpoints. Nonzero rank errors
must include rank and hostname and either use separate files or a
cluster-provided log collector.

## Seeds and comparison

Record Python, NumPy, framework CPU/CUDA, sampler, worker, augmentation, and
environment seeds. Multi-GPU results across different GPU, CUDA, NCCL, or
kernel versions should target experiment-level or statistical reproducibility;
do not promise bitwise identity.

For a model comparison:

1. Freeze dataset membership before choosing hyperparameters.
2. Keep the same observation/action schema, camera order, normalization, and
   task metrics.
3. Tune only on the training and validation sets.
4. Do not use test outcomes to select a checkpoint or stopping rule.
5. Report repeated seeds or confidence intervals when stochastic variance can
   change the conclusion.

## Checkpoints

Training checkpoints and inference exports are different:

- A resumable checkpoint includes model, optimizer, scheduler, scaler/EMA,
  progress, random state, configuration, data identity, normalization,
  precision, and distributed topology.
- An inference export includes selected model/EMA weights plus input/output
  schemas, camera order, preprocessing, normalization, action bounds, intended
  frequency, and hashes.

Write checkpoints to a temporary file or directory on the destination
filesystem, validate completeness, then rename atomically. Retain `latest`,
`best`, and a bounded number of milestones. A preemption or signal handler must
not label a run successful if its final checkpoint failed.

## Evaluation and failures

Metrics need an observable definition. Use only metrics supported by the
available state:

- The current MuJoCo smoke benchmark can report joint tracking, pre-shape,
  timing, commanded speed, TCP displacement, and diagnostic contact count.
- Object lift, hold, transport, placement, and release require a maintained
  object task and object-state predicate.
- Physical current/force register statistics are proxies, not calibrated
  contact force.
- Slip, passive-joint configuration, and tactile metrics are unavailable from
  current RH56 feedback alone.

Keep failure categories explicit, for example configuration, reset, input
stale, infeasible candidate, controller fault, hand fault, timeout, object
drop, task timeout, operator abort, or infrastructure failure. Preserve the
first terminal reason.

Simulation and physical results must never be aggregated under one unlabeled
success rate. A physical PASS applies only to the exact device, controller
state, software/configuration, duration, workspace, and motion envelope of its
gate.

## Minimal workflow

```bash
# Host/source inventory
.venv/bin/embodied-lab doctor --output /tmp/embodied-lab-environment.json

# Deterministic simulator smoke
.venv/bin/embodied-lab benchmark \
  configs/benchmark/smoke.yaml \
  --output /tmp/embodied-lab-benchmark.json

# Canonical data validation and frozen episode split
.venv/bin/embodied-lab dataset validate <episode-directory>
.venv/bin/embodied-lab dataset manifest <dataset-root> <manifest.json>
.venv/bin/embodied-lab dataset statistics <manifest.json> <statistics.json>
```

Policy-specific procedures are in
[`docs/training/TRAINING_INTEGRATION.md`](../training/TRAINING_INTEGRATION.md);
distributed execution is in
[`docs/training/DISTRIBUTED_TRAINING.md`](../training/DISTRIBUTED_TRAINING.md);
benchmark scope is in
[`docs/benchmark/BENCHMARKS.md`](../benchmark/BENCHMARKS.md).
