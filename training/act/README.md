# Repository LeRobot training integration

This directory is the repository-owned boundary to the pinned official
LeRobot trainer. The active experiment is the single human-audited
nominal52 physical-bottle task. Historical mixed-quality and val4 configs are
kept under `configs/archive/training/`; they are not active launcher inputs.

The pinned runtime is:

- image: `jaka-lerobot-dev:snapshot-before-raw-mount`
- image ID: `sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e`
- LeRobot: `0.6.2`
- source: `third_party/lerobot`
- commit: `f66e5128ecb2456e8c54a63d15404fa59c16aebc`

The active configuration boundary is deliberately small:

- `configs/training/shared/physical_bottle.yaml` owns the task prompt, immutable
  audit/materialization rules, 37/15 session split, and both standard and
  optional ACT+Force data views;
- `configs/training/act/physical_bottle.yaml` selects the standard ACT view for
  local ACT/OpenPI smoke checks;
- `configs/training/act/lerobot.json` is the current strong ACT
  trainer configuration.

Check the pinned dependencies before training:

```bash
scripts/check_training_dependencies.sh --require-image
```

Start the current offline baseline with:

```bash
scripts/train_physical_bottle_lerobot.sh strong-act
```

The launcher checks the image, LeRobot submodule, cached ResNet18 weights, and
the immutable nominal52 master. It builds a disposable ACT view using the
validation episode list from `shared/physical_bottle.yaml`, validates it through the
real LeRobot loader, refuses existing output directories, runs with
`--network none`, and never connects to a robot.

The model uses two RGB images, 12-D state, 12-D absolute/native action,
60-step chunks, ImageNet-pretrained ResNet18, batch 16, and 100,000 optimizer
steps. The 15 validation episodes are ordered last in the generated view for
LeRobot's episode-level evaluation behavior. Existing generated views,
checkpoints, logs, and container caches remain under ignored `outputs/` paths.

The source dataset still preserves the six-channel RH56 force signal. The
canonical config exposes it as `views.act_force` for a later force-aware
experiment, but the current ACT and pi0.5 baselines do not consume force.

For the completed historical mixed-quality runs, see the dated research logs
and the archived snapshots under `configs/archive/training/`. Those snapshots
are retained for provenance and are not selected by the active launcher.

## 中文说明

当前只有一个正式训练任务：人工审核后的 nominal52 bottle task。活动配置收敛为：

```text
configs/training/shared/physical_bottle.yaml
configs/training/act/physical_bottle.yaml
configs/training/act/lerobot.json
```

`shared/physical_bottle.yaml` 同时拥有任务 prompt、审核边界、materialization、37/15
session split，以及标准 ACT 和可选 ACT+Force view。force 数据仍保留在源数据中，
但当前 ACT/pi0.5 baseline 不输入 force。

从仓库根目录运行：

```bash
scripts/train_physical_bottle_lerobot.sh strong-act
```

mixed、val4 等已完成的历史训练配置只放在
`configs/archive/training/`，不会被当前入口误用。
