# Repository LeRobot training integration

This directory documents the repository-owned boundary to the official
LeRobot trainer. The upstream LeRobot source is kept as the Git submodule
`third_party/lerobot`, pinned to the exact commit used by the audited runtime.
The pinned runtime is the audited image `jaka-lerobot-dev:snapshot-before-raw-mount`,
image ID
`sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e`,
with LeRobot `0.6.2` from official commit
`f66e5128ecb2456e8c54a63d15404fa59c16aebc`. The image contains the current
NVIDIA PyTorch runtime used by the Thor host.

The maintained entrypoint is:

```bash
scripts/train_physical_bottle_lerobot.sh both
```

Verify both external training repositories and the ACT image before training:

```bash
scripts/check_training_dependencies.sh --require-image
```

The launcher mounts `third_party/lerobot/src` read-only into the container and
puts it first on `PYTHONPATH`; the image and source commit are checked before
any dataset view is built. Set `LEROBOT_SOURCE` only for a separately audited
checkout with the same pinned commit.

It performs, in order:

1. checks the pinned image and the already-materialized immutable source;
2. builds a disposable LeRobot v3 view from `data/training/physical_bottle_v2/act`;
3. builds the matching ACT+Force view from `.../act_force`;
4. validates each view through the real `LeRobotDataset` loader;
5. starts `python -m lerobot.scripts.lerobot_train` for ACT and ACT+Force.

Use `act` or `act-force` to run one branch. The script uses `--network none`,
does not access robot devices, and refuses to overwrite an existing generated
view or checkpoint directory. Outputs are ignored by Git under
`outputs/training/physical_bottle_v2/`.

The standard ACT view has two RGB inputs, `observation.state` `[12]`, and
absolute/native `action` `[16,12]`. The ACT+Force view uses the same rows,
images, and action targets and additionally exposes the six raw
`FORCE_ACT` counts as LeRobot's `observation.environment_state` `[6]`. ACT
0.6.2 consumes that feature through its native environment-state token; the
master dataset remains unchanged and still stores `observation.force` as a
separate six-vector with age, validity, and timestamp provenance.

The training configs are in
`configs/training/lerobot/act_physical_bottle_v2.json` and
`configs/training/lerobot/act_force_physical_bottle_v2.json`. They reproduce
the previous strong baseline architecture: 16-step chunks, batch 16, fp32,
AdamW at `1e-4`, seed 1000, no temporal ensemble, no pretrained backbone, and
2,000 optimizer steps. Deployment may consume only a prefix of the predicted
chunk; that is separate from this training configuration.

The pinned runtime is also recorded in `training/lerobot/runtime.yaml`. The
most recent completed run and its checkpoint paths are recorded in
`research_log/physical_bottle_v2_training.md`.

### Episode-level validation run

The first v2 run was train-only. A reproducible validation run is available
for the four reviewed clean demonstrations 89, 98, 114, and 116:

```bash
scripts/train_physical_bottle_lerobot.sh val4
```

This uses `configs/training/physical_bottle_v2_val4.yaml` and the matching
`*_val4.json` trainer configs. The derived views put the other 21 logical
episodes first and the four held-out episodes last, matching LeRobot 0.6.2's
episode-level `eval_split: 0.16` behavior. The split is 18,064 training rows /
21 episodes and 2,680 validation rows / 4 episodes. State, action, and force
normalization statistics are computed from training rows only. Episode
lengths may differ; action chunks stay within each episode and LeRobot pads
only a short final horizon according to its existing contract.

The completed run is recorded in
`research_log/physical_bottle_v2_training_val4.md`; generated views, logs, and
checkpoints are under
`outputs/training/physical_bottle_v2/{lerobot,act_val4_run,act_force_val4_run}`.

## Audit of the previous workflow

The previous run did use a LeRobot checkout outside this repository and a
manually invoked Docker command. The reproducibility evidence is retained in
`research_log/act_thor_environment.md` and
`research_log/act_five_demo_overfit.md`. The old workflow hard-coded the v1
five-demo master and copied commands from `/home/thor/LeRobot/scripts`; it did
not provide a repository-owned entrypoint for the current v2 matched views.
This integration keeps the official trainer but moves view construction,
version checks, container invocation, and the ACT/ACT+Force mapping into this
repository.

## 中文说明

本目录定义仓库与官方 LeRobot trainer 的边界。第三方 LeRobot 源码通过
`third_party/lerobot` Git submodule 固定 commit，不复制成普通源码目录。
固定使用已经审计过的 `jaka-lerobot-dev:snapshot-before-raw-mount` 镜像和 LeRobot 0.6.2。

从仓库根目录运行：

```bash
scripts/train_physical_bottle_lerobot.sh both
```

训练前可统一检查外部训练依赖和 ACT 镜像：

```bash
scripts/check_training_dependencies.sh --require-image
```

入口会把 `third_party/lerobot/src` 以只读方式挂载到容器并置于
`PYTHONPATH` 首位，同时检查 source commit 和镜像 digest。只有使用同一
固定 commit 的单独审计 checkout 时才设置 `LEROBOT_SOURCE`。

入口会先检查镜像、构建并验证两个临时 LeRobot v3 view，然后依次启动 ACT 与 ACT+Force
训练。它使用 `--network none`，不访问机器人，不覆盖已经存在的 view 或 checkpoint。

ACT 使用双 RGB、12-D state 和 16-step 的 12-D absolute/native action。ACT+Force 使用完全
相同的样本、图像和 action，并将六维 raw `FORCE_ACT` 通过 LeRobot 原生的
`observation.environment_state` 输入 token 提供给 ACT。master 数据集中的 force、age、validity
和 timestamp 仍保持独立，不改 raw 数据。

如果需要真正检查泛化而不是只看训练 loss，可运行：

```bash
scripts/train_physical_bottle_lerobot.sh val4
```

该配置按 episode 留出 89、98、114、116 四条示教作为验证集，21 条示教训练、4 条示教
验证；两种模型使用完全相同的划分。不同示教持续时间不会跨 episode 拼接 action chunk，
末尾短 horizon 按 LeRobot 的既有 padding 约定处理。
