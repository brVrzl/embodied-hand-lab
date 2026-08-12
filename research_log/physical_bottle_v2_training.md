# Physical bottle v2 repository training run

Date: 2026-08-12

## Result

The repository-owned entrypoint completed both offline training jobs without
hardware access:

```bash
scripts/train_physical_bottle_lerobot.sh both
```

| job | steps | batch | rows | episodes | final loss | final checkpoint |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| ACT | 2,000 | 16 | 20,744 | 25 | 0.152 | `outputs/training/physical_bottle_v2/act_run/checkpoints/002000/pretrained_model` |
| ACT+Force | 2,000 | 16 | 20,744 | 25 | 0.151 | `outputs/training/physical_bottle_v2/act_force_run/checkpoints/002000/pretrained_model` |

Both jobs reached `training_step.json: step = 2000`, saved the 1,000-step and
2,000-step checkpoints, and returned exit status 0. The final logs contain no
NaN/Inf or data-loader errors. Final observed metrics were:

- ACT: L1 `0.145`, KL `0.006`, gradient norm `3.268`.
- ACT+Force: L1 `0.145`, KL `0.007`, gradient norm `3.591`.

These are pilot optimization traces, not a validation benchmark and not a
claim of physical task success.

## Repository integration and previous workflow audit

The previous training commands lived outside this repository in the
`/home/thor/LeRobot` checkout and were launched manually inside Docker. The
audited runtime was the image recorded in
`training/lerobot/runtime.yaml`: LeRobot 0.6.2 at official commit
`f66e5128ecb2456e8c54a63d15404fa59c16aebc`. The old workflow hard-coded an
older five-demo view and did not own the current v2 materialization boundary.

The repository now owns the bridge and entrypoint:

- `tools/lerobot_physical_bottle.py` builds/verifies a disposable LeRobot v3
  view from an immutable v2 master tree.
- `scripts/train_physical_bottle_lerobot.sh` checks the pinned image, uses a
  network-disabled container, validates the view, and starts the official
  trainer for ACT and ACT+Force.
- `configs/training/lerobot/` contains the two reproducible trainer configs.
- `training/lerobot/README.md` documents the mapping and rerun procedure.

Generated views, logs, and checkpoints are under the ignored
`outputs/training/physical_bottle_v2/` tree. No raw episode or master source
file is an output target.

## Exact model inputs and action contract

The master/native semantics remain unchanged:

```text
observation.state: [12] = JAKA measured joints [6] + RH56 measured positions [6]
action:            [12] = accepted absolute/native JAKA target [6]
                         + accepted absolute/native RH56 target [6]
```

The six RH56 channels use the maintained canonical order:
`index, middle, ring, pinky, thumb_close, thumb_lateral`.

The ACT LeRobot view contains:

```text
observation.images.workspace: [3, 240, 320]
observation.images.wrist:     [3, 240, 320]
observation.state:            [12]
action target:                [16, 12]
```

The ACT+Force view contains exactly the same images, rows, episodes, state,
and targets, plus:

```text
observation.environment_state: [6] = raw RH56 FORCE_ACT counts
```

This is LeRobot 0.6.2's native environment-state input token. Force is not
concatenated into `observation.state`; the rich master still retains force
age, validity, and source timestamps for future use. The final checkpoint
configs confirm ACT has only state while ACT+Force has the extra `ENV` feature.

Both views use 16-step absolute action chunks. LeRobot repeat-last padding is
used at each logical episode boundary; no action chunk crosses an episode or
logical segment.

## Validation evidence

The disposable views were built and checked before training with the actual
LeRobot 0.6.2 loader:

- ACT: `PASS`, 20,744 rows, 25 episodes, state `[12]`, action `[16, 12]`.
- ACT+Force: `PASS`, 20,744 rows, 25 episodes, state `[12]`, force `[6]`,
  action `[16, 12]`.

The focused repository tests for the integration passed (`7 passed`), and
`bash -n scripts/train_physical_bottle_lerobot.sh` plus `git diff --check`
passed. The full repository suite passed after training: `666 passed, 4
skipped` (two existing multiprocessing deprecation warnings).

## Reproduction and smoke commands

From the repository root, after the v2 master is materialized:

```bash
scripts/train_physical_bottle_lerobot.sh act
scripts/train_physical_bottle_lerobot.sh act-force
scripts/train_physical_bottle_lerobot.sh both
```

The entrypoint reuses a derived view only when its loader-validation report is
present and never overwrites an existing training output directory. To make a
new run, select a new output/config root; do not alter raw data.

The official LeRobot trainer is the smoke/training command itself:

```bash
python -m lerobot.scripts.lerobot_train \
  --config_path=configs/training/lerobot/act_physical_bottle_v2.json
python -m lerobot.scripts.lerobot_train \
  --config_path=configs/training/lerobot/act_force_physical_bottle_v2.json
```

Those direct commands must be run through the pinned container; the wrapper
is the preferred reproducible entrypoint.

## 中文摘要

已经从仓库入口完成两路离线训练，未访问硬件：ACT 和 ACT+Force 均为 2,000 steps、
batch 16，读取同样的 20,744 行和 25 个 logical episodes，最终 checkpoint 分别在
`outputs/training/physical_bottle_v2/act_run/checkpoints/002000/pretrained_model`
和 `.../act_force_run/checkpoints/002000/pretrained_model`。

ACT 使用双 RGB、12-D state 和 16-step 的 12-D absolute/native action；ACT+Force
额外使用独立的六维 raw `FORCE_ACT` 环境状态输入，不改变 12-D state/action。
两路 view 已通过真实 LeRobot 0.6.2 loader 验证，episode 边界 action chunk 不串接。

训练环境固定记录在 `training/lerobot/runtime.yaml`，仓库入口是
`scripts/train_physical_bottle_lerobot.sh both`。raw episode 和 master 源数据保持只读，
生成的 view、日志和 checkpoint 位于 Git 忽略的 `outputs/training/physical_bottle_v2/`。
