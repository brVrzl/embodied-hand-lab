# ACT training

## English

This directory owns the current LeRobot ACT boundary for the audited
physical-bottle task. The source task/config split is:

- `configs/training/shared/physical_bottle.yaml`: task, audit, split, and views;
- `configs/training/act/physical_bottle.yaml`: ACT view selection;
- `configs/training/act/lerobot.json`: current strong ACT trainer.

Verify dependencies and launch the offline baseline with:

```bash
scripts/check_training_dependencies.sh --require-image
scripts/train_physical_bottle_lerobot.sh strong-act
```

The view uses two RGB images, 12-D state, 12-D absolute/native action, and
60-step chunks. Outputs and checkpoints stay under ignored `outputs/`.
Physical rollout is a separate operator workflow.

## 中文

本目录负责经过审核的 physical-bottle task 的当前 LeRobot ACT boundary。source task/config split 为：

- `configs/training/shared/physical_bottle.yaml`：task、audit、split 和 view；
- `configs/training/act/physical_bottle.yaml`：ACT view 选择；
- `configs/training/act/lerobot.json`：当前 strong ACT trainer。

验证 dependency 并启动离线 baseline：

```bash
scripts/check_training_dependencies.sh --require-image
scripts/train_physical_bottle_lerobot.sh strong-act
```

该 view 使用两路 RGB image、12-D state、12-D absolute/native action 和 60-step chunk。输出和 checkpoint 保存在
被 Git 忽略的 `outputs/` 下。物理 rollout 属于独立的 operator workflow。
