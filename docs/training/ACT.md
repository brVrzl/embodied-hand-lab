# ACT training

## English

The current ACT task is the audited physical-bottle dataset. Its configuration
is split by responsibility:

- `configs/training/shared/physical_bottle.yaml`;
- `configs/training/act/physical_bottle.yaml`;
- `configs/training/act/lerobot.json`.

The native boundary is two RGB images, 12-D state, and 12-D absolute/native
action. The action is six JAKA values followed by the six RH56 active-actuator
values. The current physical adapter uses the checkpoint's full action chunk
with the canonical temporal executor; it does not use legacy consume-K behavior.

Training is container-bound and training-only:

```bash
scripts/check_training_dependencies.sh --require-image
scripts/train_physical_bottle_lerobot.sh strong-act
scripts/evaluate_physical_bottle_nominal52_checkpoints.sh
```

The last command is the current offline teacher-forced checkpoint evaluation
workflow. It uses the reusable horizon-coverage and nominal-checkpoint
analyzers; their output is analysis, not physical rollout.

## 中文

当前 ACT task 是经过审核的 physical-bottle dataset，配置按职责拆分：

- `configs/training/shared/physical_bottle.yaml`；
- `configs/training/act/physical_bottle.yaml`；
- `configs/training/act/lerobot.json`。

native boundary 是两路 RGB image、12-D state 和 12-D absolute/native action。action 前六维为 JAKA，后六维为
RH56 active-actuator。当前 physical adapter 使用 checkpoint 的完整 action chunk 和 canonical temporal
executor，不使用旧的 consume-K 行为。

训练依赖 container 且只进行 training：

```bash
scripts/check_training_dependencies.sh --require-image
scripts/train_physical_bottle_lerobot.sh strong-act
scripts/evaluate_physical_bottle_nominal52_checkpoints.sh
```

最后一个命令是当前离线 teacher-forced checkpoint evaluation workflow。它使用可复用的 horizon-coverage 和
nominal-checkpoint analyzer；输出属于 analysis，不是真机 rollout。
