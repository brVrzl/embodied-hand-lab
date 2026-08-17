# Dataset and training views

## English

The canonical episode contains synchronized observations, robot state/action,
camera provenance, timestamps, and quality metadata. Training manifests split
at episode boundaries and statistics are computed from the training split.
Validators check required fields, finite values, image availability, timing,
and source provenance at the ingestion/materialization boundary.

The current physical-bottle task prompt is:
`Pick up the bottle and place it on the cardboard box.`
The maintained native state and action each have 12 dimensions: six JAKA arm
values followed by six RH56 active-actuator values in
`index, middle, ring, pinky, thumb_close, thumb_lateral` order. Actions are
absolute native targets. Raw six-channel `FORCE_ACT` remains in source data;
the first ACT and π0.5 baselines do not consume it.

Current view commands:

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  audit-physical-bottle --config configs/training/shared/physical_bottle.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  materialize-physical-bottle --config configs/training/shared/physical_bottle.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  validate-physical-bottle data/training/physical_bottle_v4_nominal52
```

The source tree is never overwritten. ACT uses the standard state/action view;
ACT+Force adds the separate force view; π0.5 uses the project OpenPI adapter
and the locked 37-train/15-validation task split.

## 中文

canonical episode 包含同步 observation、robot state/action、camera provenance、timestamp 和 quality
metadata。training manifest 在 episode boundary 上切分，statistics 只从 training split 计算。validator
在 ingestion/materialization boundary 检查必需字段、finite 值、图像可用性、timing 和源数据 provenance。

当前 physical-bottle task prompt 为：
`Pick up the bottle and place it on the cardboard box.`
维护中的 native state 和 action 都是 12 维：前六维为 JAKA arm，后六维为 RH56 active-actuator，顺序为
`index, middle, ring, pinky, thumb_close, thumb_lateral`。action 是 absolute native target。源数据仍保留
六通道 `FORCE_ACT`；第一版 ACT 和 π0.5 baseline 不读取它。

当前 view 命令：

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  audit-physical-bottle --config configs/training/shared/physical_bottle.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  materialize-physical-bottle --config configs/training/shared/physical_bottle.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli \
  validate-physical-bottle data/training/physical_bottle_v4_nominal52
```

不会覆盖源数据树。ACT 使用标准 state/action view；ACT+Force 增加独立 force view；π0.5 使用项目 OpenPI
adapter 和固定的 37-train/15-validation task split。
