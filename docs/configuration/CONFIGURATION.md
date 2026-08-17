# Configuration

## English

Configuration is grouped by current responsibility:

- `configs/sim/`: Quest/MuJoCo offline and live-simulation profiles;
- `configs/hand/`: RH56 real retargeting and PC-direct operation;
- `configs/perception/`: camera/RGB-D processing;
- `configs/data_collection/`: review-first episode collection;
- `configs/training/shared/physical_bottle.yaml`: task prompt, audited source
  selection, split, and ACT/force views;
- `configs/training/act/`: current ACT view and LeRobot trainer;
- `configs/training/pi05/`: current π0.5 base checkpoint and LoRA settings.

Load configuration through the repository CLI or the documented wrapper. Do
not invent a second config source, silently apply controller settings, or use
historical aliases. Generated datasets, checkpoints, caches, and logs belong
under ignored `outputs/` or `data/training/` paths and are not committed.

Useful read-only checks:

```bash
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab dataset --help
scripts/check_training_dependencies.sh --help
```

## 中文

配置按当前职责分组：

- `configs/sim/`：Quest/MuJoCo 离线和 live simulation profile；
- `configs/hand/`：RH56 真机 retargeting 和 PC-direct operation；
- `configs/perception/`：相机/RGB-D 处理；
- `configs/data_collection/`：先审核的 episode 采集；
- `configs/training/shared/physical_bottle.yaml`：task prompt、审核源数据选择、split 和 ACT/force view；
- `configs/training/act/`：当前 ACT view 和 LeRobot trainer；
- `configs/training/pi05/`：当前 π0.5 base checkpoint 和 LoRA 设置。

通过仓库 CLI 或文档中的 wrapper 读取配置。不要创建第二套配置来源、静默应用 controller setting，
或使用历史 alias。生成的数据集、checkpoint、cache 和 log 应位于被 Git 忽略的 `outputs/` 或
`data/training/`，不得提交。

可执行的只读检查：

```bash
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab dataset --help
scripts/check_training_dependencies.sh --help
```
