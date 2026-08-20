# Command reference

## English

The main offline CLI is:

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

Current maintained entrypoints:

| Function | Entry point |
| --- | --- |
| Simulation | `scripts/run_quest_jaka_sim_demo.sh` |
| Arm-only physical teleoperation | `scripts/run_quest_jaka_bounded_teleop.sh` |
| Combined JAKA + RH56 teleoperation and physical collection | `scripts/run_quest_jaka_rh56_teleop.sh` |
| Quest CTRL input gate | `tools/quest_controller_transport_gate.py` |
| RH56 offline/explicit test | `scripts/run_quest_rh56_hand_test.sh` |
| RGB-D checks | `tools/check_realsense_stream.py`, `tools/process_rgbd_tabletop.py` |
| Episode review/synchronization | `.venv/bin/embodied-lab dataset review-staging`, `sync-staging` |
| Physical-bottle audit/materialization | `.venv/bin/embodied-lab dataset audit-physical-bottle`, `materialize-physical-bottle`, `validate-physical-bottle` |
| ACT/ACT+Force/OpenPI dataset smoke checks | `.venv/bin/embodied-lab dataset act-smoke`, `act-force-smoke`, `openpi-smoke` |
| ACT/LeRobot training | `scripts/train_physical_bottle_lerobot.sh` |
| ACT checkpoint evaluation | `scripts/evaluate_physical_bottle_nominal52_checkpoints.sh` |
| π0.5 training | `training/pi05/scripts/train_weekend.sh` |
| Training status | `training/pi05/scripts/status.sh` |
| Strong ACT command-disabled shadow | `tools/act_live_shadow.py` |
| Strong ACT physical rollout | `tools/act_physical_rollout.py` |
| ACT/Quest DAgger rollout | `scripts/run_act_dagger.sh` |
| Saved-rollout replay/analysis | `tools/act_temporal_executor_replay.py`, `tools/analyze_act_bottle_rollout.py` |

Read each entry's `--help` before using it. Help and simulation do not grant
hardware authority.

The maintained physical ACT rollout command is a Python entrypoint rather than
a shell wrapper. It reads the checkpoint contract dynamically and applies the
existing projection, delta-limit, contact-safety, freshness, and native JAKA
gates:

```bash
PYTHONPATH=src:tools .venv/bin/python tools/act_physical_rollout.py \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --checkpoint <checkpoint>/pretrained_model \
  --model-container <pinned-lerobot-image> \
  --output outputs/act_physical_rollouts/<run-label> \
  --duration-sec <bounded-duration> \
  --act-execution-mode async_temporal_ensemble \
  --query-rate-hz 25 --command-rate-hz 30 \
  --label UNLABELED --notes "operator notes"
```

Use `--write-path-mode none --stationary-write-test` for a command-disabled
startup/shadow check. Do not use `consume-K` options; canonical or async
absolute-time temporal aggregation preserves the full predicted chunk.
After a saved run, use `act_temporal_executor_replay.py` and
`analyze_act_bottle_rollout.py` offline. The physical command requires a
separate current operator authorization and must not be inferred from this
documentation.

The ACT/Quest DAgger wrapper is started with:

```bash
VLA_PYTHON=.venv/bin/python bash scripts/run_act_dagger.sh \
  --runtime-config configs/data_collection/physical_dagger_collection.yaml \
  --checkpoint <checkpoint>/pretrained_model \
  --model-container <pinned-lerobot-image> \
  --output outputs/rollouts/<run-label> \
  --total-duration-sec 180 \
  --command-rate-hz 30
```

Policy inference runs at 30 Hz. Expert takeover runs at 60 Hz while ACT
inference is stopped, and policy inference resumes after the expert segment.

## 中文

主离线 CLI 为：

```bash
.venv/bin/embodied-lab --help
.venv/bin/embodied-lab doctor
.venv/bin/embodied-lab sim --help
.venv/bin/embodied-lab dataset --help
```

当前维护的入口：

| 功能 | 入口 |
| --- | --- |
| 仿真 | `scripts/run_quest_jaka_sim_demo.sh` |
| 仅机械臂真机 teleoperation | `scripts/run_quest_jaka_bounded_teleop.sh` |
| JAKA + RH56 联合 teleoperation 和真机数据采集 | `scripts/run_quest_jaka_rh56_teleop.sh` |
| Quest CTRL input gate | `tools/quest_controller_transport_gate.py` |
| RH56 离线/显式测试 | `scripts/run_quest_rh56_hand_test.sh` |
| RGB-D 检查 | `tools/check_realsense_stream.py`、`tools/process_rgbd_tabletop.py` |
| episode review/同步 | `.venv/bin/embodied-lab dataset review-staging`、`sync-staging` |
| physical-bottle audit/materialization | `.venv/bin/embodied-lab dataset audit-physical-bottle`、`materialize-physical-bottle`、`validate-physical-bottle` |
| ACT/ACT+Force/OpenPI 数据 smoke 检查 | `.venv/bin/embodied-lab dataset act-smoke`、`act-force-smoke`、`openpi-smoke` |
| ACT/LeRobot 训练 | `scripts/train_physical_bottle_lerobot.sh` |
| ACT checkpoint evaluation | `scripts/evaluate_physical_bottle_nominal52_checkpoints.sh` |
| π0.5 训练 | `training/pi05/scripts/train_weekend.sh` |
| Training status | `training/pi05/scripts/status.sh` |
| Strong ACT command-disabled shadow | `tools/act_live_shadow.py` |
| Strong ACT 真机 rollout | `tools/act_physical_rollout.py` |
| ACT/Quest DAgger 推理接管采集 | `scripts/run_act_dagger.sh` |
| 已保存 rollout replay/分析 | `tools/act_temporal_executor_replay.py`、`tools/analyze_act_bottle_rollout.py` |

使用前先阅读对应入口的 `--help`。help 和仿真不会授予硬件操作权限。

维护中的 physical ACT rollout 入口为 Python 工具，会动态读取 checkpoint
contract，并继续使用现有 projection、delta-limit、contact-safety、freshness 和
native JAKA gate：

```bash
PYTHONPATH=src:tools .venv/bin/python tools/act_physical_rollout.py \
  --runtime-config configs/data_collection/physical_collection.yaml \
  --checkpoint <checkpoint>/pretrained_model \
  --model-container <pinned-lerobot-image> \
  --output outputs/act_physical_rollouts/<run-label> \
  --duration-sec <bounded-duration> \
  --act-execution-mode async_temporal_ensemble \
  --query-rate-hz 25 --command-rate-hz 30 \
  --label UNLABELED --notes "operator notes"
```

启动前可使用 `--write-path-mode none --stationary-write-test` 做不写入命令的
sanity/shadow。不要使用 `consume-K`；canonical/async absolute-time aggregation
都会保留完整 action chunk。保存后的 rollout 使用
`act_temporal_executor_replay.py` 和 `analyze_act_bottle_rollout.py` 离线分析。
真机命令仍必须由当前 operator 单独授权，文档本身不构成授权。

ACT/Quest DAgger wrapper 启动命令为：

```bash
VLA_PYTHON=.venv/bin/python bash scripts/run_act_dagger.sh \
  --runtime-config configs/data_collection/physical_dagger_collection.yaml \
  --checkpoint <checkpoint>/pretrained_model \
  --model-container <pinned-lerobot-image> \
  --output outputs/rollouts/<run-label> \
  --total-duration-sec 180 \
  --command-rate-hz 30
```

policy 推理为 30 Hz。专家接管时停止 ACT 推理，并以 60 Hz 运行专家控制；专家
片段结束后恢复 policy 推理。
