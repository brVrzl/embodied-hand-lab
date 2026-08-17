# Physical training dataset materialization

The physical staging tree under `data/raw_episodes/` is immutable source data.
The offline materializer in `episode_dataset.training_materialization` creates
one derived master dataset under `data/training/`, with episode-level splits
and model views layered over the same rows and videos.

## Source-to-training mapping

| Source field | Master field | Meaning |
| --- | --- | --- |
| `observation.state[0:6]` | `observation.state` | measured JAKA joints, radians |
| `observation.state[6:12]` | `observation.state` | measured RH56 normalized closure in the maintained canonical order |
| `action[0:6]` | `action` | accepted absolute JAKA joint target, radians |
| `action[6:12]` | `action` | accepted absolute RH56 normalized target |
| `observation.force` | `observation.force` | raw RH56 `FORCE_ACT` counts, zero-order-held when its source timestamp is unchanged |
| `timing.rh56_force_act_timestamp_ns` | `force_timestamp_ns` | original causal RH56 force source timestamp |
| `timing.rh56_force_act_age_ns` | `force_age_s` | age at canonical time, seconds |
| `timing.rh56_force_act_valid` | `force_valid` | source validity; no force interpolation is performed |
| `camera.*` and the MP4s | `observation.images.workspace`, `observation.images.wrist` | canonical RGB frames, with no image interpolation |
| `timing` | `timing` | compact JSON provenance retained for audit and synchronization research |

The six RH56 channels are always ordered as:

```text
index, middle, ring, pinky, thumb_close, thumb_lateral
```

The master feature definitions are:

```text
observation.state = [
  jaka_joint_1, jaka_joint_2, jaka_joint_3, jaka_joint_4, jaka_joint_5, jaka_joint_6,
  rh56_index, rh56_middle, rh56_ring, rh56_pinky, rh56_thumb_close, rh56_thumb_lateral,
]
action = [
  jaka_target_1, jaka_target_2, jaka_target_3, jaka_target_4, jaka_target_5, jaka_target_6,
  rh56_target_index, rh56_target_middle, rh56_target_ring, rh56_target_pinky,
  rh56_target_thumb_close, rh56_target_thumb_lateral,
]
observation.force = [
  rh56_force_index, rh56_force_middle, rh56_force_ring, rh56_force_pinky,
  rh56_force_thumb_close, rh56_force_thumb_lateral,
]
```

The recorded action is an absolute/native target. The master dataset never
converts it to a delta or permanently normalizes it. Standard ACT ignores the
force columns. ACT+Force adds the force vector plus its age and validity. The
initial openpi adapter uses the two images, the 12-D state, the task prompt,
and the 12-D absolute action; it deliberately omits force.

## Reproducible commands

From the repository root:

```bash
.venv/bin/embodied-lab dataset materialize-training \
  --config configs/training/physical_bottle_v1.yaml
.venv/bin/embodied-lab dataset validate-training \
  data/training/physical_bottle_v1
.venv/bin/embodied-lab dataset act-smoke \
  --config configs/training/act_physical_bottle_v1.yaml
.venv/bin/embodied-lab dataset act-force-smoke \
  --config configs/training/act_force_physical_bottle_v1.yaml
.venv/bin/embodied-lab dataset openpi-smoke \
  --config configs/training/openpi_physical_bottle_v1.yaml
```

The materializer uses the persisted task-release timestamp, not a fixed
five-second subtraction. Rows after that timestamp are recorded in the crop
report but are not placed in the master training rows. An unresolved release
timestamp excludes the episode with `crop_review_required`.

The pilot manifest is intentionally conservative: episodes 65, 66, 67, 69,
and 70 are enabled; 68 is excluded as recovery/reclutch-heavy; 71 remains
review-required and disabled by default. This is a data-processing choice,
not a claim that the excluded raw episodes are useless.

## Human-audited nominal16 baseline (2026-08-13)

The earlier `physical_bottle_v2` view is retained for reproducibility but is a
mixed-quality diagnostic dataset, not 25 clean expert demonstrations. The
current nominal-success baseline is the derived, immutable-source view:

```text
data/training/physical_bottle_v2_nominal16/
```

Its authority is
`configs/training/physical_bottle_v2_nominal16.yaml`: 13 direct nominal source
episodes plus two nominal segments from source 99 and the second nominal
segment from source 102. Source 102's first segment and all other reviewed
trajectories are excluded as `manual_audit_non_nominal`; they remain untouched
in raw storage. Sources 99 and 102 have explicit demonstration/reset/
demonstration boundaries, and no action chunk crosses those boundaries.

Task trimming is provenance-preserving. Normal episodes end at the persisted
task-release row, before the approximately five-second manual recovery tail.
The few significant pre-task holds end one row before the first accepted target
change, without removing approach motion. Sources 99 and 102 use reviewed frame
and timestamp boundaries because their first completion was not independently
persisted. Rebuild and audit with:

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli audit-physical-bottle \
  --config configs/training/physical_bottle_v2_nominal16.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli materialize-physical-bottle \
  --config configs/training/physical_bottle_v2_nominal16.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli validate-physical-bottle \
  data/training/physical_bottle_v2_nominal16
PYTHONPATH=src .venv/bin/python tools/analyze_physical_bottle_curation.py \
  --config configs/training/physical_bottle_v2_nominal16.yaml \
  --mixed-master data/training/physical_bottle_v2/act/master \
  --nominal-master data/training/physical_bottle_v2_nominal16/act/master \
  --output outputs/training/physical_bottle_v2_nominal16/analysis/curation
```

The clean validation split is deterministic at acquisition-session level:
sessions containing sources 87/88 and 108/109 are held out (2,800 rows), and
the other 12 trajectories are training data (9,377 rows). Both source-99
segments stay together. Start the controlled scratch comparison first:

```bash
scripts/train_physical_bottle_lerobot.sh clean-scratch
```

`clean-pretrained` uses exactly the same rows and split and is refused until
the clean-scratch 2k checkpoint and its offline transition report exist:

```bash
scripts/train_physical_bottle_lerobot.sh clean-pretrained
```

Both commands use the pinned LeRobot 0.6.2, network-disabled container. The
old `strong-pretrained` mode on mixed-quality `val4` is retired. No physical
rollout is authorized by these offline commands.

## Human-audited nominal33 expansion (2026-08-13)

The versioned `physical_bottle_v3_nominal33` view extends nominal16 with 17
accepted trajectories from the later source-121--149 cohort. The operator
excluded 122--129 and 132; source 130 has partial metadata but no complete
training payload. Sources 135 and 136 are retained as review-required and are
not in the clean baseline because a person is visible during the task interval,
not only in a removable reset tail.

For each accepted new trajectory, synchronized actions and workspace video were
used to preserve the complete approach through release/final task motion while
removing pre-task hold and post-task stationary/manual-reset rows. The 17 new
crops contain 10,476 rows; 3,132 of their 13,608 raw rows (23.0%) are outside
the reviewed task intervals. Together with the immutable nominal16 view, the
new master contains 33 trajectories and 22,653 matched ACT/ACT+Force rows.

The split remains acquisition-session grouped: 24 trajectories/16,670 rows are
train and 9 trajectories/5,983 rows are validation. Reproduce and validate it
offline with:

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli audit-physical-bottle \
  --config configs/training/physical_bottle_v3_nominal33.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli materialize-physical-bottle \
  --config configs/training/physical_bottle_v3_nominal33.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli validate-physical-bottle \
  data/training/physical_bottle_v3_nominal33
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli act-smoke \
  --config configs/training/act_physical_bottle_v3_nominal33.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli act-force-smoke \
  --config configs/training/act_force_physical_bottle_v3_nominal33.yaml
```

The authoritative curation and exact source frame/timestamp boundaries are in
`configs/training/physical_bottle_v3_nominal33.yaml`; generated datasets remain
ignored by Git. See `research_log/physical_bottle_nominal33_audit.md` and
`research_log/physical_bottle_nominal33_materialization.md` for the results.

## Human-audited nominal52 expansion (2026-08-13)

`physical_bottle_v4_nominal52` extends nominal33 with the accepted recordings
150--180. The operator-marked unusable recordings remain excluded. Source 172
is split into `172_a`, `172_b`, and `172_c`; source 174 contributes `174_a` and
`174_b`, while its incomplete third task is excluded. The reset/manual-recovery
intervals are gaps in the derived view, not part of either neighboring task.

The 52 included logical trajectories contain 33,111 matched rows and about
1,102.1 seconds after removing reviewed setup holds and stationary/manual-reset
tails. The train/validation split is acquisition-session grouped (37/15
trajectories); all corrected segments from source 172/174 stay together in the
validation group. Episode 161 is included under the operator's "other episodes
normal" instruction, but its raw `aborted_robot_safety` metadata warning is
preserved in the audit. Episode 153 is excluded despite locally present files
because it was reported missing by the operator.

Rebuild and validate the immutable-source derived view with:

```bash
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli audit-physical-bottle \
  --config configs/training/physical_bottle_v4_nominal52.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli materialize-physical-bottle \
  --config configs/training/physical_bottle_v4_nominal52.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli validate-physical-bottle \
  data/training/physical_bottle_v4_nominal52
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli act-smoke \
  --config configs/training/act_physical_bottle_v4_nominal52.yaml
PYTHONPATH=src .venv/bin/python -m episode_dataset.cli act-force-smoke \
  --config configs/training/act_force_physical_bottle_v4_nominal52.yaml
```

The exact decisions and source frame/timestamp ranges are in
`configs/training/physical_bottle_v4_nominal52.yaml` and
`research_log/physical_bottle_nominal52_materialization.md`.

### Strong ACT baseline

The repository-owned strong vision/state ACT baseline uses the frozen
session-grouped nominal52 split, ImageNet-pretrained ResNet18, a 60-action
prediction chunk, the canonical-size 512/3200 transformer, and a 100k-step
budget. The physical adapter uses the full checkpoint chunk with absolute-time
temporal aggregation; it has no consume-K execution path. The pinned
LeRobot training config retains `n_action_steps=1` only as a required
configuration value; the physical worker does not use its action queue.
Starting training never starts a robot:

```bash
scripts/train_physical_bottle_lerobot.sh strong-act
scripts/evaluate_physical_bottle_nominal52_checkpoints.sh
```

The exact split is
`configs/training/physical_bottle_v4_nominal52_split.yaml`, the trainer config
is `configs/training/lerobot/act_physical_bottle_v4_nominal52_strong.json`, and
the audit/selection protocol is in
`research_log/physical_bottle_nominal52_strong_act_20260813.md`. Checkpoints and
derived LeRobot views stay under ignored `outputs/training/`; do not commit
them. ACT+Force training remains gated until the strong ACT configuration and
checkpoint-selection result are fixed.

## Historical v2 LeRobot training entrypoint

The retained mixed-quality physical-bottle materialization is
`data/training/physical_bottle_v2/`. Its two master trees contain the same
20,744 logical samples from 25 accepted logical segments. The repository-owned
LeRobot bridge is documented in [training/lerobot/README.md](../training/lerobot/README.md)
and is launched with:

```bash
scripts/train_physical_bottle_lerobot.sh both
```

The command runs inside the pinned LeRobot 0.6.2 container, creates disposable
views under `outputs/training/physical_bottle_v2/`, validates them with the
actual LeRobot loader, and starts ACT followed by ACT+Force. The ACT+Force
view exposes raw six-channel force as LeRobot's separate
`observation.environment_state`; it does not alter the master 12-D state or
the absolute/native 12-D action. Generated views and checkpoints are ignored
outputs; `data/raw_episodes/` and the master source trees are read-only.

## 中文说明

`data/raw_episodes/` 下的 physical staging 数据是不可变的源数据。离线
materializer 在 `data/training/` 下生成一个 master 数据集，再通过 episode
级 split 和 model view 支持 ACT、ACT+Force 与 openpi。RH56 六通道顺序严格沿用
当前代码：`index, middle, ring, pinky, thumb_close, thumb_lateral`。

master 中保留 12-D state、12-D absolute/native action、6-D raw FORCE_ACT、
force timestamp/age/validity、两路图像和 timing provenance。ACT 忽略 force；
ACT+Force 额外读取 force；第一版 openpi adapter 不读取 force。裁剪使用已保存的
task-release timestamp，不盲目减去五秒；如果 timestamp 缺失则进入 review-required，
不会静默猜测。

当前 v2 的 LeRobot 训练入口是仓库内的
`scripts/train_physical_bottle_lerobot.sh both`。它在固定的 LeRobot 0.6.2
Docker 环境中构建和验证临时 view，然后从仓库入口依次启动 ACT 和 ACT+Force；生成的
view/checkpoint 位于 `outputs/training/physical_bottle_v2/`，原始数据和 master 源数据保持不变。
ACT+Force 通过单独的 `observation.environment_state` 提供六维 raw force，不会把 force
拼入 12-D state。

2026-08-13 人工审核后，`physical_bottle_v2` 仅保留为混合质量诊断数据，不能再称为
“25 条干净专家示教”。当前 nominal baseline 是
`data/training/physical_bottle_v2_nominal16/`，由 13 条直接 nominal episode、99
的两段和 102 的第二段组成。99/102 的人工恢复区间不属于任何派生 episode；普通
episode 在持久化 task-release 行结束，约 5 秒人工 reset 尾段不进入训练。少数明显的
任务前静止段也通过清单中的精确 frame/timestamp 边界排除，但不会裁掉 approach。

clean split 按采集 session 划分：87/88 与 108/109 为验证集（2,800 行），其余 12
条为训练集（9,377 行）；99 的两段始终同 split。先运行
`scripts/train_physical_bottle_lerobot.sh clean-scratch`，完成离线 transition 诊断后才允许
`clean-pretrained`。旧 mixed val4 上的 `strong-pretrained` 已停用。以上均为离线训练，
不会授权真机运动。

同日新增的 `physical_bottle_v3_nominal33` 是 nominal16 的版本化扩展。新批次中
122--129、132 按人工审核排除，130 缺少完整载荷；135/136 的人物出现在实际任务
区间内，暂列 review-required，不进入干净基线。其余 17 条新轨迹保留完整 approach
到 release/最后任务动作，并裁掉任务前静止和任务后人工恢复。新轨迹由原始 13,608
行裁为 10,476 行；与 nominal16 合并后共有 33 条、22,653 行完全匹配的 ACT 与
ACT+Force 样本。训练/验证按采集 session 隔离，分别为 24 条/16,670 行和 9 条/5,983
行。权威边界与复现命令见本页英文小节和
`configs/training/physical_bottle_v3_nominal33.yaml`。
