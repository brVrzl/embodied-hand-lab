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

## Current v2 LeRobot training entrypoint

The current reviewed physical-bottle materialization is
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
