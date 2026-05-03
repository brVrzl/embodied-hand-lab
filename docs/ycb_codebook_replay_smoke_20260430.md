# YCB Mesh + RH56 Codebook Replay Smoke

Date: 2026-04-30

## What Changed

- Added `tools/collect_ycb_codebook_replay_dataset.py`.
- It loads converted ManiSkill YCB collision meshes from `data/external/maniskill_ycb_mujoco_assets.json`.
- It builds JAKA + RH56 MuJoCo scenes with real YCB mesh collision objects.
- It can evaluate:
  - `--hand-code-mode target`: continuous heuristic close profile.
  - `--hand-code-mode nearest`: nearest codebook centroid.
  - `--hand-code-mode active`: scan all active codebook entries.

## Important Fix

The ordered DQ-RISE-style codebook stores centroids in:

```text
index, middle, ring, pinky, thumb_close, thumb_lateral
```

The existing planner/controller expects:

```text
pinky, ring, middle, index, thumb_close, thumb_lateral
```

The YCB collector, handref codebook replay benchmark, and MuJoCo codebook viewer now reorder codebook centroids before converting them into MuJoCo controls.

Files patched:

- `tools/collect_ycb_codebook_replay_dataset.py`
- `tools/benchmark_handref_codebook_replay.py`
- `tools/view_mujoco_rh56_pose_contact.py`

## Smoke Results

### YCB Mesh Import

Command:

```bash
.venv/bin/python tools/prepare_maniskill_ycb_mujoco_assets.py
```

Result:

- train + heldout: 40 converted objects
- MuJoCo compile failures: 0

### Dice Contact Calibration

The old analytic `062_dice` proxy succeeds with the current planner. The real YCB dice mesh initially failed because the mesh collision is smaller and more exact than the padded analytic proxy.

Known successful analytic candidate:

```text
box_precision_pinch_p0_w1_z1_o3
```

YCB mesh result with continuous target close and 4 mm mesh contact padding:

```bash
.venv/bin/python tools/collect_ycb_codebook_replay_dataset.py \
  --objects 062_dice \
  --split all \
  --hand-code-mode target \
  --candidate-name-contains box_precision_pinch_p0_w1_z1_o3 \
  --mesh-contact-padding 0.004 \
  --table-clearance -0.000976 \
  --max-base-candidates 1 \
  --max-evals-per-object 1 \
  --duration 5.0 \
  --out-dir data/ycb_codebook_replay/tune_dice_single_pad4mm_zaligned
```

Result:

- success: 1 / 1
- lift: 0.0724 m

Same candidate with nearest codebook projection:

- success: 0 / 1
- nearest code: 2
- failure: no opposing contact

Same candidate scanning all active codebook entries after fixing hand order:

```bash
.venv/bin/python tools/collect_ycb_codebook_replay_dataset.py \
  --objects 062_dice \
  --split all \
  --hand-code-mode active \
  --candidate-name-contains box_precision_pinch_p0_w1_z1_o3 \
  --mesh-contact-padding 0.004 \
  --table-clearance -0.000976 \
  --max-base-candidates 1 \
  --max-evals-per-object 20 \
  --duration 5.0 \
  --out-dir data/ycb_codebook_replay/tune_dice_single_active_pad4mm_zaligned_orderfix
```

Result:

- success: 2 / 13 active codes
- best code: 10
- best lift: 0.0948 m
- second successful code: 9, lift 0.0732 m

## Interpretation

Do not train the next policy with `nearest` code projection as the label source. It can map a good continuous grasp profile to a code that is too open for the object.

Use `active` code scanning for simulation data generation:

```text
object mesh + object pose + wrist candidate + hand code -> success / lift / contacts
```

Then train the model to select the hand code directly.

## Next Data Generation Policy

Recommended first batch:

- object set: ManiSkill YCB train split
- hand code mode: `active`
- mesh contact padding: tune by object size; start with 0.002-0.004 m
- labels:
  - success
  - lift_m
  - max_lift_m
  - final_contacts
  - failure_mode
- keep both positive and negative code labels

For small objects, the collision padding is not cosmetic. It compensates for the RH56 fingertip/pad proxy mismatch and should be treated as a sim calibration parameter.

# 中文版本

## 本次变更

新增了 `tools/collect_ycb_codebook_replay_dataset.py`，用于加载 ManiSkill YCB collision mesh，并构建 JAKA + RH56 MuJoCo 场景。

该工具可以评估不同 hand-code mode：

- `target`：连续启发式 close profile。
- `nearest`：最近 codebook centroid。
- `active`：后续推荐的主动 code selection 模式。

## 目标

该 smoke test 的目标是验证：

- YCB mesh 能否进入当前 MuJoCo 场景。
- RH56 hand-code 能否在真实 mesh 物体上 replay。
- contact / lift / failure_mode 标签能否正常导出。
- 正负样本是否都能保留下来，用于后续学习。

## 下一步数据生成策略

推荐第一批配置：

- object set：ManiSkill YCB train split。
- hand code mode：`active`。
- mesh contact padding：按物体尺寸调节，建议从 0.002-0.004 m 开始。
- labels：
  - success。
  - lift_m。
  - max_lift_m。
  - final_contacts。
  - failure_mode。
- 同时保留 positive 和 negative code label。

## 注意

对小物体来说，collision padding 不是单纯视觉或美观参数。它用于补偿 RH56 fingertip/pad proxy 与真实接触面的 mismatch，应当视为 sim calibration parameter。

该数据只能作为仿真 replay 和训练前验证，不应直接报告为真实机器人成功率。
