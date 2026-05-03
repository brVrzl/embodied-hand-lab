# RH56 Hand Codebook 2026-04-30

## Artifact

First-pass RH56 6D hand-state codebook:

`data/models/rh56_hand_codebook_unitree_state_k16.npz`

Manifest:

`data/models/rh56_hand_codebook_unitree_state_k16.json`

Canonical order:

`[index, middle, ring, pinky, thumb_close, thumb_lateral]`

## Method

This is not the full DQ-RISE two-layer residual VQ-VAE yet. It is a fast KMeans baseline with reserved RH56 thumb-lateral anchors.

Inputs:

- `unitreerobotics/G1_WBT_Inspire_Pickup_Pillow_MainCamOnly`
- `unitreerobotics/G1_WBT_Inspire_Collect_Clothes_MainCamOnly`
- `unitreerobotics/G1_WBT_Inspire_Put_Clothes_into_Washing_Machine_MainCamOnly`
- `unitreerobotics/G1_WBT_Inspire_Put_Drinks_Into_Fridge`
- `unitreerobotics/G1_WBT_Inspire_Put_Vegetables_Into_Basket`

Source field:

`hand_state`, not `hand_cmd`.

Reason: Unitree Inspire `action.hand_cmd` keeps `thumb_lateral` at 0 in the checked datasets. `hand_state` contains only very small lateral variation, but it is still more informative than command.

Training command:

```bash
.venv/bin/python tools/train_rh56_hand_codebook.py \
  --input data/external/rh56_hand_sequences_smoke/unitreerobotics__G1_WBT_Inspire_Pickup_Pillow_MainCamOnly.npz \
  --input data/external/rh56_hand_sequences_unitree \
  --source hand_state \
  --k 16 \
  --output data/models/rh56_hand_codebook_unitree_state_k16.npz
```

## Thumb Lateral Handling

Measured Unitree normalized hand-state statistics:

- total frames: 733154
- `thumb_lateral` max: 0.089
- `thumb_lateral` std: 0.004511

This is too weak to learn meaningful thumb lateral states from data alone. The codebook therefore reserves 5 RH56 anchors:

```text
[0.00, 0.00, 0.00, 0.00, 0.00, 0.00]
[0.00, 0.00, 0.00, 0.00, 0.00, 1.00]
[0.00, 0.00, 0.12, 0.15, 0.40, 1.00]
[0.10, 0.10, 0.55, 0.60, 0.68, 1.00]
[0.75, 0.75, 0.80, 0.80, 0.55, 0.65]
```

These anchors are not learned from Unitree data. They are conservative RH56 execution states derived from the project pose library so that the discrete action space can still express thumb rotation / opposition.

## Current Limitation

This is a pragmatic first codebook, not a final learned representation.

- Unitree contributes strong finger close/open variation.
- Unitree does not contribute useful thumb lateral command variation.
- HRDexDB F1 has closer hand morphology but raw hand values must be calibrated before mixing.
- Reserved lateral codes should be validated in MuJoCo and then on real RH56 before they are used as policy outputs.

## Next Step

Use this codebook to encode hand trajectories and test three variants in MuJoCo:

1. continuous 6D hand command
2. KMeans-only codes without reserved anchors
3. current KMeans + RH56 thumb-lateral anchors

The expected benefit is not higher imitation accuracy; it is safer, lower-dimensional hand action selection for RH56.

## DQ-RISE-Style Follow-Up

A two-layer residual VQ-VAE version has also been trained:

`data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz`

Notes:

`docs/rh56_dqrise_rvqvae_codebook_20260430.md`

# 中文版本

## 产物

当前已有第一版 RH56 6D hand-state codebook：

```text
data/models/rh56_hand_codebook_unitree_state_k16.npz
data/models/rh56_hand_codebook_unitree_state_k16.json
```

canonical order 为：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

## 方法

这不是完整的 DQ-RISE 两层 residual VQ-VAE，而是一个快速 KMeans baseline，并额外加入 RH56 thumb-lateral anchor。

输入数据包括多个 Unitree Inspire 数据集，使用 `hand_state` 而不是 `hand_cmd`。原因是已检查的 Unitree command 数据中，`thumb_lateral` 基本没有有效运动；`hand_state` 虽然 lateral 变化也很小，但仍比 command 更有信息量。

## 拇指 lateral 处理

Unitree normalized hand-state 的统计显示：

- 总帧数：733154。
- `thumb_lateral` 最大值：0.089。
- `thumb_lateral` 标准差：0.004511。

这个变化太小，无法仅靠 Unitree 数据学习有意义的 thumb lateral 状态。因此 codebook 手动保留了 5 个 RH56 anchor，用来表达拇指旋转、对掌和 pinch-like 状态。

这些 anchor 不是从 Unitree 数据学出来的，而是根据项目中已有 RH56 pose library 设计的保守可执行状态。

## 当前限制

- Unitree 提供了较强的手指开合变化。
- Unitree 没有提供足够的 thumb lateral command variation。
- HRDexDB F1 更接近目标手型，但原始数值需要先校准。
- 预留 lateral code 在用于 policy output 前，必须先经过 MuJoCo 和真机验证。

## 下一步

使用该 codebook 编码手部轨迹，并在 MuJoCo 中比较三种变体：

1. continuous 6D hand command。
2. 不含保留 anchor 的 KMeans-only code。
3. 当前 KMeans + RH56 thumb-lateral anchor。

预期收益不是更高的 imitation reconstruction accuracy，而是为 RH56 提供更安全、更低维的 hand action selection。

## 后续 DQ-RISE 风格版本

也已经训练了两层 residual VQ-VAE 版本：

```text
data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz
```

详细说明见：

```text
docs/rh56_dqrise_rvqvae_codebook_20260430.md
```
