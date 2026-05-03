# RH56 DQ-RISE-Style RVQ-VAE Codebook 2026-04-30

## Artifact

DQ-RISE-style two-layer residual VQ-VAE codebook:

`data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz`

Manifest:

`data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.json`

Model weights:

`data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.pt`

Canonical order:

`[index, middle, ring, pinky, thumb_close, thumb_lateral]`

## Method

This version follows the DQ-RISE codebook shape more closely than the KMeans baseline:

- layer 1 codebook size: 4
- layer 2 residual codebook size: 4
- combined hand states: 16
- input: 6D RH56 canonical hand state
- source data: Unitree Inspire normalized `hand_state`

Training command:

```bash
.venv/bin/python tools/train_rh56_hand_residual_vqvae_codebook.py \
  --input data/external/rh56_hand_sequences_smoke/unitreerobotics__G1_WBT_Inspire_Pickup_Pillow_MainCamOnly.npz \
  --input data/external/rh56_hand_sequences_unitree \
  --source hand_state \
  --epochs 80 \
  --max-samples 200000 \
  --output data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz
```

## Thumb Lateral Handling

Unitree Inspire data does not provide enough thumb-lateral variation:

- measured `thumb_lateral` max: 0.089
- measured `thumb_lateral` std: 0.004511

To avoid collapse, the RVQ-VAE training uses:

- RH56 thumb-lateral anchors as augmented samples
- higher reconstruction weight on `thumb_lateral`

This produces several lateral/opposition codes:

- `code 02`: low finger close, high thumb lateral
- `code 03`: light pinch with high thumb lateral
- `code 04`: stronger pinch-like lateral state, light self-contact
- `code 06`: mid-close lateral state, light self-contact
- `code 09`: stronger lateral state, self-contact
- `code 11`: power-close lateral state, self-contact

## Occupancy Caveat

The actual Unitree samples mostly occupy a few combinations. Several of the 16 decoded combinations are valid RVQ-VAE combinations but have zero occupancy on the sampled Unitree data. Treat those as extrapolated candidate hand states and validate them in MuJoCo before using them in a policy.

High-occupancy data states:

- `code 05`: occupancy 0.24822
- `code 08`: occupancy 0.05763
- `code 13`: occupancy 0.69312

## MuJoCo Static Contact Check

Outputs:

- `data/collision_diagnostics/rh56_codebook_dqrise_rvqvae_contacts_proxy.json`
- `data/collision_diagnostics/rh56_codebook_dqrise_rvqvae_contacts_unifuc_pad_proxy.json`

Most decoded states are clean in static pose. Codes with visible hand self-contact risk:

- `code 04`
- `code 06`
- `code 09`
- `code 11`

## Viewer

Cycle all codes:

```bash
DISPLAY=:1 scripts/view_mujoco_rh56_pose_contact.sh \
  --mode codebook \
  --codebook data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz \
  --collision-mode proxy \
  --pose-period 2.0
```

Inspect one code:

```bash
DISPLAY=:1 scripts/view_mujoco_rh56_pose_contact.sh \
  --mode codebook \
  --codebook data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz \
  --codebook-index 9
```

## Status

This is the first DQ-RISE-style learned codebook. It should be compared against the simpler KMeans+anchors codebook in MuJoCo grasp tests before becoming the default policy hand representation.

# 中文版本

## 产物

当前已有 DQ-RISE 风格的两层 residual VQ-VAE codebook：

```text
data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.npz
data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.json
data/models/rh56_hand_codebook_dqrise_rvqvae_unitree_state_k16.pt
```

canonical order 为：

```text
[index, middle, ring, pinky, thumb_close, thumb_lateral]
```

## 方法

该版本比 KMeans baseline 更接近 DQ-RISE 的 codebook 形式：

- 第 1 层 codebook size 为 4。
- 第 2 层 residual codebook size 为 4。
- 组合后共有 16 个 hand states。
- 输入为 6D RH56 canonical hand state。
- 数据源为 Unitree Inspire normalized `hand_state`。

## 拇指 lateral 处理

Unitree Inspire 数据中的 thumb-lateral 变化不足：

- 最大值约 0.089。
- 标准差约 0.004511。

为了避免 codebook collapse，RVQ-VAE 训练加入了 RH56 thumb-lateral anchor augmentation，并提高了 `thumb_lateral` 的 reconstruction weight。

因此模型生成了一些 lateral / opposition code，例如 light pinch、mid-close lateral、power-close lateral 等。但部分 code 在真实 Unitree 样本中 occupancy 为 0，应视为外推候选，必须先经过 MuJoCo 和真机验证。

## MuJoCo 静态接触检查

大多数 decoded state 在静态姿态下没有明显碰撞。存在可见手部自碰撞风险的 code 包括：

- `code 04`
- `code 06`
- `code 09`
- `code 11`

这些 code 不应直接作为默认 policy output，需要在具体物体和 palm pose 下继续验证。

## 状态

这是第一版 DQ-RISE 风格 learned codebook。它需要和更简单的 KMeans+anchor codebook 在 MuJoCo grasp test 中比较，然后才能决定是否作为默认 hand representation。
