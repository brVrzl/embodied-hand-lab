# RH56 External Dataset Training Plan

Date: 2026-04-30

## Decision

Use both HRDexDB F1 and Unitree Inspire data, but do not mix them as if they were equivalent.

HRDexDB F1 should be the target-domain dataset because the Hugging Face dataset is explicitly `robot_type=inspire_f1`, uses LeRobot format, and provides 507 episodes / 324423 frames with 6 hand joints in both observation and action. It is the closest public data source to our RH56 target hand. Its six hand values are not normalized 0-1 in the public Parquet; the smoke subset shows raw values roughly in the 600-1750 range, so it needs axis-order verification and normalization before it is mixed with Unitree data.

Unitree Inspire should be the auxiliary hand-prior dataset. Its dataset cards explicitly define Inspire hand order per hand as:

`[index, middle, ring, little, thumb open/close, thumb lateral tilt]`

with range `0.0-1.0` from open to close. This matches our project canonical RH56 order except that the project names `little` as `pinky`.

Project self-collected data should stay out of the first training run because its scale and reliability are not yet enough for policy learning.

## Why Not Use Only One

Using only HRDexDB is closer to our hand, but the public LeRobot schema names the six hand axes as `hand_joint_0..5`; before deployment we must visually verify the order and semantics against `hahahataeyun/hrdexdb_test`, then normalize raw hand units into our canonical RH56 0-1 command range.

Using only Unitree gives clearer hand semantics and more task variety, but the arm/body embodiment is Unitree G1, not JAKA mini2. Its arm actions should not be cloned directly. The safe use is hand sequence pretraining, hand quantization, and grasp phase priors.

## Training Stages

1. Hand sequence extraction.

   Convert external LeRobot data into project canonical hand sequences:

   `hand_state[6]`, `hand_cmd[6]`, `episode_index`, `frame_index`, `source_dataset`, `side`.

   For Unitree dual-hand data, infer the active hand per episode from motion energy and optionally keep both hands as separate hand-only sequences.

2. DQ-RISE style hand quantizer.

   Train the first model on 6D RH56 canonical hand states/commands only. Unitree supplies scale and action diversity; HRDexDB F1 supplies target-domain hand morphology. This gives a discrete hand-code prior before training any visual or arm policy.

3. Grasp-conditioned policy.

   Use HRDexDB F1 as the main source for relative end-effector plus quantized hand-code supervision. Convert xArm6 motion to EE-relative deltas. Do not use xArm6 joint values on JAKA.

4. MuJoCo replay filter.

   Replay predicted RH56 hand sequences on the current MuJoCo RH56 model and reject candidates with severe self-collision, table collision, or obvious object-through-hand artifacts. Mild mesh penetration is acceptable at this stage if grasp behavior remains physically plausible.

5. Real-machine warm start.

   After camera deployment, start with open-loop or state-conditioned grasp candidates from the trained hand-code prior. Real rollout data is evaluation/calibration first, not training data, until it passes manual replay review.

## Initial Dataset Order

Use this order:

1. `hahahataeyun/hrdexdb_test`: smoke-test schema, download size, raw range, and hand axis order.
2. `unitreerobotics/G1_WBT_Inspire_Pickup_Pillow_MainCamOnly`: first trainable hand-prior dataset because its Inspire hand order and 0-1 range are explicit.
3. `hahahataeyun/hrdexdb`: primary target-domain training data after hand-axis verification and normalization.
4. Add Unitree clothes/drinks/vegetables datasets after the extraction and VQ training run is stable.

## Smoke Results

The metadata inspection and small extraction were run locally.

- `hahahataeyun/hrdexdb_test`: extracted 2538 frames / 5 episodes. Hand values are raw units, not normalized; observed state min/max per axis were about `[629, 1204, 1420, 1533, 1504, 1492]` to `[1088, 1356, 1742, 1690, 1691, 1643]`.
- `unitreerobotics/G1_WBT_Inspire_Pickup_Pillow_MainCamOnly`: extracted 157060 frames / 609 episodes. Values are normalized 0-1; active-hand inference selected mostly right hand. Thumb lateral remained 0 in this task, so this dataset alone is weak for thumb lateral control.

## Codebook Status

First-pass K=16 RH56 hand-state codebook has been built at:

`data/models/rh56_hand_codebook_unitree_state_k16.npz`

This uses Unitree Inspire `hand_state` rather than `hand_cmd` because the checked Unitree command data has no meaningful thumb-lateral motion. The codebook reserves RH56 thumb-lateral anchors so the discrete action space can express thumb opposition even before HRDexDB F1 raw units are calibrated.

Detailed notes: `docs/rh56_hand_codebook_20260430.md`.

## Non-Goals For This Round

- Do not train directly on our self-collected episodes.
- Do not clone Unitree G1 whole-body or arm commands to JAKA.
- Do not claim sim-to-real success from external data alone.
- Do not train a large VLA model before the hand quantizer and MuJoCo replay gates are working.

## Sources Checked

- HRDexDB paper: https://arxiv.org/abs/2604.14944
- HRDexDB Hugging Face dataset: https://huggingface.co/datasets/hahahataeyun/hrdexdb
- HRDexDB test subset: https://huggingface.co/datasets/hahahataeyun/hrdexdb_test
- Unitree Inspire Pillow dataset: https://huggingface.co/datasets/unitreerobotics/G1_WBT_Inspire_Pickup_Pillow_MainCamOnly
- Unitree datasets index: https://huggingface.co/unitreerobotics/datasets

# 中文版本

## 决策

本轮使用 HRDexDB F1 和 Unitree Inspire 数据，但不能把它们当作同一种数据直接混合。

HRDexDB F1 更接近目标域，因为其 Hugging Face 数据集明确标注 `robot_type=inspire_f1`，使用 LeRobot 格式，并包含 507 个 episode / 324423 帧，observation 和 action 中都有 6 个手部关节值。它是目前最接近 RH56 的公开数据源。不过，它的 6 个手部数值不是 0-1 归一化值；smoke subset 中观察到的原始范围大约在 600-1750 之间，因此混入训练前必须先确认轴顺序和归一化方式。

Unitree Inspire 数据应作为辅助 hand prior。其 dataset card 明确定义了每只手的顺序：

```text
[index, middle, ring, little, thumb open/close, thumb lateral tilt]
```

范围是 0.0-1.0，从 open 到 close。这个顺序与本项目 canonical RH56 order 基本一致，只是 `little` 在本项目中命名为 `pinky`。

## 为什么不能只用一个数据源

只用 HRDexDB 的优点是更接近 RH56，但公开 schema 只命名为 `hand_joint_0..5`，部署前必须视觉确认每一轴的语义和归一化方式。

只用 Unitree 的优点是手部语义更清楚、任务更多，但其 arm/body embodiment 是 Unitree G1，不是 JAKA mini2。安全用法是学习 hand sequence、hand quantization 和 grasp phase prior，不要克隆 Unitree G1 的全身或手臂动作到 JAKA。

## 训练阶段

1. 提取 hand sequence。
   - 转成项目 canonical hand sequence：`hand_state[6]`、`hand_cmd[6]`、`episode_index`、`frame_index`、`source_dataset`、`side`。
   - 对 Unitree 双手数据，根据运动能量推断 active hand，必要时把左右手拆成单手序列。

2. 训练 DQ-RISE 风格 hand quantizer。
   - 先只训练 6D RH56 canonical hand state/command。
   - Unitree 提供规模和动作多样性。
   - HRDexDB F1 提供目标手型校准。
   - 目标是在任何视觉/arm policy 之前得到离散 hand-code prior。

3. 训练 grasp-conditioned policy。
   - HRDexDB F1 可作为主要来源。
   - 把 xArm6 motion 转成 relative end-effector delta。
   - 不使用 xArm6 joint values 直接训练 JAKA。

4. MuJoCo replay filter。
   - 在 RH56 MuJoCo 模型上 replay 预测手部序列。
   - 剔除严重自碰撞、撞桌或明显穿模候选。

5. 真机 warm start。
   - 相机部署后，先用 trained hand-code prior 生成 open-loop 或 state-conditioned grasp candidate。
   - 真实 rollout 首先作为评估和校准，不立即作为训练数据，直到通过 manual replay review。

## 本轮不做

- 不直接训练自采 episode。
- 不克隆 Unitree G1 whole-body 或 arm command 到 JAKA。
- 不仅凭外部数据声称 sim-to-real 成功。
- 在 hand quantizer 和 MuJoCo replay gate 稳定前，不训练大型 VLA。
