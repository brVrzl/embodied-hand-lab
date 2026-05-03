# DQ-RISE / Quantized Hand State Analysis for JAKA mini2 + RH56

Date: 2026-04-29

Paper: Learning Dexterous Manipulation with Quantized Hand State, Feng et al., arXiv:2509.17450.

Project page: https://rise-policy.github.io/DQ-RISE/

Code: https://github.com/rise-policy/DQ-RISE

## Core Takeaway

The paper is highly relevant to this project, but the most useful part is not the full RISE point-cloud policy stack. The useful idea is the action-space design:

- Keep arm motion continuous because arm localization is the hard part.
- Replace high-dimensional continuous hand prediction with a compact per-task hand-state codebook.
- Re-index the discrete hand codes so nearby scalar predictions decode to similar hand poses.
- Train the policy to predict continuous arm action chunks plus relaxed hand-code values, then snap hand-code values to the nearest quantized hand state at execution.

This matches the current project direction: RH56 should not be trained as six independent fast finger actions in the first stage. It should be treated as a small library of executable hand states, then gradually learned from data.

## Paper Details Relevant to Reproduction

Hardware used in the paper:

- Flexiv Rizon 4 arm.
- 6-DoF OyMotion ROHand.
- Two Intel RealSense D415 global cameras.
- Wrist-mounted RealSense D435 used for calibration only.
- Meta Quest 3 joystick for arm teleoperation.
- OyMotion GForce glove for direct hand joint teleoperation.

Learning setup:

- Base policy: RISE, using fused/cropped point clouds from two calibrated cameras.
- Demonstrations: 50 teleoperated demonstrations per task.
- Tasks: Pull Tissue, Open Jar, Collect Toy, Pour Rice, Open Oven, Toast Bread.
- Hand quantizer: two-layer residual VQ-VAE.
- Codebook: 4 codes per layer, merged into K = 16 quantized hand states per task.
- VQ-VAE training: Adam, learning rate 3e-4, batch size 256, 1500 epochs.
- Evaluation: 20 real trials per task.

Reported result:

- DQ-RISE average success: 85.83%.
- RISE baseline: 55.00%.
- RISE-S separate arm/hand diffusion: 61.67%.
- DQ-RISE-C hand-code classification: 2.50%.

The important negative result is that classifying discrete hand states performed badly. The paper argues the hand code should be relaxed into a continuous scalar/order and trained jointly with arm actions, not trained as a separate classification head.

## Fit to Current Project

Current local hardware and software:

- JAKA mini2 arm.
- Inspire RH56, 6 canonical hand DOFs.
- RH56 transport currently goes through JAKA tool RS485.
- RH56 control frequency is configured at 5 Hz, with `command_pause_sec: 0.8`.
- Existing schema already stores normalized/canonical RH56 state.
- Existing grasp library includes `open`, `close`, `pinch`, `power_grasp`, `tripod`, `lateral`, and several validated `pinch_box_*` presets.
- The active plan now recommends `delta_palm_pose + hand_code + close_strength` instead of raw six-finger policy output.
- Existing MuJoCo/ManiSkill work already generates RH56 hand-reference grasp candidates.

Compatibility:

- The hand dimensionality is close: ROHand 6 DoF vs RH56 6 canonical DOFs.
- The policy insight is directly compatible: quantized hand states are a cleaner version of the existing RH56 primitive direction.
- Full RISE reproduction is not immediately compatible because it depends on calibrated two-camera point clouds, a specific robot deployment stack, and smoother teleoperation than the current RH56 RS485 path.

Main mismatch:

- DQ-RISE assumes demonstrations with rich continuous hand motion from a glove. This project currently has low-frequency RH56 commands and manually validated hand presets.
- DQ-RISE uses point-cloud RISE. This project currently has stronger support for state-only / structured samples than for calibrated two-camera point-cloud policy deployment.
- DQ-RISE evaluates long-horizon dexterous tasks. This project should first target palm-frame grasp-lift and functional grasp tasks that can be validated on the current JAKA mini2 + RH56 system.

## Reproduction Verdict

Strict reproduction: not recommended now.

To reproduce the paper closely, the project would need the full RISE environment, two calibrated depth cameras, glove or equivalent high-quality hand teleoperation, 50 demos per task, and a real-time deployment loop. That is possible in principle, but it would move effort away from the current bottleneck: stable RH56 grasp-lift data.

Functional reproduction: recommended.

A practical RH56 version should reproduce the paper's core action representation, not the exact hardware:

```text
observation -> policy -> [arm_delta_chunk, relaxed_hand_code_chunk]
relaxed_hand_code -> nearest RH56 hand code -> normalized RH56 6-DoF command
```

For the first version, use manual or k-means/PCA codebooks before residual VQ-VAE. VQ-VAE only becomes useful after the project has enough clean hand-state data.

## Proposed RH56-DQ Baseline

### Stage 1: Manual Codebook

Define a per-task RH56 codebook from existing presets:

```yaml
fixed_box_lift:
  0: open
  1: pinch_box_thumb_rotate_v2
  2: pinch_box_v4
  3: release

foam_block_envelope:
  0: open
  1: pre_shape
  2: power_grasp
  3: envelope_close
  4: release

cylinder_lift:
  0: open
  1: lateral
  2: power_grasp
  3: release
```

This is a hand-designed approximation of DQ-RISE's learned quantized states.

### Stage 2: Offline Quantization

Once there are at least 50 clean demonstrations for a task:

1. Extract `hand_states.inspire6.normalized_positions`.
2. Fit KMeans or VQ-VAE with K in `[4, 8, 16]`.
3. Sort centroids by the first PCA component over raw 6-DoF hand state.
4. Replace each continuous hand state with an ordered scalar hand code.
5. Train policy to predict continuous arm delta plus relaxed hand code.
6. During execution, round/clip hand code and decode to the RH56 centroid/preset.

KMeans is enough for the first experiment because RH56 has only 6 DOFs and current tasks have few stable hand poses.

### Stage 3: State-Only Policy Before Vision

Use existing structured data first:

```text
input:
  robot_q_current
  ee_pose
  current_hand_code
  object_pose_init or fixture id
  stage

output:
  ee_delta_xyz/rpy
  relaxed_hand_code
```

This directly tests whether the action representation helps, without introducing camera calibration as a confounder.

### Stage 4: Vision / Point Cloud

Only after state-only replay is stable:

- Add one fixed RGB-D camera.
- Then add two-camera point cloud if needed.
- Consider importing RISE-style point-cloud encoder after the action interface is validated.

## Recommended Two-Week Plan

Week 1:

- Finish real replay for `pinch_box_thumb_rotate_v2 -> pinch_box_v4 -> lift`.
- Add a `hand_code` field to exported samples.
- Build a manual RH56 hand-code YAML from existing presets.
- Train a small state-only BC model that predicts `ee_delta + relaxed_hand_code`.
- Compare against the current primitive baseline.

Week 2:

- Collect 50 clean grasp-lift demos and 30 labeled failures for one object.
- Fit KMeans hand-state codebook with K = 4 and K = 8.
- Compare:
  - raw six-DOF hand regression,
  - manual hand code,
  - learned ordered hand code.
- Evaluate 20 real rollouts per variant on fixed-start foam block or paper box lift.

## Expected Project Impact

This paper supports and strengthens the current project direction. It gives a defensible research reason for not predicting six RH56 finger commands directly:

- High-dimensional hand actions can dominate arm learning.
- Separating arm and hand heads can break coordination.
- A compact ordered hand-state representation gives the policy enough dexterity while keeping arm localization learnable.

For this project, the fastest useful path is:

```text
public Inspire-like hand data -> RH56 hand-code -> object-relative palm frame -> real replay -> pseudo-tactile correction -> lightweight vision/state policy
```

This can become a clear ablation in the project:

- continuous six-finger hand output vs fixed hand primitive vs ordered quantized hand-code.
- fixed top-down palm vs object-relative palm frame.
- no pseudo-tactile correction vs RH56 residual/force/current correction.

The paper is therefore a strong methodological anchor for the current palm-frame hand-code transfer direction.

# 中文版本

## 核心结论

DQ-RISE / Quantized Hand State 对当前项目很有价值，因为它给出了一个明确的研究理由：**不要让策略直接预测 RH56 的 6 指连续高维命令，而应该使用有序、低维、离散或半离散的 hand-code 表示。**

这和当前主线完全一致：

```text
公开 Inspire 类手部数据 -> RH56 hand-code -> object-relative palm frame -> 真机 replay -> pseudo-tactile correction -> 轻量视觉/状态策略
```

## 对 RH56 的启发

高维手部动作会带来几个问题：

- 手指动作维度会压过机械臂位姿学习。
- 分离 arm head 和 hand head 容易破坏两者协调。
- 连续 6D 手指输出对低频 RS485 控制和真实接触误差比较敏感。

更合适的做法是：

- 用 hand-code 表示 RH56 手型。
- 用 palm-frame 表示机械臂末端/掌心与物体的关系。
- 让策略输出低维 `delta_palm_pose + hand_code + close_strength`。

## 当前可用路径

第一阶段不需要完整复现 DQ-RISE。更实际的 RH56 版本是复现它的核心 action representation：

```text
observation -> policy/retrieval -> [palm delta, hand-code]
hand-code -> normalized RH56 6-DoF command
```

可做的消融包括：

- continuous six-finger hand output vs fixed primitive vs ordered quantized hand-code。
- fixed top-down palm vs object-relative palm frame。
- no pseudo-tactile correction vs RH56 residual/force/current correction。

## 结论

这篇论文是当前 palm-frame hand-code transfer 方向的重要方法依据。它支撑我们把 RH56 动作空间先压缩成 hand-code，再结合 palm-frame 和 pseudo-tactile correction 做真实机器人实验。
