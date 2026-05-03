# UniFucGrasp Project Analysis for JAKA mini2 + Inspire RH56

Date: 2026-04-30

Paper: UniFucGrasp: Human-Hand-Inspired Unified Functional Grasp Annotation Strategy and Dataset for Diverse Dexterous Hands.

Project page: https://haochen611.github.io/UFG/

Code: https://github.com/cxcAxxy/UniFucGrasp

arXiv: https://arxiv.org/abs/2508.03339

## Bottom Line

UniFucGrasp is useful for this project, but it should be treated as a medium-term functional grasp source, not as the immediate replacement for the current RH56 primitive and MuJoCo grasp benchmark.

Best use:

- Functional grasp priors for bottles, mugs, drills, spray bottles, flashlights, tools, and handled objects.
- Candidate hand poses for Inspire-style underactuated hands.
- A reference for human-hand-to-RH56 mapping and coupled-joint modeling.
- A later baseline for "functional grasp" tasks after stable lift and replay are solved.

Not best use:

- Immediate paper-box / foam-block lift MVP.
- End-to-end policy learning.
- Direct real-robot deployment without object pose, point cloud, hand model, and sim-to-real validation.

## What the Paper Contributes

UniFucGrasp targets functional grasps: grasps that preserve the object's later use, such as holding a cup by the handle, pressing a drill button, or pouring from a mug. This differs from generic force-closure grasping, where the main goal is only stable holding.

The paper contributes:

- A unified human-to-robot hand mapping strategy.
- A multi-hand functional grasp dataset.
- A functional gesture generation model conditioned on object/hand point clouds.
- Sim and real experiments on underactuated hands, including InspireHand.

Key dataset scale:

- 1108 objects.
- 21 daily-use categories.
- More than 100K functional grasp pose annotations.
- More than 70 validated functional grasp demonstrations per object.
- Hands include ShadowHand, InspireHand, and HnuHand.

The paper was accepted to IEEE Robotics and Automation Letters according to the arXiv page.

## InspireHand Relevance

The paper explicitly models InspireHand as an underactuated hand. It discusses the mismatch between human hand DoFs and InspireHand:

- Human hand: 20 DoFs in their mapping formulation.
- InspireHand: 12 joint-level DoFs.
- InspireHand actuator space: 6 active actuator commands.

For InspireHand, the paper:

- Uses direct thumb mapping because the thumb DoFs are closer to the human thumb.
- Uses fingertip-alignment calibration for the other fingers.
- Optimizes an index-finger mapping matrix with reported coefficients:
  - alpha = 0.3530
  - beta = 0.4310
  - gamma = 0.2827
  - delta = 0.2584
  - epsilon = 0.4130
  - zeta = -0.0018
- Measures a joint coupling matrix `J in R^(12x6)`.
- Computes the pseudoinverse `J+` to map joint-level commands into 6 actuator commands.

This is directly relevant to RH56 because the project already has a 6-DOF canonical hand schema and raw/canonical order conversion. The useful part is not the exact coefficients, but the method:

```text
human hand pose -> robot joint pose -> coupled actuator command -> sim validation -> real validation
```

## Fit to Current Repository

Current project assets that align well:

- [configs/hand/rh56_real.yaml](/home/w/projects/embodied_lab/configs/hand/rh56_real.yaml) already defines 6 RH56 canonical commands and presets.
- [src/rh56_driver/hand_schema.py](/home/w/projects/embodied_lab/src/rh56_driver/hand_schema.py) already normalizes raw RH56 values and handles canonical/raw order.
- [tools/rh56_handref_grasp_planner.py](/home/w/projects/embodied_lab/tools/rh56_handref_grasp_planner.py) already implements object-conditioned grasp candidate generation and MuJoCo validation.
- [docs/rh56_handref_grasp_planner.md](/home/w/projects/embodied_lab/docs/rh56_handref_grasp_planner.md) already includes `light_can_50mm`, cylinder power envelope, and multiple object categories.

Main gaps before direct use:

- UniFucGrasp assumes accurate object meshes/point clouds and object pose.
- Real experiments used a UR5 + InspireHand/HnuHand setup, RealSense, calibration board, Aruco tags, FreeScan X3 object scans, and FoundationPose for object pose.
- This project currently prioritizes low-frequency reliable RH56 execution through JAKA tool RS485, not high-throughput functional grasp model deployment.
- Current sim-to-real hand contact is still being calibrated; learned functional grasps can look plausible but fail if hand/object collision geometry is wrong.

## Recommended Use in This Project

### Phase 1: Use as Functional Object Set

Add a UniFucGrasp-inspired object subset to the local benchmark:

- mug or cup with handle
- spray bottle
- flashlight
- drill-like handle object
- bottle
- remote or tool handle

For each object, label a `functional_region`:

- handle
- trigger/button
- cap
- body
- spout/nozzle

This can extend the existing MuJoCo benchmark from generic lift to task-relevant contact.

### Phase 2: Use as Pose Prior, Not Controller

Use UniFucGrasp-generated or hand-authored functional poses as candidate presets:

```text
object point cloud + functional region -> candidate wrist pose + RH56 normalized hand pose
candidate -> MuJoCo contact/lift/function validation -> real replay preset
```

Do not run generated grasps directly on the robot. All candidates should pass:

- collision check
- contact distribution check
- lift check
- function-specific check
- real low-speed replay

### Phase 3: Add Functional Validation

Generic lift success is insufficient for this paper's target. Add task-specific success:

- mug: grasp handle and rotate/pour without slipping.
- drill: index/thumb posture reaches button; button press succeeds.
- spray bottle: fingers reach trigger region.
- bottle: cap/body grasp leaves cap accessible or enables twist.
- flashlight: power button reachable while maintaining hold.

### Phase 4: Learn RH56 Functional Codebook

Combine UniFucGrasp with the quantized-hand-state idea:

- Convert validated RH56 functional hand poses into ordered hand codes.
- Use codes such as `open`, `pre_handle`, `handle_grasp`, `trigger_press`, `pour_hold`, `release`.
- Train a small policy or retrieval model that selects functional code + wrist pose.

## Priority Recommendation

Current priority:

1. Keep the existing RH56 analytical planner and primitive pipeline for foam block, cylinder, can, and paper box.
2. Use UniFucGrasp as the next benchmark expansion after stable lift/replay.
3. Start with one functional task: mug handle grasp or drill-button press.
4. Only then evaluate the UniFucGrasp model/code directly.

Practical project sequence:

```text
stable RH56 lift benchmark
-> functional object benchmark
-> UniFucGrasp-inspired pose priors
-> sim validation
-> real low-speed replay
-> functional manipulation task
```

## Verdict

UniFucGrasp is a good fit for the project's medium-term research direction because it explicitly supports InspireHand and functional grasping. It should be cited and used when the project moves from "can the hand reliably lift objects?" to "can the hand grasp objects in a way that enables the next task?"

For immediate progress, the best action is not to port the full model. The best action is to extract its functional-grasp framing and add a small functional benchmark on top of the current RH56 MuJoCo/replay pipeline.

# 中文版本

## 结论

UniFucGrasp 适合作为本项目中期研究方向的参考，因为它明确支持 InspireHand 和 functional grasping。它回答的不是“能不能把物体拿起来”，而是“能不能用有功能意义的方式抓住物体，从而支持下一步任务”。

当前阶段不建议直接移植完整 UniFucGrasp 模型。更合理的做法是先抽取它的 functional-grasp framing，在当前 RH56 MuJoCo/replay pipeline 上加一个小型 functional benchmark。

## 对当前项目的价值

UniFucGrasp 对以下内容有直接启发：

- 物体功能区域标注。
- 任务相关的 palm/wrist pose。
- Inspire/RH56 风格手型。
- functional grasp success 的定义。

它适合在项目从稳定 lift/replay 转向功能抓取时使用。

## 推荐推进顺序

```text
稳定 RH56 lift benchmark
-> functional object benchmark
-> UniFucGrasp-inspired pose priors
-> sim validation
-> real low-speed replay
-> functional manipulation task
```

第一批任务可以选择：

- mug handle grasp。
- drill-button press。
- cup body grasp with opening avoidance。

## 不建议立即做的事

- 不要直接把完整 UniFucGrasp model 接到真机。
- 不要在 RH56 hand-code、palm-frame、MuJoCo replay 还没稳定前训练复杂功能抓取模型。
- 不要把仿真成功直接当作真实机器人成功。

## 当前定位

UniFucGrasp 在当前 active plan 中不是主模型，而是 functional grasp 扩展阶段的 prior 和 related work。
