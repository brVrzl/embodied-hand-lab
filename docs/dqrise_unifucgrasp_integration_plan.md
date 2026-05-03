# DQ-RISE + UniFucGrasp Integration Plan

Date: 2026-04-30

## Verdict

The two papers can be combined, but only if their roles are separated:

- UniFucGrasp should provide functional grasp priors: object functional regions, candidate wrist poses, and RH56/Inspire-style hand postures.
- DQ-RISE should provide the temporal action representation: continuous arm motion plus ordered, quantized hand-state codes.

Do not combine them as two full models immediately. A direct "UniFucGrasp model -> DQ-RISE visuomotor policy -> real robot" stack is too heavy for the current project stage and would hide failures behind perception, contact modeling, hand calibration, and policy learning.

The workable integration is:

```text
object mesh/point cloud + functional region
-> UniFucGrasp-style candidate hand/wrist pose
-> MuJoCo validation
-> RH56 hand-code library
-> DQ-RISE-style state/vision policy predicts arm_delta + relaxed_hand_code
-> low-speed real replay
```

## Why This Can Work

The roles are complementary:

| Component | What it solves | Project use |
| --- | --- | --- |
| UniFucGrasp | Which hand pose is functionally meaningful for an object | Generate or label candidate RH56 functional poses |
| DQ-RISE | How to make policy learning avoid raw high-DoF hand actions | Predict ordered RH56 hand codes instead of 6 raw finger commands |
| Current RH56 planner | Which candidates are physically plausible for this hardware | Sim filter before real replay |
| Failure-aware protocol | Which real executions are usable for learning | Data curation and ablations |

This fits the current hardware constraints because RH56 through JAKA tool RS485 is low-frequency. A quantized hand-code policy is much more realistic than high-rate finger teleoperation.

## What Not To Do

Avoid these for the next 1-2 weeks:

- Full RISE point-cloud policy training.
- Full UniFucGrasp model port before sim validation.
- FoundationPose-only object tracking as a hard dependency.
- Functional tasks requiring precise force control, e.g. tight cap twisting.
- In-hand rotation or high-speed continuous finger control.

## Minimal Sim Experiment

### Task 1: Functional Side Grasp on Can/Bottle Proxy

Goal:

- Grasp a light can/bottle side surface while leaving top/cap area accessible.

Existing assets:

- `light_can_50mm`
- `light_cylinder_36mm`
- existing RH56 MuJoCo candidate validation

Success:

- Lift >= 8 cm.
- No slip for 2 s.
- Fingertip contacts avoid forbidden functional region, e.g. top cap zone.

Hand codes:

```text
0 open
1 pre_side_grasp
2 power_envelope
3 hold
4 release
```

### Task 2: Mug Handle / Handle Proxy

Goal:

- Contact or hook a handle-like region without simply wrapping the whole object body.

Use a simplified MuJoCo object first:

- cylinder body
- handle proxy as a torus-like or box U-shape
- functional region label on handle

Success:

- At least one selected finger/thumb contact on handle region.
- Object can be lifted or rotated slightly without losing contact.

Hand codes:

```text
0 open
1 pre_handle
2 handle_hook_or_pinch
3 hold
4 release
```

### Task 3: Drill Button / Button Proxy

Goal:

- Hold a handle while index/thumb can reach a button region.

Use a simplified MuJoCo object:

- handle body
- small raised button geom

Success:

- Stable handle grasp.
- Index fingertip reaches button region.
- Optional: button geom displacement crosses threshold.

Hand codes:

```text
0 open
1 pre_tool_handle
2 handle_grasp
3 index_press
4 release
```

## Training / Policy Baseline

First baseline should be state-only or structured-observation, not full vision:

Input:

```text
ee_pose
robot_q
object_pose
functional_region_pose
current_hand_code
stage
```

Output:

```text
ee_delta_xyz_rpy
relaxed_hand_code
```

Compare:

- scripted staged primitive
- raw 6-DOF hand regression
- manual ordered RH56 hand codes
- learned KMeans/VQ hand codes after enough demos

This directly tests the DQ-RISE hypothesis in the project setting: quantized hand state should improve learnability and reliability over raw hand actions.

## Camera Deployment Plan

When the RGB-D camera is deployed, keep the first real experiments deliberately constrained:

1. Calibrate camera intrinsics and camera-to-robot extrinsics.
2. Use fixed object fixtures or ArUco/AprilTag markers first.
3. Record RGB, depth, point cloud, object fixture ID, and low-dimensional robot state.
4. Do not make FoundationPose or category-level pose estimation a blocker for the first week.
5. Replay sim-validated poses at low speed.
6. Use manual success/failure labels and keep `use_for_bc` strict.

Minimum real trials:

- 10 empty hand-code replay trials.
- 10 object contact-only trials.
- 20 lift trials for one object.
- 20 functional-region trials after lift is stable.

## Additional Literature To Use

### AnyDexGrasp

Use for contact-centric representation and hand-specific trial filtering. It separates universal scene/contact reasoning from per-hand grasp decision models and reports 75-95% real grasp success across hands with hundreds of attempts on 40 training objects.

Project relevance:

- Very useful for the "object/contact candidate -> RH56-specific validator" split.
- More relevant than full end-to-end policy learning for the current stage.

### FunGrasp

Use for functional grasp task design and sim-to-real cautions. It combines human-to-robot retargeting, dynamic grasp control, and sim-to-real methods, with evaluation on Allegro and Inspire hands.

Project relevance:

- Good medium-term target after stable static functional grasp.
- Too heavy to reproduce immediately because it adds RL/dynamic control.

### D(R,O) Grasp / T(R,O) Grasp

Use as later cross-embodiment grasp generators. These methods take hand description and object point cloud or robot-object transformations to synthesize cross-hand grasps.

Project relevance:

- Good for future learned candidate generation.
- Current project should first keep the existing RH56 analytical candidate generator and use these as references, not dependencies.

### DexFuncGrasp / Web2Grasp

Use as related work and object/task inspiration for functional grasping.

Project relevance:

- DexFuncGrasp is a functional grasp dataset baseline.
- Web2Grasp shows web human-object data can bootstrap functional grasps, then simulator augmentation improves success.
- Both are useful later, but not necessary for the first sim experiment.

## Recommended Next Step

Implement the smallest combined experiment:

```text
light_can_50mm functional side grasp
-> add functional_region labels
-> generate/choose 4-5 RH56 hand codes
-> run MuJoCo lift + forbidden-region/contact-region scoring
-> export replay_dataset with hand_code
-> train state-only BC or scripted hand-code replay
```

If this works in sim, move the exact same hand-code sequence to the real robot after camera calibration, starting with fixed object pose and low-speed replay.

This gives a clear research story:

```text
UniFucGrasp supplies functional grasp priors.
DQ-RISE supplies quantized hand-state policy representation.
RH56 validation supplies low-cost hardware grounding.
```

# 中文版本

## 结论

DQ-RISE 和 UniFucGrasp 可以结合，但必须明确分工，不能一开始就把两个完整模型强行串起来。

合理分工是：

- UniFucGrasp 提供 functional grasp prior，包括物体功能区域、候选 wrist/palm pose、Inspire/RH56 风格手型。
- DQ-RISE 提供 temporal action representation，也就是连续 arm motion 加有序 quantized hand-state code。
- RH56 真机验证负责低成本硬件落地，包括可执行性、碰撞、接触和 lift 成功率。

不建议直接做：

```text
UniFucGrasp model -> DQ-RISE visuomotor policy -> real robot
```

因为这会把 perception、contact modeling、hand calibration、policy learning 和 real robot control 的错误全部混在一起，难以定位问题。

## 当前推荐的最小版本

建议先做一个小而明确的功能抓取实验：

```text
light_can_50mm functional side grasp
-> 添加 functional_region label
-> 生成或选择 4-5 个 RH56 hand-code
-> 运行 MuJoCo lift 与 forbidden-region/contact-region scoring
-> 导出带 hand_code 的 replay_dataset
-> 训练 state-only BC 或 scripted hand-code replay
```

如果仿真有效，再在完成相机标定后，把同一 hand-code sequence 迁移到真实机器人，先从固定物体姿态和低速 replay 开始。

## 研究故事

这条线可以形成清楚的论文叙事：

```text
UniFucGrasp 提供功能抓取先验。
DQ-RISE 提供量化手部状态表示。
RH56 验证提供低成本真实硬件 grounding。
```

在当前 active plan 中，UniFucGrasp 不是主模型，而是用于后续 functional grasp 任务扩展的 prior 来源。
