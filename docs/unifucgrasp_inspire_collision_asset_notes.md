# UniFucGrasp InspireHand Assets for RH56 Collision Work

Date: 2026-04-30

Local import:

```text
data/external/unifucgrasp_inspire/
  manifest.json
  UNIFUCGRASP_LICENSE
  urdf/inspire_right_force_sensor.urdf
  meshes/*.STL
  pointcloud/inspire.pth
```

Import command:

```bash
python tools/import_unifucgrasp_inspire_assets.py --unifuc-root /tmp/UniFucGrasp --overwrite
```

## Why These Assets Matter

The current project model in `data/sim_assets/jaka_rh56.xml` is already a good geometry/kinematics source for the installed RH56. It has the correct JAKA mount, RH56 actuator order, active joints, and project-specific collision proxies.

UniFucGrasp's InspireHand assets should not replace it wholesale. Their value is narrower and practical:

- richer fingertip / force-sensor pad meshes;
- an independent InspireHand URDF from another research codebase;
- a 12-joint Inspire hand model with mimic/coupling structure;
- pre-sampled hand point cloud `inspire.pth` useful for learning and geometry comparison;
- a reference for converting 12 joint-level annotations to 6 RH56 actuator-level commands.

## Main Difference From Current RH56 Model

Current local RH56 model:

- Based on local Inspire RH56 standard right-hand URDF / RoboTwin MJCF.
- Uses 6 position actuators:
  - thumb rotation
  - thumb bend
  - index
  - middle
  - ring
  - pinky
- Uses simplified analytic collision proxies for stable MuJoCo grasp tests.
- Already embedded in the JAKA mini2 model.

UniFucGrasp Inspire asset:

- Uses `right_*` link names and force-sensor meshes.
- Represents 12 joint-level values:
  - 4 thumb joints
  - 2 index joints
  - 2 middle joints
  - 2 ring joints
  - 2 little/pinky joints
- Includes tactile/force sensor mesh names such as:
  - `thumb_force_sensor_*`
  - `index_force_sensor_*`
  - `middle_force_sensor_*`
  - `ring_force_sensor_*`
  - `little_force_sensor_*`
- Is not mounted to JAKA and should be treated as a standalone hand reference.

## Recommended Collision Use

Use UniFucGrasp assets for A/B collision development:

1. Compare fingertip pad surface locations against the current cyan capsule proxies.
2. Use force-sensor pad meshes as visual guides for better capsule/sphere proxy placement.
3. Use `inspire.pth` to compare sampled hand point clouds under canonical poses.
4. Add a new collision mode later:

```text
proxy_current
mesh_current
unifuc_pad_proxy
unifuc_mesh_reference
```

The first target should be `unifuc_pad_proxy`: keep the current project kinematic chain and mount, but tune the fingertip proxy positions/radii using UniFucGrasp pad meshes as reference.

Do not immediately switch `data/sim_assets/jaka_rh56.xml` to the UniFucGrasp URDF because:

- mounting transform to JAKA is different;
- link names and axes differ;
- local real hand command order is already calibrated;
- full mesh collision is likely too brittle for grasp validation;
- switching the full hand would mix geometry changes with controller/order changes.

## Mapping For Dataset Use

UniFucGrasp Inspire target poses are expected to be:

```text
[x, y, z, qw, qx, qy, qz, 12 joint values]
```

The imported manifest records this 12D active joint order:

```text
right_thumb_1_joint
right_thumb_2_joint
right_thumb_3_joint
right_thumb_4_joint
right_index_1_joint
right_index_2_joint
right_middle_1_joint
right_middle_2_joint
right_ring_1_joint
right_ring_2_joint
right_little_1_joint
right_little_2_joint
```

Project-level RH56 6D canonical order:

```text
index
middle
ring
pinky
thumb_close
thumb_lateral
```

Approximate mapping for training:

```text
index         <- right_index_1_joint + right_index_2_joint
middle        <- right_middle_1_joint + right_middle_2_joint
ring          <- right_ring_1_joint + right_ring_2_joint
pinky         <- right_little_1_joint + right_little_2_joint
thumb_close   <- right_thumb_2_joint + right_thumb_3_joint + right_thumb_4_joint
thumb_lateral <- right_thumb_1_joint
```

This is only a first mapping. It must be calibrated against real RH56 commands and photos before claiming exact sim-to-real correspondence.

## How This Supports UniFucGrasp + DQ-RISE

Recommended sequence:

1. Import UniFucGrasp assets.
2. Convert official UniFucGrasp Inspire annotations to project RH56 6D normalized commands.
3. Train a UniFucGrasp-style model to predict:

```text
object point cloud -> wrist pose + RH56 6D hand pose
```

4. Validate predicted poses in MuJoCo with project collision modes.
5. Quantize validated RH56 hand poses into DQ-RISE-style ordered hand codes.
6. Train/compare:

```text
raw RH56 6D regression
vs ordered hand-code prediction
```

The UniFucGrasp assets help steps 2-4; DQ-RISE helps step 5-6.

## Immediate Next Engineering Step

Build a small converter/validator:

```text
UniFucGrasp .npy sample
-> parse object class/name + rtj
-> keep only hand_name == inspire
-> map 12D Inspire joints to 6D RH56 canonical command
-> run project MuJoCo validation
-> save samples.jsonl for training
```

This should happen before full model training. It makes the data useful even if the full UniFucGrasp training code remains brittle or tied to the authors' local paths.

# 中文版本

## 目标

该文档记录如何使用 UniFucGrasp 中的 InspireHand 资产来辅助 RH56 collision / contact 工作。

这些资产的价值不在于立即训练完整 UniFucGrasp 模型，而在于：

- 提供 InspireHand 风格的碰撞几何参考。
- 辅助校准 RH56 fingertip / pad proxy。
- 支持功能抓取候选的 MuJoCo 验证。
- 为后续 hand-code 与 functional grasp prior 建立数据转换路径。

## 推荐工程步骤

先构建一个小型 converter / validator：

```text
UniFucGrasp .npy sample
-> 解析 object class/name + rtj
-> 只保留 hand_name == inspire
-> 将 12D Inspire joints 映射到 6D RH56 canonical command
-> 运行项目 MuJoCo validation
-> 保存 samples.jsonl 供训练或分析
```

这一步应先于完整模型训练。即使 UniFucGrasp 原始训练代码较脆弱、依赖作者本地路径，转换后的数据仍然能用于本项目。

## 与当前主线的关系

UniFucGrasp 资产主要帮助：

- functional grasp region。
- hand/object contact prior。
- RH56 collision proxy 校准。
- functional object benchmark。

DQ-RISE / hand-code 方向主要帮助：

- ordered hand-code。
- policy action representation。
- low-dimensional hand control。

两者可以结合，但要分阶段：先资产转换和 MuJoCo 验证，再进入真实 RH56 replay 和 policy 学习。
