# RH56 MuJoCo Model Accuracy Check

Date: 2026-04-28

## Short Answer

当前本地 `data/sim_assets/jaka_rh56.xml` 可以认为是 **几何/运动学上可信的 RH56 模型来源**，因为它来自本地 RoboTwin 生成的 RH56 MJCF，而该 MJCF 又来自 RH56 官方/本地 URDF 和 STL mesh。

However, it should **not yet be treated as a fully validated contact/dynamics model**. The public RH56DFX paper reports a sim-to-real validated MuJoCo model, but our local XML is not confirmed to be the same released model and does not include the paper's system-identified feedforward, damping, armature, joint friction, and force-control validation settings.

## Local Evidence

本地模型：

- `data/sim_assets/jaka_rh56.xml`
- source model: `/home/w/projects/RoboTwin/robot_sim/generated/jaka_minicobo_rh56.xml`

检查结果：

- `data/sim_assets/jaka_rh56.xml` 与 RoboTwin 生成模型在规范化 asset path 后完全一致。
- RH56 子模型来自 `/home/w/projects/RoboTwin/robot_sim/generated/rh56.xml`。
- `generated/rh56.xml` 的 link、joint、mesh、inertial、mimic/coupling 信息与本地官方 URDF 对齐。
- 本地官方 URDF 路径：
  `/home/w/projects/RoboTwin/灵巧手资料/RH56DFQ系列最新资料/URDF/三代手标准版/inspire_hand_3_standard/src/inspire_hand_r2024.11.5/urdf/inspire_hand_r.urdf`

## What Seems Accurate

可信部分：

1. **Visual geometry / 外观几何**
   - 使用 RH56 STL meshes。
   - Mesh names include palm, thumb links, and four finger proximal/distal links.

2. **Kinematic tree / 运动学树**
   - Thumb has two active joints plus coupled PIP/DIP.
   - Four non-thumb fingers each have MCP + coupled DIP.

3. **DOF count / 自由度数量**
   - Official manual states total joints = 12 and DOF = 6.
   - Local MJCF models 12 joints with 6 actuators and equality coupling constraints.

4. **DOF order / 控制顺序**
   - Official manual and Unitree RH56DFX controller both use:
     `[pinky, ring, middle, index, thumb_bend, thumb_rotation]`
   - This matches the physical mapping we discovered on the real hand.

5. **Joint ranges / 关节范围**
   - Local MJCF ranges match the URDF structure:
     - thumb rotation: `0..1.1`
     - thumb bend: `0..0.5`
     - index/middle/ring/pinky MCP: approximately `0..1.68/1.7`
     - coupled distal joints via equality constraints.

## What Is Not Yet Validated Locally

不应直接相信的部分：

1. **Contact realism / 接触真实性**
   - Mesh collision contact may not match real fingertip pads.
   - Current benchmark uses fingertip proxy spheres, which are useful for bring-up but not a calibrated contact model.

2. **Actuator dynamics / 执行动力学**
   - Local MJCF uses simple position actuators, e.g. hand `kp=8` in the base XML.
   - The RH56DFX paper explicitly identifies feedforward, damping, armature, and joint friction from hardware trajectories.

3. **Force feedback / 力反馈**
   - The model does not yet include calibrated intrinsic force conversion or force-control behavior.

4. **Mount transform / 安装外参**
   - The hand-to-flange mount in local setup remains an engineering approximation unless measured.

5. **Sim-to-real grasp success / 仿真到真实抓取成功率**
   - The public paper reports sim-to-real validation, but our local XML has not yet been shown to reproduce those exact contact force and grasp benchmarks.

## External Evidence

1. RH56DFX paper/project:
   - Reports a sim-to-real validated MuJoCo model for RH56DFX.
   - Reports hardware characterization, latency, force overshoot, calibrated joint limits/coupling constraints, system identification, and 300 real grasp trials across 15 objects.
   - Reports 87% grasp success for its validated planning/control stack.

2. Official Inspire RH56 manual:
   - Confirms RH56 has 12 total joints, 6 DOF, 6 force sensors, and repeated fingertip positioning accuracy of 0.2 mm.
   - Confirms register order for actual angle feedback:
     `ANGLE_ACT(0..5) = little, ring, middle, index, thumb bend, thumb rotation`.

3. Unitree RH56DFX controller:
   - Confirms the same 6-motor ordering:
     `pinky, ring, middle, index, thumb-bend, thumb-rotation`.

4. Dex-URDF:
   - Includes an Inspire Hand URDF model as part of a dexterous-hand model collection.
   - Useful as independent evidence that Inspire Hand URDF models are commonly used, but it is not evidence of contact/dynamics validation.

## Practical Conclusion for This Project

可以采用这个判断：

- **Use the current model for kinematics, workspace analysis, width-to-grasp planning, object pose candidate search, and policy interface development.**
- **Do not yet trust it for final contact success, force closure, slip, or sim-to-real performance claims.**

换句话说：

> 当前模型“形状和关节大概率是对的”，但“接触和动力学还需要本项目自己校准”。

## Next Validation Steps

1. Compare real open/close photos against MuJoCo rendered poses for 5 canonical hand configurations.
2. Measure fingertip positions in image or with simple caliper fixtures for open, half-close, and full-close states.
3. Implement width-to-grasp sweep like the RH56DFX paper:
   - input object width;
   - output thumb rotation, finger closure, wrist tilt/offset;
   - validate contact fingers and lift.
4. Add calibrated fingertip contact proxies rather than relying on raw mesh collision.
5. If Correll Lab releases code, compare their RH56DFX MJCF parameters against our local XML:
   - joint limits;
   - coupling constraints;
   - damping;
   - armature;
   - friction;
   - actuator control ranges;
   - fingertip collision geometry.
