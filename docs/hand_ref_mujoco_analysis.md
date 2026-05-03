# Hand Pose Generation References and MuJoCo Check / 手部姿态生成文献与 MuJoCo 检查

Date / 日期: 2026-04-28

## Conclusion / 结论

中文：`hand_ref.md` 中列出的论文值得使用，但不应该无条件替代当前已经成功的真机 primitive。更稳妥的短期策略是：保留真机上成功的纸盒 top-down pinch 作为校准 baseline，同时在 MuJoCo 中实现基于文献思想的候选姿态生成和接触验证流程。

English: The papers listed in `hand_ref.md` are useful, but they should not blindly replace the current real-hardware primitive. A more reliable short-term strategy is to keep the successful real paper-box top-down pinch as the calibration baseline while implementing literature-inspired candidate generation and contact validation in MuJoCo.

推荐策略 / Recommended strategy:

1. 中文：保留真机成功的 top-down paper-box pinch 作为 calibration baseline。  
   English: Keep the real successful top-down paper-box pinch as the calibration baseline.
2. 中文：优先使用 RH56-specific analytical planning，尤其是 object width to grasp、slow contact closing 和 contact-aware validation。  
   English: Use RH56-specific analytical planning first, especially object-width-to-grasp mapping, slow contact closing, and contact-aware validation.
3. 中文：等 RH56 MuJoCo 模型、物体位姿和 collision geometry 可信后，再接入跨 embodiment 的学习式候选生成方法。  
   English: Use learned cross-embodiment candidate generators later, after the RH56 MuJoCo model, object pose, and collision geometry are reliable.

## Reference Priority / 文献优先级

| Priority / 优先级 | Method / 方法 | Use Now? / 现在使用? | Reason / 原因 |
| --- | --- | --- | --- |
| 1 | RH56DFX Analytical Planning + Hybrid Force Control | Yes / 是 | 中文：直接针对 Inspire RH56DFX，最有用的是 hand characterization、width-based analytical grasp planning 和 slow contact/force-aware closing。English: Directly targets Inspire RH56DFX; the most useful parts are hand characterization, width-based analytical planning, and slow contact/force-aware closing. |
| 2 | AnyDexGrasp | Partial / 部分使用 | 中文：概念上有用，强调 contact-centric representation 和每种手少量真实 trial；但仍需要物体几何、感知和 per-hand trial data。English: Conceptually useful because it uses contact-centric representation and limited real trials per hand, but it still needs object geometry, perception, and per-hand trial data. |
| 3 | D(R,O) Grasp / T(R,O) Grasp | Later / 后续 | 中文：适合跨手型 grasp generation，但需要可靠 hand model、object point cloud、IK 和候选验证。English: Strong for cross-hand grasp generation, but requires reliable hand models, object point clouds, IK, and generated grasp validation. |
| 4 | UniFucGrasp | Later / 后续 | 中文：适合 bottle/cup/tool 等 functional grasp，不是当前纸盒 lift 的第一优先级。English: Useful for functional grasps on bottles, cups, and tools, but not the first priority for the current paper-box lift. |
| 5 | DexGrasp-Zero / CEDex / UniMorphGrasp | Reference / 参考 | 中文：适合 related work 和中长期扩展，但当前系统 bring-up 阶段偏重。English: Good related work and medium-term extensions, but too heavy for the current system bring-up stage. |

## Why Not Directly Switch to Learned Pose Generation / 为什么不直接切到学习式姿态生成

中文：当前瓶颈不只是“缺少姿态候选”。系统仍然需要解决 RH56 command/state calibration、拇指先旋转再闭合、稳定 approach pose、object frame 定义、sim-to-real collision/contact mismatch，以及物体模型和位姿精度问题。

English: The current bottleneck is not just the lack of pose candidates. The system still needs RH56 command/state calibration, correct thumb rotation before closure, reliable approach poses, object-frame definition, sim-to-real collision/contact alignment, and accurate object model/pose estimates.

中文：大多数学习式 pose-generation 方法默认 hand model 和 object point cloud/mesh 足够准确。如果这些输入不可靠，模型会生成看起来合理但真机仍然失败的 grasp。

English: Most learned pose-generation methods assume accurate hand models and object point clouds/meshes. If these inputs are unreliable, the generated grasps may look plausible but still fail on the real robot.

## MuJoCo Trial Performed / 已执行的 MuJoCo 测试

脚本 / Script:

```bash
.venv/bin/python tools/sim_pinch_box_pose.py --sweep --duration 3.5
```

输出 / Generated outputs:

- `data/mujoco_debug/pinch_box_v1/pinch_box_pose_sweep.json`
- `data/mujoco_debug/pinch_box_v1/sweep_*.xml`

编码的真机 primitive / Encoded real-hardware primitive:

- Arm grasp preset / 机械臂抓取位置: `pinch_grasp_box_v2`
- Arm lift preset / 机械臂抬升位置: `pinch_lift_box_v1`
- Hand stage 1 / 灵巧手阶段 1: `pinch_box_thumb_rotate_v2`
- Hand stage 2 / 灵巧手阶段 2: `pinch_box_v4`
- Physical hand norm / 物理手部归一化参数: `[0, 0, 0.12, 0.15, 0.4, 1.0]`

物理 DOF 顺序 / Physical DOF order:

```text
[pinky, ring, middle, index, thumb_bend, thumb_rotate]
```

MuJoCo actuator 顺序 / MuJoCo actuator order:

```text
[thumb_rotate, thumb_bend, index, middle, ring, pinky]
```

## MuJoCo Result / MuJoCo 结果

中文：当前原始 MuJoCo 模型没有复现真机中成功的纸盒 lift。sweep 找到的候选都偏弱，主要表现为 thumb-dominant contact，缺少 index/middle 的有效对向接触。

English: The original MuJoCo model did not reproduce the real successful paper-box lift. The sweep only found weak candidates, mostly with thumb-dominant contact and without effective opposing index/middle contact.

| Rank / 排名 | Offset / 偏移 | Final box z / 最终高度 | Contact Summary / 接触总结 |
| --- | --- | --- | --- |
| 1 | `[0.018, 0.009, 0.006]` | `0.0562` | thumb contact only; no index/middle box contact / 只有拇指接触，无食指/中指接触 |
| 2 | `[0.009, 0.009, 0.0]` | `0.0502` | thumb contact only; no index/middle box contact / 只有拇指接触，无食指/中指接触 |
| 3 | `[0.0, 0.0, 0.006]` | `0.0562` | thumb contact only; no index/middle box contact / 只有拇指接触，无食指/中指接触 |

解释 / Interpretation:

- 中文：这不说明真机抓取不好，因为真机已经成功拿起纸盒。  
  English: This is not evidence that the real grasp is bad, because the real robot already lifted the paper box.
- 中文：这说明当前 MJCF/contact setup 还不足以预测真实成功率。  
  English: It shows that the current MJCF/contact setup is not calibrated enough to predict real success.
- 中文：最大 mismatch 是仿真显示 thumb-dominant contact，而真实抓取更像 thumb + opposing fingers 的 top pinch。  
  English: The largest mismatch is that simulation reports thumb-dominant contact, while the real grasp appears to be a top pinch with thumb plus opposing fingers.

## Fixes Applied / 已应用修复

- 中文：加入 physical RH56 DOF 到 MuJoCo actuator 的映射。  
  English: Added physical RH56 DOF to MuJoCo actuator mapping.
- 中文：加入 box pose offsets 和 sweep mode。  
  English: Added box pose offsets and sweep mode.
- 中文：修复 `data.xpos` view-copy bug，该 bug 会导致 rotate/close 两阶段指尖位置看起来完全一样。  
  English: Fixed a `data.xpos` view-copy bug that made rotate/close fingertip positions appear identical.
- 中文：删除 non-sweep path 中重复执行的仿真。  
  English: Removed duplicate simulation execution in the non-sweep path.

## Updated Direction / 更新后的方向

中文：根据当前用户决策，项目应优先在 MuJoCo 中完成不同物体的抓取任务，再考虑迁移到真机。当前已经新增 `tools/mujoco_rh56_grasp_benchmark.py`，把文献思想落地成一个小型可运行流程：点云生成、宽度估计、解析候选生成、接触验证和 lift success。

English: Based on the current project decision, the next priority is to solve grasping for multiple object types in MuJoCo before transferring to the real robot. The new `tools/mujoco_rh56_grasp_benchmark.py` implements a small runnable version of the literature-inspired pipeline: point-cloud generation, width estimation, analytical candidate generation, contact validation, and lift success.

## Next MuJoCo Steps / 下一步 MuJoCo 工作

1. 中文：为每个最佳候选生成图片或视频，确认不是数值假成功。  
   English: Generate images or videos for the best candidates to verify that successes are visually meaningful.
2. 中文：加入 cylinder power grasp family，提升圆柱体抓取稳定性。  
   English: Add a cylindrical power-grasp family to improve cylinder stability.
3. 中文：提高 success lift threshold，从 `2 cm` 逐步提高到 `5 cm` 和 `8 cm`。  
   English: Raise the success lift threshold from `2 cm` to `5 cm` and then `8 cm`.
4. 中文：加入随机物体位姿扰动，统计每类物体 20 seeds 的成功率。  
   English: Add randomized object pose perturbations and report success rates over 20 seeds per object.
5. 中文：将仿真最优候选导出为真机 preset，包括 thumb rotation stage、close norm、object-relative pregrasp offset 和预期接触手指。  
   English: Export the best simulation candidates into real-hardware presets, including thumb rotation stage, close norm, object-relative pregrasp offset, and expected contact fingers.
