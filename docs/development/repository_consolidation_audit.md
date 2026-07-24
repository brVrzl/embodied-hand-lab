# Repository consolidation audit

Date: 2026-07-24. Worktree: `/home/thor/projects/embodied_lab`. Branch:
`feature/jaka-teledex-control-foundation`.

## Recovery checkpoint

Before deletion, local HEAD and
`origin/feature/jaka-teledex-control-foundation` were both
`e1afa45dc5e1c8ea58cf7c2fe82a044f26253f7c` with ahead/behind `0/0`.
Every tracked deletion in this cleanup is therefore recoverable from GitHub.
The concurrent modification to `tools/teleop_mujoco_jaka_rh56.py`, untracked
`learned_policy/`, linked worktrees, captures and local artifacts were excluded.

## Removed areas

The operator explicitly retired these independent research paths:

- geometry/tactile pregrasp prediction and dataset generation;
- ManiSkill/SAPIEN tasks, agent, viewer, recorder and dependencies;
- Xbox and TeleDex input/control paths;
- episode/data recorder and its provisional collection protocol;
- tennis-ball grasp flows and grasp benchmark;
- legacy RH56 collision-mode comparisons, proxy diagnostics and generated-stage
  research tools;
- pure research plans and current-document descriptions of removed features.

Tests dedicated only to those deleted features were removed with them. Safety,
shared Quest/JAKA, simulation parity, native EDG, RH56 driver, RealSense,
digital-twin and retained HEBI/iPhone tests remain.

## Retained authority and assets

- Quest HTS/CTRL to immutable `AcceptedArmTarget`, MuJoCo adapter and JAKA
  representation-only adapter;
- MuJoCo JAKA+RH56 runtime and committed `visual_coacd` collision asset, builder,
  manifest and safety regression;
- ROS2/RViz, HEBI phone experiments, iPhone RH56 experiments, physical RH56
  driver and bounded JAKA diagnostics;
- RealSense calibration/point-cloud work and the integrated digital twin;
- physical gate and incident evidence;
- Correll RH56DFX source assets plus upstream MIT license as a reference only.

The Correll reference is not a mounted runtime collision mode. The sole
supported RH56 runtime collision representation is the committed CoACD asset.

## HEBI smoothness review

HEBI's perceived smoothness comes from a separate lag-follow architecture:
target low-pass filtering, Cartesian velocity/acceleration shaping, lead and
workspace limits, tracking-error pauses, and optional SDK Servo filtering.
Copying those after `AcceptedArmTarget` would violate Quest/JAKA parity and
reintroduce hardware-only shaping.

Two ideas are reusable and already represented in the current stack: explicit
deadman/reference state, and tracking error staged as warning/hold/fault. HEBI
does not provide evidence that the unresolved Quest/JAKA collision issue should
be hidden by a second filter. The shared output-feasibility and physical
controller contracts remain the correct authority.

## Documentation policy

Current instructions remain indexed by `docs/README.md`. Digital-twin current
status is consolidated in `docs/digital_twin/README.md`; calibration, capture,
registration and measurement evidence remain beside it. Unique physical gate
and incident evidence is preserved under `docs/history/`. Deleted research
plans do not remain as competing current guidance.

---

# 中文版：仓库精简审计

日期：2026-07-24。记录时的 worktree 为 `/home/thor/projects/embodied_lab`，分支为
`feature/jaka-teledex-control-foundation`。

## 可恢复检查点

删除前，本地 HEAD 和 `origin/feature/jaka-teledex-control-foundation` 都是
`e1afa45dc5e1c8ea58cf7c2fe82a044f26253f7c`，ahead/behind 为 `0/0`。因此这次
整理中删除的所有已跟踪内容都可以从 GitHub 恢复。并发修改
`tools/teleop_mujoco_jaka_rh56.py`、未跟踪的 `learned_policy/`、关联 worktree、
采集文件和本地产物均未纳入。

## 已移除区域

操作者明确停用了以下相互独立的研究路径：

- 几何/触觉 pregrasp 预测及数据集生成；
- ManiSkill/SAPIEN 任务、agent、viewer、recorder 和依赖；
- Xbox 和 TeleDex 输入/控制路径；
- episode/数据记录器及其临时采集协议；
- 网球抓取流程和 grasp benchmark；
- 旧 RH56 碰撞模式比较、代理诊断以及生成阶段研究工具；
- 纯研究方案以及当前文档中对已移除功能的描述。

仅服务于这些已删除功能的测试也一并移除。安全、共享 Quest/JAKA、仿真一致性、原生
EDG、RH56 驱动、RealSense、数字孪生以及保留的 HEBI/iPhone 测试仍在。

## 保留的权威实现与资产

- Quest HTS/CTRL 到不可变 `AcceptedArmTarget`，以及 MuJoCo adapter 和仅做表示转换的
  JAKA adapter；
- MuJoCo JAKA+RH56 运行时、已提交的 `visual_coacd` 碰撞资产、构建器、manifest 和安全
  回归；
- ROS2/RViz、HEBI 手机实验、iPhone RH56 实验、真机 RH56 驱动和受限 JAKA 诊断；
- RealSense 标定/点云工作以及集成数字孪生；
- 真机 gate 和事故证据；
- 仅作为参考的 Correll RH56DFX 源资产及其上游 MIT 许可证。

Correll 参考不是运行时挂载的碰撞模式。唯一受支持的 RH56 运行时碰撞表示是已提交的
CoACD 资产。

## HEBI 平滑性复核

HEBI 的平滑感来自另一套滞后跟随架构：目标低通滤波、笛卡尔速度/加速度整形、超前量和
工作空间限制、跟踪误差暂停以及可选 SDK Servo 滤波。把这些放到
`AcceptedArmTarget` 之后会破坏 Quest/JAKA 一致性，并重新引入硬件专用整形。

有两个思路可以复用，并且已经体现在当前栈中：显式 deadman/reference 状态，以及把跟踪
误差分为 warning/hold/fault。HEBI 不能证明应该用第二级滤波掩盖尚未解决的
Quest/JAKA 碰撞问题；共享输出可行性和真机控制器契约仍是正确权威。

## 文档策略

当前说明统一由 `docs/README.md` 索引。数字孪生当前状态汇总在
`docs/digital_twin/README.md`；标定、采集、配准和测量证据保留在其旁边。不可替代的
真机 gate 与事故证据保留在 `docs/history/`。已删除研究方案不再作为互相竞争的当前
指导。
