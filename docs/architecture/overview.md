# Architecture overview

## Current primary stack

The current primary stack combines a JAKA Mini2 six-joint arm, Inspire RH56DFX
hand, Meta Quest 3 hand/head tracking, a left Touch controller clutch, MuJoCo,
and an optional physical JAKA ServoJ/EDG adapter. The physical adapter is
separately authorized and is not used by simulation or the default tests.

The live policy is configured by
`configs/sim/quest_hts_jaka_mini2_live_demo.yaml`. The main entry points are
`tools/quest_jaka_mujoco_sim.py` for simulation and
`tools/quest_jaka_hardware.py` for deliberately gated physical stages.

## Control flow

```text
Quest HTS hand/head datagrams + CTRL v1 sidecar
        |
QuestDatagramReceiverWorker (bounded FIFO, receive timestamp)
        |
HTS/controller validation, freshness, ordering
        |
release-before-press clutch and reference capture
        |
relative wrist transform, latched head yaw, robot basis mapping
        |
translation/quaternion filters and deadbands
        |
bounded SE(3) continuation + shared MuJoCo IK
        |
limits, collision, Jacobian, branch, pose, output feasibility
        |
immutable AcceptedArmTarget
       / \
MuJoCo    JAKA joint adapter -> native 125 Hz EDG worker
```

The MuJoCo model is both the simulation plant and, on the hardware path, the
plant-free kinematic/collision model. The physical process does not step the
MuJoCo plant or copy its `qpos`.

## Authoritative modules

| Area | Source | Configuration | Principal tests |
|---|---|---|---|
| HTS/CTRL transport | `src/motion_input`, `src/quest_jaka_sim/live_input.py` | `configs/motion_input/quest_hts_right_hand.yaml` | `test_hts_protocol.py`, `test_hts_canonical.py`, `test_quest_controller_transport.py` |
| clutch/mapping/filter/IK | `src/quest_jaka_sim` | `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` | `test_quest_jaka_shared_pipeline.py`, mapping/filter/IK tests |
| common target contract | `src/teleoperation/accepted_target.py` | live simulation config | shared-pipeline tests |
| output feasibility | `src/teleoperation/output_feasibility.py` | live simulation config | `test_quest_jaka_output_feasibility.py` |
| MuJoCo adapter | `src/quest_jaka_sim/output.py` | simulation configs | simulation/shared-pipeline tests |
| JAKA adapter | `src/teleoperation/jaka/quest_adapter.py` | live simulation config plus gated CLI args | hardware CLI/shared-pipeline tests |
| native transport | `native/jaka_servo_worker/main.cpp` | CLI plus shared-memory protocol | native worker/resampler tests |
| RH56 | `src/rh56_driver`, `src/rh56_sim`, simulation retargeter | `configs/hand`, `configs/sim/quest_rh56_retarget.yaml` | RH56 schema/backend/H0/H2/retarget tests |

Other current project areas—digital twin, vision, iPhone RH56 experiments, and
ROS2/RViz bring-up—remain outside the primary Quest/JAKA shared pipeline. HEBI
phone teleoperation is retired and retained only as a compatibility reference.

The committed live configuration enables an integrated 6-arm + 6-hand
simulation. The explicit arm-only model builder remains the JAKA-only
production invariant: six JAKA actuators, no RH56 actuator or command path.

---

# 中文版：架构概览

## 当前主链

当前主链由 JAKA Mini2 六轴机械臂、Inspire RH56DFX、Meta Quest 3 手/头跟踪、
左 Touch 控制器 clutch、MuJoCo 和可选的 JAKA ServoJ/EDG 真机适配器组成。真机适配器
必须单独授权，仿真和默认测试不会使用它。

实时策略由 `configs/sim/quest_hts_jaka_mini2_live_demo.yaml` 管理。主要入口为：

- 仿真：`tools/quest_jaka_mujoco_sim.py`
- 真机 gate：`tools/quest_jaka_hardware.py`

## 控制流

```text
Quest HTS 手/头数据 + CTRL v1 sidecar
        |
有界 FIFO 和接收时间戳
        |
HTS/控制器有效性、新鲜度和顺序检查
        |
release-before-press clutch 与参考捕获
        |
相对手腕变换、锁存 head yaw、机器人基映射
        |
平移/四元数滤波和 deadband
        |
有界 SE(3) continuation + 共享 MuJoCo IK
        |
关节限位、碰撞、Jacobian、分支、位姿和输出可行性
        |
不可变 AcceptedArmTarget
       / \
MuJoCo    JAKA joint adapter -> 125 Hz native EDG worker
```

硬件路径使用 MuJoCo 模型做无 plant 的运动学/碰撞计算，但不会推进 MuJoCo plant，也不会
复制模拟 `qpos`。

## 权威模块

| 区域 | 代码/配置 |
|---|---|
| HTS/CTRL 传输 | `src/motion_input`、`src/quest_jaka_sim/live_input.py` |
| clutch、映射、滤波、IK | `src/quest_jaka_sim`、实时仿真 YAML |
| 共享目标契约 | `src/teleoperation/accepted_target.py` |
| 输出可行性 | `src/teleoperation/output_feasibility.py` |
| MuJoCo 适配器 | `src/quest_jaka_sim/output.py` |
| JAKA 适配器 | `src/teleoperation/jaka/quest_adapter.py` |
| 原生传输 | `native/jaka_servo_worker/main.cpp` |
| RH56 | `src/rh56_driver` 和仿真 retargeter |

数字孪生、视觉、iPhone RH56 和 ROS2/RViz 属于并行区域，不覆盖主链契约。HEBI 手机
遥操作已经停用，仅作为兼容参考保留。

已提交实时配置启用 6 个机械臂 + 6 个手部 actuator 的集成仿真。显式 arm-only builder
仍保留 JAKA-only invariant：只有 6 个 JAKA actuator，没有 RH56 actuator 或 command
path。
