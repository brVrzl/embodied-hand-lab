# Current capabilities

## English

Embodied Lab currently supports these boundaries:

| Function | Maintained boundary | Validation level |
| --- | --- | --- |
| Quest input | HTS/CTRL parsing, ordering, clutch state, recording, and replay | Offline tested |
| Arm control | Shared target generation, continuation IK, feasibility, and JAKA accepted-target adapter | Offline and simulation tested; physical validation is partial |
| RH56 | PC-direct USB/RS485 protocol, six active actuator channels, feedback, and simulation model | Offline tested; physical validation is partial |
| Simulation | Headless MuJoCo smoke, Quest replay, and interactive viewer | Simulation validated |
| Data | Review-first episodes, causal synchronization, manifests, statistics, and derived ACT/LeRobot views | Offline tested; raw/master inputs remain immutable |
| ACT | Strong ACT training and bounded rollout/shadow entrypoints | Offline/runtime contracts tested; no claim of physical policy success |
| π0.5 | `pi05_base` JAX LoRA adapter, normalization, checkpoint/resume, and Thor supervision | Training infrastructure; container/GPU dependent |
| Cameras | RealSense identity/stream checks and RGB-D processing | Offline tested; dual-camera collection is not fully physically validated |

The repository does not claim that simulation, replay, a fake worker, or a
training loss is a physical pass. Force data remains available in source
datasets; the first π0.5 baseline does not consume force.

## 中文

Embodied Lab 当前支持以下边界：

| 功能 | 维护边界 | 验证等级 |
| --- | --- | --- |
| Quest 输入 | HTS/CTRL 解析、排序、clutch 状态、recording 和 replay | 已完成离线测试 |
| 机械臂控制 | 共享 target 生成、continuation IK、feasibility 和 JAKA accepted-target adapter | 已完成离线/仿真测试；真机验证不完整 |
| RH56 | PC-direct USB/RS485 协议、六个 active actuator channel、feedback 和仿真模型 | 已完成离线测试；真机验证不完整 |
| 仿真 | headless MuJoCo smoke、Quest replay 和交互 viewer | 已完成仿真验证 |
| 数据 | 先审核 episode、因果同步、manifest、statistics 和 ACT/LeRobot 派生 view | 已完成离线测试；raw/master 输入不可变 |
| ACT | Strong ACT training 以及有界 rollout/shadow 入口 | runtime contract 已测试；不代表 policy 真机成功 |
| π0.5 | `pi05_base` JAX LoRA adapter、normalization、checkpoint/resume 和 Thor supervisor | 训练基础设施，依赖 container/GPU |
| 相机 | RealSense identity/stream 检查和 RGB-D 处理 | 已完成离线测试；双相机采集尚未完整真机验证 |

仓库不会把仿真、replay、fake worker 或 training loss 声称为真机 PASS。force 数据仍保存在源数据中，
第一版 π0.5 baseline 不读取 force。
