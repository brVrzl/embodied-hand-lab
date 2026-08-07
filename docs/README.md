# Documentation index

This directory contains the current operating documentation for Embodied Lab.
Each current page is written in English first and has the corresponding
Chinese version after it. Read [current status](status/current_status.md) and
[real-hardware safety](safety/REAL_HARDWARE_SAFETY.md) before interpreting
physical evidence or opening a device.

Files under [history](history/README.md) are dated evidence or superseded
design records. They are preserved as written and never override current code,
safety rules, or operator procedures.

## Start here

- [Installation](setup/INSTALLATION.md)
- [Configuration](configuration/CONFIGURATION.md)
- [Command reference](reference/command_reference.md)
- [Troubleshooting](TROUBLESHOOTING.md)
- [Current status](status/current_status.md)
- [Validation matrix](status/validation_matrix.md)

## Architecture and safety

- [System architecture](architecture/SYSTEM_ARCHITECTURE.md)
- [Shared target pipeline](architecture/shared_target_pipeline.md)
- [Quest/JAKA coordinate frames](architecture/coordinate_frames.md)
- [Simulation and hardware parity](architecture/simulation_hardware_parity.md)
- [Safety model](safety/safety_model.md)
- [Real-hardware safety](safety/REAL_HARDWARE_SAFETY.md)
- [Physical test gates](safety/physical_test_gates.md)
- [Controller configuration boundary](safety/controller_configuration.md)
- [Incident response](safety/incident_response.md)
- [Known limitations](status/known_limitations.md)

The arm acceptance boundary is immutable `AcceptedArmTarget`. MuJoCo and the
physical JAKA adapter receive the same accepted six-joint target; the physical
adapter does not follow MuJoCo state, remap the target, or solve IK.

## Operation

- [Quest host setup](operation/quest_setup.md)
- [Hardware prerequisites](operation/hardware_prerequisites.md)
- [MuJoCo simulation and replay](operation/simulation_demo.md)
- [JAKA arm teleoperation](operation/jaka_arm_teleoperation.md)
- [RH56 operation](operation/rh56_operation.md)
- [Combined JAKA/RH56 teleoperation](operation/jaka_rh56_combined_teleop.md)
- [RH56 H0 simulation self-test](operation/rh56_h0_self_test.md)
- [Manual functional validation](validation/MANUAL_FUNCTIONAL_VALIDATION.md)

Reading a page or running `--help`, tests, replay, or simulation does not
authorize JAKA, RH56, Quest, or camera access.

## Data and input

- [Physical episode collection](data/DATA_COLLECTION.md)
- [Dataset schema](data/DATASET_SCHEMA.md)
- [Motion input platform](motion_input/README.md)
- [UMIP protocol](motion_input/UMIP_PROTOCOL.md)
- [Quest controller transport](motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md)

The maintained physical collection format is review-first RGB plus a
low-dimensional robot state/action table. Quest packets, TCP, and depth are
not stored in the default training view; conversion happens only after human
review.

## Reference and status

- [Log schemas](reference/log_schemas.md)
- [Glossary](reference/glossary.md)
- [Third-party notices](../THIRD_PARTY_NOTICES.md)
- [Validation matrix](status/validation_matrix.md)
- [Known limitations](status/known_limitations.md)
- [History index](history/README.md)

## Documentation rules

- Keep one current authority per topic.
- Put English first and the matching Chinese text after it in the same file.
- Use repository-relative paths and commands verified against current code.
- Label offline, simulation, replay, partial physical, physical PASS, and
  unvalidated evidence literally.
- Do not rewrite historical evidence to match later behavior.
- Do not describe implementation, fake workers, replay, or simulation as a
  physical PASS.

---

# 文档索引

本目录保存 Embodied Lab 当前的操作说明。每个当前页面均先写英文，再提供对应的中文内容。
在解释真机证据或打开设备前，先阅读[当前状态](status/current_status.md)和[真机安全](safety/REAL_HARDWARE_SAFETY.md)。

[历史目录](history/README.md)中的文件是带日期的证据或已废弃设计记录，保持原样保存，不覆盖当前代码、安全规则和操作流程。

## 从这里开始

- [安装](setup/INSTALLATION.md)
- [配置](configuration/CONFIGURATION.md)
- [命令参考](reference/command_reference.md)
- [故障排查](TROUBLESHOOTING.md)
- [当前状态](status/current_status.md)
- [验证矩阵](status/validation_matrix.md)

## 架构与安全

- [系统架构](architecture/SYSTEM_ARCHITECTURE.md)
- [共享目标管线](architecture/shared_target_pipeline.md)
- [Quest/JAKA 坐标系](architecture/coordinate_frames.md)
- [仿真与真机一致性](architecture/simulation_hardware_parity.md)
- [安全模型](safety/safety_model.md)
- [真机安全](safety/REAL_HARDWARE_SAFETY.md)
- [真机测试门](safety/physical_test_gates.md)
- [控制器配置边界](safety/controller_configuration.md)
- [事故响应](safety/incident_response.md)
- [已知限制](status/known_limitations.md)

机械臂的接受边界是不可变的 `AcceptedArmTarget`。MuJoCo 和物理 JAKA 适配器收到同一个已接受的六关节目标；物理适配器不会跟随 MuJoCo 状态、重新映射目标或重新求 IK。

## 操作

- [Quest 主机设置](operation/quest_setup.md)
- [真机硬件前置条件](operation/hardware_prerequisites.md)
- [MuJoCo 仿真与回放](operation/simulation_demo.md)
- [JAKA 机械臂遥操作](operation/jaka_arm_teleoperation.md)
- [RH56 操作](operation/rh56_operation.md)
- [JAKA/RH56 联合遥操作](operation/jaka_rh56_combined_teleop.md)
- [RH56 H0 仿真自检](operation/rh56_h0_self_test.md)
- [手动功能验证](validation/MANUAL_FUNCTIONAL_VALIDATION.md)

阅读文档或运行 `--help`、测试、回放和仿真，都不会授权访问 JAKA、RH56、Quest 或相机。

## 数据与输入

- [物理 episode 采集](data/DATA_COLLECTION.md)
- [数据集格式](data/DATASET_SCHEMA.md)
- [运动输入平台](motion_input/README.md)
- [UMIP 协议](motion_input/UMIP_PROTOCOL.md)
- [Quest 控制器传输](motion_input/QUEST_CONTROLLER_TRANSPORT_HOST.md)

当前维护的真机采集格式是人工 review 优先的 RGB 视频加低维机器人 state/action 表。默认训练视图不保存 Quest packet、TCP 和深度数据，只有人工确认后才离线转换。

## 参考与状态

- [日志结构](reference/log_schemas.md)
- [术语表](reference/glossary.md)
- [第三方声明](../THIRD_PARTY_NOTICES.md)
- [验证矩阵](status/validation_matrix.md)
- [已知限制](status/known_limitations.md)
- [历史索引](history/README.md)

## 文档规则

- 每个主题只保留一个当前权威页面。
- 同一文件先写英文，再写对应中文。
- 使用已根据当前代码核对过的仓库相对路径和命令。
- 如实标注离线、仿真、回放、部分真机、真机 PASS 和未验证证据。
- 不修改历史证据以适配后来的行为。
- 不把实现、fake worker、回放或仿真描述为真机 PASS。
