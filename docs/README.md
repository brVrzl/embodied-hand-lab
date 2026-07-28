# Documentation

This index separates current instructions from dated evidence. Start with the
status page, then use the topic-specific page. Historical reports retain the
claims and paths that were true when written; they do not override current code
or this index.

Important current reading pages place a corresponding Chinese version after
the English text. Historical evidence remains in its original language so that
the recorded result is not rewritten after the fact.

## Current project documentation

### Architecture

- [Overview](architecture/overview.md)
- [Shared target pipeline](architecture/shared_target_pipeline.md)
- [Simulation/hardware parity](architecture/simulation_hardware_parity.md)
- [Coordinate frames](architecture/coordinate_frames.md)

### Operation

- [Quest/JAKA MuJoCo simulation](operation/simulation_demo.md)
- [RH56 H0 simulation self-test](operation/rh56_h0_self_test.md)
- [Quest host setup](operation/quest_setup.md)
- [Hardware prerequisites](operation/hardware_prerequisites.md)
- [JAKA arm teleoperation](operation/jaka_arm_teleoperation.md)
- [RH56 operation](operation/rh56_operation.md)
- [Troubleshooting](operation/troubleshooting.md)

### Safety and status

- [Safety model](safety/safety_model.md)
- [Physical test gates](safety/physical_test_gates.md)
- [Controller configuration boundary](safety/controller_configuration.md)
- [Incident response](safety/incident_response.md)
- [Current status](status/current_status.md)
- [Known limitations](status/known_limitations.md)
- [Validation matrix](status/validation_matrix.md)

### Development and reference

- [Repository layout](development/repository_layout.md)
- [Setup](development/setup.md)
- [Build](development/build.md)
- [Testing](development/testing.md)
- [Configuration](development/configuration.md)
- [Logging and replay](development/logging_and_replay.md)
- [Contribution workflow](development/contribution_workflow.md)
- [Repository consolidation audit](development/repository_consolidation_audit.md)
- [Projects workspace consolidation audit (2026-07-28)](audits/projects_workspace_consolidation_2026-07-28.md)
- [Projects workspace cleanup (2026-07-28)](audits/projects_workspace_cleanup_2026-07-28.md)
- [Legacy Quest worktree audit (2026-07-28)](audits/legacy_quest_worktrees_audit_2026-07-28.md)
- [Main documentation refresh (2026-07-28)](audits/main_documentation_refresh_2026-07-28.md)
- [Quest/JAKA teleoperation rearchitecture research (offline)](research/teleop_rearchitecture.md)
- [Teleoperation command-health ABI v1 (offline research)](research/teleop_command_abi.md)
- [JAKA clutch-recovery transport contract (offline research)](research/jaka_clutch_recovery_transport_contract.md)
- [Command reference](reference/command_reference.md)
- [Configuration reference](reference/config_reference.md)
- [Log schemas](reference/log_schemas.md)
- [Glossary](reference/glossary.md)
- [Third-party notices](../THIRD_PARTY_NOTICES.md)

## Other current project areas

- [Digital twin workspace](digital_twin/README.md)
- [Motion-input platform](motion_input/README.md)
- [RGB-D readiness](d435_depth_pointcloud_readiness.md)

## History and evidence

[The history index](history/README.md) classifies preserved physical gates,
incidents, raw measurements, handoffs, and superseded designs. Historical
evidence is intentionally not mixed into normal operator instructions.

## Documentation rules

- Keep one authoritative current page per topic.
- Use repository-relative paths and commands verified against current `--help`.
- State whether a result is offline, simulation, physical, failed, or
  unverified.
- Never rewrite raw evidence or a failed historical outcome.
- Move superseded material to history and update its index; delete only when
  complete duplication and absence of active references are proven.

---

# 中文版：文档索引

本索引把当前操作说明与历史证据分开。建议先阅读“当前状态”，再进入具体主题。历史报告只
保留其生成时真实的结论和路径，不覆盖当前代码和本索引。

## 当前项目文档

### 架构

- [架构概览](architecture/overview.md)
- [共享目标管线](architecture/shared_target_pipeline.md)
- [仿真/真机一致性](architecture/simulation_hardware_parity.md)
- [坐标系](architecture/coordinate_frames.md)

### 操作

- [Quest/JAKA MuJoCo 仿真](operation/simulation_demo.md)
- [RH56 H0 仿真自检](operation/rh56_h0_self_test.md)
- [Quest 主机设置](operation/quest_setup.md)
- [真机前置条件](operation/hardware_prerequisites.md)
- [JAKA 机械臂遥操作](operation/jaka_arm_teleoperation.md)
- [RH56 操作](operation/rh56_operation.md)
- [故障排查](operation/troubleshooting.md)

### 安全与状态

- [安全模型](safety/safety_model.md)
- [真机测试 gate](safety/physical_test_gates.md)
- [控制器配置边界](safety/controller_configuration.md)
- [事故响应](safety/incident_response.md)
- [当前状态](status/current_status.md)
- [已知限制](status/known_limitations.md)
- [验证矩阵](status/validation_matrix.md)

### 开发与参考

- [仓库布局](development/repository_layout.md)
- [环境设置](development/setup.md)
- [构建](development/build.md)
- [测试](development/testing.md)
- [配置](development/configuration.md)
- [日志与回放](development/logging_and_replay.md)
- [贡献流程](development/contribution_workflow.md)
- [仓库整理审计](development/repository_consolidation_audit.md)
- [Projects 工作区收敛审计（2026-07-28）](audits/projects_workspace_consolidation_2026-07-28.md)
- [Projects 工作区清理（2026-07-28）](audits/projects_workspace_cleanup_2026-07-28.md)
- [旧 Quest worktree 审计（2026-07-28）](audits/legacy_quest_worktrees_audit_2026-07-28.md)
- [Main 文档审阅（2026-07-28）](audits/main_documentation_refresh_2026-07-28.md)
- [Quest/JAKA 遥操作重构调研（仅离线）](research/teleop_rearchitecture.md)
- [遥操作 command-health ABI v1（仅离线调研）](research/teleop_command_abi.md)
- [JAKA clutch 恢复 transport contract（仅离线调研）](research/jaka_clutch_recovery_transport_contract.md)
- [命令参考](reference/command_reference.md)
- [配置参考](reference/config_reference.md)
- [日志结构](reference/log_schemas.md)
- [术语表](reference/glossary.md)
- [第三方声明](../THIRD_PARTY_NOTICES.md)

## 其他当前区域

- [数字孪生工作区](digital_twin/README.md)
- [运动输入平台](motion_input/README.md)
- [RGB-D 准备状态](d435_depth_pointcloud_readiness.md)

## 历史证据

[历史索引](history/README.md)对真机 gate、事故、原始测量和已取代设计进行分类。历史证据
不会混入普通操作说明。

## 文档规则

- 每个主题只保留一个当前权威页面。
- 使用仓库相对路径，并用当前 `--help` 核实命令。
- 明确区分离线、仿真、真机、失败和未验证状态。
- 不重写原始证据或历史失败结果。
- 被取代材料进入历史区；只有证明完全重复且无活动引用时才删除。
