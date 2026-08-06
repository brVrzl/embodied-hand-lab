# Documentation index

中文导航：[中文文档索引](zh/README.md)。命令、路径和安全规则以当前英文权威页为准，
中文索引用于快速定位和双语补充。

This index separates current authority from dated evidence. Read
[current status](status/current_status.md) before interpreting an old report
or planning physical work. Files under `history/` record what happened at a
particular time; they do not override current code, safety contracts, or
operator pages.

## First use

- [Installation](setup/INSTALLATION.md)
- [Configuration](configuration/CONFIGURATION.md)
- [Troubleshooting](TROUBLESHOOTING.md)
- [Command reference](reference/command_reference.md)
- [Current validation matrix](status/validation_matrix.md)
- [Maintenance policy](maintenance/README.md)
- [Runtime stop-condition audit](maintenance/runtime_stop_condition_audit.md)

## Architecture and safety

- [System architecture](architecture/SYSTEM_ARCHITECTURE.md) — components,
  control/data flow, adapter boundary, RH56 semantics, and missing integration
- [Shared accepted-target pipeline](architecture/shared_target_pipeline.md)
- [Simulation/hardware parity contract](architecture/simulation_hardware_parity.md)
- [Quest/JAKA coordinate frames](architecture/coordinate_frames.md)
- [Generic UMIP coordinate frames](motion_input/COORDINATE_FRAMES.md)
- [Real-hardware safety](safety/REAL_HARDWARE_SAFETY.md)
- [Safety model](safety/safety_model.md)
- [Physical test gates](safety/physical_test_gates.md)
- [Controller-configuration boundary](safety/controller_configuration.md)
- [Incident response](safety/incident_response.md)
- [Known limitations](status/known_limitations.md)

The current arm authority ends at immutable `AcceptedArmTarget`. MuJoCo and
physical JAKA receive the same accepted J1--J6 radians; the physical adapter
must not follow MuJoCo state, remap, filter, or solve IK.

## Simulation and operation

- [Quest/JAKA MuJoCo recording, replay, and live simulation](operation/simulation_demo.md)
- [RH56 H0 simulation self-test](operation/rh56_h0_self_test.md)
- [Quest host setup](operation/quest_setup.md)
- [Hardware prerequisites](operation/hardware_prerequisites.md)
- [JAKA arm teleoperation](operation/jaka_arm_teleoperation.md)
- [RH56 hand-only session debug](operation/rh56_session_debug.md)
- [RH56 PC-direct operation](operation/rh56_operation.md)
- [Combined JAKA and RH56 teleoperation](operation/jaka_rh56_combined_teleop.md)

Physical pages retain exact acknowledgements and bounded procedures. Reading
them or running `--help` does not authorize a device connection.

## Data and learning

- [Dataset collection entry](data/DATA_COLLECTION.md)
- [Canonical dataset schema](data/DATASET_SCHEMA.md)
- [Collection and quality guide](data/COLLECTION_GUIDE.md)
- [Policy-training integration](training/TRAINING_INTEGRATION.md)
- [Distributed-training readiness](training/DISTRIBUTED_TRAINING.md)
- [Offline benchmark harness](benchmark/BENCHMARKS.md)
- [Experiment and result discipline](experiments/EXPERIMENTS.md)
- [Execution roadmap](roadmap/NEXT_STEPS.md)

The repository provides data validation/export boundaries and a distributed
communication smoke test. It does not currently ship a maintained ACT,
Diffusion Policy, or OpenPI trainer.

Working reports and dated development records are kept outside the current
documentation set in [`../dev_tmp/`](../dev_tmp/). They are evidence only and
never override the current status or safety pages.

## Development and reference

- [Build](development/build.md)
- [Testing](development/testing.md)
- [Logging and replay](development/logging_and_replay.md)
- [Contribution workflow](development/contribution_workflow.md)
- [Configuration reference](reference/config_reference.md)
- [Log schemas](reference/log_schemas.md)
- [Glossary](reference/glossary.md)
- [Third-party notices](../THIRD_PARTY_NOTICES.md)

## Parallel research areas

- [Digital-twin workspace](digital_twin/README.md)
- [RGB-D readiness](d435_depth_pointcloud_readiness.md)
- [Motion-input platform](motion_input/README.md)
- [Quest Unity integration](../integrations/quest_unity/README.md)
- [Offline teleoperation rearchitecture](research/teleop_rearchitecture.md)
- [Command-health ABI research](research/teleop_command_abi.md)
- [Clutch-recovery transport research](research/jaka_clutch_recovery_transport_contract.md)

These areas do not silently replace the primary Quest/JAKA control authority.
`learned_policy/` is preserved inference research and remains outside the
maintained physical command path.

## History and evidence

[The history index](history/README.md) groups physical gates, incidents,
measurements, and superseded designs. Raw outcomes retain their original
claims and validation level. Current synthesis belongs in the status,
architecture, safety, and maintenance pages above.

## Documentation rules

- Keep one current authority per topic and link to it instead of copying it.
- Use repository-relative paths and commands checked against current help.
- Label offline, simulation, replay, physical PASS, physical FAIL, and
  unverified evidence literally.
- Do not rewrite historical failure evidence to match later behavior.
- Do not describe implementation, fake workers, replay, or simulation as a
  physical PASS.

## 中文导读

请先阅读[当前状态](status/current_status.md)、[安装](setup/INSTALLATION.md)和
[真机安全](safety/REAL_HARDWARE_SAFETY.md)。`history/` 中的文件只保存当时证据，
不覆盖当前代码和操作说明。测试、回放、仿真、`doctor` 与 `--help` 均不构成真机授权。

开发过程报告和带日期记录统一放在仓库根目录的 [`dev_tmp/`](../dev_tmp/)；它们只是证据，
不能覆盖当前状态页和安全页。
