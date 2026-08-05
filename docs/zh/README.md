# 中文文档索引 / Chinese documentation index

本页是当前文档的中文导航；English pages remain the exact command and API
authority. 每个当前主题都必须从下表进入对应权威页面，过程记录不放在当前文档中。

## 当前状态与安全 / Status and safety

| 中文主题 | Current authority |
| --- | --- |
| 当前状态与验证等级 | [`status/current_status.md`](../status/current_status.md) |
| 验证矩阵 | [`status/validation_matrix.md`](../status/validation_matrix.md) |
| 已知限制 | [`status/known_limitations.md`](../status/known_limitations.md) |
| 真机安全总则 | [`safety/REAL_HARDWARE_SAFETY.md`](../safety/REAL_HARDWARE_SAFETY.md) |
| 安全模型 | [`safety/safety_model.md`](../safety/safety_model.md) |
| 真机测试门 | [`safety/physical_test_gates.md`](../safety/physical_test_gates.md) |
| 控制器配置边界 | [`safety/controller_configuration.md`](../safety/controller_configuration.md) |
| 故障响应 | [`safety/incident_response.md`](../safety/incident_response.md) |

## 架构、操作与数据 / Architecture, operation, and data

| 中文主题 | Current authority |
| --- | --- |
| 系统架构 | [`architecture/SYSTEM_ARCHITECTURE.md`](../architecture/SYSTEM_ARCHITECTURE.md) |
| 共享目标管线 | [`architecture/shared_target_pipeline.md`](../architecture/shared_target_pipeline.md) |
| 仿真/真机一致性 | [`architecture/simulation_hardware_parity.md`](../architecture/simulation_hardware_parity.md) |
| Quest/JAKA 坐标系 | [`architecture/coordinate_frames.md`](../architecture/coordinate_frames.md) |
| 安装 | [`setup/INSTALLATION.md`](../setup/INSTALLATION.md) |
| Quest 设置 | [`operation/quest_setup.md`](../operation/quest_setup.md) |
| MuJoCo 仿真、回放与采集 | [`operation/simulation_demo.md`](../operation/simulation_demo.md) |
| JAKA 操作 | [`operation/jaka_arm_teleoperation.md`](../operation/jaka_arm_teleoperation.md) |
| RH56 操作 | [`operation/rh56_operation.md`](../operation/rh56_operation.md) |
| 双臂/手联合操作 | [`operation/jaka_rh56_combined_teleop.md`](../operation/jaka_rh56_combined_teleop.md) |
| 数据采集入口 | [`data/DATA_COLLECTION.md`](../data/DATA_COLLECTION.md) |
| 数据集规范 | [`data/DATASET_SCHEMA.md`](../data/DATASET_SCHEMA.md) |
| 数据采集与质量 | [`data/COLLECTION_GUIDE.md`](../data/COLLECTION_GUIDE.md) |
| 训练集成 | [`training/TRAINING_INTEGRATION.md`](../training/TRAINING_INTEGRATION.md) |
| 分布式训练准备 | [`training/DISTRIBUTED_TRAINING.md`](../training/DISTRIBUTED_TRAINING.md) |

## 开发与参考 / Development and reference

| 中文主题 | Current authority |
| --- | --- |
| 构建 | [`development/build.md`](../development/build.md) |
| 测试 | [`development/testing.md`](../development/testing.md) |
| 回放与日志 | [`development/logging_and_replay.md`](../development/logging_and_replay.md) |
| 配置参考 | [`reference/config_reference.md`](../reference/config_reference.md) |
| 命令参考 | [`reference/command_reference.md`](../reference/command_reference.md) |
| 日志结构 | [`reference/log_schemas.md`](../reference/log_schemas.md) |
| 术语 | [`reference/glossary.md`](../reference/glossary.md) |
| 故障排查 | [`TROUBLESHOOTING.md`](../TROUBLESHOOTING.md) |
| 数字孪生当前状态 | [`digital_twin/README.md`](../digital_twin/README.md) |
| RGB-D 当前状态 | [`d435_depth_pointcloud_readiness.md`](../d435_depth_pointcloud_readiness.md) |

## 文档规则 / Documentation rules

- 当前状态、架构、安全、操作和配置页面描述可重复的当前行为；不要把实时开发
  日志、一次性实验、未完成计划或旧报告写进这些页面。
- `dev_tmp/` 和 `docs/history/` 只保存记录性材料。它们不能覆盖当前安全规则，
  也不能把仿真、回放或 fake worker 证据升级成真机 PASS。
- 当前页面缺少中文细节时，先补充本页的中文摘要和链接，再修改英文权威内容；
  命令、路径、参数名保持英文原样。

- Current pages describe repeatable behavior; development logs, one-off
  experiments, unfinished plans, and old reports belong outside them.
- `dev_tmp/` and `docs/history/` are evidence only. They never override safety
  rules or turn simulation/replay/fake-worker evidence into a physical PASS.
- Keep commands, paths, and option names unchanged in both languages.
