# 文档入口

本目录保存当前 `JAKA mini2 + Inspire RH56` 重建工作的项目说明、实验协议和审计记录。旧计划只能作为背景参考，不能作为某个子系统已经完成的证据。

## 优先阅读

- [项目 README](../README.md)
- [项目重建状态总览](project_rebuild_status.md)
- [Correll RH56DFX 资产整合评估](rh56dfx_correll_integration_assessment.md)
- [RH56DFX Correll 资产审计](literature/rh56dfx_correll_assets_audit_20260709.md)
- [RH56 预抓取预测协议](rh56_pregrasp_prediction_protocol.md)
- [真实机器人数据采集协议](../real_robot_data_collection_protocol.md)
- [LeRobot 数据与工作空间标定](lerobot_data_and_workspace_calibration.md)
- [网球数字孪生计划](tennis_ball_digital_twin_plan.md)
- [近期灵巧抓取文献边界](literature/dexterous_grasping_recent_work_20260609.md)

## 当前工程边界

- 仓库正在从仿真资产误删后的状态中重建。文档必须区分可用代码路径和恢复锚点。
- `data/sim_assets/jaka_rh56.xml` 是当前 JAKA+RH56 mounted integration model，因为下游工具仍依赖它；它不是最终验证模型。
- `data/sim_assets/correll_rh56dfx/` 是 RH56DFX reference hand asset set，用于浮动手 FK 规划和指尖 force/torque sensor 验证。
- 真实硬件脚本在没有明确只读说明前，都应按可能产生运动命令处理。
- 没有真实 replay 数据时，不要把仿真结果描述为真实 RH56 抓取性能。

## 写作规则

新增或更新文档时，明确标注资产或子系统状态：

- `validated`：有测试覆盖或近期真实硬件检查记录。
- `current anchor`：当前代码依赖，但仍需审计。
- `reference`：有用的外部模型或方法，不直接挂载到当前机器人栈。
- `plan`：计划或建议流程，尚未验证。
