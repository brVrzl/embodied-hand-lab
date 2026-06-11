# 文档入口

当前文档只保留可直接指导真实设备复现和论文方向判断的材料。早期探索笔记已从入口中移除，避免误导。

## 当前必读

- [近期可信文献与方法边界](literature/dexterous_grasping_recent_work_20260609.md)
- [Jetson AGX Thor 集成准备清单](jetson_agx_thor_integration_plan_20260611.md)
- [teleop_tools README](../src/teleop_tools/README.md)
- [真实数据采集协议](../real_robot_data_collection_protocol.md)

## 当前工程边界

- iPhone/HEBI Mobile I/O 遥操作是已调通功能，保留。
- Xbox/RViz shadow 遥操作是已调通功能，保留。
- JAKA/RH56 bring-up、ROS2 bridge、RH56 PC direct 链路保留。
- `data/sim_assets/jaka_rh56.xml` 是遥操作 IK/RViz shadow 依赖，保留。
- 旧仿真、外部数据集、训练探索内容不再作为默认入口。

新增文档应优先回答两个问题：

1. 是否能帮助真实 JAKA+RH56 复现或采集数据。
2. 是否能支持当前论文主线的可信论证。
