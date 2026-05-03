# sim_maniskill

最小 ManiSkill 仿真采集入口。

- 创建 `PickCube-v1` 这类桌面任务环境
- 提取 `rgb/depth/qpos/qvel/tcp_pose/success`
- 复用现有 `EpisodeRecorder` 和结构化导出链路

当前定位：

- 先验证“仿真采数据 -> 导出 -> 后续训练”链路
- 不替代真实硬件 bring-up
- 不要求一开始就还原 `JAKA mini2 + RH56`
