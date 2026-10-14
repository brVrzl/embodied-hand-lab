# JAKA Mini2 + RH56DFX 多策略抓瓶仿真验证

工作区 `/home/thor/projects/embodied-bottle-sim`，分支 `feat/physical-bottle-multipolicy-sim`。
原 `/home/thor/projects/embodied_lab` 未提交修改保留。本轮只运行离线数据读取、模型训练和 MuJoCo，没有连接真实设备。

已完成：资产/划分核验、ACT100000与π4500真实权重推理、训练推理transform parity、原生12D闭环runner、物理场景、独立expert完整任务成功、40个无碰撞初始化实例、ACT两组各20正式rollout。
π正式评测与DP10000步训练正在运行，尚不预填其结果。完整结果完成后写入RESULTS.md。

关键入口：

```bash
cd /home/thor/projects/embodied-bottle-sim
MUJOCO_GL=egl PYTHONPATH=src:tools /home/thor/projects/embodied_lab/.venv/bin/python \
  tools/bottle_sim_rollout.py --model act --phase nominal \
  --instances results/physical_bottle_sim/initial_instances.json \
  --acceptance results/physical_bottle_sim/environment_acceptance.json \
  --output outputs/physical_bottle_sim/new_act_nominal
```

`--model`支持act/pi05/dp，`--phase`支持nominal/perturbed；默认40模拟秒、固定8步再查询。输出目录必须不存在。DP正式权重必须由正常训练生成，不能使用pilot替代；其完整训练命令在DP_REPORT.md。

报告：[资产](ASSET_AND_CHECKPOINT_AUDIT.md)、[接口](OBS_ACTION_CONTRACT.md)、[场景](SCENE.md)、[评测协议](EVALUATION_PROTOCOL.md)、[失败分析](FAILURE_ANALYSIS.md)。

本地完整闭环记录/视频在此worktree的 `outputs/physical_bottle_sim/`；原始开发图像对照、固定样本推理和环境验收视频在 `/home/thor/projects/embodied_lab/outputs/physical_bottle_sim/`。它们不上传，仓库只保留代码、配置、轻量结果与证据索引。大权重、原始数据、真实图像与凭据均不提交。
