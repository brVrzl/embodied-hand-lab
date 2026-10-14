# JAKA Mini2 + RH56DFX 多策略抓瓶仿真验证

工作区 `/home/thor/projects/embodied-bottle-sim`，分支 `feat/physical-bottle-multipolicy-sim`。
原 `/home/thor/projects/embodied_lab` 未提交修改保留。本轮只运行离线数据读取、模型训练和 MuJoCo，没有连接真实设备。

已完成：资产/原始episode划分核验；ACT100000与π4500真实权重推理；训练/推理transform parity；原生12D闭环runner；物理场景与独立expert验收；DP正常10000更新训练；三模型各20标称+20扰动正式rollout，共120次。六组完整任务成功均为0/20；无正式技术错误或数值失稳。详细结论见[RESULTS.md](RESULTS.md)。

剩余科学/工程限制：相机、物体和接触参数仍有估计，学习策略尚未完成完整抓放，未做仿真适配，也未开展本轮真机验证。这些是交付结论和后续研究边界，不是仍在后台运行的任务。全部本轮训练/评测进程已正常结束。

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

验证：7项相关行为/手部接口测试通过；实际12通道方向与范围、transform round-trip、checkpoint固定样本、DP前后向/拟合/保存恢复、物理expert及120次正式闭环记录均已检查。最终核对六组实例与冻结清单逐项一致，每集1200控制tick/150实际查询，原始动作NPZ与trace齐全；不将测试数量当任务成功。
