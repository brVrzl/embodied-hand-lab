# 本地证据与公开轻量结果索引

源码/轻量报告工作区：`/home/thor/projects/embodied-bottle-sim`。
原有资产根：`/home/thor/projects/embodied_lab`。以下原始证据只保留本地；没有上传原始数据、权重或真实图像。

| 证据 | 本地路径 | 用途/边界 |
|---|---|---|
| 既有训练与选择 | 原资产根/outputs/training/pi05_rh56/{status.json,CHECKPOINT_SELECTION.md,reports/best_checkpoint.json,logs/weekend.log} | π4500选择与训练终末索引19999 |
| ACT训练权重 | 原资产根/outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/ | 模型与processor，不上传 |
| π训练权重 | 原资产根/outputs/training/pi05_rh56/checkpoints/pi05_rh56/weekend/4500/ | 本项目LoRA权重，不使用LIBERO模型 |
| 原始split与溯源 | 原资产根/data/training/physical_bottle_v4_nominal52/ | 原始/master与统计不改写 |
| 固定样本与parity | 原资产根/outputs/physical_bottle_sim/policy_contract/ | 实际ACT/π推理；DP pilot与processor/history/source parity；所有模型不是闭环结果 |
| 真实/仿真图像对照 | 原资产根/outputs/physical_bottle_sim/real_vs_sim_workspace.jpg、real_vs_sim_wrist.jpg | 内部开发RGB，外参为估计，不发布 |
| 环境原始失败/成功 | 原资产根/outputs/physical_bottle_sim/environment_replay_ep0.mp4、environment_expert_slowrelease.mp4 | 环境/expert，不能当策略成功 |
| 带标签环境视频 | 原资产根/outputs/physical_bottle_sim/acceptance_labeled/ | environment_replay.mp4失败；environment_expert_slowrelease.mp4成功 |
| ACT正式闭环 | 工作区/outputs/physical_bottle_sim/act_nominal_v2/、act_perturbed_v2/ | episodes/attempts、原始动作NPZ、物理trace、代表性失败MP4 |
| π正式闭环 | 工作区/outputs/physical_bottle_sim/pi05_nominal_v3/、pi05_perturbed_v3/ | 同上；修复混合单位早停后的版本 |
| DP正常训练 | 工作区/outputs/diffusion_bottle/train_seed20261014/ | 实际10000次更新、metrics.jsonl、最终report和pretrained_model；smoke目录独立 |
| DP正式闭环 | 工作区/outputs/physical_bottle_sim/dp_nominal/、dp_perturbed/ | 两组各20正式实例已完成，均0/20；包括全部40集视频 |
| 旧尝试 | 工作区/outputs/physical_bottle_sim/{act_dev_first,pi05_dev_first,act_nominal,act_perturbed,pi05_nominal,pi05_nominal_v2}/ | 全部保留，earlier_attempts.json解释统计去向 |

每个完整策略attempt的NPZ保留raw_chunks、raw_actions、executed_actions、逐查询latency；trace保留当前模拟状态/接触/物体动力学。模型只接收RGB与proprio，物体真值仅用于评估。

`episode_results/`存可提交的逐集CSV/JSONL和run元数据；`asset_identity.json`、`FINGERPRINTS.json`存请求的权重/归一化、数据与环境身份。`historical_physical_evidence.json`仅记录历史ACT部署与安全退出，不把UNLABELED或normal_completion清理状态当作任务成功。

代表性策略视频统一位于工作区 `outputs/physical_bottle_sim/representative_videos/`，文件名为 `{act,pi05,dp}_env0_model20261014_failure.mp4`，画面明确标注两个seed及最终失败状态。它们是原始首个正式实例完整40秒视频的加标签副本，不剪除失败片段；来源见 `representative_videos.json`。成功视频仅有上表 environment/expert 验收，不存在可据此声称策略成功的视频。

最终轻量统计见 `formal_summary.json`；`attempt_inventory.json` 对齐120次正式、3次开发及34次旧版本尝试。评测前后的运行脚本stdout保存在本地 `outputs/physical_bottle_sim/run_console_logs/`，不上传。
