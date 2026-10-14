# JAKA Mini2 灵巧手模仿学习：真机数据、ACT部署与多策略仿真验证

可直接用于简历的表述：

- 建立双路RGB与机器人状态同步的抓瓶示教训练流程，审核整理52条逻辑轨迹、33,111帧，锁定37条训练/15条验证划分；完成ACT训练、π0.5 LoRA训练及Diffusion Policy基线训练，统一核验数据、归一化与权重身份。[数据与训练证据](ASSET_AND_CHECKPOINT_AUDIT.md)
- 完成ACT在JAKA Mini2 + RH56DFX上的真实部署与有界运行调试，定位观测新鲜度、动作执行时序及退出流程问题，保留硬件命令记录与异常退出证据。[历史真机证据](historical_physical_evidence.json)
- 重建MuJoCo抓瓶搬运场景，将ACT、π0.5与Diffusion Policy接入当前仿真图像和状态驱动的闭环；统一12维动作、30 Hz仿真控制与固定8步执行，完成独立专家物理验收，并用阶段指标、动作投影和全部尝试记录分析直接迁移失败。[接口](OBS_ACTION_CONTRACT.md) · [协议](EVALUATION_PROTOCOL.md) · [验收](environment_acceptance.json) · [闭环记录索引](EVIDENCE_INDEX.md)

## 数字与经历的证据边界

|事实|准确数字与含义|内部证据|
|---|---|---|
|数据|52条/33,111帧；训练37条/23,802帧，验证15条/9,309帧；原始采集episode不跨split|[asset_identity.json](asset_identity.json) 的data及source_episode_overlap|
|ACT训练|实际100,000次更新；使用nominal52最终checkpoint。实际保存配置chunk60、n_action_steps=2；本轮从完整chunk执行8步|[资产审计](ASSET_AND_CHECKPOINT_AUDIT.md)；原资产根 `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/training_state/training_step.json`|
|π0.5训练|历史训练完成20,000次更新；本轮沿用历史验证选中目录4500，其train_state与optimizer counter实读均为4,501；目录标签与实际更新数分开陈述|[pi05_step_counter_audit.json](pi05_step_counter_audit.json)|
|DP训练|本轮实际10,000次更新、80,000样本访问；固定单seed、最终预算checkpoint；不同模型不以步数宣称算力公平|[dp_evidence/train_seed20261014.json](dp_evidence/train_seed20261014.json) · [训练轨迹](dp_evidence/train_metrics.jsonl)|
|真实闭环接入|ACT、π0.5、DP均已用实际权重执行当前仿真观测闭环；各首次短冒烟均为60个控制tick、8次实际模型查询，模拟时长2秒；这是接入证据，不是完整任务成功|工作区 `outputs/physical_bottle_sim/{act_dev_first,pi05_dev_first,dp_dev_first}/episodes.jsonl`；正式逐集结果见[证据索引](EVIDENCE_INDEX.md)|
|环境物理验收|慢释放expert完整任务1/1；原始示教动作replay为0/1，失败也保留。两者均通过控制目标和physics step执行，均不计入策略成功率|[environment_acceptance.json](environment_acceptance.json) 的expert_slowrelease及replay|
|历史ACT真机调试|早期ACT有901条accepted target，但UNLABELED且存在状态新鲜度缺陷；nominal52 ACT有5条policy command后触发freshness abort，native计数6包含startup alignment|[historical_physical_evidence.json](historical_physical_evidence.json)|

证据根目录：本轮工作区 `/home/thor/projects/embodied-bottle-sim`；原资产根 `/home/thor/projects/embodied_lab`。原始数据、权重和视频保留本地，不随简历或Git发布。

## 目前不能写的结论

- 不能把训练、离线推理、闭环接入或expert成功写成任一策略完整抓放成功。正式成功率必须引用最终逐集分子/分母，不能用本页的短冒烟代替。
- 可以写ACT真实部署与安全调试；不能写ACT已取得完整真机抓放成功率，也不能据此扩展为π0.5或DP完成真机任务验证。本轮没有真机操作。
- 不能写原始checkpoint直接迁移成功、照片级数字孪生、通用操作能力、模型优劣排名或LIBERO时序优化收益。本场景包含估计几何、相机和接触参数，DP只有单训练seed。
