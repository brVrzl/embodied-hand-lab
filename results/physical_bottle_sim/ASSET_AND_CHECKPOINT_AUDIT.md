# 资产与 checkpoint 审计

本轮只读来源根目录 `/home/thor/projects/embodied_lab`，独立实现工作区 `/home/thor/projects/embodied-bottle-sim`。以下路径无前缀时相对来源根目录。没有连接设备、改写原始数据、split、统计或已有权重。文件身份的 SHA256 是本任务明确要求的 checkpoint 指纹，见 `asset_identity.json`；未扫描原始视频指纹。

## 数据身份

实际读取 `outputs/training/pi05_rh56/audited_manifest.json`、`data/training/physical_bottle_v4_nominal52/manifests/logical_segments.json` 与 ACT 派生视图 `outputs/training/physical_bottle_v4_nominal52/lerobot/act_strong_view/meta/embodied_lab_provenance.json`：52 个逻辑片段、33111 帧、30 Hz；train 37/23802，validation 15/9309。两种模型的逻辑片段集合逐 split 相等（ACT 的派生 episode index 与 master index 不同，不能直接比较数字索引）。按原始 source_episode 分组检查，train/validation 交集为空。99 的两段均在 train；172 的三段与174的两段均在 validation。没有重划分数据。

任务为 `Pick up the bottle and place it on the cardboard box.`；12D 是 JAKA 六关节 + RH56 六主动通道，不是18D。训练源是 `data/training/physical_bottle_v4_nominal52/act/master`。Force 保留在源文件但不作为本轮模型输入。

## π0.5

- 已有选择文件 `outputs/training/pi05_rh56/reports/best_checkpoint.json` 与 `CHECKPOINT_SELECTION.md` 选中 `outputs/training/pi05_rh56/checkpoints/pi05_rh56/weekend/4500`；本轮沿用，不用仿真成功率挑选。
- 官方 `pi05_base`，JAX，`gemma_2b_lora + gemma_300m_lora`，native12D/internal32D，horizon16；原生输出经过实际训练变换还原。不是 LIBERO checkpoint。
- 历史训练日志实际finalize最终目录index19999并记录`completed_process`；源码先更新再用零基loop index保存，因此完成20000次optimizer update。所用checkpoint目录4500的TensorStore标量实读 `train_state.step=4501`，两项optimizer count也均为4501，即所用权重完成4501次更新（不是4500次）。只读三个标量，没有恢复大优化器数组；证据见 `pi05_step_counter_audit.json`。当前磁盘只保留4500，不能声称本轮使用最终19999权重。历史40个候选固定 validation probe（每条8窗口，共120，seed20360817）选4500，mean flow matching loss0.011417476918298537；全帧确认未完成。
- `params` 实际存在约6.0 GiB，另有 assets/train_state。权重分片与norm资产逐文件指纹见JSON。原有 norm 来自23802个训练帧，位于该 checkpoint `assets/local/pi05_rh56_train/norm_stats.json`。
- OpenPI pinned commit `15a9616a00943ada6c20a0f158e3adb39df2ccac`；历史运行 `/home/thor/openpi/repo` 有两处已记录 Thor patch，不能描述为完全未修改upstream。环境证据 `outputs/training/pi05_rh56/environment_summary.json`。
- 容器 `jaka-openpi:thor-cuda13`，历史image ID `sha256:1eb9c2ef62a6d01a0ae231aea290ebfb155e2e2e745ddf32a50be1c902476a5d`。batch4、seed20260814、LoRA训练；warmup100、peakLR1e-5。
- 两路640×480 RGB：workspace→base_0_rgb，wrist→left_wrist_0_rgb，第三槽masked padding；具体当前推理契约与实测结果另见 OBS_ACTION_CONTRACT.md。

## ACT

- 使用已有 `checkpoints/last -> 100000`，全路径 `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model`。规则为沿用已完成训练最后权重，不按本轮sim筛选。磁盘另有10000至90000每10000的checkpoint。
- `training_state/training_step.json` 明确step100000、batch16、1 process；`logs/act_strong_resume_010000.log` 记录100000、eval_loss0.2356和End of training。非仅依配置推断。
- LeRobot ACT，ResNet18 ImageNet初始化，VAE latent32，Transformer512，chunk60；实际checkpoint配置n_action_steps=2、无temporal ensemble（仓库训练模板目前写1，不能用模板覆盖权重元数据）。本轮worker调用完整predict_action_chunk返回60步，再由统一执行器取前8步；不使用checkpoint内部select_action队列，不修改权重预测长度。seed1000，AdamW LR1e-5。
- `model.safetensors` 与pre/post normalization资产指纹见JSON。历史视图明确 `normalization_stats_scope: train logical segments only`；RGB采用ImageNet统计，state/action采用MEAN_STD。不重新拟合已有统计。
- ACT派生图像320×240，OpenCV INTER_AREA从640×480缩小并一次BGR→RGB；来源证据为派生视图metadata。
- 容器已验证 `jaka-lerobot-dev:snapshot-before-raw-mount` image ID `sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e`，LeRobot0.6.2；仓库锁定official commit `f66e5128ecb2456e8c54a63d15404fa59c16aebc`。历史训练时项目Git revision未在所读checkpoint元数据固定，不能从当前HEAD倒推；本轮基线HEAD为 `ffbf6fb33a0752a92a352a96291e23a0a4917a81`。

## DP可行性与证据边界

在已有ACT容器中、network none条件下成功导入官方 `lerobot.policies.diffusion.modeling_diffusion.DiffusionPolicy` 与 `DiffusionConfig`。该lazy import最初未暴露缺失的可选diffusers；实际构造失败后，仅在本轮隔离目录加入diffusers0.39.0，未升级ACT/OpenPI环境。后续真实训练与验证结果见 DP_REPORT.md。既有资产证明ACT/π训练和历史离线评估，不自动证明本轮闭环或真机成功。已读文档中“12-D arm plus six”表述歧义，以metadata的总维度12为准。

## 历史ACT真机证据（只读核验）

`historical_physical_evidence.json` 从两个实际rollout_summary/native_metrics抽取非敏感计数及报告路径：nominal52 `canonical001` 有5条policy command、6条native accepted target（含startup alignment），因freshness gate中止；早期 `20260812_act` 有901条accepted target但UNLABELED，且历史报告指出JAKA状态新鲜度与清理问题。两个checkpoint身份不同。可写真实部署与有界调试经历；不能写完整任务成功率。native `normal_completion` 仅说明清理完成，不是抓放成功。本轮没有新增真机操作。

本轮新增DP已完成正式10000次optimizer update、80000样本访问，使用同37/15 split、seed20261014与固定最终checkpoint规则。实际训练加验证3039.63s，checkpoint身份/normalization指纹见 `asset_identity.json` 的dp_checkpoint_identity，详细过程及失败尝试见 `DP_REPORT.md`。
