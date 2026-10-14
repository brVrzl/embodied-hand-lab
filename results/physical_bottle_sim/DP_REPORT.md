# Diffusion Policy 实验

## 冻结训练契约

使用已有 LeRobot 0.6.2 容器的官方 DiffusionPolicy，不升级 ACT/OpenPI。CPU准备通过官方LeRobotDataset加载锁定37条训练轨迹/23802帧；15条/9309帧仅验证，source_episode无跨split。两路RGB历史2帧，state12D，动作16帧offset[-1..14]；官方采样输出slice[1:9]为当前起8动作。历史必须是相邻30Hz时刻t-1/t，不能把相隔8步的查询历史冒充它。端点padding附action_is_pad并用于loss mask；reset清空历史。使用既有明确train-only统计，state/action为MIN_MAX、RGB ImageNet MEAN_STD。

配置 `configs/training/diffusion/physical_bottle.json` 于正常训练前冻结：seed20261014，batch8，10000 optimizer steps（80000样本访问，约3.36次训练集大小，不宣称算力公平），最终预算checkpoint，不用仿真筛选。DDPM100训练/推理去噪步，horizon16，UNet宽度256/512/1024，ResNet18 ImageNet预训练并保留BatchNorm，89,407,180参数。AdamW LR1e-4、warmup500、cosine，gradient clip10。无宽泛参数搜索。最终120窗口验证loss仅诊断、不选择checkpoint。

## 全部已启动技术尝试

|尝试|实际结果|保留证据|
|---|---|---|
|Docker --gpus all|Thor runtime hook拒绝，进程未启动；改--runtime nvidia|本报告与本轮工具记录|
|prepare_attempt1|CPU数据/shape/mask加载通过|outputs/diffusion_bottle/prepare_attempt1/report.json|
|smoke_attempt1|0步；lazy import未暴露缺失diffusers，构造模型报错|outputs/diffusion_bottle/smoke_attempt1/report.json|
|smoke_attempt2|0步；官方拒绝pretrained backbone替换GroupNorm，改回官方默认False|outputs/diffusion_bottle/smoke_attempt2/report.json及.log|
|smoke_attempt3|40步loss1.147727→0.0032736；保存成功，100步DDPM推理bitwise检查差0.0003288而失败|outputs/diffusion_bottle/smoke_attempt3/report.json、metrics.jsonl及.log|
|smoke_attempt4|40步前后向、单批拟合、保存恢复全部通过；权重逐tensor精确相等，推理允许atol1e-3/rtol1e-4，实测最大差0.0003597|outputs/diffusion_bottle/smoke_attempt4/report.json、metrics.jsonl及.log|
|train_seed20261014|正常10000步训练、120窗口验证、checkpoint保存恢复完成，exit0|outputs/diffusion_bottle/train_seed20261014/及同名.log|

仅按现有LeRobot声明的 `diffusers>=0.38,<0.40` 安装 `diffusers0.39.0 --no-deps --target outputs/diffusion_bottle/deps`；无需改其他包，DDPM import通过。依赖与权重不入Git。保存恢复检查不要求100次随机数值积分bitwise相等，严格比较模型权重且报告推理误差；没有隐藏失败。

Pilot40步纯优化7.34s，总含保存恢复11.19s，峰值CUDA分配2.55GB，loss1.147727→0.0032747；输出(8,8,12)有限。此pilot只证明实现可训练，不能称正常训练完成。正常运行允许与ACT/π仿真共享GPU，因此该时段延迟和训练时间必须标为共享资源实测，不用于公平单模型速度排名。

## 复现

在独立worktree运行，下列source只读、容器无网络，不挂设备接口：

```bash
docker run --rm --name bottle-dp-train --network none --runtime nvidia --ipc host \
  -v /home/thor/projects/embodied_lab:/source:ro \
  -v /home/thor/projects/embodied-bottle-sim:/work \
  -e PYTHONPATH=/work/outputs/diffusion_bottle/deps -e HF_HUB_OFFLINE=1 \
  -e TORCH_HOME=/source/outputs/training/lerobot_container_home/.cache/torch \
  --entrypoint python jaka-lerobot-dev:snapshot-before-raw-mount \
  /work/training/diffusion/train_bottle.py \
  --config /work/configs/training/diffusion/physical_bottle.json \
  --output /work/outputs/diffusion_bottle/train_seed20261014 --mode train
```

输出目录必须不存在，防止覆盖历史。最终checkpoint为该目录下pretrained_model，已完成正常训练与保存恢复，可用于冻结协议正式评估。

## 已完成正常训练

`train_seed20261014` 从独立初始化开始，没有继承单批smoke权重。实际10000次optimizer update，80000次样本访问；优化耗时3012.30s，含最终验证和保存恢复3039.63s（50.66min），峰值CUDA分配2,548,920,832 bytes。运行与本轮ACT/π仿真部分时间共享GPU，不能当作独占吞吐基准。

最后一批训练noise-MSE 0.005362146；固定15条validation每条8窗口，共120窗口，seed20261017，mean noise-MSE 0.017679397，不用于checkpoint选择。这些loss不是任务成功率，也不能和ACT L1或π flow loss直接比较。权重保存恢复逐tensor严格相等，100步DDPM抽样推理最大差0.000308037，满足已声明atol1e-3/rtol1e-4。输出(8,8,12)有限。

轻量证据：`dp_evidence/train_seed20261014.json`、`dp_evidence/train_metrics.jsonl`。权重与normalization文件、源码与配置指纹：`asset_identity.json`中dp_checkpoint_identity。训练脚本归档于537dc1a（训练启动后归档，文件内容与运行脚本一致，不冒称启动时HEAD）。实际权重留在 `outputs/diffusion_bottle/train_seed20261014/pretrained_model`，不入Git。

闭环任务指标由统一runner的正式DP结果目录产生，汇总见 RESULTS.md；本训练报告不把训练/固定样本推理冒充任务成功。
