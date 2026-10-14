# Diffusion Policy 实验

本轮使用已有 LeRobot 0.6.2 容器的官方 DiffusionPolicy。最初模块import成功，但第一次GPU smoke实际构造模型时发现可选diffusers缺失，0次优化。已按仓库要求 `>=0.38,<0.40` 仅安装diffusers0.39.0 wheel到本轮独立 `outputs/diffusion_bottle/deps`，用PYTHONPATH挂载，不改ACT/OpenPI环境或其他包。此前 `docker --gpus all` 在Thor runtime hook被拒绝、进程未启动；已改为 `--runtime nvidia`。这些技术尝试保留，不当成策略失败或训练完成。

CPU准备已通过官方LeRobotDataset加载37条训练轨迹/23802帧与锁定15条验证轨迹；source_episode无跨split。两路RGB历史2帧，state12D，动作16帧offset[-1..14]；官方采样输出slice[1:9]恰为当前起8动作。端点padding附action_is_pad并用于loss mask；reset清空历史。使用既有明确train-only统计，state/action为MIN_MAX、RGB ImageNet MEAN_STD。

预声明配置 `configs/training/diffusion/physical_bottle.json`：seed20261014，batch8，候选预算10000 optimizer steps，最终预算结束checkpoint，无仿真挑选；正常训练前由40步固定小批量pilot测资源决定是否可行。网络DDPM100去噪步，horizon16，UNet宽度256/512/1024，ResNet18 pretrained，groupnorm。完整状态随后更新。
