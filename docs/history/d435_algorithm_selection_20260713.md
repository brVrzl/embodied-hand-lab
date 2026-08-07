# D435 Depth Algorithm Selection

> **Status: historical measurement snapshot dated 2026-07-13.** It records the
> compared algorithms and device evidence from that session. For the current
> implementation boundary, see
> [`DATA_COLLECTION.md`](../data/DATA_COLLECTION.md).

日期：2026-07-13

## 当前决策

桌面操作按以下层次部署：

1. 动态控制默认使用 D435 ASIC 深度和 `disparity -> spatial -> depth`。
2. 静态标定/扫描可增加 temporal filter；机械臂或物体运动时关闭。
3. 点云使用基座工作区裁剪、5 mm voxel、统计离群剔除、半径离群剔除和桌面 RANSAC。
4. RGB guided completion 本轮拒绝用于几何，只保留为实验支路。
5. FoundationStereo 保留为重点二阶段对照；LingBot-Depth 暂缓下载。

## 同帧实测

设备：D435 `346522072675`，848x480@30，Default preset，FW `5.15.1.55`，SDK
`2.58.1.10581`。45 帧短序列中视野有人体和桌面杂物，因此时序结果是动态场景判断，
不是清理后最终验收值。

| 方法 | 0.3-1.5 m 有效率 | 时序标准差中位数 | P95 | 结论 |
|---|---:|---:|---:|---|
| Raw | 84.0% | 2.76 mm | 14.07 mm | 对照 |
| Disparity + Spatial | 84.0% | 2.56 mm | 13.96 mm | 动态默认 |
| + Temporal | 85.0% | 1.46 mm | 14.73 mm | 仅静态 |
| + RGB guided fill | 90.3% | 2.70 mm | 20.36 mm | 几何拒绝 |

Guided fill 新增约 26k 有效像素，但可视化显示它会把部分无观测/遮挡区扩成平滑深度块，
不能把填充率当作准确率。点云方面，Spatial 体素云为 54,955 点，统计加半径剔除后为
53,239 点；该支路只删稀疏飞点，不补造表面。

浏览器报告由 `tools/compare_d435_depth_algorithms.py` 生成，包含固定量程深度图和可旋转点云。

## 开源候选评估

### FoundationStereo / ESS

[Isaac ROS DNN Stereo Depth](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_stereo_depth)
明确提供 FoundationStereo、Fast-FoundationStereo、ESS，并给出 RealSense 双红外入口。
FoundationStereo 相比单目算法更适合 D435，因为左右图、焦距和基线可以产生度量视差。

官方 [FoundationStereo 文档](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_foundationstereo/index.html)
给出的约束是：

- 标准模型固定为 576x960 或 320x736；高分辨率需要超过 16 GB GPU 内存。
- RealSense 示例采集 640x360 双红外、关闭 emitter、限制为 15 FPS；否则推理可能积压。
- 输入必须完成双目校正，尺寸相同且可被 32 整除。
- 运行依赖 Isaac ROS Jazzy、TensorRT engine 和接受 EULA 后下载的模型。
- Fast-FoundationStereo 是研究许可、不可商用；标准 FoundationStereo 才是产品候选。

当前 Thor 有足够统一内存，但没有可用的 Isaac ROS 包源，Docker daemon 对当前用户无权限，
也没有安装 TensorRT Python/ROS 运行栈。原生 PyTorch 路径仍会遇到与 LingBot 相同的大运行时下载。
因此 FoundationStereo 技术上可用，但不是“小下载替代”；清理桌面并完成轻量基线后，再单独建立
Isaac ROS 环境，优先试 320x736 标准模型并与 D435 ASIC 同场录制对照。

ESS 的官方 AGX Thor T5000 576p benchmark 为 157 FPS，但这是 ESS 图的结果，不能外推为
FoundationStereo 或本机实际吞吐。若需要 30 FPS 控制回路，ESS 比 FoundationStereo 更值得先做
工程验证，FoundationStereo更适合质量上限对照。

### LingBot-Depth

[LingBot-Depth](https://github.com/Robbyant/lingbot-depth) 的 RGB + noisy/sparse metric depth 输入
与任务高度匹配，但 v0.5 模型文件约 1.28 GB；当前 Thor 可用 CUDA PyTorch wheel 另约 338 MB，
还需 torchvision/xformers。下载已按用户要求停止并删除残缺文件。它仍是离线精修候选，不进入
首轮部署。

### Depth Anything V2 Small

[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2) Small 为 24.8M 参数且
Apache-2.0，但基础输出是相对深度，仍依赖 PyTorch。它只能作为 D435 无效区的边缘先验，不能
直接替换度量深度；本轮不下载。

### 传统深度与点云

RealSense 官方 [post-processing filters](https://github.com/realsenseai/librealsense/blob/master/doc/post-processing-filters.md)
是当前低延迟主链。点云离群剔除采用与 PCL
[Statistical Outlier Removal](https://pointclouds.org/documentation/tutorials/statistical_outlier.html)
相同的邻域距离判据，并增加固定半径最少邻居判据；实现使用现有 SciPy `cKDTree`，没有新增模型。

多帧 TSDF/占据建图以后再接
[Isaac ROS nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox)；它不负责单帧深度修复，
不放入当前控制回路。

## SDK 与固件

最新 [librealsense v2.58.2](https://github.com/realsenseai/librealsense/releases/tag/v2.58.2)
仍标为 beta，支持 Ubuntu 24.04 和 JetPack 7.0，并修复 Python wrapper 潜在死锁；其 D435 USB
兼容表建议 FW `5.17.3.10` 或更新。官方
[D400 firmware page](https://dev.realsenseai.com/docs/firmware-releases-d400/) 将 `5.17.3.10`
列为 2026-06 稳定版本并与 SDK 2.58.1 配套；当前 `5.15.1.55` 也是官方量产版本。

处理顺序：

1. 当前算法对比保持 SDK `2.58.1.10581` + FW `5.15.1.55`，不改变基线。
2. 已保存 `device_settings_before_firmware_change.json`；升级前另录空桌面/标定板基线。
3. 先在隔离环境验证 SDK 2.58.2 的枚举、848x480@30、时间戳和 10-30 分钟稳定性。
4. 再安排 FW 5.17.3.10，升级时只连接目标 D435，禁止相机/机器人任务同时运行。
5. 升级后重复深度质量、内参、同步、启停和 soak test；失败则使用官方旧固件包回退。

本轮不自动刷固件，原因不是新固件不可用，而是当前设备可采集，且一次只改变一个变量更容易
定位质量或稳定性回归。
