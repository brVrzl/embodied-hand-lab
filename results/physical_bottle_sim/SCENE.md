# 物理抓瓶场景与环境验收

本场景复用 `assets/jaka_rh56_visual_coacd.xml` 的 JAKA Mini2、RH56DFX 安装变换、惯量、网格碰撞与六条手指 equality；未替换机器人。机器人底座坐标原点在桌面，新增有质量的自由瓶体和固定刚性纸盒。纸盒不模拟变形，瓶体为圆柱近似，不模拟液体。此为有依据的近似任务重建，不能称高保真数字孪生。

配置：`assets/bottle_sim/scene.json`。场景生成器：`src/embodied_bottle_sim/scene.py`。校准只查看训练集 master episode 0（原始 episode 67）的起始 RGB 和机器人 FK。瓶半径 27 mm、高 230 mm、圆柱质量 350 g（另加 5 g 瓶盖，总质量 355 g）、盒半尺寸 75×90×35 mm 均为估计；工作台与瓶盒相对位置由视频和抓取/释放姿态 FK 估计，没有找到对应设备的数值相机外参。workspace 与 wrist 相机都是估计针孔模型，不能描述为完成真实标定。workspace 当前 640×480 RGB、50° 竖直视角，wrist 65°；预处理由模型 adapter 负责。腕部相机固定在手掌上，随物理手掌运动。

原模型手部 CoACD 碰撞摩擦 `1.8 .08 .004`、接触参数、碰撞掩码完全复用；瓶体 `0.8 .005 .0001`，没有为成功增加摩擦。既有碰撞模型自身不包含所有真实软组织和腱传动接触细节。本次没有添加吸附、瓶体焊接、隐藏约束或逐帧 qpos 搬运。只在 reset 初始化 qpos；之后通过 ctrl 与 mj_step 执行。

动作：前六维为绝对弧度关节位置，后六维为 index/middle/ring/pinky/thumb_close/thumb_lateral 的归一化闭合量 [0,1]，不是原始 0–1000 通道值。模型物理状态使用真实模拟关节位置，手部转换回闭合量。canonical 顺序复用 `rh56_sim/model.py`。闭合量映射至现有驱动关节角范围；四指线性关系是近似。拇指驱动线性范围与 `assets/rh56_thumb_table_calibration.json` 相符，PIP/DIP 三次耦合由原模型 equality 保留。每个动作都按关节限位和闭合量 [0,1] 投影并记录 raw/executed/projected。非有限动作显式失败。

原 XML 手臂 kp=40 用于既有预览；本任务有限力矩位置伺服采用 kp=500、kv=30、±60 Nm，属于仿真控制器估计而非真实 JAKA 已识别控制器。重力保留。物理 dt=1/600 s，每个 30 Hz 动作执行 20 子步，允许等待模型推理。

`environment_checks.json` 保存单关节/手指正向增量及静态支撑检查。12 个正向通道增量都引起同方向测量变化；手指动作通过 ctrl 执行，未写入从动关节。静态瓶中心高度 0.114941 m，对照初始 0.115 m。手臂重力跟踪存在约 0.019 rad 静态误差，未以瞬移消除。

`environment_acceptance.json` 使用与策略 runner 相同的 `Stages` 顺序指标，另外要求最后 2 s 连续稳定；保留两次开发尝试：

- 原始示教目标回放 795 步 / 26.50 s，瓶中心最大抬高 0.108 m，发生搬运及短暂盒面接触，但释放/退手时碰倒并掉落，完整任务失败 0/1。
- 独立 expert 使用同一 episode 的前 551 个目标，随后保持抓取 3 s，在冻结手臂姿态下用 6 s 渐开手、保持 3 s，再用 6 s 退回示教末态并保持 3 s。1181 步 / 39.37 s；最终瓶中心 [0.18061,0.57202,0.18494] m，盒面接触、手部零接触，最后连续 2 s 线速度 <0.02 m/s、角速度 <0.15 rad/s，完整环境任务成功 1/1。最大手臂跟踪误差约 0.0593 rad。

该脚本属于 environment/expert 验收，只说明场景物理上可以完成任务，绝不计入任何学习策略成功率。expert 的保持与慢释放规则也不得注入 policy runner。

复现：

```bash
PYTHONPATH=src /home/thor/projects/embodied_lab/.venv/bin/python tools/bottle_sim_acceptance.py \
  --dataset /home/thor/projects/embodied_lab/data/training/physical_bottle_v4_nominal52/act/master \
  --output results/physical_bottle_sim/environment_acceptance.json \
  --video /home/thor/projects/embodied_lab/outputs/physical_bottle_sim/acceptance_labeled
```

私有本地证据目录：`/home/thor/projects/embodied_lab/outputs/physical_bottle_sim/`，包括 `environment_replay_ep0.mp4/.json`、`environment_expert_slowrelease.mp4/.json`、`real_vs_sim_workspace.jpg`、`real_vs_sim_wrist.jpg`。真实 RGB 对照含内部原始素材，不提交发布。第一版腕部相机方向错误的图片保留为 `*_attempt0_bad_extrinsics.jpg`；修复只依据开发图像，不依据策略结果。首次失败视频与成功视频均保留。

`initial_instances.json` 固定 20 个来自训练划分的不同起始机器人状态；不是同一个确定性状态的 20 次重复。扰动版在相同状态上使用 ±5 mm XY、±5° yaw；前者小于训练 FK 抓取代理位置范围（`train_pose_coverage.json`），但真实瓶位分布未测量，不能称经测量验证的分布范围。圆柱绕竖轴旋转近似对称，因此实例区别主要来自实际起始关节姿态和 XY。所有方法必须使用同一清单。

盒面支撑判据同时要求真实接触、瓶中心 XY 在盒面内、按圆柱姿态计算的最低点距盒顶 <12 mm，避免侧面碰撞被计为放置。带环境标签的新视频保存在 `acceptance_labeled/`；旧尝试视频不覆盖。

## 初始化技术修复（清单 v2）

首版采用不同训练起始姿态配同一固定纸盒，后续几何检查发现索引 1、6、9、13 的手指/手掌与纸盒初始穿透约 10.18、10.45、25.28、5.94 mm。该问题在已有 ACT 尝试启动后发现；旧清单保留为 `initial_instances_v1.json`，旧 rollout 均保留并另列，不混入修复后正式口径。不能将这些初始化问题全部归因模型。

修复只替换这四个起始姿态：按照原训练清单顺序，选择接下来的四条零接触起始轨迹 master episode 29、30、31、32。未使用任何策略成功结果选择，场景、相机、控制器、目标瓶盒位置与原种子/扰动均未改变。`initialization_audit.json` 保留旧碰撞对/深度和 v2 全部 40 个实例检查：全部状态有限、位于原生限位内、reset 时零碰撞（包括手瓶碰撞）。各模型统一使用 `initial_instances.json` v2 重新评测。
