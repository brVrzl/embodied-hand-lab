# 近期可信文献与方法边界

更新日期：2026-06-09

本页只保留能直接影响 `JAKA mini2 + Inspire RH56` 真机复现、数据采集和论文论证的工作。引用优先级：

1. 已发表期刊/会议论文。
2. 有明确作者、项目页、代码/数据或主流实验基准的 arXiv 论文。
3. 2026 新 preprint 只作为趋势观察，不直接作为结果可信度依据。

## 对当前项目的结论

- 真机第一阶段应优先复现 teleop 数据采集、固定网球 grasp-and-hold、低维 arm/hand action schema，而不是直接上 VLA。
- DP/ACT 可以作为小数据 imitation baseline；前提是先把 iPhone/HEBI、Xbox、RH56 状态、JAKA 关节/TCP 状态记录稳定。
- VLA/robot foundation model 可以作为相关工作和后续 fine-tuning 方向，但当前硬件和数据规模不足以把它作为第一版主线。
- 对称物体网球适合第一轮：姿态歧义少，主要检验接近、闭合、保持、释放和安全停机。
- RH56 没有高分辨率触觉时，不应声称完成 tactile dexterity；可以把电流/力/状态反馈称为 pseudo-tactile 或 proprioceptive feedback。

## 方法表

| 方向 | 代表工作 | 来源可信度 | 对本项目的用法 |
| --- | --- | --- | --- |
| Diffusion Policy | Chi et al., “Diffusion Policy: Visuomotor Policy Learning via Action Diffusion”, IJRR/arXiv | 高；IJRR 2025，arXiv 2023，代码/数据公开 | 作为固定物体少样本 visuomotor baseline；动作空间先用低维 palm delta + RH56 preset/counts。 |
| ACT | Zhao et al., “Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware”, RSS 2023/arXiv | 高；真实机器人、低成本 teleop、ACT 明确 | 作为 action chunking baseline；适合从遥操作轨迹学习短序列动作。 |
| DROID | Khazatsky et al., “DROID: A Large-Scale In-the-Wild Robot Manipulation Dataset”, RSS 2024/arXiv | 高；大规模真实数据、开源数据/代码 | 支撑“先建设稳定数据管线”的论证；不直接迁移到 RH56 标签。 |
| Octo | Octo Model Team, “Octo: An Open-Source Generalist Robot Policy”, RSS 2024/arXiv | 高；Open X-Embodiment，开源 generalist policy | 作为未来 fine-tuning 参考；当前数据量不足时不作为首选复现目标。 |
| OpenVLA | Kim et al., “OpenVLA: An Open-Source Vision-Language-Action Model”, arXiv 2024 | 中高；开源 VLA、明确实验 | 可写相关工作；真机第一阶段只做接口预留。 |
| pi0 | Black et al., “pi0: A Vision-Language-Action Flow Model for General Robot Control”, arXiv 2024 | 中高；强团队、多 embodiment 真实任务，但复现成本高 | 作为 VLA/flow matching 趋势，不作为当前硬件的直接 baseline。 |
| ADAPT-Teleop | Junge and Hughes, “ADAPT-Teleop”, npj Robotics 2025 | 高；期刊，teleop + anthropomorphic hand | 支撑保留 iPhone/HEBI/手部 teleop 的必要性：高质量遥操作本身就是数据资产。 |
| Visuomotor diffusion for multifinger hands | Koczy et al., “Learning Dexterous In-Hand Manipulation with Multifingered Hands via Visuomotor Diffusion”, IROS 2025/arXiv | 中高；IROS 2025/DOI，贴近多指手 | 作为 RH56 后续 in-hand manipulation 相关工作；当前网球 grasp-hold 先不要求 in-hand reorientation。 |
| ViTacFormer | Heng et al., “ViTacFormer”, arXiv/OpenReview 2025 | 中；新工作，触觉设定强 | 作为视觉-触觉趋势参考；本项目无同等级触觉，不能照搬结论。 |

## 真机复现优先级

1. `HEBI/iPhone ARKit -> relative palm target -> JAKA EDG servo`：先做 arm-only shadow，再小速度实机。
2. `iPhone camera / MediaPipe -> RH56 counts`：先跑 safety gate，再允许 ROS2 command。
3. `Xbox -> ROS2 intent -> RViz mirror -> real bridge`：作为稳定对照链路。
4. 固定网球数据采集：每条 episode 至少记录 RGB、JAKA joint/TCP、RH56 angle/force/current/status、teleop command、deadman 状态。
5. Policy baseline：先 ACT/DP 小模型；VLA 只做接口和文献定位。

## 参考来源

- Cheng Chi, Zhenjia Xu, Siyuan Feng, Eric Cousineau, Yilun Du, Benjamin Burchfiel, Russ Tedrake, Shuran Song. “Diffusion Policy: Visuomotor Policy Learning via Action Diffusion.” arXiv:2303.04137 / IJRR. https://arxiv.org/abs/2303.04137
- Tony Z. Zhao, Vikash Kumar, Sergey Levine, Chelsea Finn. “Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware.” arXiv:2304.13705. https://arxiv.org/abs/2304.13705
- Alexander Khazatsky et al. “DROID: A Large-Scale In-the-Wild Robot Manipulation Dataset.” arXiv:2403.12945. https://arxiv.org/abs/2403.12945
- Octo Model Team et al. “Octo: An Open-Source Generalist Robot Policy.” arXiv:2405.12213. https://arxiv.org/abs/2405.12213
- Moo Jin Kim et al. “OpenVLA: An Open-Source Vision-Language-Action Model.” arXiv:2406.09246. https://arxiv.org/abs/2406.09246
- Kevin Black et al. “pi0: A Vision-Language-Action Flow Model for General Robot Control.” arXiv:2410.24164. https://arxiv.org/abs/2410.24164
- Kai Junge, Josie Hughes. “ADAPT-Teleop: robotic hand with human matched embodiment enables dexterous teleoperated manipulation.” npj Robotics 3, 31 (2025). https://doi.org/10.1038/s44182-025-00034-3
- Piotr Koczy, Michael C. Welle, Danica Kragic. “Learning Dexterous In-Hand Manipulation with Multifingered Hands via Visuomotor Diffusion.” arXiv:2503.02587 / IROS 2025. https://arxiv.org/abs/2503.02587
- Liang Heng, Haoran Geng, Kaifeng Zhang, Pieter Abbeel, Jitendra Malik. “ViTacFormer: Learning Cross-Modal Representation for Visuo-Tactile Dexterous Manipulation.” arXiv:2506.15953. https://arxiv.org/abs/2506.15953
