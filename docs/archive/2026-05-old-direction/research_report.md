# Research Report: JAKA mini2 + Inspire RH56 Manipulation Direction

## Executive Recommendation

当前最现实、最可发表、最容易跑通的方向是：

**面向低成本机械臂-灵巧手系统的 failure-aware demonstration collection pipeline，并把 RH56 先退化为 envelope gripper。**

第一篇小论文不要主打新模型，也不要做复杂 in-hand manipulation。主打内容应是：在 JAKA mini2 + RH56 上建立一个可复现的真实机器人数据采集 benchmark，包括任务分阶段、抓取 primitive、失败模式标注、replay validation、数据筛选、BC baseline。这个方向能直接解释你现在的痛点：为什么采不到稳定 success 数据，以及怎样把 success 数据工程化地产出。

## 1. 近两年论文到底在解决什么问题

### 真实机器人 manipulation 数据为什么难采

1. **成功率低不是偶然，而是系统性问题。** UMI、DexUMI、RwoR、ACE 都不是先换模型，而是先重新设计 demonstration interface。原因是普通 teleop 采到的数据经常不可执行、不可复现或和部署接口不一致。
2. **接触 dynamics 对小误差极敏感。** 抓取点偏 5-10 mm、手掌姿态偏几度、闭合时序慢 0.5 s，都可能从稳定抓取变成推倒、滑落或夹空。
3. **灵巧手动作空间过大。** RH56 是 6 路控制，真实手指还有耦合、延迟、摩擦和死区。直接学连续 6 指 action 会放大数据噪声。
4. **人手到机器手存在 embodiment gap。** DexUMI、DEXOP、TypeTele、SPIDER 都在处理同一个问题：人手动作不能直接变成机器人手可执行动作。
5. **teleoperation 延迟和不直观。** 你的配置里 RH56 控制 5 Hz，且 JAKA tool RS485 有 `command_pause_sec: 0.8`。这对实时高频 finger teleop 不友好，更适合低频 grasp primitive。
6. **缺少触觉/力反馈时，复杂接触任务会很脆。** ADAPT-Teleop 的启发是靠硬件 compliance 弥补力反馈不足。对 RH56，应先用软物体、包络抓取、低速接近来制造被动稳定性。
7. **任务定义经常过难。** “pick / grasp / lift”如果没有固定物体初始姿态、预抓取位姿、lift 高度、保持时间，就无法形成可训练数据。
8. **success label 和 failure mode 不清晰。** FAIL-Detect 和 interactive imitation learning survey 都说明：失败检测、失败类型、弱成功和可用数据筛选是部署可靠性的核心。
9. **数据质量比模型结构更关键。** ICLR 2025 Scaling Laws 明确指出环境/物体覆盖和数据组成比单一场景无限增加 demo 更重要；对你现在更关键的是先保证每条 success 可 replay。

### 近两年被接收的工作主要靠什么创新

- **更好的数据采集 interface：** UMI 用手持 gripper，DexUMI 用人手 + 外骨骼，ACE 用低成本 visual-exoskeleton，ARCap 用 AR feedback。
- **更好的 teleop / retargeting：** TypeTele 用 manipulation type 避免不合理 retargeting；ByteDexter 用优化式人手 retargeting；SPIDER 用 physics-informed sampling 让轨迹可执行。
- **把 human demonstration 转成 robot demonstration：** RwoR 直接把人手视频转为 UMI gripper 风格 demonstration。
- **sim-real 混合训练：** Sim-and-Real Co-Training 说明少量真实数据可以和大量模拟数据混训，但 action space 和 camera/action schema 必须先统一。
- **数据筛选、质量标注、human correction：** Interactive imitation learning 的趋势是让人纠正失败，而不是把失败轨迹和成功轨迹混在一起训练。
- **小任务做深：** 可发表的小系统往往把采集、标注、replay、评测做完整，而不是一开始追求通用 VLA。

## 2. 不适合你现在做的方向

- 大规模通用 VLA 或 dexterous foundation model。
- 需要几百小时真实 teleop 数据的项目。
- 需要高级触觉阵列、力控、昂贵 glove 或定制外骨骼的项目。
- 复杂 in-hand manipulation，例如手内旋转、翻转、打字式精细操作。
- long-horizon multi-object manipulation。
- 没有可靠相机标定时，直接做视觉闭环 Diffusion Policy。
- 直接训练 RH56 6 指连续控制策略，特别是在 hand feedback 低频/延迟未量化前。

## 3. 适合你现在做的方向

- **简化任务 + 高质量数据采集。** 固定物体、固定起点、固定 wrist orientation、明确 pregrasp pose。
- **真实设备 teleop demonstration pipeline。** 先用低维 action：末端 delta pose + grasp type / close strength。
- **失败模式分析与数据筛选。** 每条轨迹都要有 `success`、`failure_mode`、`manual_success`、`use_for_bc`。
- **low-cost dexterous hand data collection。** 不做外骨骼也可以贡献一个低成本 arm-hand 采集 protocol。
- **sim-to-real / real-to-sim 对齐。** 用 ManiSkill/SAPIEN 定义任务、动作 schema、success label，再映射到真机。
- **把 RH56 作为多指包络式夹爪。** 先让它稳定完成 grasp-lift-place，再谈 dexterity。
- **JAKA mini2 + RH56 小型 manipulation benchmark。** 这对中文会议、机器人应用期刊、ICRA/CoRL/IROS workshop 更现实。

## 4. 重新定义真实机器人任务

| 任务 | 难度 | 相机 | 可先无视觉 | 对象 | 起始/目标 | success | failure mode | 为什么适合 | 20/50/100 demos | 论文价值 |
|---|---|---|---|---|---|---|---|---|---|---|
| Fixed Foam Cube Grasp-Lift | 低 | 不必须 | 是 | 50-60 mm 泡沫块/海绵块，<30 g，高摩擦 | 物体放在治具中心；目标为 lift 5-8 cm 并保持 2 s | 离桌 >5 cm，保持 2 s，无明显滑落 | 夹空、推倒、提前碰撞、lift 中滑落 | 软、轻、容错高，适合 RH56 包络抓取 | 20 可训练状态 BC smoke test；50 可做 baseline；100 可评估泛化到小扰动 | 很适合作为第一任务 |
| Light Cylinder Power Grasp + Lift | 低-中 | 不必须 | 是 | 直径 45-60 mm EVA 圆柱/轻质瓶，<80 g | 圆柱竖直放入圆形定位圈；目标为垂直抬升 8 cm | object bottom > table 8 cm，保持 3 s，姿态倾角 <25 deg | 手指未包住、圆柱旋转滑出、碰倒 | power grasp 和 RH56 多指包络天然匹配 | 20 可做 primitive 调参；50 可训练 BC；100 可做 object size ablation | 适合作第二任务 |
| Bottle/Cup Pick-and-Place to Fixed Tray | 中 | 建议有顶视/侧视视频，但训练可先无视觉 | 是，若起点固定 | 轻质塑料瓶/纸杯，直径 55-70 mm，<100 g | 起点定位圈；目标托盘 15x15 cm | 物体进入托盘且稳定 2 s，无人工接管 | 抓取偏心、放置时撞托盘、松手时带倒 | 接近真实 pick-place，但仍可控 | 20 不建议训练；50 可训练固定起点 BC；100 可做轻量 Diffusion/BC 对比 | 适合第一篇主实验 |
| Push Object to Zone | 低 | 可选 | 是 | 橡胶块/泡沫块/小盒子，50-80 mm | 起点固定；目标区 12x12 cm | 物体中心进入目标区 | 推偏、翻倒、手掌碰撞 | 不要求稳定 grasp，可作为 fallback | 20 可训练；50 稳定；100 可做泛化 | 适合降级任务和失败对照 |
| Envelope-Gripper Pick-and-Place | 中 | 可选 | 是，固定起点 | 泡沫块、圆柱、小杯三类 | 固定 pregrasp + staged close + lift + place | 完整 pick-place，保持/释放正确 | close 太早推物体、lift 滑落、place 碰撞 | 把 RH56 限制为 2-3 个 grasp primitives，降低动作空间 | 50 可开始 BC；100 可写 ablation | 最适合论文主线 |
| Failure-Correction-Success Collection | 中 | 视频强烈建议 | 可以状态训练，视频复核 | 上述对象 | 每次失败后人工纠正继续到成功 | 标注失败点、修正段、最终成功 | 无法定位失败、修正过大 | 直接对应 failure-aware IIL/data curation | 50 条三段轨迹可做分析；100 可训练/筛选对比 | 最有研究味道 |

## 5. 最现实的 3 个项目方向

### 方向 A：高成功率 demonstration collection pipeline

**题目草案：** A Failure-Aware Demonstration Collection Pipeline for Low-Cost Arm-Dexterous-Hand Manipulation

核心问题：低成本机械臂 + 灵巧手系统上，如何从不稳定抓取中系统地产出可 replay、可筛选、可训练的 success demonstrations。

创新点：

- staged task curriculum: open/close -> contact -> lift 3 cm -> lift 8 cm -> place。
- demonstration schema: action/state/video/success/failure/use_for_bc。
- replay validation: 每条 success 至少 replay 1 次或抽样 replay。
- failure-aware curation: `hard_failure`、`weak_success`、`clean_success` 分开。

MVP：泡沫块和轻质圆柱两个任务，各 50 条，训练状态 BC；比较 raw all data vs clean_success only vs clean+correction。

投稿：中文机器人会议/自动化应用期刊现实；ICRA/CoRL/IROS workshop 现实；RA-L 暂时不现实，除非实验规模和泛化显著扩展。

### 方向 B：RH56 退化为 envelope gripper 的真实抓取 benchmark

**题目草案：** When Dexterous Hands Should Behave Like Grippers: Reliable Envelope Grasping on a Low-Cost Arm-Hand System

核心问题：入门灵巧手未必适合一开始做 dexterity；通过 hand primitive、任务简化、数据筛选能否得到高成功率抓取和可训练数据。

创新点：

- RH56 grasp type library：`power_grasp`、`tripod`、`lateral`、`envelope_close`。
- 比较 continuous finger command vs primitive command。
- 比较自由 wrist vs 固定 wrist orientation。
- object ladder：foam cube -> foam cylinder -> plastic cup。

MVP：固定 foam cube + cylinder；30 trials per setting；目标 success rate >80% 后采 BC。

投稿：workshop/应用类会议很合适；论文论点清楚。

### 方向 C：Failure-aware imitation learning

**题目草案：** Failure-Aware Imitation Learning for Low-Cost Dexterous Grasping

核心问题：失败数据是扔掉、混入训练，还是用来做筛选/诊断/修正？

创新点：

- failure taxonomy for arm-hand grasp-lift-place。
- weak_success / near_failure 的影响分析。
- failure-correction-success 轨迹切分。
- 证明数据质量比数量更重要：50 clean demos 可能胜过 150 mixed demos。

MVP：采 100 条，其中 clean success、weak success、failure、correction 都标注；训练 3 个 BC baseline。

投稿：IROS/CoRL workshop 最合适；如果指标扎实，可冲中文核心/机器人应用期刊。

## 6. 最终推荐路线

1. **现在最应该做：方向 A + B 合并。** 题目聚焦 pipeline，方法上把 RH56 退化为 envelope gripper。
2. **第一篇小论文：** “Failure-Aware Demonstration Collection for Reliable Envelope Grasping with a Low-Cost Arm-Dexterous-Hand System”。
3. **第一周采集：** 只做固定 foam cube 的 open/close、close-only、lift 3 cm、lift 8 cm，不做 place。
4. **第一个对象：** 50-60 mm 高摩擦泡沫块，质量 <30 g。
5. **success 标准：** 物体离桌 >=5 cm，保持 2 s，期间无滑落、无人工接管、无碰撞报警。
6. **action space：** `delta_ee_pose` 低频控制 + `grasp_type` + `close_strength`。第一版不要输出 6 指连续 action。
7. **是否需要相机：** 第一周训练不需要相机；必须保存侧视/顶视视频用于人工复核。第二阶段再接 RGB-D 或 wrist camera。
8. **先 BC 还是 Diffusion Policy：** 先状态 BC。Diffusion Policy 等 100 条 clean success + 相机标定稳定后再上。
9. **多少条 demo 开始训练：** 50 条 clean success 可以开始训练固定任务 BC；20 条只用于 pipeline smoke test。
10. **50 条还没有 success 怎么降级：** 不训练。退到 close-only；lift 高度降到 3 cm；换更大更软物体；固定 wrist；固定 pregrasp；把 RH56 close speed/force/pose 重标定。

## Sources Used

- DexUMI project and arXiv: https://dex-umi.github.io/ , https://arxiv.org/abs/2505.21864
- DEXOP project and arXiv: https://dex-op.github.io/ , https://arxiv.org/abs/2509.04441
- TypeTele arXiv/OpenReview: https://arxiv.org/abs/2507.01857
- RwoR project and arXiv: https://rwor.github.io/ , https://arxiv.org/abs/2507.03930
- UMI RSS proceedings: https://www.roboticsproceedings.org/rss20/p045.html
- Sim-and-Real Co-Training: https://rpl.cs.utexas.edu/publications/2025/06/21/maddukuri-rss25-simreal/ , https://arxiv.org/abs/2503.24361
- Data Scaling Laws: https://openreview.net/forum?id=pISLZG7ktL
- ACE: https://ace-teleop.github.io/ , https://arxiv.org/abs/2408.11805
- FAIL-Detect: https://openreview.net/forum?id=A2iUXYdWZD
