# Project Direction Recommendation

## Final Decision

推荐做：

**Failure-aware demonstration collection pipeline for low-cost arm-dexterous-hand envelope grasping.**

中文题目可以写成：

**面向低成本机械臂-灵巧手系统的失败感知示教采集与包络抓取基准**

这条路线把方向 A、B、C 合并：以 pipeline 为论文主线，以 RH56 envelope gripper 化为工程策略，以 failure label / data curation 作为研究贡献。

## Why This Is the Best Route

你的当前瓶颈不是模型，而是没有稳定 success data。直接训练 Diffusion Policy 或 VLA 会把硬件、任务、数据标注问题全部混在一起，最后无法判断失败来自控制、对象、相机、模型还是数据。

这条路线可发表的原因：

- 问题真实：低成本 arm-hand 系统采不到成功数据是普遍痛点。
- 方法闭环：任务分解、primitive 设计、failure taxonomy、replay validation、BC baseline 都能落地。
- 实验可控：不需要几百小时数据，不需要 glove/触觉阵列/高端力控。
- 论文故事清晰：不要一开始追 dexterity，而是证明“对低成本硬件，先把灵巧手当包络夹爪并做 failure-aware curation，能显著提高可训练数据质量”。

## Paper 1 Scope

### Title Draft

Failure-Aware Demonstration Collection for Reliable Envelope Grasping with a Low-Cost Arm-Dexterous-Hand System

### Core Question

在 JAKA mini2 + Inspire RH56 上，如何把不稳定的 pick/grasp/lift 采集，转化为稳定、可 replay、可训练的真实机器人 demonstration 数据？

### Contributions

1. 一个低成本 arm-hand demonstration collection pipeline。
2. RH56 grasp primitive library：`open`、`pre_shape`、`envelope_close`、`power_grasp`、`release`。
3. 分阶段真实设备采集 protocol：close-only -> lift 3 cm -> lift 8 cm -> place。
4. failure taxonomy 与 `use_for_bc` 数据筛选标准。
5. BC baseline：raw mixed data vs clean success data vs clean+correction data。

## MVP Experiment

对象：

- foam cube: 50-60 mm, <30 g。
- foam cylinder: diameter 45-60 mm, height 70-100 mm, <80 g。
- optional: plastic cup / light bottle。

任务：

- Task 1: fixed foam cube grasp-lift 8 cm。
- Task 2: cylinder power grasp-lift 8 cm。
- Task 3: fixed pick-and-place into tray。

数据规模：

- Week 1: 60-100 diagnostic trials, 不急着训练。
- MVP: 每任务 50 clean success + 30 labeled failures。
- 完整论文：2-3 tasks，每任务 100 trials，至少 300 real robot trials。

Baselines：

- scripted primitive without learning。
- BC with all trajectories。
- BC with only clean success。
- BC with clean success + correction segment。
- optional: Diffusion Policy only after camera and 100+ clean demos are stable。

Ablations：

- object: cube vs cylinder vs cup。
- grasp command: continuous 6 finger action vs primitive grasp type。
- wrist orientation: free vs fixed。
- data: raw vs curated vs weak_success included。
- lift height: 3 cm vs 8 cm。

Metrics：

- task success rate。
- grasp success rate before lift。
- lift success rate。
- slip count。
- time to grasp。
- max/mean final object height。
- replay success rate。
- clean_success ratio。
- failure_mode distribution。
- BC rollout success over 20 trials。

## Venue Reality

- 中文会议 / 机器人应用期刊：现实。
- ICRA / IROS / CoRL workshop：现实，尤其是 data collection、robot evaluation、dexterous manipulation workshop。
- RA-L：第一版不现实，除非你扩展到多对象、多任务、系统开源、强 ablation 和真实 rollouts。

## 3-Week Plan

Week 1:

- 完成 hand primitive 标定。
- 完成 foam cube close/lift protocol。
- 建立 metadata schema 和 manual review。
- 目标：lift 8 cm success rate >=70%。

Week 2:

- 采 foam cube + cylinder clean demos。
- 训练状态 BC。
- 做 raw vs curated 第一个对比。
- 目标：BC 在固定起点 20 rollouts 中 >=50% success。

Week 3:

- 加 pick-and-place into tray。
- 完成 failure taxonomy 分析图。
- 整理第一版 workshop paper skeleton。
- 目标：至少 2 个任务有清楚 baseline 表。

## 6-Week Plan

- 扩展到 3 类对象。
- 加 replay validation。
- 加 weak_success / near_failure ablation。
- 如相机标定稳定，尝试 RGB/state BC 或小型 Diffusion Policy。
- 写出完整中文论文或 workshop paper。

## 3-Month Plan

- 加 ManiSkill/SAPIEN digital cousin。
- 做 sim data + real clean data co-training 小实验。
- 开源 dataset schema、protocol、failure taxonomy。
- 冲 IROS/ICRA workshop 或中文核心/应用类期刊。

## Risk and Fallback

最大风险：真实 grasp-lift success rate 仍低于 60%。

降级顺序：

1. 换更大更软泡沫块。
2. lift 8 cm 降到 3 cm。
3. place 任务降级为 lift-hold。
4. finger continuous 降级为 single `envelope_close`。
5. wrist 自由姿态降级为固定姿态。
6. BC 降级为 scripted primitive + data collection paper。
