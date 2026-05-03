# Archived Direction Notes

Date: 2026-05-04

These files are retained as historical references only.

They are no longer the active research direction because the project has moved from a failure-aware envelope-grasp collection paper toward:

**Palm-Frame Hand-Code Transfer for Data-Efficient Dexterous Grasping on JAKA mini2 + Inspire RH56**.

Use the active plan instead:

```text
docs/active_research_and_control_plan.md
```

Archived files:

- `research_report.md`: old recommendation for failure-aware demonstration collection.
- `project_direction_recommendation.md`: old final decision centered on envelope grasping.
- `week1_todo.md`: old week-one foam-cube collection checklist.
- `failure_debug_checklist.md`: old failure-debug checklist, replaced by `docs/hardware_bringup_checklist.md`.

Useful ideas that remain valid:

- success/failure/manual review metadata.
- replay validation.
- clean/weak/failure data curation.
- conservative real-robot bring-up.

Invalidated assumptions:

- first paper novelty is a data-collection pipeline.
- RH56 should primarily be treated as a simple envelope gripper.
- JAKA tool RS485 should be the main RH56 experimental data path.
- low-frequency segmented control is enough for the main research direction.

# 中文版本

这些文件仅作为历史参考保留。

它们不再代表当前研究方向，因为项目已经从 failure-aware envelope-grasp collection paper 转向：

**Palm-Frame Hand-Code Transfer for Data-Efficient Dexterous Grasping on JAKA mini2 + Inspire RH56**。

当前应使用的主计划是：

```text
docs/active_research_and_control_plan.md
```

归档文件包括：

- `research_report.md`：旧的 failure-aware demonstration collection 推荐。
- `project_direction_recommendation.md`：旧的 envelope grasping final decision。
- `week1_todo.md`：旧的第一周 foam-cube collection checklist。
- `failure_debug_checklist.md`：旧 failure-debug checklist，已由 `docs/hardware_bringup_checklist.md` 替代。

仍然有效的内容：

- success/failure/manual review metadata。
- replay validation。
- clean/weak/failure 数据筛选。
- 保守真实机器人 bring-up。

已经失效的假设：

- 第一篇论文创新点是数据采集 pipeline。
- RH56 主要应被当成简单 envelope gripper。
- JAKA 航插 RS485 是 RH56 主要实验数据链路。
- 低频分段控制足以支撑当前主研究方向。
