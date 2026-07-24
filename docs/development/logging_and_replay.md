# Logging, evidence, and replay

Runtime logs and captures are generated output and ignored by default. Commit
only a small, purpose-specific regression fixture or unique physical evidence
with clear provenance.

For physical evidence record:

- schema/version and monotonic/wall-clock domain;
- branch, commit, config and executable hashes;
- stage acknowledgements and motion/time bounds;
- operator/controller state and stop reason;
- the relationship between narrative report, metrics JSON, accepted targets,
  emitted targets, and timing CSV/JSONL;
- PASS, FAIL, partial, superseded, or unverified status.

Never edit raw JSON/CSV/JSONL to match a later explanation. Add a follow-up
report instead. Historical Quest/JAKA evidence lives under
`docs/history/incidents/quest_jaka_20260722_23/`; foundation gates live under
`docs/history/gates/jaka_foundation_20260716/`.

Offline analyzers include:

```bash
.venv/bin/python tools/analyze_jaka_edg_resampling.py --help
.venv/bin/python tools/analyze_quest_jaka_output_feasibility.py --help
.venv/bin/python tools/analyze_quest_jaka_output_acceleration.py --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

Select inputs explicitly. Do not commit large personal motion captures or
generated logs solely because they exist locally.

---

# 中文版：日志、证据与回放

运行日志和采集文件是生成物，默认被忽略。只有具备明确来源的小型专用回归样本或不可
替代的真机证据才应提交。

真机证据应记录：

- schema/版本以及单调时钟和墙上时钟域；
- 分支、提交、配置和可执行文件哈希；
- gate 确认项及运动/时间边界；
- 操作者/控制器状态和停止原因；
- 叙述报告、指标 JSON、接受目标、发射目标及计时 CSV/JSONL 之间的关系；
- PASS、FAIL、部分通过、已取代或未验证状态。

绝不能修改原始 JSON/CSV/JSONL 使其符合后来的解释；应另加跟进报告。Quest/JAKA 历史
证据位于 `docs/history/incidents/quest_jaka_20260722_23/`，基础 gate 位于
`docs/history/gates/jaka_foundation_20260716/`。

离线分析工具包括：

```bash
.venv/bin/python tools/analyze_jaka_edg_resampling.py --help
.venv/bin/python tools/analyze_quest_jaka_output_feasibility.py --help
.venv/bin/python tools/analyze_quest_jaka_output_acceleration.py --help
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

必须显式选择输入。不要仅因本地存在大型个人动作采集或生成日志就将其提交。
