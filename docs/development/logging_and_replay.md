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
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

Select inputs explicitly. Do not commit large personal motion captures or
generated logs solely because they exist locally.

The simulation `live-6dof` bundle consists of the raw HTS+CTRL JSONL, the
60 Hz shared event JSONL, and the JSON report. The report embeds the small
`quest_jaka_joint_recording.v1` manifest with commit/config hash, duration,
joint/channel order, rates, file paths, and `simulation_only=true`. In
`jaka-equivalent-125hz` mode an additional
`*.arm_emitted_125hz.jsonl` records source accepted sequence/time, emitted
sequence/time, q/dq/ddq/jerk, PWL segment state, transition state, actual q,
and command-actual error. The raw input is output-mode neutral.

Operational commands for creating the bundle and replaying the same capture
through either arm adapter are in the
[Quest/JAKA MuJoCo simulation guide](../operation/simulation_demo.md).

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
.venv/bin/python tools/replay_quest_jaka_output_feasibility_native.py --help
```

必须显式选择输入。不要仅因本地存在大型个人动作采集或生成日志就将其提交。

仿真 `live-6dof` bundle 由 raw HTS+CTRL JSONL、60 Hz shared event JSONL 和 JSON report
组成。report 内嵌小型 `quest_jaka_joint_recording.v1` manifest，记录 commit/config hash、
时长、关节/通道顺序、频率、文件路径和 `simulation_only=true`。选择
`jaka-equivalent-125hz` 时还会生成 `*.arm_emitted_125hz.jsonl`，其中包含 source accepted
序列/时间、emitted 序列/时间、q/dq/ddq/jerk、PWL segment/transition 状态、actual q 和
command-actual error。raw 输入不绑定 output mode。

生成该 bundle，并把同一份 capture 通过两种 arm adapter 回放的操作命令见
[Quest/JAKA MuJoCo 仿真指南](../operation/simulation_demo.md)。
