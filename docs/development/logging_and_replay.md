# Logging, evidence, and replay

Runtime logs and captures are generated output and ignored by default. Commit
only a small, purpose-specific regression fixture or unique physical evidence
with clear provenance.

For physical evidence record:

- schema/version and monotonic/wall-clock domain;
- branch/commit when available, otherwise a source-bundle identity, plus
  config and executable hashes;
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

中文摘要：生成日志默认不入库；真机证据必须保留原始文件、时钟域、配置/程序标识、
授权边界和停止原因。不得编辑原始证据来匹配后续解释，离线回放也不能替代真机验证。
