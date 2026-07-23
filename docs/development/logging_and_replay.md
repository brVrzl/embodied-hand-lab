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
