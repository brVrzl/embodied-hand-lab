# Log schemas

The repository uses several purpose-specific schemas rather than one universal
log. Authoritative definitions live with their writers and tests.

| Data | Authority | Notes |
|---|---|---|
| `AcceptedArmTarget` / heartbeat | `src/teleoperation/accepted_target.py` | immutable accepted six-joint target; monotonic timestamps |
| shared session events | `src/quest_jaka_sim/smooth_session.py` | clutch/control state, acceptance/rejection reasons |
| native worker metrics | `native/jaka_servo_worker/main.cpp` | timing, health, tracking, resampling, defensive boundary counts |
| accepted/emitted JSONL | hardware/replay tools and native worker | preserve sequence and timestamp domain |
| HTS/UMIP recordings | `src/motion_input` | observation schema, not robot command |
| RH56 commands | `src/rh56_driver` | canonical six-channel order and explicit units |

Committed historical examples are indexed under `docs/history/`. Treat them as
evidence for their recorded schema version, not as an API to infer by example.
When adding a field, update the producer, parser/analysis code, tests, and this
reference if the public meaning changes.

---

# 中文版：日志结构

仓库使用多种专用 schema，而不是一个通用日志。权威定义与对应 writer 和测试放在一起。

| 数据 | 权威位置 | 说明 |
|---|---|---|
| `AcceptedArmTarget` / heartbeat | `src/teleoperation/accepted_target.py` | 不可变六关节接受目标；使用单调时间戳 |
| 共享会话事件 | `src/quest_jaka_sim/smooth_session.py` | clutch/控制状态、接受/拒绝原因 |
| 原生 worker 指标 | `native/jaka_servo_worker/main.cpp` | 计时、健康、跟踪、重采样和防御边界计数 |
| accepted/emitted JSONL | 硬件/回放工具与原生 worker | 保留序列号和时间戳域 |
| HTS/UMIP 录制 | `src/motion_input` | 观测 schema，不是机器人命令 |
| RH56 命令 | `src/rh56_driver` | 规范六通道顺序和显式单位 |

已提交历史样例由 `docs/history/` 索引。它们只证明其记录版本下的 schema，不能仅凭样例
反推当前 API。新增字段时，如果公开含义发生变化，必须同时更新 producer、parser/分析
代码、测试和本参考页。
