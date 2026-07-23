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
