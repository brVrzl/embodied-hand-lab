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
| RH56 PC-direct episode | `src/rh56_driver/pc_direct_control.py` | `rh56_pc_direct_episode.v1`; selected/requested action, measured raw feedback, transport state |
| combined JAKA/RH56 event | `tools/quest_jaka_hardware.py` | shared arm event plus current `rh56_telemetry`; one monotonic host timeline |
| simulation recording manifest | `tools/quest_jaka_mujoco_sim.py` | `quest_jaka_joint_recording.v1`, embedded in report |
| 125 Hz simulated arm emission | `src/quest_jaka_sim/output.py` | production PWL source/emitted state and MuJoCo tracking |
| single-episode training capture | `src/episode_dataset` | raw streams, causal 30 Hz canonical samples, lossless depth sidecar, explicit completed/aborted/invalid status |

Committed historical examples are indexed under `docs/history/`. Treat them as
evidence for their recorded schema version, not as an API to infer by example.
When adding a field, update the producer, parser/analysis code, tests, and this
reference if the public meaning changes.

Native cycle telemetry classifies safe bounded motion as
`transition_limited_output`. Only a transition with `destination_gap_rad`
greater than `1e-6` rad and `selected_progress_rad` no greater than `1e-9` rad
is `output_no_progress_hold` and advances the existing 2 s / 250-cycle hold
escalation. The worker metrics expose `transition_limited_progress_points` and
`true_output_hold_count`; older acceleration-hold counters remain aliases for
the true hold count so current readers do not reinterpret limited motion as a
hold.

Shared event records expose per-attempt seed FK, Jacobian, IK iteration/final
FK, workspace, collision, output-feasibility, remaining-check, and total wall
times. Physical hardware events additionally expose receiver/router, status
sync, session, adapter dispatch, JSON serialization/write, and complete outer
tick times. `CONTROL_COMPUTE_BUDGET_EXHAUSTED` denotes a discarded,
non-authoritative candidate followed by a normal `HOLD_REJECTED` heartbeat.

Native final jerk checks retain the configured nominal limit and fault level,
with only a documented numeric comparison envelope: absolute tolerance
`1e-6 rad/s3` plus relative tolerance `2.5e-7 * abs(limit)`. This envelope is
based on the observed `1.3e-5 rad/s3` finite-difference discrepancy at the
configured `62.831853 rad/s3` limit; it is not a motion parameter. Native cycle
telemetry records `raw_output_jerk_rad_s3`, the nominal limit, both tolerance
terms, and `output_joint_jerk_hard_boundary_with_tolerance_rad_s3`. Worker
metrics additionally retain `last_output_check_raw_jerk_rad_s3`, including the
last attempted check when a hard fault occurs.

The RH56 read-only summary records the requested by-id path, resolved tty, USB
VID/PID/serial, sample/rate/latency/repeat metrics, initial/final `ANGLE_ACT`,
raw ERROR/STATUS values, timeout/checksum/protocol counters, and
`register_write_count`. Valid read-only evidence requires that count to be zero.
Per-frame records distinguish requested, actually selected/written, and
measured values, and include current/load, transport/control state, timestamps,
read latency, status/error, fault, and episode validity.

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
| RH56 PC-direct episode | `src/rh56_driver/pc_direct_control.py` | `rh56_pc_direct_episode.v1`；selected/requested action、实测 raw feedback 和 transport state |
| JAKA/RH56 联合 event | `tools/quest_jaka_hardware.py` | shared arm event 加当前 `rh56_telemetry`；共用 host monotonic 时间线 |
| 仿真录制 manifest | `tools/quest_jaka_mujoco_sim.py` | report 内嵌 `quest_jaka_joint_recording.v1` |
| 125 Hz 仿真机械臂 emission | `src/quest_jaka_sim/output.py` | production PWL source/emitted 状态及 MuJoCo tracking |
| 单 episode 训练采集 | `src/episode_dataset` | raw streams、因果 30 Hz canonical、无损 depth sidecar，以及 completed/aborted/invalid 显式状态 |

已提交历史样例由 `docs/history/` 索引。它们只证明其记录版本下的 schema，不能仅凭样例
反推当前 API。新增字段时，如果公开含义发生变化，必须同时更新 producer、parser/分析
代码、测试和本参考页。

native cycle telemetry 将安全受限且持续运动的输出分类为
`transition_limited_output`。只有 `destination_gap_rad` 大于 `1e-6` rad，同时
`selected_progress_rad` 不大于 `1e-9` rad 的 transition 才属于
`output_no_progress_hold`，并推进原有 2 秒/250 周期升级策略。worker metrics 新增
`transition_limited_progress_points` 与 `true_output_hold_count`；旧 acceleration-hold
计数保留为 true hold 的别名，避免当前 reader 再把正常 limited motion 解释为 hold。

shared event 会记录每次 attempt 的 seed FK、Jacobian、IK 迭代/最终 FK、workspace、
collision、output-feasibility、remaining-check 和 total wall time。真机 hardware event 还会
记录 receiver/router、status sync、session、adapter dispatch、JSON 序列化/写入及完整
outer tick 耗时。`CONTROL_COMPUTE_BUDGET_EXHAUSTED` 表示 candidate 已被丢弃、未成为
权威目标，随后发送正常 `HOLD_REJECTED` heartbeat。

native 最终 jerk 检查保留配置中的 nominal limit 和 fault 等级，只增加明确的数值比较
包络：absolute tolerance 为 `1e-6 rad/s3`，relative tolerance 为
`2.5e-7 * abs(limit)`。该包络来自配置 `62.831853 rad/s3` 附近观测到的
`1.3e-5 rad/s3` finite-difference 差异；它不是运动参数。native cycle telemetry 会记录
`raw_output_jerk_rad_s3`、nominal limit、两个 tolerance 项，以及
`output_joint_jerk_hard_boundary_with_tolerance_rad_s3`。worker metrics 额外保留
`last_output_check_raw_jerk_rad_s3`，包括 hard fault 发生时最后一次尝试的检查值。

RH56 read-only summary 记录 by-id、resolved tty、USB VID/PID/serial、反馈 sample/rate/
latency/repeat、初末 `ANGLE_ACT`、raw ERROR/STATUS、timeout/checksum/protocol 计数和
`register_write_count`；只有写入数为 0 才是有效只读证据。逐帧记录会区分 requested、实际
selected/written 和 measured 数值，并包含 current/load、transport/control state、时间戳、
read latency、status/error、fault 和 episode validity。
