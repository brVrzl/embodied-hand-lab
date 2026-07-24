# JAKA arm teleoperation

The current physical entry point is `tools/quest_jaka_hardware.py`. It is not a
normal quick-start command. Inspecting help is safe:

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

Its stages are deliberately separated (`p2-shadow`, `e2-isolated`, `p4-live`,
and `post-payload-diagnostic`) and require stage-specific acknowledgements.
Never copy an old historical invocation without reconciling it with current
`--help`, current config, and the approved gate.

## Current runtime contract

- Target generation is 60 Hz; native output is approximately 125 Hz (8 ms).
- Physical output consumes the shared immutable accepted six-joint target.
- Joint-teleop mode performs zero native JAKA IK calls.
- Output is absolute `edg_servo_j(..., ABS, 1)`.
- Resampling is piecewise-linear toward the latest destination with no stale
  queue replay.
- Post-EDG `q_hold` is authoritative and first engagement must be continuous.
- `HOLD_REJECTED` holds the last safe target with a fresh heartbeat.
- Actual liveness loss, tracking fault, controller alarm, collision, SDK error,
  or hard timing failure stops and cleans up.
- The sole SDK session performs lightweight health polling every two command
  cycles; extended collision/estop queries occur only after unhealthy status.

See [current status](../status/current_status.md) before proposing a physical
stage. The next recommended gate is not yet authorized and must occur in a new
session.

## Current bounded manual command entry

The repository provides a wrapper only for the currently recommended bounded
post-payload diagnostic. Inspecting help does not connect:

```bash
./scripts/run_quest_jaka_post_payload_manual.sh --help
```

After verifying the controller and receiving explicit authorization for this
exact gate, the operator may run:

```bash
./scripts/run_quest_jaka_post_payload_manual.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 60 \
  --approval I_AUTHORIZE_ONE_POST_PAYLOAD_TELEOP_RERUN \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

Verify the IP addresses instead of treating these recorded values as permanent
configuration. The wrapper always selects `post-payload-diagnostic`, limits the
run to at most 60 seconds, uses a 1.0 rad/s run-specific output boundary,
enables the pre-SDK acceleration abort, and creates timestamped logs. It cannot
transition to another gate. Release the left-index clutch or press Ctrl+C to
stop. Any alarm or hard fault requires evidence review, not an automatic retry.

---

# 中文版：JAKA 机械臂遥操作

当前真机入口是 `tools/quest_jaka_hardware.py`，它不是普通 quick start。安全的帮助检查：

```bash
.venv/bin/python tools/quest_jaka_hardware.py --help
```

stage 被明确分成 `p2-shadow`、`e2-isolated`、`p4-live` 和
`post-payload-diagnostic`，每个都要求精确 acknowledgement。不得直接复制历史命令，
必须与当前 `--help`、配置和已批准 gate 核对。

## 当前运行契约

- 共享目标生成 60 Hz，native 输出约 125 Hz（8 ms）。
- 真机消费共享不可变六关节目标。
- joint-teleop 模式 native JAKA IK 调用数为零。
- 输出为 absolute `edg_servo_j(..., ABS, 1)`。
- 分段线性重采样靠近 latest destination，不重放旧队列。
- 进入 EDG 后的 `q_hold` 是启动权威，首次 engage 必须连续。
- `HOLD_REJECTED` 使用新鲜 heartbeat 保持最后安全目标。
- 真正的活性丢失、tracking fault、控制器报警、碰撞、SDK 或硬时序错误会停止并清理。
- 唯一 SDK 会话每两个命令周期执行一次轻量健康轮询。

在提出真机 gate 前先阅读[当前状态](../status/current_status.md)。当前推荐 gate 仍需新的
显式授权。

## 当前受限手动命令入口

以下 `--help` 不会连接真机：

```bash
./scripts/run_quest_jaka_post_payload_manual.sh --help
```

在控制器检查完成、并获得这个精确 gate 的明确授权后，操作者可以运行：

```bash
./scripts/run_quest_jaka_post_payload_manual.sh \
  --robot-ip 192.168.71.50 \
  --edg-state-ip 192.168.71.19 \
  --duration-sec 60 \
  --approval I_AUTHORIZE_ONE_POST_PAYLOAD_TELEOP_RERUN \
  --estop-accessible \
  --workspace-clear \
  --rh56-command-path-absent
```

IP 是当前记录值，运行前必须核实，不能当作永久配置。wrapper 固定选择
`post-payload-diagnostic`，最多运行 60 秒，使用 1.0 rad/s 的运行期输出边界，启用 SDK
下发前加速度中止，并生成带时间戳日志；它不能自动进入其他 gate。

停止方式：释放左手食指 clutch 或按 Ctrl+C。出现任何报警或硬故障后必须分析证据，不能
自动重试。
