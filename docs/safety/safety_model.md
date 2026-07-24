# Safety model

## Fault classes

| Class | Examples | Required behavior |
|---|---|---|
| Input/producer liveness | stale Quest, producer or IPC timeout | stop physical output and clean up |
| Recoverable candidate infeasibility | IK, collision, singularity direction, output velocity/acceleration | `HOLD_REJECTED`, fresh heartbeat, hold last safe target |
| Native/controller hard fault | tracking error, servo alarm, collision, estop, power/enable loss, SDK error, hard timing fault | stop before another point and clean up |
| Operator action | clutch release, bounded gate end, explicit stop | stop/hold per stage and clean up |

The distinction prevents a rejected target from masquerading as dead
communication while ensuring that dead communication cannot masquerade as a
recoverable hold.

## Defense in depth

Shared policy rejects unsafe candidates before constructing
`AcceptedArmTarget`. The native worker independently checks continuity,
velocity, acceleration, tracking, liveness, timing, and controller health as
defensive assertions. Native checks must not silently reshape a target and
thereby make simulation and hardware different.

The startup hold measured after entering EDG is the physical authority.
Reference capture cannot waive startup continuity. Latest-destination
resampling is bounded and causal; rejected or old targets are never queued for
future playback.

## Configuration ownership

Payload, center of mass, installation, TCP, and controller safety limits belong
to controller/operator configuration. Current software reads health/state but
does not silently write these settings. A recorded value is evidence of a
previous operator report, not a guarantee of present controller state.

---

# 中文版：安全模型

## 故障分类

| 类别 | 示例 | 必须采取的行为 |
|---|---|---|
| 输入/producer 活性 | Quest 陈旧、producer 或 IPC 超时 | 停止真机输出并清理 |
| 可恢复候选不可行 | IK、碰撞、奇异方向、输出速度/加速度 | `HOLD_REJECTED`、新鲜 heartbeat、保持最后安全目标 |
| Native/控制器硬故障 | tracking error、servo alarm、collision、estop、power/enable 丢失、SDK、硬时序故障 | 在发送下一点前停止并清理 |
| 操作者动作 | clutch release、gate 到时、显式停止 | 按 stage 停止/保持并清理 |

这样可以避免把候选拒绝误判成通信死亡，同时也防止真正的通信死亡被伪装成可恢复 hold。

## 纵深防御

共享策略在构造 `AcceptedArmTarget` 前拒绝不安全候选。native worker 独立检查连续性、速度、
加速度、跟踪、活性、时序和控制器健康，作为最终防御。native 检查不能静默重塑目标，否则
会破坏仿真/真机一致性。

进入 EDG 后测得的 startup hold 是真机权威。捕获参考不能豁免启动连续性。latest-destination
重采样必须有界且因果；被拒绝或旧目标不能排队后续回放。

## 配置所有权

Payload、质心、安装方向、TCP 和控制器安全限制属于控制器/操作者。当前软件可以读取健康/
状态，但不会静默写入。历史记录值只是此前操作者证据，不保证控制器当前仍相同。
