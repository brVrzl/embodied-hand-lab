# Simulation and hardware parity

## Contract

Simulation and physical execution share validated Quest facts, clutch state,
reference capture, mapping, filters, continuation, IK, collision and
singularity checks, output feasibility, and the immutable accepted target.
The maintained physical collection runtime has one explicit policy overlay:
it does not apply the live-demo `maximum_target_displacement_m` as a
clutch-relative task-travel envelope. The value remains available to
simulation/replay. The arm and JAKA adapters still diverge only at the output
adapter and do not add mapping, filtering, IK, or shaping.

```text
                         +-> MuJoCo accepted-joint adapter
AcceptedArmTarget -------+
                         +-> JAKA accepted-joint adapter -> EDG worker
```

The hardware path must not:

- read or follow MuJoCo `qpos`;
- independently recompute IK;
- repeat the latest target as a staircase;
- replay a queue of stale destinations;
- write controller configuration.

In current joint-teleop mode, native JAKA inverse-kinematics call count must
remain zero. The worker emits `edg_servo_j(..., ABS, 1)` at a target period of
8 ms. Its piecewise-linear resampler continuously moves from the last emitted
point toward the latest accepted destination. A newer destination replaces the
active segment without building a backlog.

## Startup and stop

The measured post-EDG joint state (`q_hold`) is authoritative. The first
accepted target must be continuous with it; clutch reference capture alone
cannot legalize a joint jump. Release, timeout, tracking error, controller
fault, SDK error, or hard loop-timing fault terminates output and runs cleanup.

The current worker uses the sole JAKA SDK session. Every two 8 ms cycles it
performs the lightweight status read. Only an unhealthy status triggers the
additional emergency-stop/collision queries. A prior two-session monitoring
design was tried physically and failed safely because the second login kept the
primary worker from reaching `CONNECTED`; it is historical, not current.

## Evidence boundary

Offline fake-worker tests verify serialization, resampling, startup,
latest-destination behavior, zero native IK, timing accounting, controller
health policy, and cleanup. Historical physical evidence validates selected
foundation and timing behavior only. The latest output-acceleration correction
has not yet received a post-fix physical gate.

---

# 中文版：仿真与真机一致性

## 共享契约

仿真和真机共用 Quest 有效输入、clutch 状态、参考捕获、映射、滤波、continuation、IK、
碰撞/奇异检查、输出可行性以及不可变已接受目标。维护中的物理采集 runtime 有一个明确的
policy overlay：不把 live-demo 的 `maximum_target_displacement_m` 用作相对 clutch 参考点的任务
行程 envelope；该值仍供仿真/回放使用。arm 和 JAKA adapter 仍只在输出适配器处分开，不在
adapter 中增加 mapping、filter、IK 或 shaping：

```text
                         +-> MuJoCo accepted-joint adapter
AcceptedArmTarget -------+
                         +-> JAKA accepted-joint adapter -> EDG worker
```

硬件路径禁止：

- 读取或跟随 MuJoCo `qpos`；
- 独立重新求解 IK；
- 以 repeat-latest 阶梯形式发送目标；
- 重放陈旧目标队列；
- 写入控制器配置。

当前 joint-teleop 模式中 native JAKA IK 调用数必须为零。worker 以 8 ms 周期执行
`edg_servo_j(..., ABS, 1)`。分段线性重采样器从最后输出点连续靠近最新已接受目标；
新目标替换活动段，不积累 backlog。

## 启动和停止

进入 EDG 后测得的关节状态 `q_hold` 是启动权威。第一个已接受目标必须与其连续，单独捕获
clutch 参考不能让关节跳变合法化。clutch release、超时、跟踪误差、控制器故障、SDK
错误或硬时序故障都会终止输出并执行清理。

当前 worker 只使用一个 JAKA SDK 会话，每两个 8 ms 周期读取一次轻量状态；只有状态异常
时才进一步查询急停/碰撞。曾经测试过双会话监控，但第二次登录阻止主 worker 进入
`CONNECTED`，因此该设计只保留在历史证据中。

## 证据边界

离线 fake worker 验证序列化、重采样、启动、latest-destination、native zero-IK、时序、
控制器健康策略和清理。历史真机证据只覆盖其明确的基础与时序范围。最新输出加速度修复尚未
完成修复后的真机 gate。
