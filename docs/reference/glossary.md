# Glossary

- **AcceptedArmTarget** — immutable six-joint target created only after shared
  mapping, IK, continuity, collision, singularity, and output feasibility pass.
- **CTRL v1** — Quest left-controller sidecar packet used for clutch/grip facts.
- **EDG** — JAKA external guidance mode used by the native joint worker.
- **HOLD_REJECTED** — fresh-heartbeat state that holds the last safe target
  after a recoverable candidate rejection.
- **HTS** — Hand Tracking Streamer packet source for Quest hand/head poses.
- **latest destination** — newest accepted target toward which the native
  resampler moves without replaying a queued backlog.
- **plant-free hardware path** — MuJoCo supplies kinematics/collision checks but
  is not stepped as a simulated plant and its `qpos` is not followed.
- **q_hold** — measured joint state after entering EDG; startup continuity
  authority.
- **shared pipeline** — all input, mapping, filter, IK, and feasibility work
  before the simulation/hardware adapter split.
- **UMIP** — device-neutral observation contract in `src/motion_input`.

---

# 中文版：术语表

- **AcceptedArmTarget** — 只有通过共享映射、IK、连续性、碰撞、奇异性和输出可行性检查
  后才创建的不可变六关节目标。
- **CTRL v1** — Quest 左控制器 sidecar 数据包，用于传递 clutch/grip 状态。
- **EDG** — 原生关节 worker 使用的 JAKA 外部引导模式。
- **HOLD_REJECTED** — 可恢复候选目标被拒绝后，保持新鲜心跳并维持最后安全目标的状态。
- **HTS** — 提供 Quest 手部/头部位姿的 Hand Tracking Streamer 数据源。
- **latest destination** — 原生重采样器当前追踪的最新接受目标；不会回放排队积压。
- **plant-free hardware path** — MuJoCo 提供运动学/碰撞检查，但不作为仿真 plant 运行，
  真机也不跟随其 `qpos`。
- **q_hold** — 进入 EDG 后测得的关节状态，是启动连续性的权威基线。
- **shared pipeline** — 仿真/硬件 adapter 分叉之前的全部输入、映射、滤波、IK 和可行性
  处理。
- **UMIP** — `src/motion_input` 中与设备无关的观测契约。
